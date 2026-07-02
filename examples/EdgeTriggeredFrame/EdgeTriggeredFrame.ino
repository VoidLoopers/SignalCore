/**
 * EdgeTriggeredFrame.ino
 * @version V1R1
 * @date 06-16-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * EdgeTriggeredFrame.ino - Phase-stable scope frames from edge triggering
 *
 * Every oscilloscope is built on one trick: instead of plotting whatever
 * samples happen to be in the buffer, find a repeatable feature — a rising edge
 * through a threshold — and align the display to it. Without that, a free-running
 * capture of a periodic signal scrolls and jitters. This sketch fills an
 * oversized capture buffer with a synthesized, slightly noisy wave each frame,
 * asks Trigger for the aligned window start, and streams that window to the
 * Serial Plotter. The trace stands still because the trigger removes the phase
 * drift between captures.
 *
 * This demonstrates:
 * - Trigger with TriggerMode, setThreshold(), setMinMax(), and setPosition()
 * - calculateOffset() returning an aligned frame inside a larger buffer
 * - valid() to detect "no edge found", and toString() for the mode label
 *
 * Output: Serial Plotter (time-series), 115200 baud — the "scope:" trace holds
 *         steady frame to frame; a "# ..." line prints when no edge is found.
 *
 * Hardware: Any Arduino-compatible board (pure synthesis, no analog input)
 *
 * Sketch outline:
 * - setup() configures the source wave and the trigger (mode, level, position)
 * - loop() refills the capture buffer, aligns a frame, and streams it
 *
 * Things to try:
 * - Switch TRIGGER_MODE to TriggerMode::FallingEdge to lock to the down-slope
 * - Raise TRIGGER_LEVEL above SIGNAL_AMPLITUDE to force "no trigger found"
 * - Set TRIGGER_POS to 0.1 to push the edge to the left of the frame
 * - Plot `buffer` directly instead of `frame` to watch an untriggered capture
 *   scroll for contrast
 *
 * Pairs with ViewPoint: SignalAnalyzer_A0
 */

#include <SignalCore.h>
using namespace signalcore;

#define SAMPLE_RATE      10000              // virtual SPS for the synthesizer
#define OUTPUT_SIZE      200                // samples shown per scope frame
#define INPUT_CAPACITY   600               // capture depth; must exceed OUTPUT_SIZE
#define CARRIER_HZ       200.0             // tone frequency (Hz)
#define SIGNAL_AMPLITUDE 1.0f             // peak amplitude of the synthesized wave
#define TRIGGER_MODE     TriggerMode::RisingEdge
#define TRIGGER_LEVEL    0.0f             // edge crossing level (centre of the wave)
#define TRIGGER_POS      0.5f             // where the edge sits in the frame, 0..1
#define NOISE_PERCENT    2.0             // jitter so triggering visibly earns its keep

SignalGenerator sig(SAMPLE_RATE);
Trigger         trig(TRIGGER_MODE, TRIGGER_LEVEL);
float           buffer[INPUT_CAPACITY];

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }

    sig.setCarrier(SignalShape::Sine, CARRIER_HZ, SIGNAL_AMPLITUDE);
    sig.setNoise(NOISE_PERCENT);

    // With an explicit threshold set, Trigger skips its auto min/max scan, so
    // tell it the signal range — that range sets the hysteresis margin.
    trig.setMinMax(-SIGNAL_AMPLITUDE, SIGNAL_AMPLITUDE);
    trig.setThreshold(TRIGGER_LEVEL);
    trig.setPosition(TRIGGER_POS);

    Serial.print("# Trigger mode: "); Serial.println(toString(trig.mode()));
}

void loop() {
    // The generator free-runs, so each capture starts at a different phase —
    // exactly the drift the trigger is there to cancel.
    for (int i = 0; i < INPUT_CAPACITY; i++) buffer[i] = (float)sig.nextSample(0);

    float* frame = trig.calculateOffset(buffer, OUTPUT_SIZE, INPUT_CAPACITY);

    if (!trig.valid()) {
        Serial.println("# no trigger found (lower TRIGGER_LEVEL or check the signal)");
        delay(200);
        return;
    }

    for (int i = 0; i < OUTPUT_SIZE; i++) {
        Serial.print("scope:");
        Serial.println(frame[i], 4);
    }

    delay(50);
}
