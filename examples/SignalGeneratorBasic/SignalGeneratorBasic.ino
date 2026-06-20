/**
 * SignalGeneratorBasic.ino
 * @version V1R1
 * @date 06-16-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * SignalGeneratorBasic.ino - Stream a synthesized sine wave to the Serial Plotter
 *
 * Drives the SignalCore SignalGenerator at a 10 kSPS virtual sample rate and
 * prints the resulting waveform one sample per line. The Arduino Serial
 * Plotter renders the output directly, which makes this sketch a useful
 * sanity check on any board the library compiles for — no analog hardware
 * needed.
 *
 * This demonstrates:
 * - SignalGenerator carrier, AM, and noise configuration
 * - Pure-software signal synthesis with no analog hardware
 * - The `using namespace signalcore;` pattern recommended for sketches
 *
 * Output: Serial Plotter (time-series), 115200 baud — one synthesized sample
 *         per line; the trace is the carrier shaped by AM and noise.
 *
 * Hardware: Any Arduino-compatible board
 *
 * Sketch outline:
 * - setup() configures a 200 Hz sine carrier with light AM and noise
 * - loop() pulls one sample at a time and prints it to Serial
 *
 * Things to try:
 * - Change CARRIER_HZ between 50 and 4000 to sweep the visible frequency
 * - Set NOISE_PERCENT to 0 to see a clean carrier, then ramp up to 20
 * - Swap WAVEFORM through Sine, Square, Triangle, and Sawtooth to compare shapes
 *
 * Pairs with ViewPoint: SpectrumAnalyzer, SignalAndNoise, SignalAnalyzer_A0
 */

#include <SignalCore.h>
using namespace signalcore;

#define SAMPLE_RATE    10000               // virtual SPS for the synthesizer
#define CARRIER_HZ     200.0               // base tone (Hz)
#define WAVEFORM       SignalShape::Sine   // Sine, Square, Triangle, or Sawtooth
#define AM_HZ          7.0                 // amplitude-modulation rate (Hz)
#define AM_PERCENT     30.0                // AM depth 0..100
#define NOISE_PERCENT  3.0                 // gaussian noise level 0..100

SignalGenerator sig(SAMPLE_RATE);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }

    sig.setCarrier(WAVEFORM, CARRIER_HZ);
    sig.setAM(SignalShape::Sine, AM_HZ, AM_PERCENT);
    sig.setNoise(NOISE_PERCENT);
}

void loop() {
    double sample = sig.nextSample(0);   // 0 = full-resolution float output
    Serial.println((float)sample, 4);

    // Pacing keeps the Serial Plotter readable on slow USB-CDC links.
    delayMicroseconds(1000000UL / SAMPLE_RATE);
}
