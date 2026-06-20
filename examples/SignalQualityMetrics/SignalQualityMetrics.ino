/**
 * SignalQualityMetrics.ino
 * @version V1R1
 * @date 06-17-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * SignalQualityMetrics.ino - Measure SINAD and ENOB from a tone's spectrum
 *
 * How good is a signal — or the converter that captured it? Two numbers answer
 * that from a single FFT. SINAD (signal-to-noise-and-distortion ratio, in dB)
 * compares the tone's power to everything else in the spectrum; ENOB (effective
 * number of bits) restates SINAD as the resolution an ideal converter would
 * need to be this clean. This sketch synthesizes a sine, removes DC, transforms
 * it, and computes both metrics on the linear magnitudes, printing them once a
 * second alongside the detected peak.
 *
 * A subtlety worth internalizing: a single non-coherent FFT understates ENOB,
 * because a tone that doesn't complete a whole number of cycles in the window
 * leaks across bins and looks like noise. CARRIER_HZ is chosen here to be
 * coherent (COHERENT_CYCLES whole cycles per window), which is why a rectangular
 * window is used — no window is needed when the tone already closes on itself.
 *
 * This demonstrates:
 * - SINAD() and ENOB() computed on LINEAR magnitudes (before any dB scaling)
 * - removeDC() before the transform and FFT::peakBin() with the DC bin skipped
 * - Coherent sampling: picking a frequency that lands exactly on a bin
 *
 * Output: Serial Monitor (metrics line), 115200 baud — SINAD (dB), ENOB (bits),
 *         and the peak bin / frequency, once per second.
 *
 * Hardware: Any Arduino-compatible board with RAM for one N512 FFT (~6 KB).
 *           AVR is capped at 256.
 *
 * Sketch outline:
 * - setup() computes the coherent carrier and configures the generator
 * - loop() synthesizes a window of samples, runs the FFT, prints the metrics
 *
 * Things to try:
 * - Raise NOISE_PERCENT and watch SINAD and ENOB fall together
 * - Set COHERENT_CYCLES to a non-integer-friendly tone (e.g. add 0.5 to
 *   CARRIER_HZ directly) to see the leakage penalty drag ENOB down
 * - Switch WINDOW to Hann or BlackmanHarris and compare the estimate
 */

#include <SignalCore.h>
using namespace signalcore;

#define SAMPLE_RATE     10000              // virtual SPS for the synthesizer
#define FFT_LEN         512                // FFT length; drives FFT<> and samples. AVR: 256
#define BIN_COUNT       (FFT_LEN / 2 + 1)  // unique magnitude bins (DC..Nyquist)
#define COHERENT_CYCLES 64                 // whole cycles in the window -> coherent
#define NOISE_PERCENT   1.0                // additive noise 0..100
#define WINDOW          WindowType::Rectangle  // coherent tone needs no window

// A frequency of CYCLES * SAMPLE_RATE / FFT_LEN lands exactly on bin CYCLES,
// so the tone completes whole cycles in the window and does not leak.
const double CARRIER_HZ = (double)COHERENT_CYCLES * SAMPLE_RATE / FFT_LEN;

FFT<FFT_LEN>    fft;
SignalGenerator sig(SAMPLE_RATE);
float           samples[FFT_LEN];

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }

    sig.setCarrier(SignalShape::Sine, CARRIER_HZ);
    sig.setNoise(NOISE_PERCENT);
    fft.setWindowType(WINDOW);

    Serial.print("Coherent carrier: "); Serial.print(CARRIER_HZ, 2);
    Serial.print(" Hz (bin ");          Serial.print(COHERENT_CYCLES);
    Serial.println(")");
}

void loop() {
    for (int i = 0; i < FFT_LEN; i++) samples[i] = (float)sig.nextSample(0);
    removeDC(samples, FFT_LEN);
    fft.calculate(samples);

    // SINAD/ENOB/findPeak all read the LINEAR magnitudes produced by calculate.
    // Note: postScaleDB is not used here as it would overwrite the buffer with dB and corrupt the calculations
    float* mags  = (float*)fft;
    float  sinad = SINAD(mags, BIN_COUNT);
    float  enob  = ENOB(mags, BIN_COUNT);

    int   peakBin = fft.peakBin();                  // skip DC (bin 0)
    float peakHz  = fft.peakFrequency(SAMPLE_RATE);

    Serial.print("SINAD: ");   Serial.print(sinad, 2); Serial.print(" dB");
    Serial.print("\tENOB: ");  Serial.print(enob, 2);  Serial.print(" bits");
    Serial.print("\tpeak: bin "); Serial.print(peakBin);
    Serial.print(" ("); Serial.print(peakHz, 1); Serial.println(" Hz)");

    delay(1000);
}
