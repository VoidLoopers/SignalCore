/**
 * WindowComparison.ino
 * @version V1R1
 * @date 06-17-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * WindowComparison.ino - Compare FFT windows on a single off-bin tone
 *
 * Window functions trade main-lobe width for sidelobe suppression, and that
 * trade is the single most useful FFT lesson there is. This sketch synthesizes
 * one tone placed deliberately *between* bins (so spectral leakage is visible),
 * then runs the same data through three windows — Rectangle, Hann, and
 * Blackman-Harris. Then for each window it reports where the peak landed and how far
 * down the worst sidelobe sits. Watch the worst-sidelobe column drop by tens of
 * dB as the window gets gentler.
 *
 * This demonstrates:
 * - FFT<N> with setWindowType() across several WindowType values
 * - FFT::peakBin() for the dominant bin (DC skipped) on linear magnitudes
 * - getWindowNameAbbr() for labeling, and a hand-rolled sidelobe search
 *
 * Output: Serial Monitor (table), 115200 baud — one row per window:
 *         name, peak bin, peak Hz, peak dB, worst-sidelobe dB.
 *
 * Hardware: Any Arduino-compatible board with RAM for one N512 FFT (~6 KB).
 *           AVR is capped at 256 by SIGNALCORE_MAX_FFT_SIZE.
 *
 * Sketch outline:
 * - setup() synthesizes the off-bin tone once into a shared buffer
 * - loop() runs the FFT three times and prints the comparison table
 *
 * Things to try:
 * - Set CARRIER_HZ to an exact bin centre (a multiple of SAMPLE_RATE/FFT_LEN)
 *   and watch Rectangle's leakage collapse — coherent tones don't leak
 * - Raise NOISE_PERCENT and see the sidelobe floor lift for every window
 * - Add WindowType::FlatTop or Nuttall to the analyzeWindow() calls
 *
 * Pairs with ViewPoint: WindowFunctionExplorer
 */

#include <SignalCore.h>
using namespace signalcore;

#define SAMPLE_RATE   10000     // virtual SPS for the synthesizer
#define FFT_LEN       512       // FFT length; drives FFT<> and samples. AVR: 256
#define CARRIER_HZ    1234.5    // deliberately off-bin so leakage is visible
#define NOISE_PERCENT 0.0       // additive noise 0..100; raise to lift the floor
#define FULL_SCALE    1.0f      // input amplitude (unit-amplitude synthesis)

// Bins on each side of the peak to exclude from the sidelobe search, so the
// main lobe's own skirts are not mistaken for a sidelobe.
#define MAIN_LOBE_GUARD 2

FFT<FFT_LEN>    fft;
SignalGenerator sig(SAMPLE_RATE);
float           samples[FFT_LEN];

void analyzeWindow(WindowType win) {
    // calculate() copies and windows its own internal buffer, so the shared
    // `samples` array survives intact for the next window.
    fft.setWindowType(win);
    fft.calculate(samples);

    float*    mags = (float*)fft;
    const int bins = FFT_LEN / 2;                  // highest AC bin (Nyquist)

    int   peakBin = fft.peakBin();                 // skips DC; range result[0..N/2]
    float peakHz  = fft.peakFrequency(SAMPLE_RATE);

    // Worst sidelobe: the largest magnitude outside the main-lobe guard band.
    float sidelobe = 0.0f;
    for (int i = 1; i <= bins; i++) {
        if (abs(i - peakBin) <= MAIN_LOBE_GUARD) continue;
        if (mags[i] > sidelobe) sidelobe = mags[i];
    }

    float peakDB     = 20.0f * log10f(max(mags[peakBin], 1e-9f) / FULL_SCALE);
    float sidelobeDB = 20.0f * log10f(max(sidelobe,      1e-9f) / FULL_SCALE);

    Serial.print(getWindowNameAbbr(win));
    Serial.print('\t'); Serial.print(peakBin);
    Serial.print('\t'); Serial.print(peakHz, 1);
    Serial.print('\t'); Serial.print(peakDB, 2);
    Serial.print('\t'); Serial.println(sidelobeDB, 2);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }

    sig.setCarrier(SignalShape::Sine, CARRIER_HZ);
    sig.setNoise(NOISE_PERCENT);

    for (int i = 0; i < FFT_LEN; i++) samples[i] = (float)sig.nextSample(0);
    removeDC(samples, FFT_LEN);
}

void loop() {
    Serial.println();
    Serial.print("Tone ");           Serial.print(CARRIER_HZ, 1);
    Serial.print(" Hz, bin spacing "); Serial.print((float)SAMPLE_RATE / FFT_LEN, 2);
    Serial.println(" Hz");
    Serial.println("Window\tbin\tHz\tpeakdB\tsidelobedB");

    analyzeWindow(WindowType::Rectangle);
    analyzeWindow(WindowType::Hann);
    analyzeWindow(WindowType::BlackmanHarris);

    // Overlaying the full magnitude curves for each window is what ViewPoint's
    // WindowFunctionExplorer draws; here we reduce each to peak + worst sidelobe.
    delay(3000);
}
