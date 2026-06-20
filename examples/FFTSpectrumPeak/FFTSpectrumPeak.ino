/**
 * FFTSpectrumPeak.ino
 * @version V1R1
 * @date 06-17-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * FFTSpectrumPeak.ino - Generate a tone, run an FFT, and report the peak bin
 *
 * Fills a 512-sample buffer with a synthesized sine wave, removes DC, applies
 * a Blackman-Harris window, runs the FFT, and prints the bin number and
 * frequency where the spectrum peaks. The reported frequency should track
 * CARRIER_HZ within one bin (~SAMPLE_RATE / FFT_LENGTH).
 *
 * This demonstrates:
 * - The signalcore::FFT<N> template with windowing (N = FFT length)
 * - removeDC() before the transform to keep the DC bin out of the peak search
 * - FFT::peakBin() / peakFrequency() for dominant-tone detection, then postScaleDB()
 *
 * Output: Serial Monitor (numeric), 115200 baud — one line per FFT giving the
 *         peak bin, its frequency in Hz, and its level in dB.
 *
 * Hardware: Any Arduino-compatible board with at least ~6 KB free RAM
 *           (AVR boards are limited to 256 by SIGNALCORE_MAX_FFT_SIZE)
 *
 * Sketch outline:
 * - setup() configures the carrier and selects a Blackman-Harris window
 * - loop() fills the input buffer, removes DC, runs the FFT, prints the peak
 *
 * Things to try:
 * - Change CARRIER_HZ to 1234.5 and watch the peak frequency report a
 *   value near the closest bin centre
 * - Swap WindowType::BlackmanHarris for WindowType::Rectangle to see
 *   leakage broaden the apparent peak
 * - Reduce FFT_LENGTH to 256 to halve the RAM footprint (one edit: FFT_LENGTH
 *   drives the FFT<> template and the sample array together)
 *
 * Pairs with ViewPoint: SpectrumAnalyzer
 */

#include <SignalCore.h>
using namespace signalcore;

#define SAMPLE_RATE   10000
#define FFT_LENGTH    512
#define CARRIER_HZ    1500.0

FFT<FFT_LENGTH>    fft;
SignalGenerator    sig(SAMPLE_RATE);
float              samples[FFT_LENGTH];

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }

    sig.setCarrier(SignalShape::Sine, CARRIER_HZ);
    fft.setWindowType(WindowType::BlackmanHarris);
}

void loop() {
    for (int i = 0; i < FFT_LENGTH; i++) {
        samples[i] = (float)sig.nextSample(0);
    }

    // removeDC keeps any constant offset out of bin 0 (and so out of the peak
    // search). Synthesized centered sines carry almost no DC, but real ADC data
    // does — the call belongs here either way.
    removeDC(samples, FFT_LENGTH);

    fft.calculate(samples);

    // A real FFT of FFT_LENGTH samples packs FFT_LENGTH/2 + 1 unique magnitude
    // bins (DC .. Nyquist). peakBin() searches that range and skips the DC bin
    // by default, so a constant offset can't be mistaken for the tone. Read it
    // before postScaleDB(), which overwrites the buffer with dB.
    int   peakBin = fft.peakBin();
    float peakHz  = fft.peakFrequency(SAMPLE_RATE);

    // postScaleDB's 2nd arg is the input full-scale (not a gain): 1.0 matches the
    // unit-amplitude synthesized carrier. For ADC data it would be the volts or
    // counts at full scale.
    fft.postScaleDB(-100.0f, 1.0f);
    float* mags   = (float*)fft;
    float  peakDB = mags[peakBin];

    Serial.print("Peak bin: ");  Serial.print(peakBin);
    Serial.print("  Hz: ");      Serial.print(peakHz, 2);
    Serial.print("  dB: ");      Serial.println(peakDB, 2);

    delay(250);
}
