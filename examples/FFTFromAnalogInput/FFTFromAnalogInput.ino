/**
 * FFTFromAnalogInput.ino
 * @version V1R1
 * @date 06-17-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * FFTFromAnalogInput.ino - Spectrum of a real ADC signal on A0
 *
 * This is the bridge from a physical signal to a spectrum. The sketch block-reads
 * FFT_LEN samples from analogRead(A0) at a paced rate, feeds the raw uint16_t
 * buffer straight into the FFT's integer overload, and reports the dominant
 * frequency. Two relationships matter: the read pacing sets the effective sample
 * rate, and the spectrum can only represent frequencies up to half that rate
 * (the Nyquist limit, here SAMPLE_RATE / 2). Anything above Nyquist folds back
 * and lies about its frequency, so pace the reads for the band you care about.
 *
 * This demonstrates:
 * - FFT::calculate(uint16_t*) reading an ADC buffer directly
 * - postScaleDB() with the ADC full-scale (in counts) as the reference
 * - FFT::peakBin() (DC skipped) and where removeDC fits the integer path
 *
 * Output: Serial Monitor (numeric), 115200 baud — dominant frequency in Hz and
 *         its level in dBFS, one line per captured block.
 *
 * Hardware: A board with an ADC on A0 (Teensy 4.x, ESP32, etc.) and a signal
 *           source — a function generator or AC-coupled audio biased into the
 *           ADC's input range. Runs on any ADC board; no DMA required.
 *
 * Sketch outline:
 * - setup() sets the ADC resolution and opens Serial
 * - loop() paces a block of reads, runs the FFT, prints the dominant tone
 *
 * Things to try:
 * - Feed different tones and confirm the reported Hz tracks the source
 * - Change SAMPLE_RATE to move the Nyquist limit; feed a tone above it and
 *   watch the reported frequency alias to a lower (wrong) value
 * - Short A0 to GND to see only the noise floor with no dominant peak
 *
 * Pairs with ViewPoint: SignalAnalyzer_A0
 */

#include <SignalCore.h>
using namespace signalcore;

#define ADC_PIN      A0          // analog input
#define ADC_BITS     12          // analogReadResolution; Teensy/ESP32 support 12
#define FFT_LEN      512         // FFT length; drives FFT<> and adcBuffer. AVR: 256
#define SAMPLE_RATE  8000        // target SPS set by read pacing; Nyquist = SR/2

// Full-scale in ADC counts. calculate(uint16_t*) casts raw counts to float, so
// the magnitudes are in counts and the dB reference must be too.
const float ADC_FULL_SCALE = (float)((1UL << ADC_BITS) - 1);

FFT<FFT_LEN> fft;
uint16_t     adcBuffer[FFT_LEN];

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }

    analogReadResolution(ADC_BITS);
    fft.setWindowType(WindowType::BlackmanHarris);
}

void loop() {
    // Pace the reads to a fixed deadline so the effective sample rate is known.
    const uint32_t periodUs = 1000000UL / SAMPLE_RATE;
    uint32_t       next     = micros();
    for (int i = 0; i < FFT_LEN; i++) {
        while ((int32_t)(micros() - next) < 0) { /* wait for this sample's slot */ }
        adcBuffer[i] = (uint16_t)analogRead(ADC_PIN);
        next += periodUs;
    }

    fft.calculate(adcBuffer);   // integer overload: counts are cast to float

    // calculate(uint16_t*) cannot remove DC in place, so the ADC's large bias
    // lands entirely in bin 0. peakBin() skips it by default; to strip DC
    // explicitly, copy the counts into a float buffer and call removeDC() before
    // calculate() — removeDC operates on the float path.
    int   peakBin = fft.peakBin();                  // skips DC (bin 0)
    float peakHz  = fft.peakFrequency(SAMPLE_RATE);

    // postScaleDB's 2nd arg is the input full-scale (counts here), giving dBFS.
    fft.postScaleDB(-100.0f, ADC_FULL_SCALE);
    float* mags   = (float*)fft;
    float  peakDB = mags[peakBin];

    Serial.print("Dominant: "); Serial.print(peakHz, 1); Serial.print(" Hz");
    Serial.print("  level: ");  Serial.print(peakDB, 2);  Serial.println(" dBFS");

    delay(500);
}
