/**
 * SpectrumSmoothingPeakHold.ino
 * @version V1R1
 * @date 06-16-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * SpectrumSmoothingPeakHold.ino - EMA vs peak-hold on a live, swept spectrum
 *
 * A raw FFT magnitude frame is noisy and twitchy. Two post-processors tame it
 * in opposite ways: an exponential moving average smooths each bin toward its
 * recent history (steady but laggy), while a peak-hold latches each bin's
 * maximum and lets it bleed down slowly (captures transients you would
 * otherwise miss). This sketch sweeps a tone across the band and watches a few
 * fixed bins: as the tone passes a watched bin you'll see the instant value
 * spike, the smoothed value lag behind it, and the held value jump up then
 * linger as the tone moves on.
 *
 * This demonstrates:
 * - ExponentialMovingAverage smoothing a whole magnitude vector per frame
 * - PeakHoldProcessor latching per-bin maxima with exponential decay
 * - removeDC() + FFT + windowing feeding both processors from one buffer
 *
 * Output: Serial Monitor (numeric), 115200 baud — per frame, the sweep tone
 *         frequency and, for each watched bin, instant / smoothed / held levels.
 *
 * Hardware: Any Arduino-compatible board with RAM for one N512 FFT plus two
 *           257-element vectors. AVR is capped at 256.
 *
 * Sketch outline:
 * - setup() selects a window and primes the swept generator
 * - loop() retunes the tone, runs the FFT, updates EMA + peak-hold, reports bins
 *
 * Things to try:
 * - Raise HOLD_DECAY toward 0.99 and watch the held values linger much longer
 * - Drop EMA_SAMPLE_WEIGHT to 0.05 for heavy smoothing that visibly trails the sweep
 * - Increase SWEEP_STEP so the tone races through bins and the held peak trails
 *
 * Pairs with ViewPoint: SpectrumAnalyzer
 */

#include <SignalCore.h>
using namespace signalcore;

#define SAMPLE_RATE   10000              // virtual SPS for the synthesizer
#define FFT_LEN       512                // FFT length; drives FFT<> and samples. AVR: 256
#define BIN_COUNT     (FFT_LEN / 2 + 1)  // unique magnitude bins (DC..Nyquist)
#define SWEEP_LO      500.0              // sweep start (Hz)
#define SWEEP_HI      3000.0            // sweep end (Hz)
#define SWEEP_STEP    25.0             // Hz advanced per frame
#define NOISE_PERCENT 5.0            // additive noise 0..100
#define EMA_SAMPLE_WEIGHT     0.2f          // EMA weight on the newest frame; lower = heavier
#define HOLD_DECAY    0.95f        // peak-hold retention; closer to 1.0 = slower decay

FFT<FFT_LEN>             fft;
SignalGenerator          sig(SAMPLE_RATE);
ExponentialMovingAverage ema(BIN_COUNT, EMA_SAMPLE_WEIGHT);
PeakHoldProcessor        hold(BIN_COUNT, HOLD_DECAY);
float                    samples[FFT_LEN];

// Fixed bins to watch the sweep pass through (about 1.0, 1.5, 2.0 kHz).
const int WATCH_BINS[] = { 51, 77, 102 };
const int WATCH_COUNT  = sizeof(WATCH_BINS) / sizeof(WATCH_BINS[0]);

double sweepHz = SWEEP_LO;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }

    fft.setWindowType(WindowType::Hann);
    sig.setNoise(NOISE_PERCENT);
}

void loop() {
    // Retune each frame. setCarrier resets the carrier phase, which is harmless
    // here because every frame is an independent windowed transform.
    sig.setCarrier(SignalShape::Sine, sweepHz);

    for (int i = 0; i < FFT_LEN; i++) samples[i] = (float)sig.nextSample(0);
    removeDC(samples, FFT_LEN);
    fft.calculate(samples);

    float* mags = (float*)fft;        // linear magnitudes, bins 0..N/2
    ema.update(mags);                 // smooths the entire vector in place
    hold.update(mags, BIN_COUNT);     // per-bin running max with decay

    float* smoothed = ema;
    float* held     = hold;

    Serial.print("sweep "); Serial.print((float)sweepHz, 0); Serial.print(" Hz");
    for (int w = 0; w < WATCH_COUNT; w++) {
        int   b  = WATCH_BINS[w];
        float hz = (float)b * SAMPLE_RATE / FFT_LEN;
        Serial.print("  | "); Serial.print((int)hz); Serial.print("Hz");
        Serial.print(" i=");  Serial.print(mags[b],     3);
        Serial.print(" s=");  Serial.print(smoothed[b], 3);
        Serial.print(" h=");  Serial.print(held[b],     3);
    }
    Serial.println();

    sweepHz += SWEEP_STEP;
    if (sweepHz > SWEEP_HI) sweepHz = SWEEP_LO;

    // Rendering the full smoothed and held spectra as live curves is what
    // ViewPoint's SpectrumAnalyzer does; here we sample a few bins numerically.
    delay(100);
}
