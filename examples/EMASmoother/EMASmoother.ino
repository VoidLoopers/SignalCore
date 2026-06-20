/**
 * EMASmoother.ino
 * @version V1R1
 * @date 06-16-26
 *
 * SignalCore is brought to you by VoidLoop. Docs: https://docs.voidloop.com/libraries/SignalCore/
 * EMASmoother.ino - Smooth a noisy analog read with ExponentialMovingAverage
 *
 * Reads A0 once per loop, packs the value into a single-element vector, and
 * pushes it through ExponentialMovingAverage. The raw and smoothed values
 * are printed side by side so the Arduino Serial Plotter shows the effect
 * of the sampleWeight parameter on transient response. This is the gentle, scalar
 * introduction to the class; its primary use — smoothing an entire spectrum
 * vector frame by frame — is shown in SpectrumSmoothingPeakHold.
 *
 * This demonstrates:
 * - Per-element smoothing via signalcore::ExponentialMovingAverage
 * - The implicit float* conversion of the class
 * - Sensible defaults for sampleWeight (lower = heavier smoothing, slower settle)
 *
 * Output: Serial Plotter (time-series), 115200 baud — two traces, raw and
 *         smoothed, so you can see the filter lag the input.
 *
 * Hardware: Any Arduino-compatible board with an analog input on A0
 *           (A floating pin works fine — the noise is what we want to filter)
 *
 * Sketch outline:
 * - setup() opens Serial
 * - loop() reads A0, updates the EMA, prints raw + smoothed values
 *
 * Things to try:
 * - Drop SMOOTHING_WEIGHT to 0.05 to see noticeably heavier smoothing
 * - Raise SMOOTHING_WEIGHT to 0.8 to see the filter mostly pass through
 * - Touch A0 with a finger and watch the smoothed line trail the raw one
 *
 * Pairs with ViewPoint: SignalAnalyzer_A0
 */

#include <SignalCore.h>
using namespace signalcore;

#define SMOOTHING_WEIGHT  0.2f    // 0.0 = freeze, 1.0 = no smoothing
#define UPDATE_INTERVAL   20      // ms between samples

ExponentialMovingAverage ema(/*size=*/1, SMOOTHING_WEIGHT);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait briefly for USB */ }
}

void loop() {
    float raw = (float)analogRead(A0);
    ema.update(&raw);

    // The class exposes both `operator[]` and an implicit float* conversion.
    // Using a dedicated pointer keeps subscript lookup unambiguous.
    float* smoothed = ema;

    Serial.print("raw:");        Serial.print(raw, 1);
    Serial.print("\tsmoothed:"); Serial.println(smoothed[0], 1);

    delay(UPDATE_INTERVAL);
}
