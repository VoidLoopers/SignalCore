/******************************************************************************
 * @file Windows.h
 * @brief Window Functions and Coefficient Generation
 *
 * Provides the WindowType enum and a coefficient generator that fills the
 * first half of a buffer with the selected window. The second half is the
 * mirror of the first and is applied symmetrically by the FFT's windowing
 * loop, which saves half the storage.
 *
 * Supported windows: rectangle, Hamming, Hann, triangle (Bartlett), Nuttall,
 * Blackman, Blackman-Nuttall, Blackman-Harris, flat-top, Welch, and the
 * 7-term Blackman-Harris (Albrecht).
 *
 * Platform: Any Arduino-compatible board (pure C++ math, no hardware dependencies)
 *
 *   This file is a part of the SignalCore project, a cross-platform digital
 *   signal processing toolkit released by VoidLoop. See www.voidloop.com for
 *   updates.
 *
 * Contributors:
 * Initial vision, feature definition, debugging and funding: Greg Kovacs
 * Architecture, Development, debugging and support: Zachariah Magee
 *
 * MIT License (MIT)
 *
 * Copyright (c) 2026 VoidLoop
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#ifndef _SIGNALCORE_WINDOWS_H_
#define _SIGNALCORE_WINDOWS_H_

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "Platform.h"


namespace signalcore {

// Choosing a window trades spectral leakage against frequency resolution:
// heavier tapers (Blackman-Harris, Nuttall) suppress leakage skirts but
// widen each peak; light tapers keep peaks narrow but leak more. Rules of
// thumb: Hann is the general-purpose default; FlatTop gives the most
// accurate amplitude readings (widest peak); Rectangle/None only for
// transients or signals exactly periodic in the capture buffer.
//
// None and Rectangle are functionally identical (no shaping applied; the
// FFT skips the multiply loop for both). Both exist so sketches can express
// intent: None = "windowing not considered", Rectangle = "I chose a
// rectangular window deliberately".
enum class WindowType : uint8_t {
    None,
    Rectangle,
    Hamming,
    Hann,
    Triangle,
    Nuttall,
    Blackman,
    BlackmanNuttall,
    BlackmanHarris,
    FlatTop,
    Welch,
    BlackmanHarris7,
};


// Short human-readable label for the window. Useful for serial diagnostics.
constexpr const char* getWindowNameAbbr(WindowType win) {
    return win == WindowType::None             ? "None"    :
           win == WindowType::Rectangle        ? "Rect"    :
           win == WindowType::Hamming          ? "Hamm"    :
           win == WindowType::Hann             ? "Hann"    :
           win == WindowType::Triangle         ? "Tria"    :
           win == WindowType::Nuttall          ? "Nutt"    :
           win == WindowType::Blackman         ? "Blkmn"   :
           win == WindowType::BlackmanNuttall  ? "Blk-Nt"  :
           win == WindowType::BlackmanHarris   ? "Blk-Har" :
           win == WindowType::FlatTop          ? "FltTop"  :
           win == WindowType::Welch            ? "Welch"   :
           win == WindowType::BlackmanHarris7  ? "BH7"     :
                                                 "Unknown";
}


// Full human-readable name. Pass abbr=true to get the short form.
constexpr const char* getWindowName(WindowType win, bool abbr = false) {
    return abbr                                ? getWindowNameAbbr(win) :
           win == WindowType::None             ? "None"             :
           win == WindowType::Rectangle        ? "Rectangle"        :
           win == WindowType::Hamming          ? "Hamming"          :
           win == WindowType::Hann             ? "Hann"             :
           win == WindowType::Triangle         ? "Triangle"         :
           win == WindowType::Nuttall          ? "Nuttall"          :
           win == WindowType::Blackman         ? "Blackman"         :
           win == WindowType::BlackmanNuttall  ? "Blackman-Nuttall" :
           win == WindowType::BlackmanHarris   ? "Blackman-Harris"  :
           win == WindowType::FlatTop          ? "Flat Top"         :
           win == WindowType::Welch            ? "Welch"            :
           win == WindowType::BlackmanHarris7  ? "Blackman-Harris 7":
                                                 "Unknown";
}


// Double-precision cosine-series coefficient tables, shared by the float
// display path below (createWindowCoefficients) and by measure/SineMetrics.h,
// which needs double precision and its own periodic-form evaluation. lobeBins
// is the main-lobe half-width in bins, used by SineMetrics as a mask radius.
struct CosineSeries {
    const double* a;
    uint8_t terms;
    uint8_t lobeBins;
};

namespace detail {
    static const double kHamming[]         = {0.54, 0.46};
    static const double kHann[]            = {0.5, 0.5};
    static const double kNuttall[]         = {0.355768, 0.487396, 0.144232, 0.012604};
    static const double kBlackman[]        = {0.42323, 0.49755, 0.07922};
    static const double kBlackmanNuttall[] = {0.3635819, 0.4891775, 0.1365995, 0.0106411};
    static const double kBlackmanHarris[]  = {0.35875, 0.48829, 0.14128, 0.01168};
    static const double kFlatTop[]         = {0.2810639, 0.5208972, 0.1980399};
    // Albrecht's 7-term Blackman-Harris, highest sidelobe about -180 dB.
    static const double kBlackmanHarris7[] = {0.27105140069342, 0.43329793923448,
                                              0.21812299954311, 0.06592544638803,
                                              0.01081174209837, 0.00077658482522,
                                              0.00001388721735};
}

// Look up the cosine-series coefficients for `w`. Returns false for windows
// that are not a cosine series (None, Rectangle, Triangle, Welch) — those
// have no `out` written.
inline bool cosineSeries(WindowType w, CosineSeries& out) {
    switch (w) {
        case WindowType::Hamming:         out = {detail::kHamming, 2, 2};         return true;
        case WindowType::Hann:            out = {detail::kHann, 2, 2};            return true;
        case WindowType::Nuttall:         out = {detail::kNuttall, 4, 4};         return true;
        case WindowType::Blackman:        out = {detail::kBlackman, 3, 3};        return true;
        case WindowType::BlackmanNuttall: out = {detail::kBlackmanNuttall, 4, 4}; return true;
        case WindowType::BlackmanHarris:  out = {detail::kBlackmanHarris, 4, 4};  return true;
        case WindowType::FlatTop:         out = {detail::kFlatTop, 3, 3};         return true;
        case WindowType::BlackmanHarris7: out = {detail::kBlackmanHarris7, 7, 7}; return true;
        default:                          return false;
    }
}

// Window value at cos(theta), theta the phase for the sample being
// evaluated: w = a0 - a1 cos(theta) + a2 cos(2 theta) - ...
// Evaluated with the Chebyshev recurrence cos((m+1)t) = 2 cos t cos(mt) - cos((m-1)t)
// so each sample costs one cos() plus a few multiply-adds per term.
//
// NOTE: this evaluates the shape only — it says nothing about whether theta
// steps through the symmetric (denominator size-1, used below by
// createWindowCoefficients for the float display path) or periodic
// (denominator N, used by measure/SineMetrics.h) convention. SineMetrics
// uses the periodic form because that is what the host reference
// utilities/adc_measure.py uses, and the compare gate between them is
// bit-exact; the two conventions give slightly different coefficients near
// the buffer edges and must not be swapped.
inline double cosineSeriesValue(const CosineSeries& w, double cosTheta) {
    double cm1 = 1.0;        // cos(0 t)
    double c0  = cosTheta;   // cos(1 t)
    double acc = w.a[0];
    double sign = -1.0;
    for (int m = 1; m < w.terms; ++m) {
        acc += sign * w.a[m] * c0;
        double c1 = 2.0 * cosTheta * c0 - cm1;
        cm1 = c0;
        c0 = c1;
        sign = -sign;
    }
    return acc;
}


// Fill the first `size/2` entries of `array` with the selected window's
// coefficients and return the sum of those entries. The FFT path applies the
// window symmetrically, so only half of the coefficients are stored.
//
// The returned sum is the coherent-gain normalizer the FFT divides by (the
// single-sided 2/sum(w) amplitude correction) — it is NOT a noise-power
// (sum of squares) normalization, so it calibrates tone amplitudes, not
// noise density.
inline float createWindowCoefficients(float* array, uint32_t size, WindowType windowType) {
    float windowSum = 0.0f;
    // size - 1 selects the symmetric window form (w[0] == w[size-1]).
    // Changing this to size switches to the periodic convention and changes
    // FFT amplitude behavior.
    float numSamplesMinus1 = (float)size - 1.0f;
    for (uint16_t i = 0; i < size / 2; i++) {
        float ratio = (float)i / numSamplesMinus1;
        switch (windowType) {
            case WindowType::None:
            case WindowType::Rectangle:
                array[i] = 1.0f;
                break;
            case WindowType::Triangle:
                array[i] = 1.0f - ((2.0f * fabsf((float)i - (numSamplesMinus1 / 2.0f))) / numSamplesMinus1);
                break;
            case WindowType::Welch: {
                float t = ((float)i - numSamplesMinus1 / 2.0f) / (numSamplesMinus1 / 2.0f);
                array[i] = 1.0f - (t * t);
                break;
            }
            case WindowType::Hamming:
            case WindowType::Hann:
            case WindowType::Nuttall:
            case WindowType::Blackman:
            case WindowType::BlackmanNuttall:
            case WindowType::BlackmanHarris:
            case WindowType::FlatTop:
            case WindowType::BlackmanHarris7: {
                // Blackman/FlatTop coefficient sets follow the legacy SDSP
                // tables, normalized so the window peaks at 1.0; they differ
                // slightly from the textbook "classic Blackman"
                // (0.42/0.50/0.08). FlatTop is a 3-term variant, not the
                // common 5-term flat-top — expect a narrower main lobe.
                // Hann peaks at 1.0 in the centre. The original SDSP copy
                // used the Hamming coefficient here, which biased amplitude
                // estimates by ~+0.67 dB. Corrected to 0.5.
                CosineSeries cs;
                cosineSeries(windowType, cs);
                double theta = (2.0 * M_PI) * (double)ratio;
                array[i] = (float)cosineSeriesValue(cs, cos(theta));
                break;
            }
        }
        windowSum += array[i];
    }
    return windowSum;
}

} // namespace signalcore

#endif // _SIGNALCORE_WINDOWS_H_
