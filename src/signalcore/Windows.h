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
 * Blackman, Blackman-Nuttall, Blackman-Harris, flat-top, and Welch.
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
                                                 "Unknown";
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
            case WindowType::Hamming:
                array[i] = 0.54f - (0.46f * cosf((2.0f * (float)M_PI) * ratio));
                break;
            case WindowType::Hann:
                // Hann peaks at 1.0 in the centre. The original SDSP copy
                // used the Hamming coefficient here, which biased amplitude
                // estimates by ~+0.67 dB. Corrected to 0.5.
                array[i] = 0.5f * (1.0f - cosf((2.0f * (float)M_PI) * ratio));
                break;
            case WindowType::Triangle:
                array[i] = 1.0f - ((2.0f * fabsf((float)i - (numSamplesMinus1 / 2.0f))) / numSamplesMinus1);
                break;
            case WindowType::Nuttall:
                array[i] = 0.355768f - (0.487396f * cosf((2.0f * (float)M_PI) * ratio))
                                     + (0.144232f * cosf((4.0f * (float)M_PI) * ratio))
                                     - (0.012604f * cosf((6.0f * (float)M_PI) * ratio));
                break;
            case WindowType::Blackman:
                // Blackman/FlatTop coefficient sets follow the legacy SDSP
                // tables, normalized so the window peaks at 1.0; they differ
                // slightly from the textbook "classic Blackman"
                // (0.42/0.50/0.08). FlatTop is a 3-term variant, not the
                // common 5-term flat-top — expect a narrower main lobe.
                array[i] = 0.42323f - (0.49755f * cosf((2.0f * (float)M_PI) * ratio))
                                    + (0.07922f * cosf((4.0f * (float)M_PI) * ratio));
                break;
            case WindowType::BlackmanNuttall:
                array[i] = 0.3635819f - (0.4891775f * cosf((2.0f * (float)M_PI) * ratio))
                                      + (0.1365995f * cosf((4.0f * (float)M_PI) * ratio))
                                      - (0.0106411f * cosf((6.0f * (float)M_PI) * ratio));
                break;
            case WindowType::BlackmanHarris:
                array[i] = 0.35875f - (0.48829f * cosf((2.0f * (float)M_PI) * ratio))
                                    + (0.14128f * cosf((4.0f * (float)M_PI) * ratio))
                                    - (0.01168f * cosf((6.0f * (float)M_PI) * ratio));
                break;
            case WindowType::FlatTop:
                array[i] = 0.2810639f - (0.5208972f * cosf((2.0f * (float)M_PI) * ratio))
                                      + (0.1980399f * cosf((4.0f * (float)M_PI) * ratio));
                break;
            case WindowType::Welch: {
                float t = ((float)i - numSamplesMinus1 / 2.0f) / (numSamplesMinus1 / 2.0f);
                array[i] = 1.0f - (t * t);
                break;
            }
        }
        windowSum += array[i];
    }
    return windowSum;
}

} // namespace signalcore

#endif // _SIGNALCORE_WINDOWS_H_
