/******************************************************************************
 * @file Platform.h
 * @brief Platform Detection and Math Helpers for SignalCore
 *
 * Detects available hardware DSP acceleration at compile time and selects the
 * backend. Automatic CMSIS-DSP enablement is intentionally conservative:
 *
 *   SIGNALCORE_USE_CMSIS   — Teensy 4.x by default, or other ARM boards when
 *                            the sketch defines SIGNALCORE_FORCE_CMSIS before
 *                            including SignalCore.h and the core links
 *                            CMSIS-DSP symbols
 *   SIGNALCORE_USE_ESP_DSP — Reserved for ESP32 with Espressif esp-dsp library
 *   SIGNALCORE_USE_GENERIC — Pure C++ fallback for every other platform
 *
 * Some ARM cores ship arm_math.h without automatically linking the DSP
 * objects, so automatic CMSIS detection is limited to Teensy 4.x where
 * Teensyduino provides the full integration. Other ARM cores must opt in
 * with SIGNALCORE_FORCE_CMSIS.
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

#ifndef _SIGNALCORE_PLATFORM_H_
#define _SIGNALCORE_PLATFORM_H_

// --- ARM CMSIS-DSP detection ---

#if defined(TEENSYDUINO)
    #define SIGNALCORE_USE_CMSIS 1
#elif (defined(__arm__) || defined(__ARM_ARCH)) && defined(SIGNALCORE_FORCE_CMSIS)
    #define SIGNALCORE_USE_CMSIS 1
#endif

// --- ESP-DSP detection (reserved for future use) ---

#if !defined(SIGNALCORE_USE_CMSIS) && (defined(ESP32) || defined(ESP_PLATFORM))
    #if defined(__has_include)
        #if __has_include("esp_dsp.h")
            // #define SIGNALCORE_USE_ESP_DSP 1  // Enable when ESP-DSP wrapper is implemented
        #endif
    #endif
#endif

// --- Generic fallback ---

#if !defined(SIGNALCORE_USE_CMSIS) && !defined(SIGNALCORE_USE_ESP_DSP)
    #define SIGNALCORE_USE_GENERIC 1
#endif

// --- Platform capability limits ---

#if defined(__AVR__)
    // ATmega328 has 2 KB RAM; a 256-point FFT already needs ~3 KB of buffers.
    // Sketches that target larger AVR parts may raise this manually.
    #define SIGNALCORE_MAX_FFT_SIZE 256
#elif defined(ESP8266)
    #define SIGNALCORE_MAX_FFT_SIZE 1024
#else
    #define SIGNALCORE_MAX_FFT_SIZE 4096
#endif

// --- Common math fallbacks ---

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifndef M_TWOPI
    #define M_TWOPI (2.0 * M_PI)
#endif


namespace signalcore {

// Wrap a phase value into [0, 2*pi). Multi-period inputs are wrapped in a
// single pass per direction; large positive or negative excursions are
// supported because phase accumulators can overshoot at high modulation
// rates relative to the sample rate.
inline double wrapPhase(double phase) {
    while (phase >= M_TWOPI) {
        phase -= M_TWOPI;
    }
    while (phase < 0.0) {
        phase += M_TWOPI;
    }
    return phase;
}

} // namespace signalcore

#endif // _SIGNALCORE_PLATFORM_H_
