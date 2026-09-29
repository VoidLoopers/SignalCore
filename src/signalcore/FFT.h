/******************************************************************************
 * @file FFT.h
 * @brief Cross-Platform FFT and Spectral Analysis
 *
 * Provides a template-based FFT class, a runtime-sized DynamicFFT variant,
 * and spectral analysis helpers. Both classes run on RealFFT<float>
 * (fft/RealFFT.h), which picks its engine per instance: CMSIS-DSP
 * arm_rfft_fast_f32 for 32..4096 points on Teensy 4.x (or other ARM cores
 * with SIGNALCORE_FORCE_CMSIS), and the portable radix-2 engine with table
 * twiddles for every other size and platform. The output packing is the
 * same either way.
 *
 * Output format, in two stages:
 *   Raw transform packing (internal, matches CMSIS arm_rfft_fast_f32):
 *     result[0]      = DC (real), result[1] = Nyquist (real),
 *     result[2..N-1] = complex pairs [re, im] for bins 1..(N/2 - 1)
 *   After forward()/calculate():
 *     result[0..N/2] = linear amplitude magnitudes, DC through Nyquist
 *                      (N/2 + 1 valid bins); entries above N/2 are stale
 *                      scratch from the transform — never read them.
 *
 * Platform: Any Arduino-compatible board
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

#ifndef _SIGNALCORE_FFT_H_
#define _SIGNALCORE_FFT_H_

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <new>
#include <float.h>
#include <stdint.h>

#include "Platform.h"
#include "Windows.h"
#include "fft/RealFFT.h"


namespace signalcore {

// Named FFT sizes. This is an *unscoped* enum on purpose: the values implicitly
// convert to an integer, so FFT<FFTSize::N512> and FFT<512> name the same type,
// and a plain integer (e.g. a FFT_LENGTH macro) can drive both the template
// argument and an array declaration from a single source of truth.
//
// The names stop at 32768: that is the default SIGNALCORE_MAX_FFT_SIZE off
// AVR and ESP8266, and the largest power of two uint16_t holds. On a board
// with a lower cap the larger names still exist but fail the FFT<N>
// static_assert. Sizes above 4096 run the generic engine on every platform
// (CMSIS-DSP covers 32..4096).
enum FFTSize : uint16_t {
    N32    = 32,
    N64    = 64,
    N128   = 128,
    N256   = 256,
    N512   = 512,
    N1024  = 1024,
    N2048  = 2048,
    N4096  = 4096,
    N8192  = 8192,
    N16384 = 16384,
    N32768 = 32768
};


// --- Magnitude of interleaved complex pairs ---

namespace detail {

#if defined(SIGNALCORE_USE_CMSIS)
    inline void cmplxMagF32(float* pSrc, float* pDst, uint32_t numSamples) {
        arm_cmplx_mag_f32(pSrc, pDst, numSamples);
    }
#else
    // Safe for in-place operation.
    inline void cmplxMagF32(float* pSrc, float* pDst, uint32_t numSamples) {
        for (uint32_t i = 0; i < numSamples; i++) {
            float re = pSrc[2 * i];
            float im = pSrc[2 * i + 1];
            pDst[i] = sqrtf(re * re + im * im);
        }
    }
#endif

} // namespace detail


// --- Fast log approximation ---

// Polynomial approximation of log2(x), accurate to ~0.01 dB across the float
// range. Used by the fast dB conversion path on platforms where log10f is
// expensive.
inline float log2fApprox(float X) {
    float Y, F;
    int E;
    F = frexpf(fabsf(X), &E);
    Y = 1.23149591368684f;
    Y *= F;
    Y += -4.11852516267426f;
    Y *= F;
    Y += 6.02197014179219f;
    Y *= F;
    Y += -3.13396450166353f;
    Y += E;
    return Y;
}

inline float log10fFast(float x) {
    return log2fApprox(x) * 0.3010299956639812f;
}


// --- Helper functions ---

inline void removeDC(float* samples, int size) {
    float m = 0.0f;
    for (int i = 0; i < size; i++) m += samples[i];
    m = m / (float)size;
    for (int i = 0; i < size; i++) samples[i] -= m;
}


// Index of the largest element in data[start .. size-1], returned as an
// absolute index. `start` lets a caller skip leading bins without slicing the
// pointer and re-adding the offset: pass 1 to ignore the DC bin (index 0) of an
// FFT magnitude buffer when searching for a tone.
inline int findPeak(float* data, int size, int start = 0) {
    double peak = -DBL_MAX;
    int bin = start;
    for (int i = start; i < size; i++) {
        if (data[i] > peak) {
            peak = data[i];
            bin = i;
        }
    }
    return bin;
}


inline float mean(float* data, int size) {
    double sum = 0.0;
    for (int i = 0; i < size; i++) sum += data[i];
    return (float)(sum / (double)size);
}


inline float signalToNoiseRatioDB(float signalAmplitude, float noiseAmplitude) {
    // 0 dB here is a divide-by-zero sentinel for "no measurable noise",
    // not a real ratio (a true 0 dB would mean signal equals noise).
    if (noiseAmplitude == 0.0f) return 0.0f;
    return 20.0f * log10fFast(signalAmplitude / noiseAmplitude);
}


// Signal-to-noise-and-distortion ratio in dB, computed from LINEAR
// magnitudes — call this before postScaleDB(), never on dB data (on dB it
// returns nonsense silently). Walks outward from the peak bin, counting
// bins as "signal" while they stay at or above twice the spectrum mean
// (the test (data[bin] - avg) >= avg is data[bin] >= 2*avg); the remaining
// bins in each direction count as noise+distortion power. Remove DC first:
// bin 0 participates if it is the largest bin. This is a threshold-based
// estimate, so its signal/noise boundary depends on the spectrum.
inline float SINAD(float* data, int size) {
    double avg = mean(data, size);
    int peakBin = findPeak(data, size);

    float signalPower = 0.0f;
    float noisePower  = 0.0f;

    int bin = peakBin;
    for (; bin < size; bin++) {
        if ((data[bin] - avg) >= avg) signalPower += data[bin] * data[bin];
        else break;
    }
    for (; bin < size; bin++) noisePower += data[bin] * data[bin];

    bin = peakBin - 1;
    for (; bin > 0; bin--) {
        if ((data[bin] - avg) >= avg) signalPower += data[bin] * data[bin];
        else break;
    }
    for (; bin > 0; bin--) noisePower += data[bin] * data[bin];

    if (noisePower == 0.0f) return 0.0f;
    return 10.0f * log10f((float)(signalPower / noisePower));
}


// Effective number of bits, derived from SINAD via the standard formula.
inline float ENOB(float* data, int size) {
    return (SINAD(data, size) - 1.76f) / 6.02f;
}


// --- FFT template class ---

// FFT_LENGTH is a plain integer (e.g. 512 or FFTSize::N512 — the unscoped enum
// value converts). The same symbol can therefore size the sample arrays in a
// sketch, so one value sizes both the transform and its input arrays.
//
// Memory: each instance embeds samples_[N] + result_[N] + windowCoef_[N/2]
// + twiddle_[N/4 + 1], approximately 2.75 * N floats (11 * N bytes): FFT<1024> ~ 11 KB,
// FFT<4096> ~ 44 KB. Prefer global/static storage for large instances to
// avoid exhausting an embedded target's stack.
template<size_t FFT_LENGTH>
class FFT {
private:
    static constexpr int N = static_cast<int>(FFT_LENGTH);

    // The transform is radix-2, so the length must be a power of two. Enum
    // values and plain integer arguments must satisfy the same constraint.
    static_assert(FFT_LENGTH >= 32 && (FFT_LENGTH & (FFT_LENGTH - 1)) == 0,
                  "FFT size must be a power of two >= 32");

    // Catch oversized FFTs at compile time so a sketch does not burn the
    // entire RAM of an UNO trying to allocate a 4096-point transform. The
    // limit lives in Platform.h: 256 for AVR, 1024 for ESP8266, 32768
    // elsewhere; it guards these static instances only.
    static_assert(N <= SIGNALCORE_MAX_FFT_SIZE,
                  "FFT size exceeds SIGNALCORE_MAX_FFT_SIZE for this platform "
                  "(see src/signalcore/Platform.h)");

public:
    FFT() : windowType_(WindowType::None), windowSum_(0.0f) {
        // The twiddle table is a member, so the engine never allocates
        // and this cannot fail for a valid N.
        rfft_.init(N, twiddle_, sizeof(twiddle_) / sizeof(twiddle_[0]));
    }

    // Which engine runs the transform (CMSIS or the generic radix-2).
    RealFFT<float>::Backend backend() const { return rfft_.backend(); }

    // Access the magnitude spectrum computed by the last calculate()/forward().
    // Only result[0 .. N/2] are meaningful (N/2 + 1 bins: DC through Nyquist);
    // entries above N/2 are leftover FFT scratch — do not read or plot them.
    // The pointer aliases internal storage: it is overwritten by the next
    // calculate() and rewritten in place (linear -> dB) by postScaleDB().
    operator float*() { return result_; }

    // Select the analysis window. `size` must match the sample count later
    // passed to calculate() — a mismatch applies a wrong-length window and
    // skews the amplitude scaling. Values above N are clamped: the
    // coefficient buffer holds N/2 entries. Leave it at the default N unless
    // you deliberately transform partial (zero-padded) buffers.
    void setWindowType(WindowType wt, uint32_t size = N) {
        if (size > (uint32_t)N) size = N;
        windowType_ = wt;
        windowSum_  = createWindowCoefficients(windowCoef_, size, wt);
    }

    // The calculate() overloads copy `size` samples in, zero-pad to N,
    // window, transform, and scale. Zero-padding keeps bin spacing at
    // sampleRate/N. Set the window for the actual sample count to normalize
    // that capture; the default unwindowed scale reads partial captures low
    // by size/N. Raw unipolar ADC data carries a
    // large DC offset that lands in bin 0 — peakBin() skips it by default,
    // or call removeDC() on a float copy first for a clean spectrum.
    void calculate(uint16_t* source, int size = N) {
        for (int i = 0; i < size; i++) samples_[i] = (float)source[i];
        for (int i = size; i < N; i++) samples_[i] = 0.0f;
        forward(size);
    }

    void calculate(int16_t* source, int size = N) {
        for (int i = 0; i < size; i++) samples_[i] = (float)source[i];
        for (int i = size; i < N; i++) samples_[i] = 0.0f;
        forward(size);
    }

    void calculate(float* source, int size = N) {
        for (int i = 0; i < size; i++) samples_[i] = source[i];
        for (int i = size; i < N; i++) samples_[i] = 0.0f;
        forward(size);
    }

    void forward(int size) {
        if (windowType_ != WindowType::None && windowType_ != WindowType::Rectangle) {
            for (int i = 0; i < size / 2; i++) {
                samples_[i] = samples_[i] * windowCoef_[i];
                samples_[size - (i + 1)] = samples_[size - (i + 1)] * windowCoef_[i];
            }
        }

        rfft_.forward(samples_, result_);

        // Scale so a full-amplitude sine centered on a bin reads its
        // time-domain amplitude in the output (same units as the input).
        // windowSum_ is the sum of the FIRST HALF of the window = (sum of
        // all N coefficients)/2, so dividing by it is the textbook
        // single-sided correction 2/sum(w): the factor 2 accounts for the
        // matching negative-frequency component, and sum(w) undoes the
        // window's coherent gain. With no window set, windowSum_ falls back
        // to N/2 (rectangle sum / 2).
        if (windowSum_ == 0.0f) windowSum_ = (float)(N / 2);
        float scale = windowSum_;

        // DC and Nyquist each appear exactly once in the real spectrum (no
        // negative-frequency twin), so both must skip the single-sided x2
        // baked into `scale` — hence the extra /2 relative to interior bins.
        // The transform packs DC into result[0] and Nyquist into result[1];
        // the rest is interleaved complex pairs.
        float DC      = fabsf(result_[0]) / (scale * 2.0f);
        float nyquist = fabsf(result_[1]) / (scale * 2.0f);

        detail::cmplxMagF32(result_, result_, (N / 2));

        for (int i = 0; i < (N / 2); i++) {
            result_[i] = result_[i] / scale;
        }

        result_[0]       = DC;
        result_[(N / 2)] = nyquist;
    }

    // Convert result[0..N/2] in place from linear magnitude to dB relative
    // to full scale: 20*log10(mag / maxScale). The default maxScale = 32767
    // assumes int16-scale samples; pass 1.0f for normalized float input, or
    // approximately 2048.0f for a full-scale sine centered in a 12-bit ADC
    // range. maxScale is peak amplitude in input units; a wrong value adds a
    // constant dB offset. Output is clamped to [floor, +10]: the floor tames
    // log10(0) = -inf on empty bins, the +10 dB ceiling absorbs slight
    // overshoot (windowing/rounding) above full scale. Linear magnitudes
    // are destroyed — call peakBin()/SINAD()/ENOB() first if you need them.
    void postScaleDB(float floor = -100.0f, float maxScale = 32767.0f) {
        for (int i = 0; i <= (N / 2); i++) {
            result_[i] = constrain(20.0f * log10f(result_[i] / maxScale), floor, 10.0f);
        }
    }

    // Same contract as postScaleDB() with the polynomial log approximation
    // (~0.01 dB error) for platforms where log10f is expensive.
    void postScaleDBFast(float floor = -100.0f, float maxScale = 32767.0f) {
        for (int i = 0; i <= (N / 2); i++) {
            result_[i] = constrain(20.0f * log10fFast(result_[i] / maxScale), floor, 10.0f);
        }
    }

    // Index of the dominant magnitude bin over the valid range result[0 .. N/2].
    // skipDC starts the search at bin 1 so a DC offset in the input cannot win;
    // leave it true for tone detection. Call before postScaleDB() if you also
    // need the linear level, since postScaleDB() rewrites the buffer with dB.
    int peakBin(bool skipDC = true) {
        return findPeak(result_, (N / 2) + 1, skipDC ? 1 : 0);
    }

    // Frequency of the dominant bin in Hz for a given capture sample rate.
    // Bin spacing is sampleRate / N. This returns a bin center without
    // interpolation; noise and nearby tones can affect which bin is largest.
    float peakFrequency(float sampleRate, bool skipDC = true) {
        return (float)peakBin(skipDC) * sampleRate / (float)N;
    }

private:
    RealFFT<float> rfft_;
    float       twiddle_[N / 4 + 1];
    float       samples_[N];
    float       result_[N];
    WindowType  windowType_;
    float       windowCoef_[N / 2];
    float       windowSum_;
};


// --- DynamicFFT: runtime-sized FFT ---

// Same transform, packing, and scaling as FFT<N>, but the length is chosen —
// and can be changed — at run time with init(). Use it when a sketch cycles
// through FFT sizes live (a UI size control, adaptive resolution); when the
// length is known at compile time, prefer FFT<N>, which needs no heap and
// cannot fail to allocate.
//
// Memory: buffers live on the heap — samples N + result N + window N/2 +
// twiddles N/4 + 1, approximately 2.75 * N floats (11 * N bytes), re-allocated
// on every call: 4096 -> 44 kB, 16384 -> 176 kB, 32768 -> 352 kB. Nothing is
// allocated until the first init(), and calculate()/forward() are safe
// no-ops before it. There is no size cap beyond what the heap holds.
//
// init() clears the window selection: coefficient length depends on the FFT
// length, so call setWindowType() again after every (re)size.
class DynamicFFT {
public:
    DynamicFFT() = default;

    ~DynamicFFT() {
        release();
    }

    // Size the transform, allocating the internal buffers. `size` must be a
    // power of two >= 32; a rejected size returns false. On failure,
    // including a refused allocation, the object is left unallocated and
    // every processing call is a no-op until a successful init().
    bool init(int size) {
        release();

        if (size < 32 || (size & (size - 1)) != 0) {
            return false;
        }

        // Report allocation failure through init()'s false return. Clear the
        // buffers only after all allocations have succeeded.
        samples_    = new (std::nothrow) float[size];
        result_     = new (std::nothrow) float[size];
        windowCoef_ = new (std::nothrow) float[size / 2];

        if (!samples_ || !result_ || !windowCoef_ ||
            !rfft_.init((uint32_t)size)) {
            release();
            return false;
        }

        memset(samples_,    0, (size_t)size * sizeof(float));
        memset(result_,     0, (size_t)size * sizeof(float));
        memset(windowCoef_, 0, (size_t)(size / 2) * sizeof(float));

        size_ = size;
        return true;
    }

    // Current transform length, or 0 before the first successful init().
    int size() const { return size_; }

    // Which engine runs the transform (CMSIS up to 4096 points on a CMSIS
    // platform, the generic radix-2 otherwise).
    RealFFT<float>::Backend backend() const { return rfft_.backend(); }

    // Access the magnitude spectrum computed by the last calculate()/forward().
    // Only result[0 .. size/2] are meaningful; same contract as FFT<N>.
    operator float*() { return result_; }

    // Select the analysis window. `size` must match the sample count later
    // passed to calculate(); 0 (the default) means the full FFT length.
    // Values above the FFT length are clamped: the coefficient buffer holds
    // size/2 entries. Must be re-called after init() — resizing discards the
    // coefficients.
    void setWindowType(WindowType wt, uint32_t size = 0) {
        if (!windowCoef_) return;
        if (size == 0 || size > (uint32_t)size_) size = (uint32_t)size_;
        windowType_ = wt;
        windowSum_  = createWindowCoefficients(windowCoef_, size, wt);
    }

    // The calculate() overloads copy `size` samples in, zero-pad to the FFT
    // length, window, transform, and scale — same contract and caveats as
    // FFT<N>::calculate(). `size` < 0 (the default) means a full buffer.
    void calculate(uint16_t* source, int size = -1) {
        if (!samples_ || !result_) return;
        int n = clampInputSize(size);
        for (int i = 0; i < n; i++) samples_[i] = (float)source[i];
        for (int i = n; i < size_; i++) samples_[i] = 0.0f;
        forward(n);
    }

    void calculate(int16_t* source, int size = -1) {
        if (!samples_ || !result_) return;
        int n = clampInputSize(size);
        for (int i = 0; i < n; i++) samples_[i] = (float)source[i];
        for (int i = n; i < size_; i++) samples_[i] = 0.0f;
        forward(n);
    }

    void calculate(float* source, int size = -1) {
        if (!samples_ || !result_) return;
        int n = clampInputSize(size);
        for (int i = 0; i < n; i++) samples_[i] = source[i];
        for (int i = n; i < size_; i++) samples_[i] = 0.0f;
        forward(n);
    }

    void forward(int size) {
        if (!samples_ || !result_) return;

        if (windowType_ != WindowType::None && windowType_ != WindowType::Rectangle) {
            for (int i = 0; i < size / 2; i++) {
                samples_[i] = samples_[i] * windowCoef_[i];
                samples_[size - (i + 1)] = samples_[size - (i + 1)] * windowCoef_[i];
            }
        }

        rfft_.forward(samples_, result_);

        // Identical scaling to FFT<N>::forward(): single-sided 2/sum(w)
        // amplitude correction, with DC and Nyquist skipping the x2 because
        // they have no negative-frequency twin.
        if (windowSum_ == 0.0f) windowSum_ = (float)(size_ / 2);
        float scale = windowSum_;

        float DC      = fabsf(result_[0]) / (scale * 2.0f);
        float nyquist = fabsf(result_[1]) / (scale * 2.0f);

        detail::cmplxMagF32(result_, result_, (size_ / 2));

        for (int i = 0; i < (size_ / 2); i++) {
            result_[i] = result_[i] / scale;
        }

        result_[0]         = DC;
        result_[size_ / 2] = nyquist;
    }

    // Convert result[0..size/2] in place from linear magnitude to dB, as
    // FFT<N>::postScaleDB() does, with one addition: the clamp ceiling is a
    // parameter. Set it high enough to retain levels above maxScale. Both
    // the spectrum and maxScale must use the same amplitude units.
    void postScaleDB(float floor = -100.0f, float maxScale = 32767.0f, float ceiling = 10.0f) {
        if (!result_) return;
        for (int i = 0; i <= (size_ / 2); i++) {
            result_[i] = constrain(20.0f * log10f(result_[i] / maxScale), floor, ceiling);
        }
    }

    // Same contract as postScaleDB() with the polynomial log approximation
    // (~0.01 dB error) for platforms where log10f is expensive.
    void postScaleDBFast(float floor = -100.0f, float maxScale = 32767.0f, float ceiling = 10.0f) {
        if (!result_) return;
        for (int i = 0; i <= (size_ / 2); i++) {
            result_[i] = constrain(20.0f * log10fFast(result_[i] / maxScale), floor, ceiling);
        }
    }

    // Add a per-bin correction to the spectrum in place. The correction and
    // the spectrum must be in the same domain — typically dB offsets applied
    // after postScaleDB(), which is how a stored flatness calibration is
    // applied. numBins is clamped to the valid bin count.
    void applyCalibration(const float* calibrationData, size_t numBins) {
        if (!result_) return;
        size_t maxBins = (size_t)(size_ / 2) + 1;
        if (numBins > maxBins) numBins = maxBins;
        for (size_t i = 0; i < numBins; i++) {
            result_[i] += calibrationData[i];
        }
    }

    // Index of the dominant magnitude bin; same contract as FFT<N>::peakBin().
    int peakBin(bool skipDC = true) {
        if (!result_) return 0;
        return findPeak(result_, (size_ / 2) + 1, skipDC ? 1 : 0);
    }

    // Frequency of the dominant bin in Hz; same contract as
    // FFT<N>::peakFrequency().
    float peakFrequency(float sampleRate, bool skipDC = true) {
        if (size_ == 0) return 0.0f;
        return (float)peakBin(skipDC) * sampleRate / (float)size_;
    }

private:
    int clampInputSize(int size) const {
        if (size < 0 || size > size_) return size_;
        return size;
    }

    void release() {
        delete[] samples_;
        delete[] result_;
        delete[] windowCoef_;
        rfft_.init(0);   // frees the twiddle table, size() -> 0
        samples_ = result_ = windowCoef_ = nullptr;
        size_       = 0;
        windowType_ = WindowType::None;
        windowSum_  = 0.0f;
    }

    RealFFT<float> rfft_;
    int         size_        = 0;
    float*      samples_     = nullptr;
    float*      result_      = nullptr;
    float*      windowCoef_  = nullptr;
    WindowType  windowType_  = WindowType::None;
    float       windowSum_   = 0.0f;
};

} // namespace signalcore

#endif // _SIGNALCORE_FFT_H_
