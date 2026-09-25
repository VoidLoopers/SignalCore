/******************************************************************************
 * @file RealFFT.h
 * @brief Real-input FFT engine, float or double, any power of two >= 32.
 *
 * RealFFT<T> transforms n real samples into the n/2 + 1 complex bins of
 * their spectrum. It is the one transform every SignalCore class runs on:
 * FFT<N> and DynamicFFT wrap RealFFT<float>, and measure/SineMetrics.h
 * runs its bin powers through RealFFT<SpectrumReal>.
 *
 * Two backends, chosen per instance by init():
 *
 *   Cmsis    arm_rfft_fast_f32, only for T = float on a CMSIS platform
 *            (Teensy 4.x, or SIGNALCORE_FORCE_CMSIS) and only for
 *            32 <= n <= 4096, the sizes CMSIS-DSP ships tables for.
 *   Generic  A radix-2 complex FFT of n/2 points followed by the real
 *            split, written once as a template. Every other case: double,
 *            sizes above 4096, and every non-CMSIS board.
 *
 * Both give the same output packing (below), so a caller never needs to
 * know which one ran; backend() reports it for diagnostics.
 *
 * Twiddles come from a quarter-wave table of n/4 + 1 values, sin(2 pi k / n)
 * for k = 0..n/4, computed in double and stored as T. Cosines are the same
 * table read from the other end. The table is the engine's only memory,
 * either allocated by init(n) or supplied by the caller through
 * init(n, table, count) for code that must not allocate. The precomputed-
 * table design follows Robin Scheibler's ESP32 FFT (MIT, 2017); the code
 * here is written fresh.
 *
 * Precision: the transform runs entirely in T. With table twiddles a float
 * transform has a round-off floor near -150 dBFS per bin at 8192 points
 * (test/host/test_realfft.cpp prints the measured figure), far below any
 * converter up to about 18 effective bits. RealFFT<double> exists for the
 * 20-bit class.
 *
 * Output packing (the CMSIS arm_rfft_fast_f32 layout):
 *   out[0]              DC, real
 *   out[1]              Nyquist (bin n/2), real
 *   out[2k], out[2k+1]  re, im of bin k for k = 1 .. n/2 - 1
 * A bin-centred sine of amplitude A reads |X[k]| = A n / 2; a constant c
 * reads out[0] = c n.
 *
 * No size cap: any power of two >= 32 that the table fits is accepted.
 * FFT<N> keeps its compile-time cap (SIGNALCORE_MAX_FFT_SIZE) because its
 * buffers are static; the runtime classes are limited by memory alone.
 *
 * Platform: Any Arduino-compatible board, and the host test rig.
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

#ifndef _SIGNALCORE_FFT_REALFFT_H_
#define _SIGNALCORE_FFT_REALFFT_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <new>

#include "../Platform.h"

#if defined(SIGNALCORE_USE_CMSIS)
// arm_math.h must be parsed at global scope or the ARM symbols would be
// declared inside a signalcore namespace and fail to link.
#include "arm_math.h"
#include "arm_const_structs.h"
#endif

namespace signalcore {

namespace detail {

// The CMSIS binding. Only the float specialisation on a CMSIS platform can
// run a transform; every other instantiation reports itself unavailable
// and RealFFT<T> takes the generic path.
template<typename T>
struct CmsisRfft {
    enum { available = 0 };
    bool init(uint32_t) { return false; }
    void forward(T*, T*) {}
};

#if defined(SIGNALCORE_USE_CMSIS)
template<>
struct CmsisRfft<float> {
    enum { available = 1 };
    arm_rfft_fast_instance_f32 inst;
    bool init(uint32_t n) {
        if (n < 32 || n > 4096) return false;
        return arm_rfft_fast_init_f32(&inst, (uint16_t)n) == ARM_MATH_SUCCESS;
    }
    void forward(float* in, float* out) {
        arm_rfft_fast_f32(&inst, in, out, 0);
    }
};
#endif

} // namespace detail


template<typename T>
class RealFFT {
public:
    enum class Backend { None, Cmsis, Generic };

    RealFFT() {}
    ~RealFFT() { release(); }

    // The table pointer would be double-freed by a copy.
    RealFFT(const RealFFT&) = delete;
    RealFFT& operator=(const RealFFT&) = delete;

    // Twiddle-table entries a transform of n points needs: n/4 + 1.
    static size_t twiddleCount(uint32_t n) { return (size_t)(n / 4) + 1; }

    // Size the transform for n points, allocating the twiddle table with
    // nothrow new. n must be a power of two >= 32; there is no upper
    // bound. Returns false, with size() left at 0, when n is rejected or
    // the allocation fails. A previous table is freed first.
    bool init(uint32_t n) {
        release();
        if (!validSize(n)) return false;
        T* table = new (std::nothrow) T[twiddleCount(n)];
        if (!table) return false;
        owned_ = true;
        return bind(n, table);
    }

    // Same, but the twiddle table lives in caller-owned storage of `count`
    // entries (at least twiddleCount(n)); nothing is allocated. The caller
    // keeps `table` alive for as long as the transform is used.
    bool init(uint32_t n, T* table, size_t count) {
        release();
        if (!validSize(n) || !table || count < twiddleCount(n)) return false;
        owned_ = false;
        return bind(n, table);
    }

    // Transform n real samples from `in` into `out` (n entries, packed as
    // described in the file header). `in` is undefined after the call on
    // every backend: CMSIS uses it as scratch, and a portable caller must
    // not rely on the generic path leaving it alone. `in` and `out` must
    // not overlap. A no-op before a successful init().
    void forward(T* in, T* out) {
        if (backend_ == Backend::Cmsis) {
            cmsis_.forward(in, out);
        } else if (backend_ == Backend::Generic) {
            forwardGeneric(in, out);
        }
    }

    // Transform length, or 0 before the first successful init().
    uint32_t size() const { return n_; }

    // Which engine forward() runs.
    Backend backend() const { return backend_; }

    // For tests only: run the generic engine even where CMSIS would apply,
    // so the two can be compared bin for bin on hardware. Takes effect on
    // the next forward(); the table is always built, so it may be called
    // after init().
    void forceGeneric(bool on) {
        forceGeneric_ = on;
        if (n_ == 0) return;
        backend_ = (!on && cmsisReady_) ? Backend::Cmsis : Backend::Generic;
    }

private:
    static bool validSize(uint32_t n) {
        return n >= 32 && (n & (n - 1)) == 0;
    }

    // Fill the table and pick the backend. `table` is already owned or
    // bound as the caller decided.
    bool bind(uint32_t n, T* table) {
        table_ = table;
        n_ = n;
        const size_t count = twiddleCount(n);
        for (size_t k = 0; k < count; ++k) {
            table_[k] = (T)sin(2.0 * M_PI * (double)k / (double)n);
        }
        cmsisReady_ = detail::CmsisRfft<T>::available && cmsis_.init(n);
        backend_ = (cmsisReady_ && !forceGeneric_) ? Backend::Cmsis : Backend::Generic;
        return true;
    }

    void release() {
        if (owned_) delete[] table_;
        table_ = nullptr;
        owned_ = false;
        n_ = 0;
        cmsisReady_ = false;
        backend_ = Backend::None;
    }

    // sin and cos of 2 pi a / n for an angle index a in [0, n/2], from the
    // quarter-wave table. Beyond n/4 the sine folds back (sin(pi - x) =
    // sin x) and the cosine changes sign.
    T sinAt(uint32_t a) const {
        const uint32_t q = n_ / 4;
        return a <= q ? table_[a] : table_[n_ / 2 - a];
    }
    T cosAt(uint32_t a) const {
        const uint32_t q = n_ / 4;
        return a <= q ? table_[q - a] : -table_[a - q];
    }

    // The generic engine: the n real samples are taken as m = n/2 complex
    // points z[j] = x[2j] + i x[2j+1], transformed in place in `out`, and
    // the real spectrum is recovered from the even/odd split
    //   E[k] = (Z[k] + conj Z[m-k]) / 2,  O[k] = (Z[k] - conj Z[m-k]) / 2i,
    //   X[k] = E[k] + W^k O[k],  W = exp(-2 pi i / n).
    void forwardGeneric(const T* in, T* out) {
        const uint32_t n = n_;
        const uint32_t m = n / 2;
        memcpy(out, in, (size_t)n * sizeof(T));

        // Bit-reversal permutation of the m complex points.
        for (uint32_t i = 1, j = 0; i < m; ++i) {
            uint32_t bit = m >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                T t;
                t = out[2 * i];     out[2 * i]     = out[2 * j];     out[2 * j]     = t;
                t = out[2 * i + 1]; out[2 * i + 1] = out[2 * j + 1]; out[2 * j + 1] = t;
            }
        }

        // Butterfly stages. A butterfly at offset j in a group of `len`
        // points uses W_len^j = exp(-2 pi i j / len), which is angle index
        // j * (n / len) in the n-point table.
        for (uint32_t len = 2; len <= m; len <<= 1) {
            const uint32_t half = len >> 1;
            const uint32_t stride = n / len;
            for (uint32_t i = 0; i < m; i += len) {
                for (uint32_t j = 0; j < half; ++j) {
                    const T c = cosAt(j * stride);
                    const T s = sinAt(j * stride);
                    T* a = out + 2 * (i + j);
                    T* b = out + 2 * (i + j + half);
                    // (c - i s) * (br + i bi)
                    const T tr = c * b[0] + s * b[1];
                    const T ti = c * b[1] - s * b[0];
                    b[0] = a[0] - tr;  b[1] = a[1] - ti;
                    a[0] += tr;        a[1] += ti;
                }
            }
        }

        // Real split. DC and Nyquist come from Z[0] alone.
        const T z0r = out[0], z0i = out[1];
        out[0] = z0r + z0i;
        out[1] = z0r - z0i;

        // Bins k and m - k share one pass; W^(m-k) = -conj(W^k).
        for (uint32_t k = 1; k < m / 2; ++k) {
            const uint32_t mk = m - k;
            const T zrk = out[2 * k],  zik = out[2 * k + 1];
            const T zrmk = out[2 * mk], zimk = out[2 * mk + 1];
            const T er = (T)0.5 * (zrk + zrmk);
            const T ei = (T)0.5 * (zik - zimk);
            const T orr = (T)0.5 * (zik + zimk);
            const T oi = (T)0.5 * (zrmk - zrk);
            const T c = cosAt(k), s = sinAt(k);
            out[2 * k]      =  er + c * orr + s * oi;
            out[2 * k + 1]  =  ei + c * oi  - s * orr;
            out[2 * mk]     =  er - c * orr - s * oi;
            out[2 * mk + 1] = -ei + c * oi  - s * orr;
        }

        // k = m/2 pairs with itself: E = re Z, O = im Z, W = -i.
        {
            const uint32_t k = m / 2;
            const T zrk = out[2 * k], zik = out[2 * k + 1];
            out[2 * k]     = zrk;
            out[2 * k + 1] = -zik;
        }
    }

    detail::CmsisRfft<T> cmsis_;
    T*       table_ = nullptr;
    uint32_t n_ = 0;
    Backend  backend_ = Backend::None;
    bool     owned_ = false;
    bool     cmsisReady_ = false;
    bool     forceGeneric_ = false;
};

} // namespace signalcore

#endif // _SIGNALCORE_FFT_REALFFT_H_
