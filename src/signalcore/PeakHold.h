/******************************************************************************
 * @file PeakHold.h
 * @brief Per-Bin Peak-Hold with Exponential Decay
 *
 * Tracks the running maximum of each element in a vector and lets the held
 * peak fade toward the current input through a single-pole exponential decay.
 * Common uses include spectrum analyzers (per-bin max-hold over an FFT),
 * envelope followers, and "max-bar" indicators.
 *
 * The hold value follows samples instantly when the input rises and decays
 * toward the input when it falls. The `decay` factor is the weight kept from
 * the previous held value each call: decay=0.95 keeps 95% of the previous
 * peak and bleeds 5% toward the new sample.
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

#ifndef _SIGNALCORE_PEAKHOLD_H_
#define _SIGNALCORE_PEAKHOLD_H_

#include <math.h>
#include <stddef.h>
#include <new>


namespace signalcore {

class PeakHoldProcessor {
public:
    // decay is the fraction KEPT from the previous peak (higher = slower
    // fade). This is the OPPOSITE convention from ExponentialMovingAverage's
    // sampleWeight, which is the fraction given to the NEW sample:
    // decay = 1 - sampleWeight.
    PeakHoldProcessor(size_t size, float decay = 0.95f, bool isActive = true)
        : active(isActive), size_(0), result_(nullptr), decay_(decay) {
        init(size);
    }

    ~PeakHoldProcessor() {
        delete[] result_;
    }

    // Owns a heap buffer of size * sizeof(float) bytes. Copying is disabled:
    // two copies sharing one buffer would double-delete it. On a failed
    // allocation the processor degrades to size() == 0 (update() becomes a
    // no-op) instead of faulting: with -fno-exceptions a failed new[]
    // returns nullptr, and the value-init form would zero-fill through it.
    PeakHoldProcessor(const PeakHoldProcessor&)            = delete;
    PeakHoldProcessor& operator=(const PeakHoldProcessor&) = delete;

    // Seed with -INFINITY so the first update() unconditionally captures
    // the input. Don't read values (operator[] / float*) until after the
    // first update — they are -inf until then.
    inline void reset() {
        for (size_t i = 0; i < size_; ++i) {
            result_[i] = -INFINITY;
        }
    }

    inline void init(size_t newSize) {
        delete[] result_;
        result_ = new (std::nothrow) float[newSize];
        size_   = result_ ? newSize : 0;
        reset();
    }

    // Only min(inputSize, size()) elements are processed — a short input
    // never reads past its own end.
    inline void update(float* samples, size_t inputSize) {
        size_t n = (inputSize < size_) ? inputSize : size_;
        for (size_t i = 0; i < n; ++i) {
            if (samples[i] >= result_[i]) {
                result_[i] = samples[i];
            } else {
                // Bleed the held peak toward the lower sample value. The
                // held value falls with time constant ~1/(1-decay) updates:
                // decay=0.95 -> ~20 updates to close most of the gap.
                result_[i] = result_[i] * decay_ + samples[i] * (1.0f - decay_);
            }
        }
    }

    // Convenience overload: samples must hold at least size() elements.
    inline void update(float* samples) { update(samples, size_); }

    inline float operator[](size_t index) const { return result_[index]; }
    inline operator float*() { return result_; }

    inline void   setDecay(float factor) { decay_ = factor; }
    inline float  getDecay() const       { return decay_; }
    inline size_t size() const           { return size_; }

    // Convenience flag for sketch-level enable/disable. The class itself
    // never checks it — callers decide whether to call update().
    bool active = true;

private:
    size_t size_;
    float* result_;
    float  decay_;
};

} // namespace signalcore

#endif // _SIGNALCORE_PEAKHOLD_H_
