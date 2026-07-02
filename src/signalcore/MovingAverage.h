/******************************************************************************
 * @file MovingAverage.h
 * @brief Exponential Moving Average for Per-Element Vector Smoothing
 *
 * Smooths a vector input through a single-pole IIR filter applied
 * element-wise. The `sampleWeight` parameter is the weight given to the
 * latest input on each update: sampleWeight=1.0 disables smoothing,
 * sampleWeight=0.0 freezes the output at the first sample, and small values
 * produce heavy smoothing with longer settling time.
 *
 * The first call after construction or `init()` copies the input into the
 * internal buffer rather than blending with zeros — this avoids the long
 * exponential ramp-up that would otherwise distort the first many frames.
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

#ifndef _SIGNALCORE_MOVINGAVERAGE_H_
#define _SIGNALCORE_MOVINGAVERAGE_H_

#include <stddef.h>
#include <string.h>


namespace signalcore {

class ExponentialMovingAverage {
public:
    // sampleWeight is the fraction given to the NEW sample (higher = faster
    // response). This is the OPPOSITE convention from PeakHoldProcessor's
    // decay, which is the fraction kept from the OLD value:
    // sampleWeight = 1 - decay.
    ExponentialMovingAverage(size_t size, float sampleWeight = 0.2f, bool isActive = true)
        : active(isActive), size_(size), sampleWeight_(sampleWeight), result_(nullptr), reinit_(true) {
        result_ = new float[size_]();
    }

    ~ExponentialMovingAverage() {
        delete[] result_;
    }

    // Owns a heap buffer of size * sizeof(float) bytes. Copying is disabled:
    // two copies sharing one buffer would double-delete it. On RAM-limited
    // boards note that `new` failure is not checked — construct once at
    // startup and reuse via init() rather than allocating per frame.
    ExponentialMovingAverage(const ExponentialMovingAverage&)            = delete;
    ExponentialMovingAverage& operator=(const ExponentialMovingAverage&) = delete;

    inline void init(size_t newSize) {
        delete[] result_;
        size_   = newSize;
        result_ = new float[size_]();
        reinit_ = true;
    }

    // Restart the average: the next update() copies its input verbatim
    // instead of blending, exactly like a freshly constructed instance.
    inline void reset() { reinit_ = true; }

    inline void  setSampleWeight(float sampleWeight) { sampleWeight_ = sampleWeight; }
    inline float getSampleWeight() const             { return sampleWeight_; }

    // Only min(inputSize, size()) elements are processed — a short input
    // never reads past its own end. On the first update after construction,
    // init(), or reset(), the input is copied verbatim to avoid the long
    // exponential ramp-up from zero.
    inline void update(float* samples, size_t inputSize) {
        size_t n = (inputSize < size_) ? inputSize : size_;
        if (reinit_) {
            memcpy(result_, samples, n * sizeof(float));
            reinit_ = false;
            return;
        }
        for (size_t i = 0; i < n; ++i) {
            // Single-pole IIR: y = w*x + (1-w)*y. Time constant is roughly
            // 1/sampleWeight updates — w=0.2 settles in ~5 frames, w=0.01
            // in ~100.
            result_[i] = sampleWeight_ * samples[i] + (1.0f - sampleWeight_) * result_[i];
        }
    }

    // Convenience overload: samples must hold at least size() elements.
    inline void update(float* samples) { update(samples, size_); }

    inline float operator[](size_t index) const { return result_[index]; }
    inline operator float*() { return result_; }

    inline size_t size() const { return size_; }

    // Convenience flag for sketch-level enable/disable. The class itself
    // never checks it — callers decide whether to call update().
    bool active = true;

private:
    size_t size_;
    float  sampleWeight_;
    float* result_;
    bool   reinit_;
};

} // namespace signalcore

#endif // _SIGNALCORE_MOVINGAVERAGE_H_
