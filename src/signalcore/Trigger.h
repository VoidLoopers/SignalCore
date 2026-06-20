/******************************************************************************
 * @file Trigger.h
 * @brief Signal Trigger with Hysteresis
 *
 * Provides rising/falling edge triggering with Schmitt-trigger hysteresis for
 * stable waveform alignment. Searches a sample buffer for the trigger event
 * and returns a pointer offset so that the event is centered in the output
 * window — suitable for oscilloscope-style displays.
 *
 * Features:
 *   - Rising and falling edge detection with configurable threshold
 *   - Schmitt-trigger hysteresis with adaptive margin relaxation
 *   - Automatic threshold at signal midpoint when no threshold is specified
 *   - Configurable trigger position within the output window
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

#ifndef _SIGNALCORE_TRIGGER_H_
#define _SIGNALCORE_TRIGGER_H_

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>


namespace signalcore {

enum class TriggerMode : uint8_t {
    None,
    RisingEdge,
    FallingEdge,
};

inline const char* toString(TriggerMode mode) {
    switch (mode) {
        case TriggerMode::RisingEdge:  return "Rising";
        case TriggerMode::FallingEdge: return "Falling";
        case TriggerMode::None:
        default:                       return "None";
    }
}


class Trigger {
public:
    static constexpr float TRIGGER_NO_THRESHOLD = -1e30f;
    static constexpr int   NO_TRIGGER_FOUND     = -1;

    Trigger(TriggerMode mode      = TriggerMode::RisingEdge,
            float       threshold = TRIGGER_NO_THRESHOLD)
        : mode_(mode), threshold_(threshold), position_(0.5f) {}

    ~Trigger() {}

    inline void begin(TriggerMode mode, float threshold = TRIGGER_NO_THRESHOLD) {
        mode_      = mode;
        threshold_ = threshold;
        reset();
    }

    inline void reset() {
        lastState_ = false;
        valid_     = false;
    }

    inline bool active() const {
        return mode_ != TriggerMode::None;
    }

    inline bool valid() const { return valid_; }
    inline TriggerMode mode() const { return mode_; }

    inline void setThreshold(float threshold) {
        threshold_ = threshold;
    }

    inline void setPosition(float position) {
        if (position < 0.0f) position = 0.0f;
        if (position > 1.0f) position = 1.0f;
        position_ = position;
    }

    /**
     * Find a trigger point in the data buffer and return a pointer offset
     * so that the trigger event is near the center of the output window.
     *
     * @param data           Pointer to the sample buffer (float voltages)
     * @param outputSize     Number of samples in the output view window
     * @param inputCapacity  Total number of samples available in data
     * @return Pointer to the start of the aligned output window, or data if no trigger found
     */
    inline float* calculateOffset(float* data, size_t outputSize, size_t inputCapacity) {
        if (!data || inputCapacity == 0 || outputSize == 0) {
            valid_ = false;
            return data;
        }
        if (mode_ == TriggerMode::None) return data;

        if (threshold_ == TRIGGER_NO_THRESHOLD) findMinMax(data, inputCapacity);

        int triggerIndex = NO_TRIGGER_FOUND;
        float m = 0.05f;
        int n = 4;
        int tries = 0;
        int maxTries = 4;

        while (triggerIndex == NO_TRIGGER_FOUND && (m > 0 && n >= 0)) {
            if (tries++ >= maxTries) break;
            triggerIndex = findTriggerIndex(data, outputSize, inputCapacity,
                                            (int)(outputSize * position_) + 1, n, m);
            if (triggerIndex == NO_TRIGGER_FOUND) {
                float mNext = m - 0.02f;
                m = (mNext > 0.01f) ? mNext : 0.01f;
                int nNext = n - 2;
                n = (nNext > 0) ? nNext : 0;
            } else {
                break;
            }
        }

        if (triggerIndex == NO_TRIGGER_FOUND) {
            valid_ = false;
            return data;
        } else {
            int viewStart = triggerIndex - (int)(outputSize * position_);
            if (viewStart < 0) viewStart = 0;
            valid_ = true;
            return &data[viewStart];
        }
    }

    inline void setMinMax(float minValue, float maxValue) {
        min_ = minValue;
        max_ = maxValue;
    }

    inline TriggerMode getTriggerMode() const { return mode_; }

private:

    void findMinMax(float* data, size_t inputCapacity) {
        if (inputCapacity == 0) {
            min_ = 0.0f;
            max_ = 0.0f;
            return;
        }
        float minVal = data[0], maxVal = data[0];
        for (size_t i = 1; i < inputCapacity; ++i) {
            if (data[i] < minVal) minVal = data[i];
            if (data[i] > maxVal) maxVal = data[i];
        }
        min_ = minVal;
        max_ = maxVal;
    }

    float getThreshold() const {
        if (threshold_ != TRIGGER_NO_THRESHOLD) return threshold_;
        return min_ + (max_ - min_) * 0.5f;
    }

    int findTriggerIndex(float* data, size_t outputSize, size_t inputCapacity,
                         int start = -1, int n = 4, float m = 0.05f) {
        if (mode_ == TriggerMode::None) return NO_TRIGGER_FOUND;

        int offset = (int)(outputSize * position_);
        if (start < 0) start = offset + n;
        int end = (int)inputCapacity - (int)(outputSize - offset) - n - 1;

        float thresh = getThreshold();
        float margin = (max_ - min_) * m;

        for (int i = start; i < end; ++i) {
            float curr = average(&data[i + 1], n);
            if (isTriggered(curr, thresh, margin)) return i;
        }

        return NO_TRIGGER_FOUND;
    }

    bool isTriggered(float curr, float threshold, float margin) {
        const bool low = false, high = true;
        switch (mode_) {
            case TriggerMode::RisingEdge:
                if (lastState_ == low && curr >= threshold + margin) {
                    lastState_ = high;
                    return true;
                } else if (curr < threshold - margin) {
                    lastState_ = low;
                }
                break;

            case TriggerMode::FallingEdge:
                if (lastState_ == high && curr <= threshold - margin) {
                    lastState_ = low;
                    return true;
                } else if (curr > threshold + margin) {
                    lastState_ = high;
                }
                break;

            default: break;
        }
        return false;
    }

    float average(const float* data, int n) const {
        if (n <= 0) return data[0];
        float sum = 0.0f;
        for (int i = 0; i < n; ++i) sum += data[i];
        return sum / (float)n;
    }

    TriggerMode mode_      = TriggerMode::None;
    float       threshold_ = TRIGGER_NO_THRESHOLD;
    float       position_  = 0.5f;
    bool        lastState_ = false;
    bool        valid_     = false;
    float       min_ = 0.0f, max_ = 3.3f;
};

} // namespace signalcore

#endif // _SIGNALCORE_TRIGGER_H_
