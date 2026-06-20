/******************************************************************************
 * @file DDS.h
 * @brief Direct Digital Synthesis and Signal Generation
 *
 * Provides a table-based DDS oscillator and a flexible SignalGenerator class
 * for producing sine, square, triangle, sawtooth, and noise waveforms with
 * optional amplitude modulation and gaussian noise injection.
 *
 * The DDS class uses a 32-bit phase accumulator and an integer wave table.
 * Tables for power-of-two sizes (64..65536 samples) are generated at compile
 * time by the DDSSineTable<> template, which keeps the table in flash on AVR.
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

#ifndef _SIGNALCORE_DDS_H_
#define _SIGNALCORE_DDS_H_

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "Platform.h"


namespace signalcore {

// TableSize stores log2 of the sample count, which is exactly what the DDS
// phase accumulator needs for `acc >> (32 - bits)`. The trailing number is
// the resulting sample count (e.g. Samples1024 → 1024 samples → 10 bits).
enum class TableSize : uint8_t {
    Samples64    = 6,
    Samples128   = 7,
    Samples256   = 8,
    Samples512   = 9,
    Samples1024  = 10,
    Samples2048  = 11,
    Samples4096  = 12,
    Samples8192  = 13,
    Samples16384 = 14,
    Samples32768 = 15,
    Samples65536 = 16,
};

constexpr uint32_t tableSizeSamples(TableSize ts) {
    return 1u << static_cast<uint8_t>(ts);
}


enum class SignalShape : uint8_t {
    Sine,
    Square,
    Triangle,
    Sawtooth,
    Noise,
    None
};


// --- Phase-accumulator DDS over a pre-built integer wave table ---

class DDS {
public:
    DDS(int* table, TableSize tableSize, uint32_t samplingFrequency)
        : table_(table),
          tableSize_(tableSize),
          acc_(0),
          samplingFrequency_(samplingFrequency) {}

    inline int nextSampleIncInt(int tuningWord) {
        acc_ += (uint32_t)tuningWord;
        uint32_t index = acc_ >> (32 - static_cast<uint8_t>(tableSize_));
        return table_[index];
    }

    inline int nextSampleIncPhase(double phaseInc) {
        int tuningWord = (int)((phaseInc / (2.0 * M_PI)) * (double)UINT32_MAX);
        return nextSampleIncInt(tuningWord);
    }

    inline int nextSampleAtFrequency(double frequency) {
        int tuningWord = (int)((double)UINT32_MAX * (frequency / (double)samplingFrequency_));
        return nextSampleIncInt(tuningWord);
    }

    inline int getTableSize() const {
        return 1 << static_cast<uint8_t>(tableSize_);
    }

private:
    int*       table_;
    TableSize  tableSize_;
    uint32_t   acc_;
    uint32_t   samplingFrequency_;
};


// Compile-time sine table sized by TableSize enum. Storage lives in the
// struct's data[] member; the implicit conversion lets the table be passed
// directly to the DDS constructor.
template<TableSize TS>
struct DDSSineTable {
    static constexpr uint32_t SAMPLES = 1u << static_cast<uint8_t>(TS);

    DDSSineTable() {
        double phase = 0.0;
        double phaseInc = (2.0 * M_PI) / (double)SAMPLES;
        for (uint32_t i = 0; i < SAMPLES; i++) {
            data[i] = (int)(sin(phase) * INT32_MAX);
            phase += phaseInc;
        }
    }
    operator int*() { return data; }
    int data[SAMPLES];
};


// --- Standalone table fillers and sample generators ---

inline void fillSineTable(double sampleFrequency, double sineFrequency,
                          int tableSize, int* table, int bufferChannels = 1) {
    double phase = 0.0;
    double phaseInc = (2.0 * M_PI) / (sampleFrequency / sineFrequency);
    for (int i = 0; i < tableSize; i += bufferChannels) {
        double x = (int)(sin(phase) * INT32_MAX);
        table[i]     = (int)x;
        table[i + 1] = (int)x;
        phase = wrapPhase(phase + phaseInc);
    }
}


// Find a table size near `tableSize` (input) such that an integer number of
// periods of `targetFrequency` fits exactly at `sampleRate`. The closest
// match minimizes spectral leakage when the table loops.
inline void getTableSizeForFrequencyAtSampleRate(int sampleRate, int targetFrequency,
                                                 int& tableSize, int& tablePeriods,
                                                 double& actualFrequency) {
    double samplesPerPeriod = (double)sampleRate / (double)targetFrequency;
    double error = 1.0;
    for (int periods = (int)(((double)tableSize / samplesPerPeriod) + 0.5); periods >= 1; periods--) {
        double actualSamplesD = (double)periods * samplesPerPeriod;
        if (actualSamplesD > (double)tableSize) actualSamplesD = (double)tableSize;
        int actualSamplesInt = (int)(actualSamplesD + 0.5);
        double thisError = actualSamplesD - (double)actualSamplesInt;
        if (fabs(thisError) <= fabs(error)) {
            error = thisError;
            tableSize = actualSamplesInt;
            tablePeriods = periods;
        }
    }
    actualFrequency = (double)sampleRate / ((double)tableSize / (double)tablePeriods);
}


inline void createSquareSamples(uint16_t* data, int size, double periods, double amplitude) {
    double phase = 0.0;
    double phaseInc = (2.0 * M_PI * periods) / (double)size;
    for (int i = 0; i < size; i++) {
        double x = phase > M_PI ? 0.0 : 1.0;
        x = x * amplitude;
        data[i] = (uint16_t)(x * UINT16_MAX);
        phase += phaseInc;
        phase = wrapPhase(phase);
    }
}


inline void createSineSamples(uint16_t* data, int size, double periods, double amp) {
    double phase = 0.0;
    double phaseInc = (2.0 * M_PI * periods) / (double)size;
    for (int i = 0; i < size; i++) {
        double x = (sin(phase) + 1.0) * 0.5 * amp;
        data[i] = (uint16_t)(x * UINT16_MAX);
        phase += phaseInc;
    }
}


inline void createSineSamples32(uint32_t* data, int size, double periods, double amp) {
    double phase = 0.0;
    double phaseInc = (2.0 * M_PI * periods) / (double)size;
    for (int i = 0; i < size; i++) {
        double x = ((sin(phase) * amp) + 1.0) * 0.5;
        data[i] = (uint32_t)(x * (double)UINT32_MAX);
        phase += phaseInc;
    }
}


// --- High-level signal generator (carrier + AM + noise) ---

class SignalGenerator {
public:
    SignalGenerator(int sampleRate)
        : sampleRate_(sampleRate),
          carrierPhase_(0.0), carrierFrequency_(0.0), carrierAmplitude_(1.0),
          carrierShape_(SignalShape::Sine),
          amPhase_(0.0f), amFrequency_(0.0f), amAmplitude_(0.0f),
          amAmplitudeDec_(0.0f), amPhaseInc_(0.0f), amShape_(SignalShape::None),
          noiseAmplitude_(0.0) {}

    inline void setCarrier(SignalShape shape, double frequency,
                           double amplitude = 1.0, double phase = 0.0) {
        carrierShape_     = shape;
        carrierFrequency_ = frequency;
        carrierAmplitude_ = amplitude;
        carrierPhase_     = phase;
    }

    inline void setAM(SignalShape shape, double frequency, double amplitude) {
        amFrequency_    = (float)frequency;
        amAmplitude_    = (float)amplitude;
        amAmplitudeDec_ = amAmplitude_ / 100.0f;
        amPhaseInc_     = (float)((2.0 * M_PI) / ((double)sampleRate_ / frequency));
        amShape_        = shape;
    }

    inline void setNoise(double amplitude) {
        noiseAmplitude_ = amplitude;
    }

    // Return the next sample. resolution=0 returns the raw float carrier;
    // any other value quantizes through a 32-bit interim to that many bits.
    inline double nextSample(uint8_t resolution) {
        carrierPhase_ += (2.0 * M_PI) / ((double)sampleRate_ / carrierFrequency_);
        carrierPhase_ = wrapPhase(carrierPhase_);

        double carrier = 0.0;
        if (carrierShape_ == SignalShape::Sine) {
            carrier = sin(carrierPhase_);
        } else if (carrierShape_ == SignalShape::Square) {
            carrier = carrierPhase_ > M_PI ? 1.0 : -1.0;
        } else if (carrierShape_ == SignalShape::Triangle) {
            carrier = (carrierPhase_ < M_PI)
                ? -1.0 + (2.0 / M_PI) * carrierPhase_
                :  3.0 - (2.0 / M_PI) * carrierPhase_;
        } else if (carrierShape_ == SignalShape::Sawtooth) {
            carrier = (carrierPhase_ / M_PI) - 1.0;
        }

        carrier = carrier * carrierAmplitude_;
        carrier = modulateCarrier((float)carrier);

        if (noiseAmplitude_ > 0.0) {
            // Box-Muller transform: needs the natural log, not log10. r1 is
            // strictly positive (RAND_MAX+1 in the denominator avoids the 1.0
            // boundary; the small-r1 case is what produces the gaussian tails).
            double r1 = (double)rand() / ((double)RAND_MAX + 1.0);
            double r2 = (double)rand() / ((double)RAND_MAX + 1.0);
            double noise = sqrt(-2.0 * log(r1)) * cos(2.0 * M_PI * r2);
            double noiseAmpDec = noiseAmplitude_ / 100.0;
            carrier = carrier + (noiseAmpDec * noise);
        }

        if (resolution) {
            uint32_t sample = (uint32_t)(((carrier * 0.5) + (0.5 * carrierAmplitude_)) * (double)UINT32_MAX);
            carrier = (double)(sample >> (32 - resolution));
        }

        return carrier;
    }

    inline float modulateCarrier(float carrier) {
        if (amAmplitude_ > 0.0f) {
            amPhase_ += amPhaseInc_;
            // Wrap to [0, 2pi). The double-precision wrap is used so that
            // high AM frequencies relative to the sample rate still close on
            // the right principal value.
            amPhase_ = (float)wrapPhase((double)amPhase_);
            float am = 0.0f;
            if (amShape_ == SignalShape::Sine) {
                am = sinf(amPhase_);
            } else if (amShape_ == SignalShape::Square) {
                am = amPhase_ > (float)M_PI ? 1.0f : -1.0f;
            } else if (amShape_ == SignalShape::Triangle) {
                am = (amPhase_ < (float)M_PI)
                    ? -1.0f + (2.0f / (float)M_PI) * amPhase_
                    :  3.0f - (2.0f / (float)M_PI) * amPhase_;
            } else if (amShape_ == SignalShape::Sawtooth) {
                am = (amPhase_ / (float)M_PI) - 1.0f;
            } else if (amShape_ == SignalShape::None) {
                am = 0.0f;
            }
            carrier = (1.0f + amAmplitudeDec_ * am) * carrier;
        }
        return carrier;
    }

    // Snap the carrier phase to pi/2 when the supplied triplet straddles a
    // local maximum (x1 is the peak). Useful for re-aligning a free-running
    // oscillator to an external sync event.
    inline bool lockPhase(float x0, float x1, float x2) {
        if (x1 > 0.0f && x0 <= x1 && x2 <= x1) {
            carrierPhase_ = M_PI / 2.0;
            return true;
        }
        return false;
    }

private:
    int sampleRate_;

    double      carrierPhase_;
    double      carrierFrequency_;
    double      carrierAmplitude_;
    SignalShape carrierShape_;

    float       amPhase_;
    float       amFrequency_;
    float       amAmplitude_;
    float       amAmplitudeDec_;
    float       amPhaseInc_;
    SignalShape amShape_;

    double      noiseAmplitude_;
};

} // namespace signalcore

#endif // _SIGNALCORE_DDS_H_
