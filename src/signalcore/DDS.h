/******************************************************************************
 * @file DDS.h
 * @brief Direct Digital Synthesis and Signal Generation
 *
 * Provides a table-based DDS oscillator and a flexible SignalGenerator class
 * for producing sine, square, triangle, sawtooth, and noise waveforms with
 * optional amplitude modulation and gaussian noise injection.
 *
 * The DDS class uses a 32-bit phase accumulator and an int32_t wave table.
 * DDSSineTable<> builds its table at construction time and stores it in
 * RAM: SAMPLES * sizeof(int32_t) bytes (Samples1024 = 4 KB). Size tables to
 * your board's RAM — nothing is placed in flash.
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


// Noise and None produce silence when used as a SignalGenerator carrier
// shape: noise injection is configured separately via setNoise(), and None
// mutes the carrier. Both exist mainly for AM-shape/UI use.
enum class SignalShape : uint8_t {
    Sine,
    Square,
    Triangle,
    Sawtooth,
    Noise,
    None
};


// --- Phase-accumulator DDS over a pre-built integer wave table ---

// Table entries and tuning words are explicitly int32_t (not int): the
// full-scale table values are scaled to INT32_MAX, which overflows the
// 16-bit int on AVR. With int32_t the module behaves identically on every
// Arduino target.
class DDS {
public:
    DDS(int32_t* table, TableSize tableSize, uint32_t samplingFrequency)
        : table_(table),
          tableSize_(tableSize),
          acc_(0),
          samplingFrequency_(samplingFrequency) {}

    // Advance the 32-bit phase accumulator by tuningWord and look up a
    // sample. tuningWord = frequency / sampleRate * 2^32, so the accumulator
    // wraps exactly once per output period — the intentional uint32_t
    // overflow IS the phase wrap, no modulo needed. Only the top
    // log2(tableSize) bits select a table entry; the low bits carry
    // sub-sample phase precision between calls. A negative tuningWord steps
    // backwards through the table. Returns the raw table value (full-scale
    // +/- INT32_MAX for DDSSineTable).
    inline int32_t nextSampleIncInt(int32_t tuningWord) {
        acc_ += (uint32_t)tuningWord;
        uint32_t index = acc_ >> (32 - static_cast<uint8_t>(tableSize_));
        return table_[index];
    }

    inline int32_t nextSampleIncPhase(double phaseInc) {
        // The int64_t intermediate keeps the double->integer conversion
        // defined when the phase increment maps above INT32_MAX (i.e. past
        // Nyquist); the truncation to int32_t wraps like the accumulator.
        int32_t tuningWord = (int32_t)(int64_t)((phaseInc / (2.0 * M_PI)) * 4294967296.0);
        return nextSampleIncInt(tuningWord);
    }

    inline int32_t nextSampleAtFrequency(double frequency) {
        int32_t tuningWord = (int32_t)(int64_t)(4294967296.0 * (frequency / (double)samplingFrequency_));
        return nextSampleIncInt(tuningWord);
    }

    inline int getTableSize() const {
        return 1 << static_cast<uint8_t>(tableSize_);
    }

private:
    int32_t*   table_;
    TableSize  tableSize_;
    uint32_t   acc_;
    uint32_t   samplingFrequency_;
};


// Sine table sized by the TableSize enum. The table is filled at
// construction time and lives in RAM as an ordinary member array:
// SAMPLES * 4 bytes (Samples1024 = 4 KB, Samples65536 = 256 KB — size to
// your board). The implicit conversion lets the table be passed directly
// to the DDS constructor.
template<TableSize TS>
struct DDSSineTable {
    static constexpr uint32_t SAMPLES = 1u << static_cast<uint8_t>(TS);

    DDSSineTable() {
        double phase = 0.0;
        double phaseInc = (2.0 * M_PI) / (double)SAMPLES;
        for (uint32_t i = 0; i < SAMPLES; i++) {
            data[i] = (int32_t)(sin(phase) * INT32_MAX);
            phase += phaseInc;
        }
    }
    operator int32_t*() { return data; }
    int32_t data[SAMPLES];
};


// --- Standalone table fillers and sample generators ---

// Fill `table` with a sine at `sineFrequency` for playback at
// `sampleFrequency`. With bufferChannels = 2 the sample is duplicated into
// both slots of an interleaved stereo buffer; `tableSize` is the total
// entry count of the buffer, not the per-channel frame count. The loop
// bound guarantees every write (including the duplicate) stays inside
// table[0 .. tableSize-1].
inline void fillSineTable(double sampleFrequency, double sineFrequency,
                          int tableSize, int32_t* table, int bufferChannels = 1) {
    double phase = 0.0;
    double phaseInc = (2.0 * M_PI) / (sampleFrequency / sineFrequency);
    for (int i = 0; i + bufferChannels <= tableSize; i += bufferChannels) {
        int32_t x = (int32_t)(sin(phase) * INT32_MAX);
        table[i] = x;
        if (bufferChannels > 1) table[i + 1] = x;
        phase = wrapPhase(phase + phaseInc);
    }
}


// Find a table size near `tableSize` (input) such that an integer number of
// periods of `targetFrequency` fits exactly at `sampleRate`. The closest
// match minimizes spectral leakage when the table loops.
// tableSize is IN/OUT: pass the maximum size you can afford; on return it
// holds the chosen size. tablePeriods and actualFrequency are outputs.
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


// amp scales the whole unipolar waveform: output spans [0, amp*UINT16_MAX].
// Note this differs from createSineSamples32(), where amp scales only the
// AC swing around mid-scale.
inline void createSineSamples(uint16_t* data, int size, double periods, double amp) {
    double phase = 0.0;
    double phaseInc = (2.0 * M_PI * periods) / (double)size;
    for (int i = 0; i < size; i++) {
        double x = (sin(phase) + 1.0) * 0.5 * amp;
        data[i] = (uint16_t)(x * UINT16_MAX);
        phase += phaseInc;
    }
}


// amp scales only the AC swing around mid-scale: output stays centered at
// UINT32_MAX/2 regardless of amp. Note this differs from createSineSamples().
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
    // Defaults: sine carrier at 0 Hz (constant 0 output) with AM and noise
    // disabled — call setCarrier() before expecting a waveform.
    SignalGenerator(int sampleRate)
        : sampleRate_(sampleRate),
          carrierPhase_(0.0), carrierFrequency_(0.0), carrierAmplitude_(1.0),
          carrierShape_(SignalShape::Sine),
          amPhase_(0.0f), amAmplitude_(0.0f),
          amAmplitudeDec_(0.0f), amPhaseInc_(0.0f), amShape_(SignalShape::None),
          noiseAmplitude_(0.0) {}

    // frequency: Hz. amplitude: linear gain (1.0 = full scale) — note this
    // differs from setAM()/setNoise(), whose amplitudes are PERCENT.
    inline void setCarrier(SignalShape shape, double frequency,
                           double amplitude = 1.0, double phase = 0.0) {
        carrierShape_     = shape;
        carrierFrequency_ = frequency;
        carrierAmplitude_ = amplitude;
        carrierPhase_     = phase;
    }

    // Configure amplitude modulation of the carrier.
    // frequency: Hz. amplitude: PERCENT modulation depth, 0..100 —
    // setAM(shape, 10, 50) gives 50% depth. Note this differs from
    // setCarrier(), whose amplitude is a linear gain (1.0 = full scale).
    // amplitude <= 0 disables AM.
    inline void setAM(SignalShape shape, double frequency, double amplitude) {
        amAmplitude_    = (float)amplitude;
        amAmplitudeDec_ = amAmplitude_ / 100.0f;
        amPhaseInc_     = (float)((2.0 * M_PI) / ((double)sampleRate_ / frequency));
        amShape_        = shape;
    }

    // Add gaussian noise to the output. amplitude is PERCENT of full scale
    // (0..100): setNoise(5) adds noise with sigma = 0.05. 0 disables noise.
    inline void setNoise(double amplitude) {
        noiseAmplitude_ = amplitude;
    }

    // Return the next sample.
    //   resolution = 0 : raw bipolar float, roughly +/- carrierAmplitude
    //                    (AM and noise can push it slightly beyond).
    //   resolution = N : the sample is offset to unipolar and quantized to
    //                    an N-bit unsigned code in [0, 2^N) — e.g. N=12
    //                    gives 0..4095, ready to feed a DAC. Note the
    //                    completely different scale from the resolution=0
    //                    case; a DC offset of half scale is baked in, so
    //                    removeDC() matters again before any FFT.
    inline double nextSample(uint8_t resolution) {
        carrierPhase_ += (2.0 * M_PI) / ((double)sampleRate_ / carrierFrequency_);
        carrierPhase_ = wrapPhase(carrierPhase_);

        // Shapes are generated directly from the phase (not band-limited).
        // Square/Triangle/Sawtooth contain harmonics above Nyquist
        // (sampleRate/2) that alias back into the output as the carrier
        // frequency rises — expect artifacts above roughly sampleRate/10
        // for these shapes. Sine is alias-free below Nyquist.
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
            // Offset the bipolar carrier to unipolar [0, 1] before scaling
            // to 32-bit codes. AM overshoot or noise can push the value
            // outside that range, and a negative double cast to uint32_t
            // wraps to garbage codes — clamp first so excursions saturate
            // at the rails instead.
            double norm = (carrier * 0.5) + (0.5 * carrierAmplitude_);
            if (norm < 0.0) norm = 0.0;
            else if (norm > 1.0) norm = 1.0;
            uint32_t sample = (uint32_t)(norm * (double)UINT32_MAX);
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
    float       amAmplitude_;
    float       amAmplitudeDec_;
    float       amPhaseInc_;
    SignalShape amShape_;

    double      noiseAmplitude_;
};

} // namespace signalcore

#endif // _SIGNALCORE_DDS_H_
