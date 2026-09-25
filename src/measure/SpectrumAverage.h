/******************************************************************************
 * @file SpectrumAverage.h
 * @brief Running mean of bin powers over several sine-wave records, with
 *        the MT-003 metrics of the averaged spectrum.
 *
 * Averaging the linear bin powers of K records does not lower the noise
 * floor: the mean level of a noise bin is the same in one record and in the
 * average. What it lowers is the scatter of the floor. A single record's
 * noise bin has a standard deviation equal to its mean (a chi-squared
 * variable with two degrees of freedom); the mean of K records has a
 * standard deviation of 1/sqrt(K) of that. On a plot the grass flattens,
 * small spurs stand out, and a peak-per-column display reads lower because
 * the maximum over a group of bins sits closer to their mean.
 *
 * The mean is kept in one buffer. The first K records form an exact block
 * mean. From record K+1 on, each new record replaces 1/K of the mean, so a
 * meter that runs indefinitely follows the last K records or so without a
 * ring of K spectra.
 *
 * The metrics of the average (SINAD, SNR, THD, SFDR, the ENOB pair) use the
 * same rules as one record: DC lobe and fundamental lobe excluded, window
 * noise bandwidth and coherent gain corrected. Because the mean of power
 * sums is the sum of mean powers, SINAD of the average equals the mean of
 * the single-record SINAD values to within their scatter.
 *
 * The mean is accumulated in double whatever SpectrumReal the records'
 * powers arrive in (see SineMetrics.h for the precision contract).
 *
 * Portable C++11, no allocation: the caller owns the mean buffer.
 *
 *   This file is a part of the SignalCore project, a cross-platform digital
 *   signal processing toolkit released by VoidLoop. See www.voidloop.com for
 *   updates.
 *
 * MIT License (MIT), Copyright (c) 2026 VoidLoop
 ******************************************************************************/

#ifndef _SIGNALCORE_MEASURE_SPECTRUMAVERAGE_H_
#define _SIGNALCORE_MEASURE_SPECTRUMAVERAGE_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "SineMetrics.h"

namespace signalcore {
namespace measure {

struct PsdAverage {
    // Attach a buffer of `capacity` doubles: nfft/2 + 1 for the largest
    // transform the average will see.
    void bind(double* buffer, size_t capacity) {
        mean_ = buffer;
        capacity_ = capacity;
        count_ = 0;
    }

    // Forget everything and set the averaging depth (1 = no averaging).
    void reset(unsigned records) {
        depth_ = records < 1 ? 1 : records;
        count_ = 0;
    }

    // Fold one record's bin powers (computeBinPowers() output, SpectrumReal)
    // into the mean. A record from a different transform length or window
    // restarts the average. Returns false when no buffer is bound or it is
    // too small.
    bool add(const SpectrumReal* power, const BinPowerInfo& info) {
        const size_t bins = info.nfft / 2 + 1;
        if (!mean_ || !power || info.nfft == 0 || bins > capacity_) return false;
        if (count_ == 0 || info.nfft != info_.nfft || info.window != info_.window) {
            for (size_t k = 0; k < bins; ++k) mean_[k] = (double)power[k];
            info_ = info;
            count_ = 1;
            return true;
        }
        // Exact block mean up to `depth_` records, then exponential
        // forgetting with the same weight.
        const unsigned n = count_ < depth_ ? count_ + 1 : depth_;
        const double w = 1.0 / (double)n;
        for (size_t k = 0; k < bins; ++k) mean_[k] += w * ((double)power[k] - mean_[k]);
        if (count_ < depth_) ++count_;
        return true;
    }

    bool valid() const { return mean_ != nullptr && count_ > 0; }
    unsigned records() const { return count_; }       // records in the mean (<= depth)
    unsigned depth() const { return depth_; }
    size_t bins() const { return count_ ? info_.nfft / 2 + 1 : 0; }
    const double* powers() const { return mean_; }
    const BinPowerInfo& info() const { return info_; }

private:
    double*  mean_ = nullptr;
    size_t   capacity_ = 0;
    unsigned depth_ = 1;
    unsigned count_ = 0;
    BinPowerInfo info_{};
};

// The averaged spectrum's headline numbers, in the same terms analyze()
// reports for one record. The amplitude comes from the fundamental lobe
// power (A = sqrt(2 S)); with no time-domain fit it carries the window's
// scalloping error, so the fit's amplitude stays the better level estimate.
struct AveragedResult {
    bool     valid;
    unsigned records;        // records in the mean
    SpectrumResult spec;
    double   ampCodes;
    double   inputDbfs;      // 20 log10(A / F)
    double   sinadDb;
    double   enobAtInput;    // (SINAD - 1.76) / 6.02
    double   enobFullScale;  // (SINAD - 1.76 + 20 log10(F / A)) / 6.02
};

// Metrics of the current mean. `scratch` must hold nfft SpectrumReal (the
// workspace's input slice serves); meta supplies the sample rate,
// resolution and the tone hint (use the last record's fitted frequency).
// Returns false when the average is empty or no fundamental is found.
inline bool analyzeAverage(const PsdAverage& avg, const RecordMeta& meta,
                           const Options& opt, SpectrumReal* scratch, AveragedResult& out) {
    memset(&out, 0, sizeof(out));
    if (!avg.valid()) return false;
    if (!spectrumMetrics(avg.powers(), avg.info(), meta, opt, scratch, out.spec)) return false;
    const double F = ldexp(1.0, meta.bits - 1);
    out.records = avg.records();
    out.ampCodes = sqrt(2.0 * out.spec.S);
    out.sinadDb = out.spec.sinadDb;
    if (out.ampCodes > 0.0) {
        out.inputDbfs = 20.0 * log10(out.ampCodes / F);
        out.enobAtInput = (out.sinadDb - 1.76) / 6.02;
        out.enobFullScale = (out.sinadDb - 1.76 + 20.0 * log10(F / out.ampCodes)) / 6.02;
    }
    out.valid = true;
    return true;
}

} // namespace measure
} // namespace signalcore

#endif // _SIGNALCORE_MEASURE_SPECTRUMAVERAGE_H_
