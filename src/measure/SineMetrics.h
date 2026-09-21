/******************************************************************************
 * @file SineMetrics.h
 * @brief ADC dynamic-performance metrics from one sine-wave record.
 *
 * Given a record of raw converter codes that contains a single sine wave,
 * this header computes the figures of merit defined in Analog Devices
 * MT-003 (Kester, "Understand SINAD, ENOB, SNR, THD, THD + N, and SFDR"):
 *
 *   SINAD   signal power over everything else in DC..fs/2 (DC excluded)
 *   ENOB    (SINAD - 1.76) / 6.02, at the measured input level and
 *           normalized to full scale (MT-003 eq. 2)
 *   SNR     signal over noise with the harmonics H2..H6 removed
 *   THD     harmonics H2..H6 over signal (reported as a negative dBc level)
 *   SFDR    signal over the largest spur, in dBc and dBFS
 *
 * Two independent estimators run on the same record:
 *
 *   1. A windowed FFT (Hann, 4-term Blackman-Harris, 4-term Nuttall or
 *      7-term Blackman-Harris) with the coherent-gain and equivalent-noise-
 *      bandwidth corrections applied, DC and fundamental lobes masked.
 *      This is the primary estimator and the only one that can separate
 *      harmonics and spurs from noise.
 *   2. A three-parameter least-squares sine fit in the time domain
 *      (x[n] = c + a cos(wn) + b sin(wn)). It needs no window and no FFT
 *      gain correction, so agreement between the two SINAD values on the
 *      same record is the built-in check of the window corrections.
 *
 * The record is never modified and no memory is allocated here. The FFT
 * path needs a caller-supplied workspace (see workspaceDoubles()); the fit
 * needs about 1 kB of stack. All accumulation is in double.
 *
 * Portable C++11: no Arduino, no VoidLoopEngine, no CMSIS dependency. The
 * same header runs on the host test rig and on the target.
 *
 * Record contract: one contiguous capture at a single sample rate. Records
 * assembled from several DMA fills, or with dropped samples, produce a
 * DISCONTINUITY / RESIDUAL_GROWTH rejection rather than a wrong number.
 * A single wrong sample (an impulse rather than a shift) is caught by the
 * spike screen when it is large, and otherwise by the fit/FFT agreement
 * check: a window weights an impulse by its position, so a non-stationary
 * residual makes the two estimators disagree while a stationary one does
 * not. The spike floor (15 % of the amplitude, the host reference tool's
 * rule) is deliberately loose: real converters throw occasional sparkle
 * codes of a few tens of codes that are part of the measurement, not a
 * capture fault, so they are reported through FitResult::residMaxCodes
 * rather than rejected (a 2 % floor rejected half the Basecamp records on
 * the bench, 2026-09-14, with no change in SINAD).
 *
 * Known limits: a spur that falls inside a harmonic's lobe is counted as
 * that harmonic (THD up, SNR up, SINAD unchanged); Hann's leakage caps the
 * SINAD it can resolve near 40 dB for a tone that is not bin-centred, which
 * the agreement check reports as ESTIMATORS_DISAGREE. Use bh4 or bh7 for
 * converter-class dynamic range.
 *
 *   This file is a part of the SignalCore project, a cross-platform digital
 *   signal processing toolkit released by VoidLoop. See www.voidloop.com for
 *   updates.
 *
 * MIT License (MIT), Copyright (c) 2026 VoidLoop
 ******************************************************************************/

#ifndef _SIGNALCORE_MEASURE_SINEMETRICS_H_
#define _SIGNALCORE_MEASURE_SINEMETRICS_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <algorithm>

#include "../signalcore/Windows.h"

namespace signalcore {
namespace measure {

// ---------------------------------------------------------------------------
// Windows
// ---------------------------------------------------------------------------
//
// The window shapes (coefficients, lobe widths) live in signalcore::Windows.h
// as the single source of truth; this header only adds the serial-protocol
// name strings ("hann", "bh4", "nuttall4", "bh7") and the wiring to run
// Windows.h's cosine-series tables through the periodic-form FFT below.
// "nuttall4" is a protocol-level name for the WindowType::BlackmanNuttall
// coefficients, not WindowType::Nuttall (Windows.h keeps those as two
// distinct, differently-tapered windows; the wire format only ever asked
// for the Blackman-Nuttall one, so the old name stays).
using signalcore::WindowType;
using signalcore::CosineSeries;
using signalcore::cosineSeries;
using signalcore::cosineSeriesValue;

// Short lower-case token used on the wire (MEAS? requests, sine_metrics_cli,
// utilities/compare_sine_metrics.py and utilities/adc_measure.py all match
// on these exact strings). Any other WindowType falls back to its
// Windows.h abbreviation.
inline const char* windowName(WindowType w) {
    switch (w) {
        case WindowType::Hann:            return "hann";
        case WindowType::BlackmanHarris:  return "bh4";
        case WindowType::BlackmanNuttall: return "nuttall4";
        case WindowType::BlackmanHarris7: return "bh7";
        default:                          return getWindowNameAbbr(w);
    }
}

// Parse one of the four wire tokens above into a WindowType. Returns false
// (leaving `out` untouched) for anything else.
inline bool parseWindowName(const char* tok, WindowType& out) {
    if (!strcmp(tok, "hann"))     { out = WindowType::Hann;            return true; }
    if (!strcmp(tok, "bh4"))      { out = WindowType::BlackmanHarris;  return true; }
    if (!strcmp(tok, "nuttall4")) { out = WindowType::BlackmanNuttall; return true; }
    if (!strcmp(tok, "bh7"))      { out = WindowType::BlackmanHarris7; return true; }
    return false;
}

// Window value at sample n of N (periodic form, denominator N):
//   w[n] = a0 - a1 cos(2pi n/N) + a2 cos(4pi n/N) - ...
// See signalcore::cosineSeriesValue for the recurrence; this measurement
// path always uses the periodic form (matches utilities/adc_measure.py,
// the compare gate is bit-exact), never the symmetric form Windows.h's
// float display path (createWindowCoefficients) uses.
inline double windowValue(const CosineSeries& w, double cosTheta) {
    return cosineSeriesValue(w, cosTheta);
}

// ---------------------------------------------------------------------------
// Result structures
// ---------------------------------------------------------------------------

// Rejecting flags make valid == false; informational flags do not.
enum ResultFlag : uint32_t {
    FLAG_CLIPPED         = 1u << 0,   // codes touched a rail (within clipMarginCodes)
    FLAG_TOO_SHORT       = 1u << 1,   // fewer than Options::minSamples
    FLAG_FEW_CYCLES      = 1u << 2,   // fewer than Options::minCycles in the record
    FLAG_NOT_CONVERGED   = 1u << 3,   // frequency refinement hit its cap or left the bracket
    FLAG_DISCONTINUITY   = 1u << 4,   // residual spike: dropped or duplicated samples
    FLAG_RESIDUAL_GROWTH = 1u << 5,   // residual rms grows along the record
    FLAG_LOW_AMPLITUDE   = 1u << 6,   // fitted amplitude below Options::minAmplitudeCodes
    FLAG_BAD_ARGS        = 1u << 7,   // null record, fs <= 0, bits out of range
    FLAG_HINT_MISMATCH   = 1u << 8,   // fit and FFT disagree on the frequency by > 1 bin
    FLAG_ESTIMATORS_DISAGREE = 1u << 9, // fit and FFT SINAD differ by > Options::agreeToleranceDb

    // Informational (do not reject)
    FLAG_NO_SPECTRUM     = 1u << 16,  // FFT skipped: no workspace supplied
    FLAG_SPECTRUM_TRUNCATED = 1u << 17, // n not a power of two; FFT used the first 2^k samples
    FLAG_LOBE_OVERLAP    = 1u << 18,  // fundamental lobe overlaps the DC lobe
    FLAG_HARMONIC_COLLISION = 1u << 19, // at least one of H2..H6 was excluded (folded onto another lobe)
    FLAG_SPUR_UNRESOLVED = 1u << 20,  // largest spur is not 12 dB above the noise median

    FLAGS_REJECTING      = 0x0000FFFFu
};

struct RecordMeta {
    double  fsHz;         // sample rate, exact (derive from the integer divider)
    uint8_t bits;         // converter resolution; full scale peak F = 2^(bits-1) codes
    double  scaleUvPerLsb;// microvolts per code, input-referred (0 if unknown)
    double  fHintHz;      // expected tone frequency, 0 = find it from the record
};

struct Options {
    WindowType window;
    int    maxHarmonic;          // highest harmonic counted (MT-003: 6)
    size_t minSamples;           // TOO_SHORT below this
    double minCycles;            // FEW_CYCLES below this
    double minAmplitudeCodes;    // LOW_AMPLITUDE below this
    int    clipMarginCodes;      // CLIPPED when min <= margin or max >= maxCode - margin
    double spikeSigma;           // DISCONTINUITY if |e| > max(spikeSigma*rms, spikeFrac*amp)
    double spikeFrac;
    double tailGrowth;           // RESIDUAL_GROWTH if tail rms > tailGrowth * head rms
    int    maxIterations;        // frequency refinement cap
    double agreeToleranceDb;     // fit-vs-FFT SINAD agreement bound at 8192 points (see agreeToleranceFor)
    double spurResolvedDb;       // spur must clear the noise median by this to be "resolved"
    size_t maxFftLength;         // cap on the transform length (0 = largest power of two <= n);
                                 // the fit always uses the whole record, and its
                                 // residual over the transform's span is what the
                                 // agreement check compares

    Options()
        : window(WindowType::BlackmanHarris), maxHarmonic(6), minSamples(1024), minCycles(8.0),
          minAmplitudeCodes(4.0), clipMarginCodes(0), spikeSigma(8.0),
          spikeFrac(0.15), tailGrowth(3.0), maxIterations(80),
          agreeToleranceDb(0.5), spurResolvedDb(12.0), maxFftLength(0) {}
};

struct FitResult {
    double fHz;           // refined frequency
    double ampCodes;      // fundamental peak amplitude A = sqrt(a^2 + b^2)
    double dcCodes;       // fitted offset c
    double phaseRad;      // atan2(-b, a): x = c + A cos(wn + phase)
    double cycles;        // f * n / fs
    double S;             // (a^2 + b^2) / 2, code^2
    double R;             // residual power SS/n, code^2
    double sinadDb;       // 10 log10(S / R)
    double spanSinadDb;   // the same over the first spanN samples only (the transform's span)
    double residRmsCodes;
    double residMaxCodes; // largest |e| over the record
    double headRmsCodes;  // residual rms over the first quarter (head-quarter fit)
    double tailRmsCodes;  // residual rms over the last quarter (same head-quarter parameters)
    int    iterations;
    bool   converged;
};

struct HarmonicLine {
    double hz;            // folded location in 0..fs/2
    double dbc;           // 10 log10(P_h / S)
    double power;         // code^2
    bool   valid;         // false when its lobe collides with DC, the fundamental or another harmonic
};

struct SpectrumResult {
    size_t nfft;          // samples transformed (power of two <= n)
    double binHz;         // fs / nfft
    double coherentGain;  // sum(w) / nfft
    double enbwBins;      // nfft * sum(w^2) / sum(w)^2
    double fHz;           // fundamental, parabolic peak interpolation
    int    peakBin;
    double S;             // fundamental lobe power, code^2
    double R;             // all other unmasked power, code^2
    double harmonicPower; // sum of valid H2..Hmax powers
    double sinadDb;
    double snrDb;         // harmonics removed from R
    double thdDbc;        // 10 log10(harmonicPower / S), negative
    double sfdrDbc;       // 10 log10(S / largest spur)
    double sfdrDbfs;      // full scale over the largest spur
    double spurHz;
    double spurDbc;
    double spurPower;
    bool   spurResolved;
    double noiseMedianDbc;// median unmasked bin power relative to S
    double dcPower;       // DC lobe power (after fitted-DC removal), diagnostics
    HarmonicLine harmonic[8]; // index h = 2..maxHarmonic used; 0,1 unused
};

struct Result {
    uint32_t flags;
    bool     valid;          // no rejecting flag set
    RecordMeta meta;
    size_t   n;
    uint16_t minCode;
    uint16_t maxCode;
    double   fullScalePeakCodes; // F = 2^(bits-1)

    FitResult      fit;
    SpectrumResult spec;
    bool           specValid;    // spectrum ran

    // Headline numbers. primaryIsSpectrum says which estimator they come from.
    bool   primaryIsSpectrum;
    double fHz;
    double ampCodes;
    double ampVolts;             // 0 when scaleUvPerLsb unknown
    double inputDbfs;            // 20 log10(A / F)
    double sinadDb;
    double enobAtInput;          // (SINAD - 1.76) / 6.02
    double enobFullScale;        // (SINAD - 1.76 + 20 log10(F / A)) / 6.02
    double fitFftDisagreeDb;     // |fit SINAD - FFT SINAD|

    // Textbook references for the tutorial (MT-003 and the FFT process gain)
    double idealSnrDb;           // 6.02 bits + 1.76
    double processGainDb;        // 10 log10(nfft / 2)
    double idealFftFloorDb;      // idealSnrDb + processGainDb, below full scale
};

// ---------------------------------------------------------------------------
// Workspace
// ---------------------------------------------------------------------------

// Largest power of two <= n (0 if n < 32), optionally capped.
inline size_t fftLengthFor(size_t n, size_t cap = 0) {
    if (cap && n > cap) n = cap;
    if (n < 32) return 0;
    size_t p = 1;
    while ((p << 1) <= n) p <<= 1;
    return p;
}

// Doubles the spectrum estimator needs for a record of n samples:
// nfft doubles for the in-place transform plus nfft/2 + 1 for the bin powers.
inline size_t workspaceDoubles(size_t n, size_t cap = 0) {
    size_t nfft = fftLengthFor(n, cap);
    return nfft ? nfft + nfft / 2 + 1 : 0;
}

// ---------------------------------------------------------------------------
// Double-precision real FFT (radix-2, in place, via a half-length complex FFT)
// ---------------------------------------------------------------------------

namespace detail {

// In-place iterative radix-2 complex FFT on m interleaved (re, im) pairs.
// m must be a power of two. Twiddles come from a per-stage rotation
// recurrence reseeded from cos/sin at every stage, accurate to ~1e-13 for
// m up to 2^15.
inline void complexFFT(double* z, size_t m) {
    // bit-reversal permutation
    for (size_t i = 1, j = 0; i < m; ++i) {
        size_t bit = m >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            std::swap(z[2 * i],     z[2 * j]);
            std::swap(z[2 * i + 1], z[2 * j + 1]);
        }
    }
    for (size_t len = 2; len <= m; len <<= 1) {
        const double ang = -2.0 * M_PI / (double)len;
        const double wr = cos(ang), wi = sin(ang);
        const size_t half = len >> 1;
        for (size_t i = 0; i < m; i += len) {
            double cr = 1.0, ci = 0.0;
            for (size_t j = 0; j < half; ++j) {
                if ((j & 63) == 0 && j) {           // reseed to bound recurrence drift
                    cr = cos(ang * (double)j);
                    ci = sin(ang * (double)j);
                }
                double* a = z + 2 * (i + j);
                double* b = z + 2 * (i + j + half);
                double tr = b[0] * cr - b[1] * ci;
                double ti = b[0] * ci + b[1] * cr;
                b[0] = a[0] - tr;  b[1] = a[1] - ti;
                a[0] += tr;        a[1] += ti;
                double ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

// Real FFT of the nfft doubles in `work` (interpreted as nfft/2 complex
// pairs after packing), writing |X[k]|^2 for k = 0..nfft/2 into `power`.
inline void realFFTPower(double* work, size_t nfft, double* power) {
    const size_t m = nfft / 2;
    complexFFT(work, m);
    // X[0] and X[nfft/2] from Z[0]
    double z0r = work[0], z0i = work[1];
    power[0] = (z0r + z0i) * (z0r + z0i);
    power[m] = (z0r - z0i) * (z0r - z0i);
    const double ang = -2.0 * M_PI / (double)nfft;
    for (size_t k = 1; k <= m / 2; ++k) {
        size_t j = m - k;
        double zkr = work[2 * k], zki = work[2 * k + 1];
        double zjr = work[2 * j], zji = work[2 * j + 1];
        // even part E = (Zk + conj(Zj)) / 2, odd part O = (Zk - conj(Zj)) / (2i)
        double er = 0.5 * (zkr + zjr), ei = 0.5 * (zki - zji);
        double orr = 0.5 * (zki + zji), oi = -0.5 * (zkr - zjr);
        double c = cos(ang * (double)k), s = sin(ang * (double)k);
        // X[k] = E + W^k O, W^k = c + i s
        double tr = orr * c - oi * s, ti = orr * s + oi * c;
        double xr = er + tr, xi = ei + ti;
        power[k] = xr * xr + xi * xi;
        // X[j], j = nfft/2 - k, by the same split with the roles swapped.
        double er2 = 0.5 * (zjr + zkr), ei2 = 0.5 * (zji - zki);
        double or2 = 0.5 * (zji + zki), oi2 = -0.5 * (zjr - zkr);
        double c2 = cos(ang * (double)j), s2 = sin(ang * (double)j);
        double tr2 = or2 * c2 - oi2 * s2, ti2 = or2 * s2 + oi2 * c2;
        double xr2 = er2 + tr2, xi2 = ei2 + ti2;
        power[j] = xr2 * xr2 + xi2 * xi2;
    }
}

inline double db10(double ratio) {
    return ratio > 0.0 ? 10.0 * log10(ratio) : -999.0;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Sine fit (time domain)
// ---------------------------------------------------------------------------

namespace detail {

struct FitSums {
    double sc, ss, scc, sss, scs, sx, sxc, sxs, sxx;
    size_t n;
};

// One pass over x[0..n) at angular frequency w (rad/sample). cos/sin by a
// rotation recurrence reseeded every 256 samples. `offset` is subtracted
// from every code before accumulating to keep the sums well conditioned.
inline void accumulate(const uint16_t* x, size_t n, double w, double offset, FitSums& s) {
    s.sc = s.ss = s.scc = s.sss = s.scs = s.sx = s.sxc = s.sxs = s.sxx = 0.0;
    s.n = n;
    const double dr = cos(w), di = sin(w);
    double cr = 1.0, ci = 0.0;
    for (size_t i = 0; i < n; ++i) {
        if ((i & 255) == 0) { cr = cos(w * (double)i); ci = sin(w * (double)i); }
        const double xv = (double)x[i] - offset;
        s.sc  += cr;        s.ss  += ci;
        s.scc += cr * cr;   s.sss += ci * ci;  s.scs += cr * ci;
        s.sx  += xv;        s.sxc += xv * cr;  s.sxs += xv * ci;
        s.sxx += xv * xv;
        double ncr = cr * dr - ci * di;
        ci = cr * di + ci * dr;
        cr = ncr;
    }
}

// Solve the 3x3 normal equations for (a, b, c). Returns false if singular.
inline bool solve(const FitSums& s, double& a, double& b, double& c) {
    // [scc scs sc][a]   [sxc]
    // [scs sss ss][b] = [sxs]
    // [sc  ss  n ][c]   [sx ]
    const double n = (double)s.n;
    double m[3][4] = {
        {s.scc, s.scs, s.sc, s.sxc},
        {s.scs, s.sss, s.ss, s.sxs},
        {s.sc,  s.ss,  n,    s.sx }
    };
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        for (int r = col + 1; r < 3; ++r)
            if (fabs(m[r][col]) > fabs(m[piv][col])) piv = r;
        if (fabs(m[piv][col]) < 1e-12) return false;
        if (piv != col) for (int k = 0; k < 4; ++k) std::swap(m[piv][k], m[col][k]);
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            double f = m[r][col] / m[col][col];
            for (int k = col; k < 4; ++k) m[r][k] -= f * m[col][k];
        }
    }
    a = m[0][3] / m[0][0];
    b = m[1][3] / m[1][1];
    c = m[2][3] / m[2][2];
    return true;
}

// Residual sum of squares at the least-squares solution, from the sums alone:
// SS = sum(x^2) - (a*sxc + b*sxs + c*sx).
inline double residualSS(const uint16_t* x, size_t n, double w, double offset) {
    FitSums s;
    accumulate(x, n, w, offset, s);
    double a, b, c;
    if (!solve(s, a, b, c)) return 1e300;
    double ss = s.sxx - (a * s.sxc + b * s.sxs + c * s.sx);
    return ss < 0.0 ? 0.0 : ss;
}

// Coarse frequency estimate from hysteresis zero crossings of the record
// about its mid-range. Used only when no hint and no spectrum are available.
inline double zeroCrossingHz(const uint16_t* x, size_t n, double fs) {
    uint16_t lo = 65535, hi = 0;
    for (size_t i = 0; i < n; ++i) { if (x[i] < lo) lo = x[i]; if (x[i] > hi) hi = x[i]; }
    const double mid = 0.5 * ((double)lo + (double)hi);
    const double hyst = 0.25 * (0.5 * ((double)hi - (double)lo));
    int state = 0;              // 0 unknown, +1 above, -1 below
    size_t firstUp = 0, lastUp = 0, ups = 0;
    for (size_t i = 0; i < n; ++i) {
        double v = (double)x[i] - mid;
        if (state <= 0 && v > hyst) {
            if (state == -1) { if (ups == 0) firstUp = i; lastUp = i; ++ups; }
            state = 1;
        } else if (state >= 0 && v < -hyst) {
            state = -1;
        }
    }
    if (ups < 2) return 0.0;
    return (double)(ups - 1) * fs / (double)(lastUp - firstUp);
}

// Brent's minimisation of SS(f) on [lo, hi] starting from mid. Returns the
// minimiser; sets iterations and converged.
inline double brentMinimize(const uint16_t* x, size_t n, double fs, double offset,
                            double lo, double hi, double mid, double tolHz,
                            int maxIter, int& iterations, bool& converged) {
    const double cgold = 0.3819660112501051;
    double a = lo, b = hi;
    double v = mid, w = v, xm = v;
    double fx = residualSS(x, n, 2.0 * M_PI * xm / fs, offset), fv = fx, fw = fx;
    double d = 0.0, e = 0.0;
    iterations = 1;
    converged = false;
    for (int it = 0; it < maxIter; ++it) {
        double m = 0.5 * (a + b);
        double tol1 = tolHz + 1e-12 * fabs(xm);
        double tol2 = 2.0 * tol1;
        if (fabs(xm - m) <= tol2 - 0.5 * (b - a)) { converged = true; break; }
        if (fabs(e) > tol1) {
            double r = (xm - w) * (fx - fv);
            double q = (xm - v) * (fx - fw);
            double p = (xm - v) * q - (xm - w) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) p = -p;
            q = fabs(q);
            double etemp = e;
            e = d;
            if (fabs(p) >= fabs(0.5 * q * etemp) || p <= q * (a - xm) || p >= q * (b - xm)) {
                e = (xm >= m) ? a - xm : b - xm;
                d = cgold * e;
            } else {
                d = p / q;
                double u = xm + d;
                if (u - a < tol2 || b - u < tol2) d = (m - xm >= 0.0) ? tol1 : -tol1;
            }
        } else {
            e = (xm >= m) ? a - xm : b - xm;
            d = cgold * e;
        }
        double u = (fabs(d) >= tol1) ? xm + d : xm + ((d >= 0.0) ? tol1 : -tol1);
        double fu = residualSS(x, n, 2.0 * M_PI * u / fs, offset);
        ++iterations;
        if (fu <= fx) {
            if (u >= xm) a = xm; else b = xm;
            v = w; fv = fw; w = xm; fw = fx; xm = u; fx = fu;
        } else {
            if (u < xm) a = u; else b = u;
            if (fu <= fw || w == xm) { v = w; fv = fw; w = u; fw = fu; }
            else if (fu <= fv || v == xm || v == w) { v = u; fv = fu; }
        }
    }
    // The bracket edges are not solutions: a minimum pinned there means the
    // seed was wrong (hint off by more than a bin, or no tone).
    if (xm - lo < 2.0 * tolHz || hi - xm < 2.0 * tolHz) converged = false;
    return xm;
}

} // namespace detail

// The agreement bound for a transform of nfft points. A windowed noise
// estimate scatters as 1/sqrt(nfft), so the bound that suits 8192 points
// rejects good records from a shorter transform (white noise alone: 6 % of
// records at 2048 points against 0.5 dB). Options::agreeToleranceDb is the
// bound at 8192 points and above; below that it widens by sqrt(8192/nfft).
inline double agreeToleranceFor(const Options& opt, size_t nfft) {
    if (nfft == 0 || nfft >= 8192) return opt.agreeToleranceDb;
    return opt.agreeToleranceDb * sqrt(8192.0 / (double)nfft);
}

// Fit a sine to x[0..n) at sample rate fs, seeded at seedHz. The bracket is
// +/- bracketBins * fs/n around the seed. Fills `fit` and returns true if
// the normal equations were solvable. spanN (0 = n) is the number of
// leading samples a shorter transform covers: spanSinadDb is the residual
// of this fit over those samples alone, so the agreement check compares
// the two estimators on the same data.
inline bool fitSine(const uint16_t* x, size_t n, double fs, double seedHz,
                    double bracketBins, int maxIterations, FitResult& fit, size_t spanN = 0) {
    memset(&fit, 0, sizeof(fit));
    if (!x || n < 8 || fs <= 0.0 || seedHz <= 0.0) return false;
    const double binHz = fs / (double)n;
    double lo = seedHz - bracketBins * binHz;
    double hi = seedHz + bracketBins * binHz;
    if (lo < 0.25 * binHz) lo = 0.25 * binHz;
    if (hi > 0.5 * fs - 0.25 * binHz) hi = 0.5 * fs - 0.25 * binHz;
    if (hi <= lo) return false;

    // Offset keeps sum(x^2) small relative to the residual; the fit is
    // invariant to it (c absorbs it).
    uint16_t lo16 = 65535, hi16 = 0;
    for (size_t i = 0; i < n; ++i) { if (x[i] < lo16) lo16 = x[i]; if (x[i] > hi16) hi16 = x[i]; }
    const double offset = 0.5 * ((double)lo16 + (double)hi16);

    const double tolHz = 1e-6 * binHz;
    int iters = 0; bool conv = false;
    double fBest = detail::brentMinimize(x, n, fs, offset, lo, hi, seedHz, tolHz,
                                         maxIterations, iters, conv);

    // Final exact pass at fBest: parameters over the full record, then the
    // residual sample by sample (also feeds the integrity screens).
    const double w = 2.0 * M_PI * fBest / fs;
    detail::FitSums s;
    detail::accumulate(x, n, w, offset, s);
    double a, b, c;
    if (!detail::solve(s, a, b, c)) return false;

    // Head-quarter parameters for the discontinuity screen: refine the
    // frequency AND the amplitude/phase/offset on the first quarter alone,
    // then evaluate that model over the whole record. A dropped or
    // duplicated sample later in the record shows up as a growing phase
    // error against the uncontaminated head model. (Refining f on the whole
    // record first lets it absorb half of such a step, which hides it.)
    const size_t q = n / 4;
    double wh = w;
    double ah = 0.0, bh = 0.0, ch = 0.0;
    bool headOk = q >= 64;
    if (headOk) {
        const double qBin = fs / (double)q;
        double hlo = fBest - qBin, hhi = fBest + qBin;
        if (hlo < 0.25 * qBin) hlo = 0.25 * qBin;
        if (hhi > 0.5 * fs - 0.25 * qBin) hhi = 0.5 * fs - 0.25 * qBin;
        int hi2 = 0; bool hc = false;
        double fHead = detail::brentMinimize(x, q, fs, offset, hlo, hhi, fBest,
                                             1e-6 * qBin, maxIterations, hi2, hc);
        wh = 2.0 * M_PI * fHead / fs;
        detail::FitSums sh;
        detail::accumulate(x, q, wh, offset, sh);
        headOk = detail::solve(sh, ah, bh, ch);
    }

    if (spanN == 0 || spanN > n) spanN = n;
    double ss = 0.0, ssSpan = 0.0, ssHead = 0.0, ssTail = 0.0, emax = 0.0;
    const double dr = cos(w), di = sin(w);
    const double hr = cos(wh), hiw = sin(wh);
    double cr = 1.0, ci = 0.0, hcr = 1.0, hci = 0.0;
    for (size_t i = 0; i < n; ++i) {
        if ((i & 255) == 0) {
            cr = cos(w * (double)i);   ci = sin(w * (double)i);
            hcr = cos(wh * (double)i); hci = sin(wh * (double)i);
        }
        const double xv = (double)x[i] - offset;
        const double e = xv - (a * cr + b * ci + c);
        ss += e * e;
        if (i < spanN) ssSpan += e * e;
        if (headOk) {
            const double eh = xv - (ah * hcr + bh * hci + ch);
            const double ae = fabs(eh);
            if (ae > emax) emax = ae;
            if (i < q) ssHead += eh * eh;
            else if (i >= n - q) ssTail += eh * eh;
        }
        double ncr = cr * dr - ci * di;
        ci = cr * di + ci * dr;
        cr = ncr;
        double nhcr = hcr * hr - hci * hiw;
        hci = hcr * hiw + hci * hr;
        hcr = nhcr;
    }

    fit.fHz = fBest;
    fit.ampCodes = sqrt(a * a + b * b);
    fit.dcCodes = c + offset;
    fit.phaseRad = atan2(-b, a);
    fit.cycles = fBest * (double)n / fs;
    fit.S = 0.5 * (a * a + b * b);
    fit.R = ss / (double)n;
    fit.sinadDb = detail::db10(fit.S / fit.R);
    fit.spanSinadDb = detail::db10(fit.S / (ssSpan / (double)spanN));
    fit.residRmsCodes = sqrt(fit.R);
    fit.residMaxCodes = emax;
    fit.headRmsCodes = q ? sqrt(ssHead / (double)q) : 0.0;
    fit.tailRmsCodes = q ? sqrt(ssTail / (double)q) : 0.0;
    fit.iterations = iters;
    fit.converged = conv;
    return true;
}

// ---------------------------------------------------------------------------
// Spectrum estimator (windowed FFT, MT-003 bookkeeping)
// ---------------------------------------------------------------------------

// The estimator runs in two steps so that the same MT-003 bookkeeping can
// serve one record's spectrum and the mean of several:
//   computeBinPowers()  window, transform, normalise -> bin powers
//   spectrumMetrics()   fundamental, masks, harmonics, spur, SINAD ...
// analyzeSpectrum() is the two in sequence.

// What computeBinPowers() learned about the transform it ran; spectrumMetrics()
// needs it to size the lobes and PsdAverage keeps a copy.
struct BinPowerInfo {
    size_t nfft;          // samples transformed (power of two <= n)
    double binHz;         // fs / nfft
    double coherentGain;  // sum(w) / nfft
    double enbwBins;      // nfft * sum(w^2) / sum(w)^2
    WindowType window;
};

// Windowed bin powers of x[0..n). `dcCodes` is subtracted before windowing
// (use the fitted DC so the DC lobe cannot swallow a low tone). `work` must
// hold workspaceDoubles(n) doubles; on return the first nfft/2+1 entries of
// work + nfft hold the bin powers in code^2: a sinusoid's lobe sums to
// A^2/2, noise bins sum to the noise power in that band. Returns false if
// the workspace or record is unusable.
inline bool computeBinPowers(const uint16_t* x, size_t n, const RecordMeta& meta,
                             const Options& opt, double dcCodes, double* work,
                             BinPowerInfo& info) {
    memset(&info, 0, sizeof(info));
    const size_t nfft = fftLengthFor(n, opt.maxFftLength);
    if (!x || !work || nfft == 0 || meta.fsHz <= 0.0) return false;
    CosineSeries win;
    if (!cosineSeries(opt.window, win)) return false;   // not a cosine-series window
    const double fs = meta.fsHz;
    const size_t half = nfft / 2;
    double* power = work + nfft;
    // Window and pack: real samples 2k, 2k+1 become complex pair k.
    double sumW = 0.0, sumW2 = 0.0;
    const double theta = 2.0 * M_PI / (double)nfft;
    const double dr = cos(theta), di = sin(theta);
    double cr = 1.0, ci = 0.0;
    for (size_t i = 0; i < nfft; ++i) {
        if ((i & 255) == 0) { cr = cos(theta * (double)i); ci = sin(theta * (double)i); }
        const double wv = windowValue(win, cr);
        sumW += wv; sumW2 += wv * wv;
        work[i] = wv * ((double)x[i] - dcCodes);
        double ncr = cr * dr - ci * di;
        ci = cr * di + ci * dr;
        cr = ncr;
    }
    detail::realFFTPower(work, nfft, power);

    // Bin power in code^2: p[k] = d |X[k]|^2 / (nfft sum(w^2)), d = 2 except DC/Nyquist.
    // Summed over a lobe this gives the sinusoid's power A^2/2; summed over
    // noise bins it gives the noise power in that band (Parseval with the
    // ENBW normalisation built in).
    const double norm = 1.0 / ((double)nfft * sumW2);
    for (size_t k = 0; k <= half; ++k) {
        const double d = (k == 0 || k == half) ? 1.0 : 2.0;
        power[k] = d * power[k] * norm;
    }

    info.nfft = nfft;
    info.binHz = fs / (double)nfft;
    info.coherentGain = sumW / (double)nfft;
    info.enbwBins = (double)nfft * sumW2 / (sumW * sumW);
    info.window = opt.window;
    return true;
}

// MT-003 bookkeeping on a bin-power array: one record's powers from
// computeBinPowers(), or the mean of several (see SpectrumAverage.h).
// `power` holds info.nfft/2 + 1 entries in code^2 and is not modified.
// `scratch` must hold info.nfft doubles (the transform half of the
// workspace is free once the powers exist). meta.fHintHz steers the
// fundamental search as in analyzeSpectrum(). Returns false when no
// fundamental can be found.
inline bool spectrumMetrics(const double* power, const BinPowerInfo& info,
                            const RecordMeta& meta, const Options& opt,
                            double* scratch, SpectrumResult& spec) {
    memset(&spec, 0, sizeof(spec));
    if (!power || !scratch || info.nfft == 0 || meta.fsHz <= 0.0) return false;
    CosineSeries win;
    if (!cosineSeries(info.window, win)) return false;
    const double fs = meta.fsHz;
    const size_t half = info.nfft / 2;
    spec.nfft = info.nfft;
    spec.binHz = info.binHz;
    spec.coherentGain = info.coherentGain;
    spec.enbwBins = info.enbwBins;
    const int L = win.lobeBins;

    // Fundamental: nearest the hint, else the largest bin above DC. The
    // search includes the DC lobe: the caller has removed the offset, and a
    // tone only a few bins up (1 kHz in a 2048-point transform at 800 kSPS
    // is bin 2.6) would otherwise lose to the largest spur.
    int peak = -1;
    if (meta.fHintHz > 0.0) {
        int kh = (int)floor(meta.fHintHz / spec.binHz + 0.5);
        int k0 = std::max(1, kh - L), k1 = std::min((int)half - 1, kh + L);
        double best = -1.0;
        for (int k = k0; k <= k1; ++k) if (power[k] > best) { best = power[k]; peak = k; }
    }
    if (peak < 0) {
        double best = -1.0;
        for (int k = 1; k < (int)half; ++k) if (power[k] > best) { best = power[k]; peak = k; }
    }
    if (peak < 1) return false;
    spec.peakBin = peak;
    // Parabolic interpolation in log magnitude for the reported frequency.
    {
        double pa = log(power[peak - 1] + 1e-300), pb = log(power[peak] + 1e-300),
               pc = log(power[peak + 1] + 1e-300);
        double den = pa - 2.0 * pb + pc;
        double delta = fabs(den) > 1e-12 ? 0.5 * (pa - pc) / den : 0.0;
        spec.fHz = ((double)peak + delta) * spec.binHz;
    }

    // Masks. The fundamental lobe wins where it overlaps the DC lobe.
    const int fLo = std::max(0, peak - L), fHi = std::min((int)half, peak + L);
    const int dcHi = L;

    double S = 0.0, dcP = 0.0, total = 0.0;
    for (int k = 0; k <= (int)half; ++k) {
        total += power[k];
        if (k >= fLo && k <= fHi) S += power[k];
        else if (k <= dcHi) dcP += power[k];
    }
    spec.S = S;
    spec.dcPower = dcP;
    double R = total - S - dcP;

    // Harmonics H2..Hmax, folded into 0..fs/2, each a +/-L lobe. A lobe that
    // touches DC, the fundamental or an earlier harmonic is excluded.
    const double f1 = spec.fHz;
    double harmonicPower = 0.0;
    const int hmax = std::min(opt.maxHarmonic, 7);
    for (int h = 2; h <= hmax; ++h) {
        double fh = fmod((double)h * f1 + 0.5 * fs, fs);
        if (fh < 0.0) fh += fs;
        fh = fabs(fh - 0.5 * fs);
        int kh = (int)floor(fh / spec.binHz + 0.5);
        int hLo = std::max(0, kh - L), hHi = std::min((int)half, kh + L);
        HarmonicLine& hl = spec.harmonic[h];
        hl.hz = fh;
        hl.valid = !(hLo <= dcHi) && !(hHi >= fLo && hLo <= fHi);
        for (int g = 2; g < h && hl.valid; ++g) {
            if (!spec.harmonic[g].valid) continue;
            int gk = (int)floor(spec.harmonic[g].hz / spec.binHz + 0.5);
            if (hHi >= gk - L && hLo <= gk + L) hl.valid = false;
        }
        double p = 0.0;
        for (int k = hLo; k <= hHi; ++k) p += power[k];
        hl.power = p;
        hl.dbc = detail::db10(p / S);
        if (hl.valid) harmonicPower += p;
    }
    spec.harmonicPower = harmonicPower;

    // Largest spur: the biggest single bin outside the DC and fundamental
    // masks, with its +/-L lobe summed (clipped to the unmasked region).
    int spurBin = -1; double spurBinP = -1.0;
    for (int k = dcHi + 1; k < (int)half; ++k) {
        if (k >= fLo && k <= fHi) continue;
        if (power[k] > spurBinP) { spurBinP = power[k]; spurBin = k; }
    }
    double spurP = 0.0;
    if (spurBin >= 0) {
        int sLo = std::max(dcHi + 1, spurBin - L), sHi = std::min((int)half, spurBin + L);
        for (int k = sLo; k <= sHi; ++k) {
            if (k >= fLo && k <= fHi) continue;
            spurP += power[k];
        }
        spec.spurHz = (double)spurBin * spec.binHz;
    }
    spec.spurPower = spurP;
    spec.spurDbc = detail::db10(spurP / S);

    // Noise median over the unmasked bins, on a compacted copy in the
    // transform half of the workspace (free once the powers exist), so the
    // bin-power array stays in order for the caller.
    {
        double* tmp = scratch;             // nfft doubles, transform no longer needed
        size_t cnt = 0;
        for (int k = dcHi + 1; k < (int)half; ++k) {
            if (k >= fLo && k <= fHi) continue;
            tmp[cnt++] = power[k];
        }
        double med = 0.0;
        if (cnt) {
            std::nth_element(tmp, tmp + cnt / 2, tmp + cnt);
            med = tmp[cnt / 2];
        }
        spec.noiseMedianDbc = detail::db10(med / S);
        spec.spurResolved = (spurBin >= 0) && (spurBinP > med * pow(10.0, opt.spurResolvedDb / 10.0));
    }

    spec.R = R;
    spec.sinadDb = detail::db10(S / R);
    spec.snrDb = detail::db10(S / (R - harmonicPower));
    spec.thdDbc = detail::db10(harmonicPower / S);
    spec.sfdrDbc = detail::db10(S / spurP);
    const double F = ldexp(1.0, meta.bits - 1);
    spec.sfdrDbfs = detail::db10((0.5 * F * F) / spurP);
    return true;
}


// Run the windowed-FFT estimator on x[0..n): computeBinPowers() followed by
// spectrumMetrics(). The bin powers stay at work + nfft on return, which is
// what the averaging in SpectrumAverage.h reads.
inline bool analyzeSpectrum(const uint16_t* x, size_t n, const RecordMeta& meta,
                            const Options& opt, double dcCodes, double* work,
                            SpectrumResult& spec) {
    memset(&spec, 0, sizeof(spec));
    BinPowerInfo info;
    if (!computeBinPowers(x, n, meta, opt, dcCodes, work, info)) return false;
    return spectrumMetrics(work + info.nfft, info, meta, opt, work, spec);
}

// ---------------------------------------------------------------------------
// Whole-record analysis
// ---------------------------------------------------------------------------

// Analyze one record. `work` may be null (spectrum skipped, fit only; the
// tone is then found from zero crossings when meta.fHintHz == 0).
inline Result analyze(const uint16_t* x, size_t n, const RecordMeta& meta,
                      const Options& opt, double* work) {
    Result r;
    memset(&r, 0, sizeof(r));
    r.meta = meta;
    r.n = n;
    r.flags = 0;

    if (!x || n == 0 || meta.fsHz <= 0.0 || meta.bits < 2 || meta.bits > 16) {
        r.flags |= FLAG_BAD_ARGS;
        r.valid = false;
        return r;
    }
    const double F = ldexp(1.0, meta.bits - 1);
    const uint16_t maxCode = (uint16_t)((1u << meta.bits) - 1u);
    r.fullScalePeakCodes = F;
    r.idealSnrDb = 6.02 * meta.bits + 1.76;

    uint16_t lo = 65535, hi = 0;
    for (size_t i = 0; i < n; ++i) { if (x[i] < lo) lo = x[i]; if (x[i] > hi) hi = x[i]; }
    r.minCode = lo; r.maxCode = hi;
    if ((int)lo <= opt.clipMarginCodes || (int)hi >= (int)maxCode - opt.clipMarginCodes)
        r.flags |= FLAG_CLIPPED;
    if (n < opt.minSamples) r.flags |= FLAG_TOO_SHORT;

    // Seed for the fit: the hint, else the FFT peak, else zero crossings.
    double seed = meta.fHintHz;
    double bracket = 1.0;
    // A first spectrum pass with the record mean removed gives the seed and
    // a DC estimate; it is redone with the fitted DC once the fit exists.
    double mean = 0.0;
    for (size_t i = 0; i < n; ++i) mean += (double)x[i];
    mean /= (double)n;

    if (seed <= 0.0 && work) {
        SpectrumResult s0;
        if (analyzeSpectrum(x, n, meta, opt, mean, work, s0)) seed = s0.fHz;
        bracket = 1.5;
    }
    if (seed <= 0.0) {
        seed = detail::zeroCrossingHz(x, n, meta.fsHz);
        bracket = 2.5;
    }

    bool fitOk = seed > 0.0 && fitSine(x, n, meta.fsHz, seed, bracket, opt.maxIterations, r.fit,
                                       fftLengthFor(n, opt.maxFftLength));
    if (!fitOk) {
        r.flags |= FLAG_NOT_CONVERGED;
    } else {
        if (!r.fit.converged) r.flags |= FLAG_NOT_CONVERGED;
        if (r.fit.cycles < opt.minCycles) r.flags |= FLAG_FEW_CYCLES;
        if (r.fit.ampCodes < opt.minAmplitudeCodes) r.flags |= FLAG_LOW_AMPLITUDE;
        const double rms = r.fit.headRmsCodes > 0.0 ? r.fit.headRmsCodes : r.fit.residRmsCodes;
        const double spikeLimit = std::max(opt.spikeSigma * rms, opt.spikeFrac * r.fit.ampCodes);
        if (r.fit.residMaxCodes > spikeLimit) r.flags |= FLAG_DISCONTINUITY;
        if (r.fit.headRmsCodes > 0.0 && r.fit.tailRmsCodes > opt.tailGrowth * r.fit.headRmsCodes)
            r.flags |= FLAG_RESIDUAL_GROWTH;
    }

    // Spectrum with the fitted DC removed.
    r.specValid = false;
    if (work) {
        const double dc = fitOk ? r.fit.dcCodes : mean;
        RecordMeta m2 = meta;
        if (fitOk) m2.fHintHz = r.fit.fHz;
        if (analyzeSpectrum(x, n, m2, opt, dc, work, r.spec)) {
            r.specValid = true;
            if (r.spec.nfft != n) r.flags |= FLAG_SPECTRUM_TRUNCATED;
            CosineSeries winL;
            cosineSeries(opt.window, winL);
            const int L = winL.lobeBins;
            if (r.spec.peakBin - L <= L) r.flags |= FLAG_LOBE_OVERLAP;
            for (int h = 2; h <= std::min(opt.maxHarmonic, 7); ++h)
                if (!r.spec.harmonic[h].valid) { r.flags |= FLAG_HARMONIC_COLLISION; break; }
            if (!r.spec.spurResolved) r.flags |= FLAG_SPUR_UNRESOLVED;
            if (fitOk && fabs(r.spec.fHz - r.fit.fHz) > r.spec.binHz) r.flags |= FLAG_HINT_MISMATCH;
        }
    } else {
        r.flags |= FLAG_NO_SPECTRUM;
    }

    // Headline numbers.
    // The fit's frequency and amplitude are the better estimates whenever
    // the fit converged (1e-6 bin resolution, no window bias); the spectrum
    // supplies SINAD and everything that needs a spectrum.
    r.primaryIsSpectrum = r.specValid;
    if (r.specValid) {
        r.fHz = fitOk ? r.fit.fHz : r.spec.fHz;
        r.sinadDb = r.spec.sinadDb;
        r.ampCodes = fitOk ? r.fit.ampCodes : sqrt(2.0 * r.spec.S);
        r.processGainDb = detail::db10((double)r.spec.nfft / 2.0);
    } else if (fitOk) {
        r.fHz = r.fit.fHz;
        r.sinadDb = r.fit.sinadDb;
        r.ampCodes = r.fit.ampCodes;
    }
    r.idealFftFloorDb = r.idealSnrDb + r.processGainDb;
    if (r.specValid && fitOk) {
        r.fitFftDisagreeDb = fabs(r.spec.sinadDb - r.fit.spanSinadDb);
        if (r.fitFftDisagreeDb > agreeToleranceFor(opt, r.spec.nfft)) r.flags |= FLAG_ESTIMATORS_DISAGREE;
    }
    if (r.ampCodes > 0.0) {
        r.inputDbfs = 20.0 * log10(r.ampCodes / F);
        r.ampVolts = meta.scaleUvPerLsb > 0.0 ? r.ampCodes * meta.scaleUvPerLsb * 1e-6 : 0.0;
        r.enobAtInput = (r.sinadDb - 1.76) / 6.02;
        r.enobFullScale = (r.sinadDb - 1.76 + 20.0 * log10(F / r.ampCodes)) / 6.02;
    }
    r.valid = (r.flags & FLAGS_REJECTING) == 0;
    return r;
}

// ---------------------------------------------------------------------------
// Aggregation over K records
// ---------------------------------------------------------------------------

struct Stat {
    double sum, min, max; unsigned count;
    Stat() : sum(0.0), min(0.0), max(0.0), count(0) {}
    void add(double v) {
        if (!count || v < min) min = v;
        if (!count || v > max) max = v;
        sum += v; ++count;
    }
    double mean() const { return count ? sum / (double)count : 0.0; }
    double spread() const { return max - min; }
};

struct Aggregate {
    unsigned used, rejected;
    uint32_t rejectFlagsSeen;   // union of rejecting flags over rejected records
    Stat sinadDb, enobFullScale, enobAtInput, sfdrDbc, snrDb, thdDbc, fHz, inputDbfs, fitFftDisagreeDb;

    Aggregate() : used(0), rejected(0), rejectFlagsSeen(0) {}
    void reset() { *this = Aggregate(); }
    void add(const Result& r) {
        if (!r.valid) { ++rejected; rejectFlagsSeen |= (r.flags & FLAGS_REJECTING); return; }
        ++used;
        sinadDb.add(r.sinadDb); enobFullScale.add(r.enobFullScale); enobAtInput.add(r.enobAtInput);
        fHz.add(r.fHz); inputDbfs.add(r.inputDbfs); fitFftDisagreeDb.add(r.fitFftDisagreeDb);
        if (r.specValid) { sfdrDbc.add(r.spec.sfdrDbc); snrDb.add(r.spec.snrDb); thdDbc.add(r.spec.thdDbc); }
    }
};

// ---------------------------------------------------------------------------
// Text formatting (plain snprintf; no stream dependency)
// ---------------------------------------------------------------------------

// Short flag list such as "CLIP|DISC" or "-" when none are set.
inline size_t formatFlags(uint32_t flags, char* out, size_t cap) {
    static const struct { uint32_t bit; const char* name; } names[] = {
        {FLAG_CLIPPED, "CLIP"}, {FLAG_TOO_SHORT, "SHORT"}, {FLAG_FEW_CYCLES, "CYCLES"},
        {FLAG_NOT_CONVERGED, "NOCONV"}, {FLAG_DISCONTINUITY, "DISC"},
        {FLAG_RESIDUAL_GROWTH, "GROWTH"}, {FLAG_LOW_AMPLITUDE, "LOWAMP"},
        {FLAG_BAD_ARGS, "ARGS"}, {FLAG_HINT_MISMATCH, "FMISMATCH"},
        {FLAG_ESTIMATORS_DISAGREE, "DISAGREE"},
        {FLAG_NO_SPECTRUM, "nofft"}, {FLAG_SPECTRUM_TRUNCATED, "trunc"},
        {FLAG_LOBE_OVERLAP, "dclobe"}, {FLAG_HARMONIC_COLLISION, "hcoll"},
        {FLAG_SPUR_UNRESOLVED, "spur?"},
    };
    size_t len = 0;
    if (cap == 0) return 0;
    out[0] = 0;
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
        if (!(flags & names[i].bit)) continue;
        int w = snprintf(out + len, cap - len, "%s%s", len ? "|" : "", names[i].name);
        if (w < 0) break;
        len += (size_t)w;
        if (len >= cap) { len = cap - 1; break; }
    }
    if (len == 0) { snprintf(out, cap, "-"); len = 1; }
    return len;
}

// One-line record summary, key=value, for serial logs and host diffs.
inline int formatRecordLine(const Result& r, unsigned index, char* out, size_t cap) {
    char fl[96];
    formatFlags(r.flags, fl, sizeof(fl));
    return snprintf(out, cap,
        "k=%u,f=%.3f,amp=%.2f,dbfs=%.2f,sinad=%.2f,enob_in=%.2f,enob_fs=%.2f,"
        "fit_sinad=%.2f,fft_sinad=%.2f,snr=%.2f,thd=%.2f,sfdr=%.2f,sfdr_dbfs=%.2f,"
        "spur_hz=%.1f,resid_rms=%.2f,resid_max=%.1f,iters=%d,flags=%s,valid=%d",
        index, r.fHz, r.ampCodes, r.inputDbfs, r.sinadDb, r.enobAtInput, r.enobFullScale,
        r.fit.sinadDb, r.specValid ? r.spec.sinadDb : 0.0,
        r.specValid ? r.spec.snrDb : 0.0, r.specValid ? r.spec.thdDbc : 0.0,
        r.specValid ? r.spec.sfdrDbc : 0.0, r.specValid ? r.spec.sfdrDbfs : 0.0,
        r.specValid ? r.spec.spurHz : 0.0, r.fit.residRmsCodes, r.fit.residMaxCodes,
        r.fit.iterations, fl, r.valid ? 1 : 0);
}

// Sink for the multi-line report: called once per line, without newline.
typedef void (*LineSink)(void* ctx, const char* line);

// MT-003 table for one record: every number names its definition, band and
// amplitude convention, per the tutorial rule.
inline void reportRecord(const Result& r, const Options& opt, LineSink sink, void* ctx) {
    char b[160];
    char fl[96];
    formatFlags(r.flags, fl, sizeof(fl));
    snprintf(b, sizeof b, "record: n=%u fs=%.3f Hz bits=%u F=%.0f codes window=%s flags=%s valid=%d",
             (unsigned)r.n, r.meta.fsHz, r.meta.bits, r.fullScalePeakCodes,
             windowName(opt.window), fl, r.valid ? 1 : 0);
    sink(ctx, b);
    snprintf(b, sizeof b, "tone: f=%.4f Hz  A=%.2f codes (%.3f V)  %.2f dBFS  DC=%.2f codes  cycles=%.2f",
             r.fHz, r.ampCodes, r.ampVolts, r.inputDbfs, r.fit.dcCodes, r.fit.cycles);
    sink(ctx, b);
    snprintf(b, sizeof b, "SINAD   %.2f dB   (%s; DC..fs/2, DC excluded; MT-003 THD+N over the Nyquist band)",
             r.sinadDb, r.primaryIsSpectrum ? "windowed FFT" : "sine fit");
    sink(ctx, b);
    snprintf(b, sizeof b, "ENOB    %.2f bits at input, %.2f bits normalized to full scale (MT-003 eq. 2)",
             r.enobAtInput, r.enobFullScale);
    sink(ctx, b);
    if (r.specValid) {
        snprintf(b, sizeof b, "SNR     %.2f dB   (H2..H%d removed)", r.spec.snrDb, opt.maxHarmonic);
        sink(ctx, b);
        snprintf(b, sizeof b, "THD     %.2f dBc  (H2..H%d rss over the fundamental, negative = below carrier)",
                 r.spec.thdDbc, opt.maxHarmonic);
        sink(ctx, b);
        snprintf(b, sizeof b, "SFDR    %.2f dBc, %.2f dBFS  (largest line at %.1f Hz%s)",
                 r.spec.sfdrDbc, r.spec.sfdrDbfs, r.spec.spurHz,
                 r.spec.spurResolved ? "" : ", not resolved above the noise median");
        sink(ctx, b);
        for (int h = 2; h <= std::min(opt.maxHarmonic, 7); ++h) {
            const HarmonicLine& hl = r.spec.harmonic[h];
            snprintf(b, sizeof b, "H%d      %.2f dBc at %.1f Hz%s", h, hl.dbc, hl.hz,
                     hl.valid ? "" : " (excluded: lobe collision)");
            sink(ctx, b);
        }
        CosineSeries winR;
        cosineSeries(opt.window, winR);
        snprintf(b, sizeof b, "window: coherent gain %.5f, ENBW %.4f bins, lobe +/-%d bins, bin %.3f Hz, nfft=%u",
                 r.spec.coherentGain, r.spec.enbwBins, winR.lobeBins,
                 r.spec.binHz, (unsigned)r.spec.nfft);
        sink(ctx, b);
        snprintf(b, sizeof b, "check:  fit SINAD %.2f dB vs FFT SINAD %.2f dB, difference %.3f dB (tolerance %.2f)",
                 r.fit.spanSinadDb, r.spec.sinadDb, r.fitFftDisagreeDb, agreeToleranceFor(opt, r.spec.nfft));
        sink(ctx, b);
        snprintf(b, sizeof b, "ideal:  SNR %.2f dB for %u bits, FFT process gain %.2f dB, FFT floor %.2f dB below full scale",
                 r.idealSnrDb, r.meta.bits, r.processGainDb, r.idealFftFloorDb);
        sink(ctx, b);
    }
}

inline void reportAggregate(const Aggregate& a, LineSink sink, void* ctx) {
    char b[160];
    char fl[96];
    formatFlags(a.rejectFlagsSeen, fl, sizeof(fl));
    snprintf(b, sizeof b, "records: used=%u rejected=%u reject_flags=%s", a.used, a.rejected, fl);
    sink(ctx, b);
    if (!a.used) return;
    snprintf(b, sizeof b, "SINAD   mean %.2f dB  min %.2f  max %.2f  spread %.2f", a.sinadDb.mean(), a.sinadDb.min, a.sinadDb.max, a.sinadDb.spread());
    sink(ctx, b);
    snprintf(b, sizeof b, "ENOB    mean %.2f bits (FS)  %.2f bits (at input)  input %.2f dBFS", a.enobFullScale.mean(), a.enobAtInput.mean(), a.inputDbfs.mean());
    sink(ctx, b);
    if (a.sfdrDbc.count) {
        snprintf(b, sizeof b, "SFDR    mean %.2f dBc  min %.2f   SNR mean %.2f dB   THD mean %.2f dBc", a.sfdrDbc.mean(), a.sfdrDbc.min, a.snrDb.mean(), a.thdDbc.mean());
        sink(ctx, b);
    }
    snprintf(b, sizeof b, "f       mean %.4f Hz  spread %.4f Hz   fit/FFT disagreement max %.3f dB", a.fHz.mean(), a.fHz.spread(), a.fitFftDisagreeDb.max);
    sink(ctx, b);
}

} // namespace measure
} // namespace signalcore

#endif // _SIGNALCORE_MEASURE_SINEMETRICS_H_
