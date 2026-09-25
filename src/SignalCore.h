/******************************************************************************
 * @file SignalCore.h
 * @brief SignalCore — main include
 *
 * Cross-platform DSP and signal generation library for Arduino-compatible
 * boards. Automatically enables CMSIS-DSP on Teensy 4.x and falls back to a
 * portable C++ implementation everywhere else by default. Other ARM boards
 * can opt in to the CMSIS path by defining SIGNALCORE_FORCE_CMSIS before
 * including this header when the selected core links CMSIS-DSP symbols.
 *
 * Supported platforms:
 *   - Teensy 4.x (automatic CMSIS-DSP acceleration)
 *   - Arduino Giga, STM32duino, Zephyr UNO Q (generic C++ by default)
 *   - ESP32, ESP32-S3 (generic C++)
 *   - RP2040, RP2350 (generic C++)
 *   - SAMD21/51 (generic C++)
 *   - AVR (generic C++, limited to small FFT sizes)
 *   - Any Arduino-compatible board with a C++11 compiler
 *
 * Modules:
 *   - Platform.h       — board detection, phase wrapping, math fallbacks
 *   - Windows.h        — window functions and coefficient generation
 *   - fft/RealFFT.h    — RealFFT<T>, the float or double real transform
 *                        every other class runs on (CMSIS or generic)
 *   - FFT.h            — template FFT class, runtime-sized DynamicFFT, and
 *                        spectral helpers
 *   - DDS.h            — DDS oscillator and SignalGenerator
 *   - Trigger.h        — rising/falling edge trigger with hysteresis
 *   - PeakHold.h       — per-bin peak-hold with exponential decay
 *   - MovingAverage.h  — ExponentialMovingAverage for vector smoothing
 *   - measure/SineMetrics.h — ADC sine-wave metrics (SINAD, ENOB, SNR, THD,
 *                        SFDR per MT-003) from a raw-code record, in
 *                        namespace signalcore::measure
 *
 * Everything lives in namespace signalcore. The conventional sketch pattern
 * is `using namespace signalcore;` after the include.
 *
 * Usage:
 *   #include <SignalCore.h>
 *   using namespace signalcore;
 *
 *   FFT<FFTSize::N1024> fft;
 *   SignalGenerator sig(SAMPLE_RATE);
 *
 *   This file is a part of the SignalCore project, a cross-platform digital
 *   signal processing toolkit released by VoidLoop. See www.voidloop.com for
 *   updates.
 *
 * Contributors:
 * Initial vision, feature definition, debugging and funding: Greg Kovacs
 * Architecture, Development, debugging and support: Zachariah Magee
 *
 * Thanks to our many collaborators and friends!
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

#ifndef _SIGNALCORE_H_
#define _SIGNALCORE_H_

#include "signalcore/Platform.h"
#include "signalcore/Windows.h"
#include "signalcore/fft/RealFFT.h"
#include "signalcore/FFT.h"
#include "signalcore/DDS.h"
#include "signalcore/Trigger.h"
#include "signalcore/PeakHold.h"
#include "signalcore/MovingAverage.h"
#include "measure/SineMetrics.h"

#endif // _SIGNALCORE_H_
