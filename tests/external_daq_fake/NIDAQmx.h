// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cstdint>

using TaskHandle = void *;
using int32 = int32_t;
using uInt32 = uint32_t;
using uInt64 = uint64_t;
using uInt8 = uint8_t;
using float64 = double;

constexpr int DAQmx_Val_NRSE = 1;
constexpr int DAQmx_Val_Diff = 7;
constexpr int DAQmx_Val_Volts = 2;
constexpr int DAQmx_Val_Rising = 3;
constexpr int DAQmx_Val_ContSamps = 4;
constexpr int DAQmx_Val_ChanPerLine = 5;
constexpr int DAQmx_Val_GroupByChannel = 6;

int32 DAQmxGetExtendedErrorInfo(char *, uInt32);
int32 DAQmxCreateTask(const char *, TaskHandle *);
int32 DAQmxCreateAIVoltageChan(TaskHandle, const char *, const char *, int32,
                             float64, float64, int32, const char *);
int32 DAQmxCreateDIChan(TaskHandle, const char *, const char *, int32);
int32 DAQmxCfgSampClkTiming(TaskHandle, const char *, float64, int32, int32,
                          uInt64);
int32 DAQmxGetSampClkRate(TaskHandle, float64 *);
int32 DAQmxStartTask(TaskHandle);
int32 DAQmxStopTask(TaskHandle);
int32 DAQmxClearTask(TaskHandle);
int32 DAQmxReadAnalogF64(TaskHandle, int32, float64, int32, float64 *, uInt32,
                         int32 *, void *);
int32 DAQmxReadDigitalLines(TaskHandle, int32, float64, int32, uInt8 *, uInt32,
                            int32 *, int32 *, void *);
