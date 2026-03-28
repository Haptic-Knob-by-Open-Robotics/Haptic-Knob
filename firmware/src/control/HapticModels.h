#pragma once

#include "app/SharedState.h"

void computeResistorCommand(const MeasuredState &measured,
                            const RuntimeConfig &config,
                            HapticCommand &command);

void computeCapacitorCommand(const MeasuredState &measured,
                             const RuntimeConfig &config,
                             HapticCommand &command);

void computeInductorCommand(const MeasuredState &measured,
                            const RuntimeConfig &config,
                            HapticCommand &command);

void computeDiodeCommand(const MeasuredState &measured,
                         const RuntimeConfig &config,
                         HapticCommand &command);

void computeRLCCommand(const MeasuredState &measured,
                       const RuntimeConfig &config,
                       HapticCommand &command);

void computeActiveModelCommand(const MeasuredState &measured,
                               const RuntimeConfig &config,
                               HapticCommand &command);
