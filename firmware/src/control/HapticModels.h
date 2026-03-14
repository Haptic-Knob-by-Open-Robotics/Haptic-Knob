/*
    HapticModels.h

    This file declares the haptic model functions used by the firmware.

    These functions take the latest measured shaft state
    (angle / velocity / acceleration) plus configuration parameters
    and compute what the motor should do in order to render the
    desired virtual behavior.

    Supported modes include:
      - resistor
      - capacitor
      - inductor
      - diode

    The output of each model should be a command for the motor layer,
    such as:
      - desired q-axis current command
      - desired voltage command
      - whether the mode uses current control or voltage control

    This file contains only model/control math.
    It should not directly talk to hardware or SimpleFOC objects.
*/

void computeResistorCommand(const MeasuredState& measured,
                            const RuntimeConfig& config,
                            HapticCommand& command);

void computeCapacitorCommand(const MeasuredState& measured,
                             const RuntimeConfig& config,
                             HapticCommand& command);

void computeInductorCommand(const MeasuredState& measured,
                            const RuntimeConfig& config,
                            HapticCommand& command);

void computeDiodeCommand(const MeasuredState& measured,
                         const RuntimeConfig& config,
                         HapticCommand& command);

/*
    Convenience dispatcher:
    picks the correct model based on active_mode.
*/
void computeActiveModelCommand(const MeasuredState& measured,
                               const RuntimeConfig& config,
                               HapticCommand& command);
                               