/*
    This file declares the fast motor-control task.

    This task is responsible for talking to the real motor subsystem at runtime:
      - update encoder state
      - run the FOC electrical control step
      - read measured motor/shaft values
      - publish measured values into SharedState
      - read the latest command from SharedState
      - apply that command to the motor

    This task is hardware-facing and timing-sensitive.
*/

void MotorControlTask(void* pvParameters){
    (void)pvParameters;

    // TODO: local variables for dt / previous velocity / acceleration estimation
    // float prev_velocity = 0.0f;
    // uint32_t last_us = micros();

    for (;;)
    {
        // TODO: step the hardware control path
        // updateHardwareControlStep();

        // TODO: read back measured values from Hardware.cpp
        // - angle
        // - wrapped angle
        // - velocity
        // - iq
        // - phase currents

        // TODO: estimate acceleration if you want it here

        // TODO: write measured values into shared state under mutex

        // TODO: read latest command + fault state from shared state under mutex

        // TODO:
        // if faulted -> stopMotor()
        // else if command.use_voltage_mode -> applyMotorVoltage(...)
        // else -> applyMotorCurrent(...)

        // TODO:
        // add a delay strategy if needed
        // for the motor task you may keep this very fast, or use vTaskDelayUntil
        taskYIELD();
    }
}
