/*
    ModelControlTask.h

    This file declares the haptic model task.

    This task is responsible for computing the virtual behavior that the user
    should feel through the knob.
*/

void ModelControlTask(void* pvParameters){
     (void)pvParameters;

    // TODO: choose a starting update rate, for example 1 kHz
    // TickType_t last_wake = xTaskGetTickCount();
    // const TickType_t period = pdMS_TO_TICKS(1);

    for (;;)
    {
        // TODO: copy measured state + config out of shared state under mutex

        // TODO: compute the active model command
        // HapticCommand new_command{};
        // computeActiveModelCommand(...);

        // TODO: timestamp the command

        // TODO: write command back into shared state under mutex

        // TODO: regulate task timing
        // vTaskDelayUntil(&last_wake, period);
        taskYIELD();
    }
}
