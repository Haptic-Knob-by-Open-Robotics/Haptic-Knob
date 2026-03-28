// #include <Arduino.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "app/Hardware.h"
// #include "app/SharedState.h"
// #include "control/HapticModels.h"

// // ========== TASK TIMING SETTINGS ===============

// // How often this task wakes up (ms)
// static constexpr uint32_t CONTROL_TASK_PERIOD_MS = 1;

// // How often we recompute the haptic model relative to the fast loop to get the desired setpoint
// // this allows the fast motor control inner loop to run more than the model control outerloop which
// // sets the setponit.
// // Ex: CONTROL_TASK_PERIOD_MS = 1ms -> fast loop is 1000hz and outerloop is 200hz
// // Means that the motor/FOC side runs every cycle and model computation runs eveyr 5 cycles
// static constexpr uint32_t OUTER_LOOP_DIVIDER = 5;

// // Task creation settings (we can change these as necessary)
// static constexpr uint32_t CONTROL_TASK_PERIOD_MS = 1;
// static constexpr uint32_t CONTROL_TASK_PRIORITY = 5;

// // FreeRTOS handle for this task
// static TaskHandle_t g_control_task_handle = nullptr;

// // ======== SHARED STATE HELPERS =======
// // The control task is time sensitive so we only lock the mutex briefly and copy whatever we need
// // and release the lock right away this is good practice that leads to better performace

// // Local struct used to read the systme flags in one shot
// struct ControlFlags{
//     bool control_enabled = false;
//     bool fault_latched = false;
// }

// // Read a local copy of the current runtime configuration
// static RuntimeCOnfig readRuntimeConfig(){

//     RuntimeConfig config{};

//     // get the lock
//     if (g_state_mutex != nullptr && xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE){

//         // Get the copy
//         config = g_runtim_config;

//         // Release the lock
//         xSemaphoreGive(g_state_mutex);
//     }
//     return config;
// }

// // Read the current system-level flags that determine whether the control task is acutally allowed
// // to driver the motor
// static ControlFlags readControlFlags(){

//     ControlFlags flags{};

//     if (g_state_mutex != nullptr && xSemaphoreTake(g_state_mutex, portMax_DELAY) == pdTRUE){
//         flags = ControlFlags;
//         xSemaphoreGive(g_state_mutex);
//     }

//     return flags;
// }

// // Publish the latest measured state so telemtry can read it
// static void MeasuredState writeMeasuredState(const MeasuredState &measured){

//     if (g_state_mutex != nullptr && xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE){
//         g_measured_state = measured;
//         xSemaphoreGIve(g_state_mutex);
//     }
// }

// // Publish the latest haptic command so telemetry/debug tasks can see
// // what the model layer most recently requested.
// static void writeHapticCommand(const HapticCommand& command)
// {
//     if (g_state_mutex != nullptr &&
//         xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE)
//     {
//         g_haptic_command = command;
//         xSemaphoreGive(g_state_mutex);
//     }
// }

// // Update the control-task heartbeat.
// // A watchdog task can use this to confirm the control loop is still alive.
// static void kickControlHeartbeat(uint32_t nowUs)
// {
//     if (g_state_mutex != nullptr &&
//         xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE)
//     {
//         g_system_state.control_last_heartbeat_us = nowUs;
//         xSemaphoreGive(g_state_mutex);
//     }
// }

// // ===================== CONTROL TASK ==================

// // Main loop flow:
// // every fast cycle:
// // 1. wait until next scheduled wakeup
// // 2. measure actual dt
// // 3. read config + enable/fault flags
// // 4. update encoder
// // 5. run motor.loopFOC()
// // 6. build MeasuredState
// // 7. every N cycles compute a new haptic command
// // 8. apply the latest command to the motor
// // 9. publish state + heartbeat
// static void controlTask(void *pvParameters){

//     (void)pvParameters;

//     TickType_t lastWakeTime = xTaskGetTickCount();

//     // convert the desired period to RTOS ticks
//     // we force at least 1 tick just in case the conversion gives 0
//     const TickType_t periodTicks = (pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS) > 0)?
//                                     pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS) : 1;

//     // Track actual elapsed time using micros() so we can estimate acceleration
//     uint32_t lastMicros = micros();

//     // Counts how many fast-loop iterations have happened.
//     // We use this to schedule the slower model update.
//     uint32_t fastLoopCounter = 0;

//     // Store the latest model output here. The control task holds this
//     // command between slower outer loop updates
//     HapticCommand heldCommand{};
//     heldCommand.iq_cmd = 0.0f;
//     heldCommand.last_update_us = micros();

//     // We need the previous velocity to estimate acceleration
//     float prevVelocity = 0.0f;
//     bool firstSample = true;

//     while (1){
//         // 1. Wait for the next control period
//         vTaskDelayUntil(&lastWakeTime, periodTicks);

//         // 2. Measure actual loop dt. Note that even though the task is periodic
//         // there will still be some jitter so we will explicitly compute dt instead
//         // of just making the assumption that its perf periodic
//         uint32_t nowUs = micros();
//         float dt = (nowUs - lastMicros) * 1e-6f;
//         lastMicros = nowUs;

//         // Defensive fallbakc in case timing becomes invalid
//         if (dt <= 0.0f || dt > 0.1f) {
//             dt = CONTROL_TASK_PERIOD_MS * 1e-3f;
//         }

//         // 3. Read shared config and system flags
//         RuntimeConfig config = readRuntimeConfig();
//         ControlFlags flags = readControlFlags();

//         // The motor should only actively drive if control is enabled
//         // and no fault has been latched
//         bool shouldDrive = flgas.control_enabled && ~flags.fault_latched;

//         // 4. Update the encoder
//         // Note we update the encoder before motor.loopFOC() so the motor control
//         // loop uses the freshest possible shaft position data
//         encoder.update();

//         // 5. Run the fast motor loop
//         // This is the lowlevel motor side FOC update. It should run every fast cycle, regardless
//         // of whether the slower haptic model is recomputed this cycle or not
//         motor.loopFOC();

//         // 6. Build the latest measured state
//         // This struct is the input to HapticModels.cpp and it rerpesents what the knob is doing rn
//         MeasuredState measured{};

//         measured.angle_rad = encoder.getAngle();
//         measured.angle_deg_wrapped = encoder.angleDegWrapped();

//         measured.velocity_rad_s = motor.shaftVelocity();

//         // Estimate angular acceleration nuymericcally from the change in velocity
//         if (firstSample) {
//             measured.acceleration_rad_s2 = 0.0f;
//             firstSample = false;
//         }
//         else {
//             measured.acceleration_rad_s2 = (measured.velocity_rad_s - prevVelocity) / dt;
//         }

//         // Save the current velocity for the next loop acceleration estimate
//         prevVelocity = measured.velocity_rad_s;

//         // Timestamp for when this measured state was produced
//         measured.last_update_us = nowUs;

//         // These will be filled from SimpleFOC
//         measured.iq_meas = 0.0f;
//         measured.ia = 0.0f;
//         measured.ib = 0.0f;
//         measured.ic = 0.0f;

//         // publish the latest measured state to shared mem
//         writeMeasuredState(measured);

//         // 7. Run the slower haptic model update
//         // So this will be our merged loop architecture
//         // fast inner loop every cycle:
//         //      encoder.update()
//         //      motor.loopFOC()
//         //      motor.move(...)
//         //
//         // slower every N cycles:
//         //     computeActiveModelCommand(...)
//         //
//         // So what we've done is that instead of putting the inner and outer loops in seperate RTOS tasks,
//         // we keep them inside one deterministic taks and just schedule the outer/model part less often
//         if ((fastLoopCounter % OUTER_LOOP_DIVIDER) == 0){

//             if (shouldDrive){
//                 // Ask the model layer to compute a new command based on:
//                 // - the latest measured state
//                 // - the user selected runtime config
//                 // Then the command is written into heldCommand
//                 computeActiveModelCommand(measured, config, heldCommand);
//             }
//             else {
//                 // If control is disabled or a fault is latched, force a safe
//                 // zero command instead of letting the model drive the motor
//                 heldCommand.iq_cmd = 0.0f;
//                 heldCommand.last_update_us = nowUs;
//             }
//         }

//         // Publish the latest command
//         writeHapticCommand(heldCommand);

//         // 8. Apply the held command to the motor
//         if (shouldDrive) {
//             motor.torque_controller = TorqueControlType::foc_current;
//             motor.move(heldCommand.iq_cmd);
//         }
//         else {
//             motor.move(0.0f);
//         }

//         // 9. update heartbeat to let the watchdog task know the control task is still alive
//         kickControlHeartbeat(nowUs);

//         // 10. advance the fast loop counter
//         fastLoopCounter++;

//     }

//     // Start function which is called once from setup() after shared state and hardware are initiliazed
//     void startControlTask(){
//         BaseType_t ok = xTaskCreatePinnedToCore(
//             controlTask, // Task entry function
//             "ControlTask", // Task name (used for debugging)
//             CONTROL_TASK_STACK_SIZE, // stack size
//             nullptr, // no param passed to task
//             CONTROL_TASK_PRIORITY,
//             &g_control_task_handle, // returned task handle
//             0 // Pin to core 0
//         );

//         if (ok != pdPASS) {
//             Serial.println("Failed to create ControlTask");

//             while (true)
//             {
//                 delay(1000);
//             }

//         }
//     }
// }

#include "app/ControlTask.h"

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/Hardware.h"
#include "app/SharedState.h"
#include "control/HapticModels.h"

namespace
{
    // ======================= TASK TIMING / SETTINGS =========================
    // CONTROL_TASK_PERIOD_MS:
    //   This is how often the control task wakes up and runs.
    //
    //   Since this is 1 ms, the fast control loop runs at:
    //       1 / 0.001 = 1000 Hz
    //
    // OUTER_LOOP_DIVIDER:
    //   We do not want to recompute the haptic model every single
    //   fast-loop cycle if we do not need to.
    //   Instead, we let the low-level hardware/control side run
    //   every cycle, and we run the higher-level model update
    //   every N cycles.
    //
    //   With OUTER_LOOP_DIVIDER = 5 and a 1 ms base loop:
    //       fast loop = 1000 Hz
    //       model loop = 1000 / 5 = 200 Hz
    //
    //   So the architecture becomes:
    //   Every 1 ms:
    //       - wake task
    //       - update hardware control step
    //       - read motor state
    //       - apply most recent held command
    //
    //   Every 5th fast-loop cycle:
    //       - recompute the haptic command from the active model
    //
    // CONTROL_TASK_STACK_SIZE:
    //   Stack allocated to the FreeRTOS task.
    //
    // CONTROL_TASK_PRIORITY:
    //   Priority of this control task relative to the rest of the system.
    //   Since this is time-sensitive, it should generally be higher than
    //   non-critical tasks like telemetry/debug printing.
    //
    constexpr uint32_t CONTROL_TASK_PERIOD_MS = 1;
    constexpr uint32_t OUTER_LOOP_DIVIDER = 5;
    constexpr uint32_t CONTROL_TASK_STACK_SIZE = 4096;
    constexpr UBaseType_t CONTROL_TASK_PRIORITY = 5;

    // Handle returned by FreeRTOS when the task is created.
    // This can be useful later if we ever want to suspend/resume/delete
    // the task or inspect it during debugging.
    TaskHandle_t g_control_task_handle = nullptr;

    // ================ LOCAL HELPER STRUCTS ===============
    // Local snapshot of the system-level flags we care about inside
    // the control loop.
    //
    // control_enabled:
    //   Whether the system is allowed to actively drive the motor.
    //
    // fault_latched:
    //   Whether some fault condition has occurred and the system has
    //   latched into a safe state.
    struct ControlFlags
    {
        bool control_enabled = false;
        bool fault_latched = false;
    };

    // ============= SHARED STATE SNAPSHOT HELPERS ============
    //
    // These helper functions keep the control task code cleaner.
    // Instead of manually reaching into shared state inside the main
    // loop, we ask for compact "snapshots" of what we need.
    //
    // The general idea is:
    //   - read shared state once
    //   - make a local copy
    //   - do the real work using the local copy
    //
    // That is good practice for real-time code because it keeps the
    // time-sensitive control loop simple and predictable.

    // Read the current runtime configuration into a local copy.
    // This configuration contains the currently selected haptic mode
    // and the parameters for that mode (for example gains/constants
    // for resistor, capacitor, inductor, diode, etc.).
    //
    // We return a copy here so the control loop can safely work with
    // that snapshot for the rest of the iteration and won't require us to get the shared state mutex each time for this data.
    RuntimeConfig readRuntimeConfigSnapshot()
    {
        RuntimeConfig config{};
        readRuntimeConfig(config);
        return config;
    }

    // Read the relevant system flags into a compact helper struct.
    // We first read the larger SystemState, then extract only the
    // fields this control task cares about right now.
    ControlFlags readControlFlags()
    {
        SystemState systemState{};
        readSystemState(systemState);

        ControlFlags flags{};
        flags.control_enabled = systemState.control_enabled;
        flags.fault_latched = systemState.fault_latched;
        return flags;
    }

    // Update the control-task heartbeat in shared state.
    // A watchdog task can monitor this heartbeat timestamp to confirm
    // that the control task is still running and not stalled.
    // If the heartbeat stops updating for too long, the watchdog can
    // decide something has gone wrong and place the system into a safe state where we stop outputting to the actuator.
    void kickControlHeartbeat(uint32_t nowUs)
    {
        SystemState systemState{};
        if (readSystemState(systemState))
        {
            systemState.control_last_heartbeat_us = nowUs;
            writeSystemState(systemState);
        }
    }

    // =================== MAIN CONTROL TASK ==============
    //
    // High-level flow of this task:
    // Every fast cycle:
    //   1. Wait until the next scheduled task release
    //   2. Measure actual elapsed time dt
    //   3. Read config + control/fault flags
    //   4. Run the low-level hardware control step
    //   5. Build the latest measured state of the knob/motor
    //   6. Publish measured state for other tasks
    //   7. Every N cycles, recompute the model command
    //   8. Apply the currently held motor command
    //   9. Update heartbeat
    //  10. Increment fast loop counter
    //
    // Important design idea here:
    // We are using a merged loop architecture.
    //
    // Instead of having:
    //   - one separate RTOS task for inner motor control
    //   - one separate RTOS task for outer haptic model control
    //
    // we keep everything in one deterministic control task and
    // simply run the slower model section less often.
    //
    // This is often cleaner and safer for embedded real-time systems
    // because it avoids unnecessary task-to-task scheduling jitter
    // between tightly related control loops.
    //
    void controlTask(void *pvParameters)
    {
        (void)pvParameters;

        // The reference tick value used by vTaskDelayUntil().
        //
        // FreeRTOS will update from this base so the task wakes
        // periodically instead of sleeping relative to whenever
        // the previous iteration ended.
        //
        // That helps keep the loop timing deterministic.
        TickType_t lastWakeTime = xTaskGetTickCount();

        // Convert the desired period in milliseconds to RTOS ticks.
        // Defensive detail:
        // If the conversion somehow gives 0 ticks, we force it to at least 1
        // so the task still delays properly.
        const TickType_t periodTicks = pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS) > 0 ? pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS) : 1;

        // We also track time using micros() because the real elapsed dt can have
        // some jitter and we want a more accurate estimate of actual loop timing,
        // especially for numerical calculations like acceleration.
        uint32_t lastMicros = micros();

        // Counts how many fast-loop iterations have run.
        // We use this counter to decide when to run the slower outer/model update.
        uint32_t fastLoopCounter = 0;

        // This stores the most recent command computed by the haptic model.
        // The important idea is:
        //   - the model does not have to recompute every fast cycle
        //   - between model updates, we "hold" the previous command
        //     and keep applying it in the fast loop
        //
        // So this acts like the current command being fed to the motor and this will change every 200 hz.
        HapticCommand heldCommand{};
        heldCommand.iq_cmd = 0.0f;
        heldCommand.last_update_us = lastMicros;

        // We keep the previous velocity so we can estimate acceleration
        // using a finite-difference approximation:
        //   acceleration = (current_velocity - previous_velocity) / dt
        float prevVelocity = 0.0f;

        // For the very first sample, we do not yet have a meaningful previous
        // velocity to compare against, so we explicitly handle that case.
        bool firstSample = true;

        // Infinite control loop.
        //
        // In FreeRTOS tasks, this is normal: the task runs forever unless explicitly deleted or the device resets.
        for (;;)
        {
            // 1. WAIT FOR THE NEXT CONTROL PERIOD
            // vTaskDelayUntil() helps keep the task periodic relative to the
            // intended schedule, rather than drifting based on how long the
            // previous iteration took.
            vTaskDelayUntil(&lastWakeTime, periodTicks);

            // 2. MEASURE ACTUAL LOOP DT
            // Even though the task is intended to run every 1 ms, real systems can still have a bit of jitter. So we compute the actual elapsed
            // time and use that for any time-dependent calculations.
            const uint32_t nowUs = micros();
            float dt = static_cast<float>(nowUs - lastMicros) * 1e-6f;
            lastMicros = nowUs;

            // Defensive fallback:
            // If dt is invalid or unreasonably large, fall back to the nominal
            // task period
            if (dt <= 0.0f || dt > 0.1f)
            {
                dt = CONTROL_TASK_PERIOD_MS * 1e-3f;
            }

            // 3. READ SHARED CONFIGURATION + SYSTEM FLAGS
            // We take snapshots of:
            //   - current runtime config (selected mode + parameters)
            //   - whether control is enabled
            //   - whether a fault has been latched
            //
            // Then we decide whether the motor is actually allowed to drive.
            RuntimeConfig config = readRuntimeConfigSnapshot();
            ControlFlags flags = readControlFlags();

            // The motor should only actively drive when:
            //   1. control has been enabled by the system, and
            //   2. no fault is currently latched
            // If either is false, we go into safe behavior later in the loop.
            // for now we set this to 1 manually to test but it shoudl be flags.control_enabled && !flags.fault_latched;
            const bool shouldDrive = 1;

            // 4. RUN THE FAST HARDWARE / MOTOR CONTROL STEP
            //   - sensor update / sampling
            //   - FOC inner-loop update
            //   - driver-side maintenance
            //
            // The important design point is:
            // this step runs every fast cycle.
            updateHardwareControlStep();

            // 5. BUILD THE LATEST MEASURED STATE
            // This struct is the "what is the knob doing right now?" snapshot.
            // It is what the haptic model uses as input.
            // We collect:
            //   - angle
            //   - wrapped angle in degrees
            //   - angular velocity
            //   - angular acceleration (estimated numerically)
            //   - measured q-axis current
            //   - measured phase currents
            //   - timestamp
            MeasuredState measured{};

            // Shaft angle in radians.
            measured.angle_rad = getMotorAngleRad();

            // Wrapped angle in degrees.
            // This is often convenient for display/debugging/UI since it stays
            // in a bounded range instead of continuously growing.
            measured.angle_deg_wrapped = getMotorAngleDegWrapped();

            // Shaft angular velocity in rad/s.
            measured.velocity_rad_s = getMotorVelocityRad();

            // Estimate angular acceleration from the change in velocity.
            // On the first iteration, we do not have a valid previous sample yet,
            // so we just set acceleration to zero.
            if (firstSample)
            {
                measured.acceleration_rad_s2 = 0.0f;
                firstSample = false;
            }
            else
            {
                measured.acceleration_rad_s2 = (measured.velocity_rad_s - prevVelocity) / dt;
            }

            // Save the velocity for the next loop iteration's acceleration estimate.
            prevVelocity = measured.velocity_rad_s;

            // Measured q-axis current.
            // This is useful both for debugging and for models/control logic
            // that care about actual torque-producing current.
            measured.iq_meas = getMeasuredIq();

            // Read the three phase currents from the hardware abstraction layer.
            const PhaseCurrent_s phaseCurrents = getPhaseCurrents();
            measured.ia = phaseCurrents.a;
            measured.ib = phaseCurrents.b;
            measured.ic = phaseCurrents.c;

            // Timestamp for when this measured snapshot was produced.
            measured.last_update_us = nowUs;

            // Publish the latest measured state so telemetry/debug/watchdog/other
            // software layers can inspect what the control loop is seeing.
            writeMeasuredState(measured);

            // 6. RUN THE SLOWER HAPTIC MODEL UPDATE
            // This is the outer-loop/model part.
            //
            // We do not run it every fast cycle. Instead, we run it only once every OUTER_LOOP_DIVDER iterations
            //
            // Fast every cycle:
            //   - updateHardwareControlStep() (reads sensor values and updates and runs foc algorithm)
            //   - read updated measurementes
            //   - apply current command
            //
            // Slower every N cycles:
            //   - computeActiveModelCommand(...)
            //
            if ((fastLoopCounter % OUTER_LOOP_DIVIDER) == 0)
            {
                if (shouldDrive)
                {
                    // Ask the active haptic model to compute a fresh command
                    // based on:
                    //   - the latest measured knob/motor state
                    //   - the current runtime configuration
                    //
                    // The result is written into heldCommand.
                    computeActiveModelCommand(measured, config, heldCommand);
                }

                else
                {
                    // If control is disabled or a fault is latched, we force the
                    // held command to a safe zero-current command instead of
                    // letting the model continue to drive the motor.
                    heldCommand.iq_cmd = 0.0f;
                    heldCommand.last_update_us = nowUs;
                }

                // Publish the latest model command.
                // We do this when the command is refreshed so other tasks can see
                // what the model most recently requested.
                writeHapticCommand(heldCommand);
            }

            // 7. APPLY THE CURRENT HELD COMMAND TO THE MOTOR
            // Important subtle point:
            //
            // Even on cycles where we do not recompute the model command,
            // we still apply the currently held command.
            //
            // That is exactly how this merged architecture works:
            //   - slow model computes command occasionally
            //   - fast loop keeps enforcing that command every cycle
            if (shouldDrive)
            {
                // Apply the torque-producing current request to the motor.
                applyMotorCurrent(heldCommand.iq_cmd);
                // static float filteredIqCmd = 0.0f;
                // filteredIqCmd += 0.08f * (heldCommand.iq_cmd - filteredIqCmd);
                // applyMotorCurrent(filteredIqCmd);
            }
            else
            {
                // Safe behavior when not allowed to drive.
                stopMotor();
            }

            // 8. KICK THE HEARTBEAT
            //
            // This tells the rest of the system that the control task is alive
            // and still executing.
            kickControlHeartbeat(nowUs);

            // 9. ADVANCE LOOP COUNTER
            //
            // This is what allows the modulo check above to schedule the
            // slower model update every N fast cycles.
            ++fastLoopCounter;
        }
    }
} // namespace

// TASK START UP
// This is called once during setup/initialization after shared state
// and hardware have already been initialized.
//
// It creates the FreeRTOS control task and pins it to core 0.
void startControlTask()
{
    const BaseType_t ok = xTaskCreatePinnedToCore(
        controlTask,             // Task entry function
        "ControlTask",           // Name used in debugging / RTOS tools
        CONTROL_TASK_STACK_SIZE, // Stack size allocated to the task
        nullptr,                 // No parameter passed into the task
        CONTROL_TASK_PRIORITY,   // Task priority
        &g_control_task_handle,  // Returned task handle
        0);                      // Pin task to core 0

    // If task creation fails, we print an error and stop here.
    // In embedded systems, silently continuing after a failed control-task
    // creation would be dangerous because the rest of the system might assume
    // control is running when it actually is not.
    if (ok != pdPASS)
    {
        Serial.println("Failed to create ControlTask");

        // Halt here so the failure is obvious.
        while (true)
        {
            delay(1000);
        }
    }
}