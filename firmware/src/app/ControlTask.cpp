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
constexpr uint32_t CONTROL_TASK_PERIOD_MS = 1;
constexpr uint32_t OUTER_LOOP_DIVIDER = 5;
constexpr uint32_t CONTROL_TASK_STACK_SIZE = 4096;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 5;

TaskHandle_t g_control_task_handle = nullptr;

struct ControlFlags
{
    bool control_enabled = false;
    bool fault_latched = false;
};

RuntimeConfig readRuntimeConfigSnapshot()
{
    RuntimeConfig config{};
    readRuntimeConfig(config);
    return config;
}

ControlFlags readControlFlags()
{
    SystemState systemState{};
    readSystemState(systemState);

    ControlFlags flags{};
    flags.control_enabled = systemState.control_enabled;
    flags.fault_latched = systemState.fault_latched;
    return flags;
}

void kickControlHeartbeat(uint32_t nowUs)
{
    SystemState systemState{};
    if (readSystemState(systemState))
    {
        systemState.control_last_heartbeat_us = nowUs;
        writeSystemState(systemState);
    }
}

void controlTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks =
        pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS) > 0 ? pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS) : 1;

    uint32_t lastMicros = micros();
    uint32_t fastLoopCounter = 0;

    HapticCommand heldCommand{};
    heldCommand.iq_cmd = 0.0f;
    heldCommand.last_update_us = lastMicros;

    float prevVelocity = 0.0f;
    bool firstSample = true;

    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, periodTicks);

        const uint32_t nowUs = micros();
        float dt = static_cast<float>(nowUs - lastMicros) * 1e-6f;
        lastMicros = nowUs;

        if (dt <= 0.0f || dt > 0.1f)
        {
            dt = CONTROL_TASK_PERIOD_MS * 1e-3f;
        }

        RuntimeConfig config = readRuntimeConfigSnapshot();
        ControlFlags flags = readControlFlags();
        // const bool shouldDrive = flags.control_enabled && !flags.fault_latched;
        const bool shouldDrive = 1;
        updateHardwareControlStep();

        MeasuredState measured{};
        measured.angle_rad = getMotorAngleRad();
        measured.angle_deg_wrapped = getMotorAngleDegWrapped();
        measured.velocity_rad_s = getMotorVelocityRad();

        if (firstSample)
        {
            measured.acceleration_rad_s2 = 0.0f;
            firstSample = false;
        }
        else
        {
            measured.acceleration_rad_s2 = (measured.velocity_rad_s - prevVelocity) / dt;
        }
        prevVelocity = measured.velocity_rad_s;

        measured.iq_meas = getMeasuredIq();

        const PhaseCurrent_s phaseCurrents = getPhaseCurrents();
        measured.ia = phaseCurrents.a;
        measured.ib = phaseCurrents.b;
        measured.ic = phaseCurrents.c;
        measured.last_update_us = nowUs;

        writeMeasuredState(measured);

        if ((fastLoopCounter % OUTER_LOOP_DIVIDER) == 0)
        {
            if (shouldDrive)
            {
                computeActiveModelCommand(measured, config, heldCommand);
            }
            else
            {
                heldCommand.iq_cmd = 0.0f;
                heldCommand.last_update_us = nowUs;
            }

            writeHapticCommand(heldCommand);
        }

        if (shouldDrive)
        {
            // printf("Should be driving");
            applyMotorCurrent(heldCommand.iq_cmd);
        }
        else
        {
            stopMotor();
        }

        kickControlHeartbeat(nowUs);
        ++fastLoopCounter;
    }
}
} // namespace

void startControlTask()
{
    const BaseType_t ok = xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        CONTROL_TASK_STACK_SIZE,
        nullptr,
        CONTROL_TASK_PRIORITY,
        &g_control_task_handle,
        0);

    if (ok != pdPASS)
    {
        Serial.println("Failed to create ControlTask");
        while (true)
        {
            delay(1000);
        }
    }
}
