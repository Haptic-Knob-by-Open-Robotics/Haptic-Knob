/*
    WatchdogTask.cpp

    This file implements the watchdog/safety supervision task.

    Main responsibilities:
      - check whether measured state is still updating
      - check whether model commands are still updating
      - detect stale or invalid data
      - raise a fault flag if something is wrong

    When a fault is raised, the motor task can use that information to stop
    applying torque and bring the system to a safe state.
*/
