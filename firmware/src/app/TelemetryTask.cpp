/*
    TelemetryTask.cpp

    This file implements the telemetry task.

    Main responsibilities:
      - read commands from Serial / Python GUI
      - parse mode changes, gain updates, PID updates, zeroing commands, etc.
      - update SharedState configuration values
      - periodically print useful telemetry for debugging and visualization

    This task should be the main place where Serial I/O happens so the
    motor-control loop stays cleaner and more deterministic.
*/
