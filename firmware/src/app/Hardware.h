/*
    Hardware.h

    This file declares the hardware-facing API for the haptic knob system.

    It owns the one-and-only access point to the real motor hardware stack:
      - SPI bus
      - magnetic encoder
      - current sense
      - BLDC driver
      - BLDC motor / SimpleFOC integration

    Other parts of the firmware should NOT directly instantiate or manage
    these hardware objects themselves. Instead, they should call the helper
    functions declared here.

    Responsibilities:
      - initialize the physical motor/sensor system
      - expose one control-step update function for the fast motor loop
      - expose helper functions to apply current/voltage commands
      - expose helper functions to read angle, velocity, and measured currents

    This file is about real hardware access, not haptic model math.
*/
