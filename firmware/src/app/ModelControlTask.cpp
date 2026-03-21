/*
    ModelControlTask.cpp

    This file implements the haptic model task.

    Main responsibilities:
      1. read the latest measured shaft state from SharedState
      2. check which haptic mode is active
      3. call the correct model function from HapticModels.cpp
      4. generate the desired motor command
      5. write that command back into SharedState

    This task should not directly manage encoder or motor hardware.
    It only decides what the hardware should do.

    Think of this task as:
      "given the current shaft motion, what should the knob feel like?"
*/


