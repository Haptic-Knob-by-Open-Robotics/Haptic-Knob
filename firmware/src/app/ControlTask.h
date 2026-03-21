#pragma once 

// Create the main deterministic control task 
// This task will own the control path so it will: 
// 1. Read the latest sensor state 
// 2. Run the fast motor loop 
// 3. Build MeasuredState
// 4. Periodically call HapticMOdels.cpp
// 5. Apply the returned command to the motor 
void startControlTask(); 