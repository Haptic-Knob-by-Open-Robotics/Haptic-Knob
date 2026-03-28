/*
    WatchdogTask.h

    This file declares the safety/watchdog task.

    This task is optional at first, but later it will supervise whether the
    control system is still updating properly.
*/
#pragma once

void WatchdogTask(void *pvParameters);
void startWatchdogTask();