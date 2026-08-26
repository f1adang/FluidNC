#include "Driver/restart.h"
#include "esp_system.h"

void restart() {
    esp_restart();
    while (1) {}
}

bool restart_was_panic() {
    return esp_reset_reason() == ESP_RST_PANIC;
}

const char* restart_reason() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:
            return "Power on";
        case ESP_RST_EXT:
            return "External pin";
        case ESP_RST_SW:
            return "Software restart";
        case ESP_RST_PANIC:
            return "Panic or unhandled exception";
        case ESP_RST_INT_WDT:
            return "Interrupt watchdog";
        case ESP_RST_TASK_WDT:
            return "Task watchdog";
        case ESP_RST_WDT:
            return "Other watchdog";
        case ESP_RST_DEEPSLEEP:
            return "Deep sleep wakeup";
        case ESP_RST_BROWNOUT:
            return "Brownout - check the power supply";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "Unknown";
    }
}
