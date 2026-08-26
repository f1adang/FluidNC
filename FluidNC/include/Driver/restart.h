bool restart_was_panic();
void restart();

// Why the board came up: "Power on", "Task watchdog", "Brownout" and so on.
// A job that vanishes without explanation is usually a reset, and the reason
// separates a firmware fault from a power or watchdog problem.
const char* restart_reason();
