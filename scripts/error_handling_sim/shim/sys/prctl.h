// Test shim for macOS: no prctl()
#define PR_SET_NAME 15
static inline int prctl(int o, ...) { return 0; }
