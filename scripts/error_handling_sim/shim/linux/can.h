// Test shim for macOS: struct can_frame (CAN is not used by the tests)
#include <stdint.h>
#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U
#define CAN_SFF_MASK 0x000007FFU
#define CAN_EFF_MASK 0x1FFFFFFFU
typedef uint32_t canid_t;
struct can_frame {
    canid_t can_id;
    uint8_t can_dlc;
    uint8_t __pad, __res0, __res1;
    uint8_t data[8] __attribute__((aligned(8)));
};
