// RM2026 robot slot layout. slot 0..4 = hero/engineer/inf3/inf4/sentry.
#ifndef VISION_INTERFACE__ROBOT_SLOTS_H_
#define VISION_INTERFACE__ROBOT_SLOTS_H_

namespace tdt_radar {

constexpr int kRobotSlotCount = 5;
constexpr int kSentrySlot     = 4;
constexpr int kHeroSlot       = 0;

inline constexpr int slot_to_robot_number(int slot) {
    constexpr int kMap[kRobotSlotCount] = {1, 2, 3, 4, 7};
    return (slot >= 0 && slot < kRobotSlotCount) ? kMap[slot] : 0;
}

inline constexpr int armor_class_to_slot(int cls) {
    switch (cls) {
        case 1: return 0;
        case 2: return 1;
        case 3: return 2;
        case 4: return 3;
        case 6: return 4;
        default: return -1;
    }
}

// referee 0x0003 HP layout per side: [0..3]=hero/eng/inf3/inf4 [4]=aerial(skip) [5]=sentry
inline constexpr int slot_to_hp_index(int slot) {
    constexpr int kMap[kRobotSlotCount] = {0, 1, 2, 3, 5};
    return (slot >= 0 && slot < kRobotSlotCount) ? kMap[slot] : -1;
}

}  // namespace tdt_radar

#endif
