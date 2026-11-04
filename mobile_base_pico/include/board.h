/*
 * Pin assignment of the HARP3 mobile base board (HARP3_Elec). The OTOS bus and
 * pins are set on the OTOS library in CMakeLists.txt.
 */

#ifndef MOBILE_BASE_BOARD_H
#define MOBILE_BASE_BOARD_H

/* Indexed by wheel: driver A1 is wheel_1_joint. Each STEP pin sits on its own
   PWM slice, so each wheel gets its own step rate. */
#define BOARD_STEP_PINS {10, 13, 7}
#define BOARD_DIR_PINS {9, 12, 6}
/* Active low. */
#define BOARD_ENABLE_PINS {11, 14, 8}

/* DIR level for a positive wheel speed is high unless inverted. To be checked
   wheel by wheel, robot on blocks, against omni_wheel_drive_controller. */
#define BOARD_DIR_INVERTED {false, false, false}

/* Int11, Int12, Int21, Int22, Int31, Int32: closed to 3.3 V while pressed. */
#define BOARD_LIMIT_SWITCH_PINS {20, 21, 18, 19, 16, 17}
#define BOARD_LIMIT_SWITCH_COUNT 6

#endif /* MOBILE_BASE_BOARD_H */
