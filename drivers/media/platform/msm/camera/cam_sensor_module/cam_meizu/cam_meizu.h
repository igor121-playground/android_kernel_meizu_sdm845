/* Copyright (c) 2024, MeizuCustoms enthusiasts.
 *
 * Meizu camera glue layer for the M1882 (Meizu 16th).
 *
 * Stock Flyme exposes a /sys/class/meizu/<attr> interface that its vendor
 * CAMX userspace talks to (AF enable/position, gyro request for OIS, OTP and
 * calibration data, ToF calibration, ...).  This file recreates the class and
 * the actuator / sensor attributes.
 *
 * NOTE: the stock kernel's explicit struct layouts differ from this tree's
 * camera driver, so this is a functional re-implementation against the tree's
 * public APIs (guided by the disassembly of the stock functions), not a
 * byte-exact lift.
 */
#ifndef _CAM_MEIZU_H_
#define _CAM_MEIZU_H_

#include <linux/device.h>

struct cam_actuator_ctrl_t;
struct cam_sensor_ctrl_t;
struct cam_ois_ctrl_t;

/* Create/destroy the "meizu" sysfs class. */
int meizu_cam_init(void);
void meizu_cam_exit(void);

/* Called from the actuator / sensor probes so the sysfs handlers know the
 * (single) control structures to operate on. */
void meizu_cam_register_actuator(struct cam_actuator_ctrl_t *a_ctrl);
void meizu_cam_register_sensor(struct cam_sensor_ctrl_t *s_ctrl);
void meizu_cam_register_ois(struct cam_ois_ctrl_t *o_ctrl);

/* OTP/calibration buffer, registered by the eeprom driver after a read.
 * meizu_cam_cal_check() runs the decoded Meizu OTP validation. */
void meizu_cam_register_otp(u8 *buf, u32 len);
int meizu_cam_cal_check(u8 *buf, u32 len);
int meizu_cam_cal_check_main(u8 *buf, u32 len);

#endif /* _CAM_MEIZU_H_ */
