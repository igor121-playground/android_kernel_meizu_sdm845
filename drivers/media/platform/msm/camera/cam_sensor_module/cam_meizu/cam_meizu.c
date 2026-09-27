// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2024, MeizuCustoms enthusiasts.
 *
 * Meizu camera class + sysfs glue (M1882 / Meizu 16th).
 */
#define pr_fmt(fmt) "CAM-MEIZU: " fmt

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include "cam_meizu.h"
#include "cam_actuator_core.h"
#include "cam_sensor_core.h"
#include "cam_ois_core.h"
#include "cam_debug_util.h"

static struct class *meizu_class;
static struct device *meizu_dev;

static struct cam_actuator_ctrl_t *g_a_ctrl;
static struct cam_sensor_ctrl_t *g_s_ctrl;
static struct cam_ois_ctrl_t *g_o_ctrl;

/* state mirrored from stock (its globals at 0xacf1ae38 / 0xacf1ae3c / 0xacf1ae40) */
static int g_af_enable_status = -1;
static u16 g_af_pos;
static int g_gyro_req;

void meizu_cam_register_actuator(struct cam_actuator_ctrl_t *a_ctrl)
{
	g_a_ctrl = a_ctrl;
}
EXPORT_SYMBOL(meizu_cam_register_actuator);

void meizu_cam_register_sensor(struct cam_sensor_ctrl_t *s_ctrl)
{
	g_s_ctrl = s_ctrl;
}
EXPORT_SYMBOL(meizu_cam_register_sensor);

void meizu_cam_register_ois(struct cam_ois_ctrl_t *o_ctrl)
{
	g_o_ctrl = o_ctrl;
}
EXPORT_SYMBOL(meizu_cam_register_ois);

/*
 * Meizu sysfs link helper (meizu_sysfslink_*).
 *
 * Stock exposes devices as symlinks under the shared "meizu" class
 * (/sys/class/meizu/<name>).  The parent kobject is the class's own kobject,
 * i.e. class->p->subsys.kobj.  struct subsys_private is private to the driver
 * core, so (as the compiled stock does) the kobject is reached through the
 * known offset: subsys.kobj sits at offset 0x18 in subsys_private
 * (list 0x10 + spinlock 0x4 + padding).
 */
static DEFINE_MUTEX(meizu_sysfslink_lock);

static struct kobject *meizu_class_kobj(void)
{
	if (!meizu_class || !meizu_class->p)
		return NULL;
	return (struct kobject *)((char *)meizu_class->p + 0x18);
}

int meizu_sysfslink_register(struct device *dev)
{
	struct kobject *parent = meizu_class_kobj();

	if (!dev || !parent)
		return -1;

	return sysfs_create_link(parent, &dev->kobj, dev->kobj.name);
}
EXPORT_SYMBOL(meizu_sysfslink_register);

int meizu_sysfslink_register_name(struct device *dev, const char *name)
{
	struct kobject *parent = meizu_class_kobj();

	if (!dev || !name || !parent)
		return -1;

	return sysfs_create_link(parent, &dev->kobj, name);
}
EXPORT_SYMBOL(meizu_sysfslink_register_name);

void meizu_sysfslink_remove_name(const char *name)
{
	struct kobject *parent = meizu_class_kobj();

	if (!parent || !name)
		return;
	sysfs_remove_link(parent, name);
}
EXPORT_SYMBOL(meizu_sysfslink_remove_name);

void meizu_sysfslink_unregister_name(const char *name)
{
	meizu_sysfslink_remove_name(name);
}
EXPORT_SYMBOL(meizu_sysfslink_unregister_name);

struct kobject *meizu_sysfslink_get_kobj(const char *name)
{
	struct kobject *parent = meizu_class_kobj();
	struct kobject *kobj;

	if (!parent || !name)
		return NULL;

	mutex_lock(&meizu_sysfslink_lock);
	kobj = kobject_create_and_add(name, parent);
	mutex_unlock(&meizu_sysfslink_lock);
	return kobj;
}
EXPORT_SYMBOL(meizu_sysfslink_get_kobj);

void meizu_sysfslink_unregister(struct device *dev)
{
	if (dev)
		device_unregister(dev);
}
EXPORT_SYMBOL(meizu_sysfslink_unregister);

/*
 * af_enable: write-only in stock (its show returns 0).  Store enables or
 * disables the AF actuator.
 */
static ssize_t af_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int32_t rc;

	if (!g_a_ctrl)
		return -ENODEV;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	rc = meizu_actuator_enable(g_a_ctrl, val);
	g_af_enable_status = rc;
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "meizu af_enable failed %d", rc);
		return rc;
	}

	return count;
}

static ssize_t af_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	/* stock show is a no-op returning 0 */
	return 0;
}
static DEVICE_ATTR_RW(af_enable);

/* af_pos: read/write the current AF position (register 0x8423). */
static ssize_t af_pos_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 pos = 0;
	int32_t rc;

	if (!g_a_ctrl)
		return -ENODEV;

	rc = meizu_get_af_pos(g_a_ctrl, &pos);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pos);
}

static ssize_t af_pos_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int32_t rc;

	if (!g_a_ctrl)
		return -ENODEV;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (val > 0x3ff) {
		CAM_ERR(CAM_ACTUATOR, "invalid AF position %d", val);
		return -EINVAL;
	}

	g_af_pos = (u16)val;
	rc = meizu_set_af_pos(g_a_ctrl, g_af_pos);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "meizu_set_af_pos failed %d", rc);
		return rc;
	}
	msleep(150);

	return count;
}
static DEVICE_ATTR_RW(af_pos);

/*
 * set_ops: write a target AF position and read back a pass/fail result plus
 * the measured position (stock format).
 */
static ssize_t set_ops_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 pos = 0;
	int32_t rc;

	if (!g_a_ctrl)
		return scnprintf(buf, PAGE_SIZE, "result=fail\n");

	rc = meizu_get_af_pos(g_a_ctrl, &pos);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "meizu_get_af_pos failed %d", rc);
		return scnprintf(buf, PAGE_SIZE, "result=fail\n");
	}

	if (g_af_pos > 0x400)
		return scnprintf(buf, PAGE_SIZE,
			"result=fail, invalid position.\ncurrent_af_position=%d\n",
			pos);

	return scnprintf(buf, PAGE_SIZE, "current_af_position=%d\nresult=%s\n",
		pos, (pos <= 0xc8) ? "pass" : "fail");
}

static ssize_t set_ops_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int32_t rc;

	if (!g_a_ctrl)
		return -ENODEV;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (val > 0x3ff) {
		CAM_ERR(CAM_ACTUATOR, "set_ops invalid position %d", val);
		return -EINVAL;
	}

	g_af_pos = (u16)val;
	rc = meizu_set_af_pos(g_a_ctrl, g_af_pos);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "meizu_set_af_pos failed %d", rc);
		return rc;
	}
	msleep(150);

	return count;
}
static DEVICE_ATTR_RW(set_ops);

/* gyro_req: arm/disarm the gyro request manager for OIS (write-only). */
static ssize_t gyro_req_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t gyro_req_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int32_t rc;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	g_gyro_req = val ? 1 : 0;
	if (g_s_ctrl) {
		rc = cam_gyro_req_mgr(g_s_ctrl, g_gyro_req);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "cam_gyro_req_mgr failed %d", rc);
			return rc;
		}
	}

	return count;
}
static DEVICE_ATTR_RW(gyro_req);

/* ois_enable: read/write the OIS enable register 0x847f. */
static ssize_t ois_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int32_t rc;
	uint32_t data = 0;

	if (!g_o_ctrl)
		return -ENODEV;

	rc = camera_io_dev_read(&g_o_ctrl->io_master_info, 0x847f, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "ois_enable read failed %d", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t ois_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int32_t rc;
	struct cam_sensor_i2c_reg_array reg_setting;
	struct cam_sensor_i2c_reg_setting write_setting;

	if (!g_o_ctrl)
		return -ENODEV;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	memset(&reg_setting, 0, sizeof(reg_setting));
	reg_setting.reg_addr = 0x847f;
	reg_setting.reg_data = val;

	write_setting.reg_setting = &reg_setting;
	write_setting.size = 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.delay = 0;

	rc = meizu_camera_io_dev_write(&g_o_ctrl->io_master_info,
		&write_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "ois_enable write failed %d", rc);
		return rc;
	}

	return count;
}
static DEVICE_ATTR_RW(ois_enable);

/* ois_cali: trigger/report the OIS factory calibration. */
static ssize_t ois_cali_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", meizu_ois_cali_check());
}

static ssize_t ois_cali_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t rc;

	if (!g_o_ctrl)
		return -ENODEV;

	rc = meizu_ois_cali_exec(g_o_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "meizu_ois_cali_exec failed %d", rc);
		return rc;
	}

	return count;
}
static DEVICE_ATTR_RW(ois_cali);

/* ois_cali_data: report the calibrated gyro offsets. */
static ssize_t ois_cali_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	s16 x = 0, y = 0;

	meizu_ois_cali_get_result(&x, &y);
	return scnprintf(buf, PAGE_SIZE, "%d %d\n", x, y);
}
static DEVICE_ATTR_RO(ois_cali_data);

/*
 * OTP / calibration.  Stock validates a fixed OTP layout (flags + per-section
 * 16-bit big-endian checksums equal to (sum_of_bytes + 1) & 0xffff).  The
 * buffer is fed by the eeprom driver after a successful read.
 */
static u8 *g_otp_buf;
static u32 g_otp_len;

void meizu_cam_register_otp(u8 *buf, u32 len)
{
	g_otp_buf = buf;
	g_otp_len = len;
}
EXPORT_SYMBOL(meizu_cam_register_otp);

static int meizu_otp_section_csum(const u8 *b, u32 start, u32 end,
	u32 cs_hi, u32 cs_lo)
{
	u32 sum = 0;
	u32 i;

	for (i = start; i < end; i++)
		sum += b[i];

	sum = (sum + 1) & 0xffff;

	return (sum == (((u32)b[cs_hi] << 8) | b[cs_lo])) ? 0 : -EINVAL;
}

int meizu_cam_cal_check(u8 *buf, u32 len)
{
	if (!buf)
		return -EINVAL;

	/* sub-module layout (decoded from sub_cam_cal_check) */
	if (len < 0xf70)
		return -EINVAL;

	if (buf[0] != 1 || buf[0x2b] != 1 ||
		buf[0x82] != 1 || buf[0x76d] != 1) {
		CAM_ERR(CAM_EEPROM, "meizu OTP flag check failed");
		return -1;
	}

	if (meizu_otp_section_csum(buf, 0x01, 0x29, 0x29, 0x2a)) {
		CAM_ERR(CAM_EEPROM, "meizu OTP AWB checksum failed");
		return -2;
	}
	if (meizu_otp_section_csum(buf, 0x2c, 0x80, 0x80, 0x81)) {
		CAM_ERR(CAM_EEPROM, "meizu OTP LSC checksum failed");
		return -3;
	}
	if (meizu_otp_section_csum(buf, 0x83, 0x76b, 0x76b, 0x76c)) {
		CAM_ERR(CAM_EEPROM, "meizu OTP module-info checksum failed");
		return -5;
	}
	if (meizu_otp_section_csum(buf, 0x76e, 0xf6e, 0xf6e, 0xf6f)) {
		CAM_ERR(CAM_EEPROM, "meizu OTP crosstalk checksum failed");
		return -8;
	}

	return 0;
}
EXPORT_SYMBOL(meizu_cam_cal_check);

/*
 * Main-camera OTP validation (meizu main_cam_cal_check).  Twelve flag bytes
 * and twelve sections whose checksum is (sum+1)&0xffff; each section's
 * 16-bit big-endian checksum sits immediately before the next flag.
 */
int meizu_cam_cal_check_main(u8 *buf, u32 len)
{
	static const u32 flags[] = {
		0x00, 0x2b, 0x82, 0x99, 0x784, 0xb87,
		0xbde, 0xbf5, 0x12e0, 0x13f3, 0x1bf6, 0x1c17,
	};
	static const u32 sec_start[] = {
		0x01, 0x2c, 0x83, 0x9a, 0x785, 0xb88,
		0xbdf, 0x1000, 0x12e1, 0x13f4, 0x1bf7, 0x1c18,
	};
	static const u32 sec_end[] = {
		0x29, 0x80, 0x97, 0x782, 0xb85, 0xbdc,
		0xbf3, 0x12de, 0x13f1, 0x1bf4, 0x1c15, 0x1c3e,
	};
	u32 i;

	if (!buf)
		return -EINVAL;

	if (len < 0x1c40)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(flags); i++) {
		if (buf[flags[i]] != 1) {
			CAM_ERR(CAM_EEPROM,
				"meizu OTP flag[%u] check failed", i);
			return -1;
		}
	}

	for (i = 0; i < ARRAY_SIZE(sec_start); i++) {
		if (meizu_otp_section_csum(buf, sec_start[i], sec_end[i],
			sec_end[i], sec_end[i] + 1)) {
			CAM_ERR(CAM_EEPROM,
				"meizu OTP main checksum[%u] failed", i);
			return -(int)(i + 2);
		}
	}

	return 0;
}
EXPORT_SYMBOL(meizu_cam_cal_check_main);

static ssize_t cam_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rc;

	if (!g_otp_buf)
		return scnprintf(buf, PAGE_SIZE, "result=fail, otp data is null\n");

	if (g_otp_len >= 0x1c40)
		rc = meizu_cam_cal_check_main(g_otp_buf, g_otp_len);
	else
		rc = meizu_cam_cal_check(g_otp_buf, g_otp_len);
	if (rc)
		return scnprintf(buf, PAGE_SIZE, "result=fail, rc=%d\n", rc);

	return scnprintf(buf, PAGE_SIZE, "result=pass\n");
}
static DEVICE_ATTR_RO(cam_cal);

static ssize_t cam_otp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int n = 0;
	u32 i;

	if (!g_otp_buf)
		return scnprintf(buf, PAGE_SIZE, "otp data is null\n");

	for (i = 0; i < g_otp_len && n < PAGE_SIZE - 4; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "%02x", g_otp_buf[i]);

	n += scnprintf(buf + n, PAGE_SIZE - n, "\n");
	return n;
}
static DEVICE_ATTR_RO(cam_otp);

static struct attribute *meizu_attrs[] = {
	&dev_attr_af_enable.attr,
	&dev_attr_af_pos.attr,
	&dev_attr_set_ops.attr,
	&dev_attr_gyro_req.attr,
	&dev_attr_ois_enable.attr,
	&dev_attr_ois_cali.attr,
	&dev_attr_ois_cali_data.attr,
	&dev_attr_cam_cal.attr,
	&dev_attr_cam_otp.attr,
	NULL,
};

static struct attribute_group meizu_attr_group = {
	.attrs = meizu_attrs,
};

int meizu_cam_init(void)
{
	int rc;

	meizu_class = class_create(THIS_MODULE, "meizu");
	if (IS_ERR(meizu_class)) {
		rc = PTR_ERR(meizu_class);
		CAM_ERR(CAM_SENSOR, "meizu class_create failed %d", rc);
		meizu_class = NULL;
		return rc;
	}

	meizu_dev = device_create(meizu_class, NULL, 0, NULL, "meizu");
	if (IS_ERR(meizu_dev)) {
		rc = PTR_ERR(meizu_dev);
		CAM_ERR(CAM_SENSOR, "meizu device_create failed %d", rc);
		class_destroy(meizu_class);
		meizu_class = NULL;
		meizu_dev = NULL;
		return rc;
	}

	rc = sysfs_create_group(&meizu_dev->kobj, &meizu_attr_group);
	if (rc) {
		CAM_ERR(CAM_SENSOR, "meizu sysfs group failed %d", rc);
		device_destroy(meizu_class, 0);
		class_destroy(meizu_class);
		meizu_class = NULL;
		meizu_dev = NULL;
		return rc;
	}

	CAM_INFO(CAM_SENSOR, "meizu camera class registered");
	return 0;
}

void meizu_cam_exit(void)
{
	if (meizu_dev) {
		sysfs_remove_group(&meizu_dev->kobj, &meizu_attr_group);
		device_destroy(meizu_class, 0);
		meizu_dev = NULL;
	}
	if (meizu_class) {
		class_destroy(meizu_class);
		meizu_class = NULL;
	}
}
