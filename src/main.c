/*
 * SmartRing Air Mouse (智能戒指空中飞鼠 - 官方驱动微码固件版)
 * Hardware: nRF52840 (Nice!Nano / XIAO) + Bosch BMI270 (I2C 地址 0x69)
 * Software: nRF Connect SDK / Zephyr RTOS
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>

LOG_MODULE_REGISTER(air_mouse, LOG_LEVEL_INF);

/* ================== 硬件设备定义 ================== */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const bmi270_dev = DEVICE_DT_GET_ONE(bosch_bmi270);

/* ================== 飞鼠参数调节 ================== */
#define MOUSE_SAMPLE_INTERVAL_MS 15    // 约 66 Hz

/* 1. 动态零偏校准参数 */
#define CALIB_STATIONARY_GYRO_THRESH 0.08f // 静止角速度门限 (rad/s)
#define CALIB_STATIONARY_ACC_THRESH  0.40f // 静止加速度变化门限 (m/s^2)
#define CALIB_STATIONARY_TICKS_REQ   20    // 连续静止 ~300ms 判定为稳定静止
#define CALIB_BIAS_ALPHA             0.02f // 零偏低通平滑跟踪速度

/* 2. 自适应抗手抖低通平滑滤波 (动态 EMA) */
#define EMA_ALPHA_SLOW               0.22f // 慢速瞄准滤波因子 (强滤波，滤除生理微颤)
#define EMA_ALPHA_FAST               0.85f // 快速甩动滤波因子 (低延迟，瞬时跟手)
#define EMA_SPEED_TRANSITION_THRESH  0.55f // 慢速->快速过渡速度阈值 (rad/s)

/* 3. 自适应死区 */
#define GYRO_DEADBAND                0.035f // 死区阈值 (rad/s)

/* 4. 非线性加速度曲线与灵敏度 (Ballistics) */
#define MOUSE_BASE_SENSITIVITY       14.0f // 基础像素灵敏度
#define MOUSE_ACCEL_POWER            1.35f // 指数加速度增益幂次
#define MOUSE_ACCEL_FACTOR           0.80f // 加速度曲线斜率

/* 5. 敲击手势参数 */
#define TAP_JERK_THRESHOLD           13.0f // 敲击瞬时冲击变化率阈值 (m/s^2)
#define TAP_DOUBLE_WINDOW_MS         320   // 双击最大判定时间 (ms)
#define TAP_DEBOUNCE_MS              60
#define TAP_COOLDOWN_MS              180
#define TAP_CLICK_HOLD_MS            45
#define TAP_FREEZE_CURSOR_MS         120   // 敲击瞬间冻结光标防止抖动位移 (ms)

enum tap_state {
	TAP_IDLE,
	TAP_WAIT_SECOND,
	TAP_COOLDOWN
};

/* ================== 蓝牙 HID 鼠标定义 ================== */
#define BASE_USB_HID_SPEC_VERSION 0x0101
#define INPUT_REP_LEN             4
#define INPUT_REP_INDEX           0

BT_HIDS_DEF(hids_obj, INPUT_REP_LEN);
static struct bt_conn *current_conn = NULL;

static const uint8_t hid_report_map[] = {
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
	0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
	0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81,
	0x25, 0x7F, 0x75, 0x08, 0x95, 0x02, 0x81, 0x06, 0x09, 0x38,
	0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
	0xC0, 0xC0
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("[BLE] 配对成功并绑定 (Bonded: %d)\n", bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	printk("[BLE] 配对事件 (reason %d)\n", reason);
}

static void pairing_confirm(struct bt_conn *conn)
{
	int err = bt_conn_auth_pairing_confirm(conn);
	printk("[BLE] 收到 Just Works 配对请求，自动同意确认 (err %d)\n", err);
}

static void passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	int err = bt_conn_auth_passkey_confirm(conn);
	printk("[BLE] 收到数值比对请求 (%06u)，自动确认同意 (err %d)\n", passkey, err);
}

static void passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	printk("[BLE] 配对密钥: %06u\n", passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	printk("[BLE] 配对取消\n");
}

/* 注册完整自动应答认证回调，全自动同意 Windows 的配对要求 */
static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = passkey_display,
	.passkey_confirm = passkey_confirm,
	.pairing_confirm = pairing_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (!err) {
		current_conn = bt_conn_ref(conn);
		int hids_err = bt_hids_connected(&hids_obj, conn);
		if (hids_err) {
			printk("[BLE] HIDS 连接分配失败 (err %d)\n", hids_err);
		}
		gpio_pin_set_dt(&led, 1);
		printk("[BLE] 已连接到主机!\n");
	} else {
		printk("[BLE] 连接失败 (err %u)\n", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("[BLE] 连接断开 (reason %u)\n", reason);
	if (current_conn) {
		bt_hids_disconnected(&hids_obj, current_conn);
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	gpio_pin_set_dt(&led, 0);

	/* 重新开启蓝牙广播，让电脑可以搜索并重新连接 */
	int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err && err != -EALREADY) {
		printk("[BLE] 重新广播失败 (err %d)\n", err);
	} else {
		printk("[BLE] 已重新开启广播，等待连接...\n");
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	printk("[BLE] 安全连接级别变更: level %u (err %d)\n", level, err);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

static void hids_init(void)
{
	struct bt_hids_init_param param = { 0 };
	param.rep_map.data = hid_report_map;
	param.rep_map.size = sizeof(hid_report_map);
	param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	param.info.b_country_code = 0x00;
	param.info.flags = (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

	struct bt_hids_inp_rep *rep = &param.inp_rep_group_init.reports[0];
	rep->size = INPUT_REP_LEN;
	rep->id = 0;
	param.inp_rep_group_init.cnt = 1;
	param.is_mouse = true;

	bt_hids_init(&hids_obj, &param);
}

static void send_mouse_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel)
{
	if (current_conn && bt_conn_get_security(current_conn) >= BT_SECURITY_L2) {
		uint8_t report[4] = { buttons, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel };
		bt_hids_inp_rep_send(&hids_obj, current_conn, INPUT_REP_INDEX, report, sizeof(report), NULL);
	}
}

/* ================== BMI270 官方驱动配置 ================== */
static int init_bmi270(const struct device *dev)
{
	if (!device_is_ready(dev)) {
		printk("ERROR: BMI270 driver device not ready!\n");
		if (dev && dev->state) {
			printk("  [驱动设备名称]: %s\n", dev->name ? dev->name : "NULL");
			printk("  [驱动初始化状态]: initialized=%d, init_res=%d\n", dev->state->initialized, dev->state->init_res);
		}
		const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
		if (device_is_ready(i2c_bus)) {
			uint8_t chip_id = 0, internal_stat = 0, err_reg = 0, pwr_conf = 0, pwr_ctrl = 0;
			i2c_reg_read_byte(i2c_bus, 0x69, 0x00, &chip_id);
			i2c_reg_read_byte(i2c_bus, 0x69, 0x21, &internal_stat);
			i2c_reg_read_byte(i2c_bus, 0x69, 0x02, &err_reg);
			i2c_reg_read_byte(i2c_bus, 0x69, 0x7C, &pwr_conf);
			i2c_reg_read_byte(i2c_bus, 0x69, 0x7D, &pwr_ctrl);
			printk("  [I2C 0x69 诊断]: CHIP_ID=0x%02X, INTERNAL_STAT=0x%02X, ERR_REG=0x%02X, PWR_CONF=0x%02X, PWR_CTRL=0x%02X\n",
			       chip_id, internal_stat, err_reg, pwr_conf, pwr_ctrl);
		}
		return -ENODEV;
	}

	printk("SUCCESS: BMI270 device %s ready! Loading attributes...\n", dev->name);

	struct sensor_value full_scale, sampling_freq, oversampling;

	/* 1. 加速度计配置: ±4G, 100Hz ODR */
	full_scale.val1 = 4;
	full_scale.val2 = 0;
	sampling_freq.val1 = 100;
	sampling_freq.val2 = 0;
	oversampling.val1 = 1;
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

	/* 2. 陀螺仪配置: 2000 dps, 100Hz ODR */
	full_scale.val1 = 2000;
	full_scale.val2 = 0;
	sampling_freq.val1 = 100;
	sampling_freq.val2 = 0;
	oversampling.val1 = 1;
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

	printk("SUCCESS: BMI270 100Hz ODR & power modes activated!\n");
	return 0;
}

static inline double sensor_to_double(const struct sensor_value *val)
{
	return (double)val->val1 + (double)val->val2 / 1000000.0;
}

/* ================== 主程序 ================== */
int main(void)
{
	printk("\n\n==============================================\n");
	printk("   SmartRing Air Mouse (BMI270 驱动微码版)\n");
	printk("==============================================\n");

	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	/* 1. 初始化 BMI270 采样率和量程 */
	int bmi_err = init_bmi270(bmi270_dev);
	if (bmi_err) {
		printk("[WARN] BMI270 init returned %d\n", bmi_err);
	}

	/* 2. 注册安全配对回调 */
	bt_conn_auth_cb_register(&conn_auth_callbacks);
	bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);

	/* 3. 蓝牙与 HID 初始化 */
	hids_init();
	bt_enable(NULL);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* 4. 开始蓝牙广播 */
	bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	printk("[BLE] 广播已开启，设备名: '%s'...\n\n", CONFIG_BT_DEVICE_NAME);

	/* 5. 飞鼠状态与滤波变量定义 */
	uint32_t loop_count = 0;
	uint8_t last_buttons = 0;
	enum tap_state tap_st = TAP_IDLE;
	int32_t tap_window_timer = 0, tap_debounce_timer = 0, tap_cooldown_timer = 0;
	int32_t freeze_cursor_timer = 0, click_release_timer = 0;
	uint8_t gesture_button_state = 0;
	double prev_accel_mag = 9.8;

	/* 动态零偏校准状态 */
	float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
	uint32_t stationary_counter = 0;
	float prev_acc_mag_stationary = 9.8f;

	/* 自适应抗手抖低通平滑状态 */
	float ema_filtered_x = 0.0f;
	float ema_filtered_y = 0.0f;

	/* 亚像素分数累加器 (确保慢速瞄准极度细腻无台阶感) */
	float subpixel_x = 0.0f;
	float subpixel_y = 0.0f;

	struct sensor_value acc_val[3] = { 0 };
	struct sensor_value gyr_val[3] = { 0 };

	while (1) {
		k_msleep(MOUSE_SAMPLE_INTERVAL_MS);
		loop_count++;

		/* 未连蓝牙时 LED 慢闪 */
		if (!current_conn && (loop_count % (500 / MOUSE_SAMPLE_INTERVAL_MS) == 0)) {
			gpio_pin_toggle_dt(&led);
		}

		int8_t mouse_dx = 0, mouse_dy = 0;
		double gx = 0.0, gy = 0.0, gz = 0.0;
		double ax = 0.0, ay = 0.0, az = 0.0;
		double jerk = 0.0;

		/* 从 BMI270 抓取数据 */
		if (sensor_sample_fetch(bmi270_dev) == 0) {
			sensor_channel_get(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, acc_val);
			sensor_channel_get(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, gyr_val);

			ax = sensor_to_double(&acc_val[0]);
			ay = sensor_to_double(&acc_val[1]);
			az = sensor_to_double(&acc_val[2]);

			gx = sensor_to_double(&gyr_val[0]);
			gy = sensor_to_double(&gyr_val[1]);
			gz = sensor_to_double(&gyr_val[2]);

			/* --- 敲击冲击检测 --- */
			double accel_mag = sqrt(ax * ax + ay * ay + az * az);
			jerk = fabs(accel_mag - prev_accel_mag);
			prev_accel_mag = accel_mag;

			bool is_tap_impulse = (jerk > TAP_JERK_THRESHOLD);

			if (tap_debounce_timer > 0) tap_debounce_timer -= MOUSE_SAMPLE_INTERVAL_MS;
			if (tap_cooldown_timer > 0) tap_cooldown_timer -= MOUSE_SAMPLE_INTERVAL_MS;
			if (freeze_cursor_timer > 0) freeze_cursor_timer -= MOUSE_SAMPLE_INTERVAL_MS;
			if (click_release_timer > 0) {
				click_release_timer -= MOUSE_SAMPLE_INTERVAL_MS;
				if (click_release_timer <= 0) gesture_button_state = 0;
			}

			switch (tap_st) {
			case TAP_IDLE:
				if (is_tap_impulse && tap_cooldown_timer <= 0) {
					tap_st = TAP_WAIT_SECOND;
					tap_window_timer = TAP_DOUBLE_WINDOW_MS;
					tap_debounce_timer = TAP_DEBOUNCE_MS;
					freeze_cursor_timer = TAP_FREEZE_CURSOR_MS;
				}
				break;
			case TAP_WAIT_SECOND:
				tap_window_timer -= MOUSE_SAMPLE_INTERVAL_MS;
				if (is_tap_impulse && tap_debounce_timer <= 0) {
					printk(">> Double Tap -> 鼠标右键\n");
					gesture_button_state = 0x02;
					click_release_timer = TAP_CLICK_HOLD_MS;
					tap_cooldown_timer = TAP_COOLDOWN_MS;
					freeze_cursor_timer = TAP_FREEZE_CURSOR_MS;
					tap_st = TAP_COOLDOWN;
				} else if (tap_window_timer <= 0) {
					printk(">> Single Tap -> 鼠标左键\n");
					gesture_button_state = 0x01;
					click_release_timer = TAP_CLICK_HOLD_MS;
					tap_cooldown_timer = TAP_COOLDOWN_MS;
					freeze_cursor_timer = TAP_FREEZE_CURSOR_MS;
					tap_st = TAP_COOLDOWN;
				}
				break;
			case TAP_COOLDOWN:
				if (tap_cooldown_timer <= 0) tap_st = TAP_IDLE;
				break;
			}

			/* --- 1. 动态零偏消除 (Zero-rate Bias Calibration) --- */
			float gyro_mag = sqrtf((float)(gx * gx + gy * gy + gz * gz));
			float acc_mag = (float)accel_mag;
			float acc_delta = fabsf(acc_mag - prev_acc_mag_stationary);
			prev_acc_mag_stationary = acc_mag;

			bool is_stationary = (gyro_mag < CALIB_STATIONARY_GYRO_THRESH) &&
					     (acc_delta < CALIB_STATIONARY_ACC_THRESH);

			if (is_stationary) {
				if (stationary_counter < CALIB_STATIONARY_TICKS_REQ) {
					stationary_counter++;
				} else {
					/* 稳定静止状态，慢速平滑跟踪学习真实零偏 */
					gyro_bias_x += ((float)gx - gyro_bias_x) * CALIB_BIAS_ALPHA;
					gyro_bias_y += ((float)gy - gyro_bias_y) * CALIB_BIAS_ALPHA;
					gyro_bias_z += ((float)gz - gyro_bias_z) * CALIB_BIAS_ALPHA;
				}
			} else {
				stationary_counter = 0;
			}

			/* 扣除零偏 */
			float unbiased_x = (float)gx - gyro_bias_x;
			float unbiased_z = (float)gz - gyro_bias_z;

			/* --- 2. 轴向映射与自适应死区判定 --- */
			float raw_input_x = unbiased_z;
			float raw_input_y = -unbiased_x;
			float input_mag = sqrtf(raw_input_x * raw_input_x + raw_input_y * raw_input_y);

			if (input_mag < GYRO_DEADBAND || is_stationary) {
				raw_input_x = 0.0f;
				raw_input_y = 0.0f;
				input_mag = 0.0f;
			}

			/* --- 3. 自适应抗手抖低通平滑 (动态 EMA 滤波) --- */
			float alpha = EMA_ALPHA_SLOW;
			if (input_mag > EMA_SPEED_TRANSITION_THRESH) {
				alpha = EMA_ALPHA_FAST;
			} else if (input_mag > 0.0f) {
				float ratio = input_mag / EMA_SPEED_TRANSITION_THRESH;
				alpha = EMA_ALPHA_SLOW + ratio * (EMA_ALPHA_FAST - EMA_ALPHA_SLOW);
			}

			ema_filtered_x = ema_filtered_x * (1.0f - alpha) + raw_input_x * alpha;
			ema_filtered_y = ema_filtered_y * (1.0f - alpha) + raw_input_y * alpha;

			/* --- 4. 非线性动力加速度曲线 (Ballistics) 与亚像素累加 --- */
			if (freeze_cursor_timer <= 0) {
				float filtered_speed = sqrtf(ema_filtered_x * ema_filtered_x + ema_filtered_y * ema_filtered_y);

				if (filtered_speed > 0.005f) {
					/* 指数加速乘数：慢速精细瞄准为 ~1.0，快速甩动时指数放大 */
					float accel_mult = 1.0f + MOUSE_ACCEL_FACTOR * powf(filtered_speed, MOUSE_ACCEL_POWER);

					float delta_x_float = ema_filtered_x * MOUSE_BASE_SENSITIVITY * accel_mult;
					float delta_y_float = ema_filtered_y * MOUSE_BASE_SENSITIVITY * accel_mult;

					/* 亚像素积分累加，不丢弃微小位移 */
					subpixel_x += delta_x_float;
					subpixel_y += delta_y_float;

					int step_x = (int)subpixel_x;
					int step_y = (int)subpixel_y;

					subpixel_x -= (float)step_x;
					subpixel_y -= (float)step_y;

					/* 限幅至 int8_t [-127, 127] */
					if (step_x > 127) step_x = 127;
					if (step_x < -127) step_x = -127;
					if (step_y > 127) step_y = 127;
					if (step_y < -127) step_y = -127;

					mouse_dx = (int8_t)step_x;
					mouse_dy = (int8_t)step_y;
				} else {
					subpixel_x = 0.0f;
					subpixel_y = 0.0f;
				}
			} else {
				/* 敲击冻结期间，清空累加器，避免敲击结束瞬间光标跳变 */
				subpixel_x = 0.0f;
				subpixel_y = 0.0f;
				ema_filtered_x = 0.0f;
				ema_filtered_y = 0.0f;
			}
		}

		/* 周期输出到上位机调试器 (约 45ms 刷新一次) */
		if (loop_count % 3 == 0) {
			printk("[BMI] G:%.2f,%.2f,%.2f | A:%.2f,%.2f,%.2f | J:%.2f | M:%d,%d\n",
			       gx, gy, gz, ax, ay, az, jerk, mouse_dx, mouse_dy);
		}

		/* 发送 HID 蓝牙报文 */
		if (current_conn && (mouse_dx != 0 || mouse_dy != 0 || gesture_button_state != last_buttons)) {
			send_mouse_report(gesture_button_state, mouse_dx, mouse_dy, 0);
			last_buttons = gesture_button_state;
		}
	}

	return 0;
}
