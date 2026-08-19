/*
 * main.c — head unit: LSM6DS3TR-C @ 416 Hz -> VQF fusion -> ESB broadcast.
 *
 * Pipeline (PRD F1/F2, PROTOCOL.md Part 2):
 *   IMU DRDY trigger (driver thread) -> 1-slot "latest wins" msgq ->
 *   fusion thread: htk_fusion_update() every sample, transmit every
 *   416/rate-th sample (default 208 Hz) as ORIENT and/or RAW, no-ack.
 *
 * Power states (PROTOCOL.md §2.4): the control loop (main thread) listens
 * for receiver beacons periodically; no beacon for HTK_STANDBY_AFTER_MS
 * drops to STANDBY (IMU down, no streaming, slow listen cadence).
 * Beacons also carry the addressed sensor commands handled here.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <nrfx.h> /* NRF_FICR */

#include "htk_protocol.h"
#include "htk_radio.h"
#include "htk_fusion.h"
#include "htk_tapdet.h"

#include "leds.h"
#include "radio.h"

LOG_MODULE_REGISTER(htk_head, CONFIG_LOG_DEFAULT_LEVEL);

#define IMU_ODR_HZ 416

/* Full scales fixed at build time (prj.conf: CONFIG_LSM6DSL_GYRO_FS=2000,
 * CONFIG_LSM6DSL_ACCEL_FS=4); RAW packets advertise exactly this. */
#define RAW_FS_BYTE HTK_FS_MAKE(HTK_FS_GYRO_2000DPS, HTK_FS_ACC_4G)

/* The Zephyr sensor API only hands out SI units, so RAW packets reconvert
 * SI back to register LSB using the configured full scales (LSM6DS3TR-C
 * datasheet sensitivities: ±2000 dps -> 70 mdps/LSB, ±4 g -> 0.122 mg/LSB).
 * May differ from the true register value by ±1 LSB (round trip). */
#define GYRO_LSB_PER_RAD_S (1.0f / (0.070f * 3.14159265f / 180.0f))
#define ACC_LSB_PER_MS2    (1.0f / (0.000122f * 9.80665f))

struct imu_sample {
	uint32_t t_us;
	float gyr[3]; /* rad/s */
	float acc[3]; /* m/s^2 */
	int16_t rgyr[3];
	int16_t racc[3];
};

/* Shallow queue, depth 4: absorbs ordinary scheduling jitter so samples are
 * not lost to a momentarily busy fusion thread — every silently dropped
 * sample is a fusion under-integration (scale error on fast motion). The
 * "latest wins" overflow policy below is retained purely as protection: a
 * full queue evicts the OLDEST sample, never delays the freshest, and every
 * eviction is counted. */
K_MSGQ_DEFINE(imu_msgq, sizeof(struct imu_sample), 4, 4);
static uint32_t imu_drops;

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));

static uint16_t my_id;
static uint16_t seq; /* fusion thread only; shared ORIENT/RAW space */

/* Volatile per-session settings (PROTOCOL.md §1.7: defaults on reboot). */
static atomic_t g_mode = ATOMIC_INIT(HTK_MODE_QUAT);
static atomic_t g_rate_hz = ATOMIC_INIT(HTK_RATE_DEFAULT);
static atomic_t g_tx_div = ATOMIC_INIT(IMU_ODR_HZ / HTK_RATE_DEFAULT);
static atomic_t g_streaming = ATOMIC_INIT(0);

/* --- Tracker ID (PROTOCOL.md §1.3) ------------------------------------- */

static uint16_t tracker_id(void)
{
	uint16_t id = (uint16_t)(NRF_FICR->DEVICEADDR[0] & 0xFFFF);

	/* 0x0000/0xFFFF are reserved (invalid target / simulator). */
	if (id == HTK_ID_INVALID) {
		id = 0x0001;
	} else if (id == HTK_ID_SIM) {
		id = 0xFFFE;
	}
	return id;
}

/* --- IMU --------------------------------------------------------------- */

static inline int16_t clamp_i16(float v)
{
	if (v > 32767.0f) {
		return INT16_MAX;
	}
	if (v < -32768.0f) {
		return INT16_MIN;
	}
	return (int16_t)v;
}

/* DRDY handler: runs in the lsm6dsl driver's own thread
 * (CONFIG_LSM6DSL_TRIGGER_OWN_THREAD), priority above the fusion thread. */
static void imu_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	ARG_UNUSED(trig);

	struct imu_sample s;
	struct sensor_value g[3], a[3];

	/* Timestamp at DRDY, before the I2C fetch (~0.2 ms at 400 kHz),
	 * so t_us tracks sample time, not bus time. */
	s.t_us = k_ticks_to_us_floor32(k_uptime_ticks());

	if (sensor_sample_fetch(dev) != 0) {
		return;
	}
	if (sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, g) != 0 ||
	    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, a) != 0) {
		return;
	}

	for (int i = 0; i < 3; i++) {
		s.gyr[i] = sensor_value_to_float(&g[i]); /* rad/s */
		s.acc[i] = sensor_value_to_float(&a[i]); /* m/s^2 */
		s.rgyr[i] = clamp_i16(s.gyr[i] * GYRO_LSB_PER_RAD_S);
		s.racc[i] = clamp_i16(s.acc[i] * ACC_LSB_PER_MS2);
	}

	/* Latest wins: evict the oldest unconsumed sample, never block DRDY. */
	if (k_msgq_put(&imu_msgq, &s, K_NO_WAIT) != 0) {
		struct imu_sample stale;

		(void)k_msgq_get(&imu_msgq, &stale, K_NO_WAIT);
		(void)k_msgq_put(&imu_msgq, &s, K_NO_WAIT);
		imu_drops++;
	}
}

/* hz = 0 powers the sensor down (standby).
 * VERIFY: lsm6dsl driver accepts 0 Hz via SENSOR_ATTR_SAMPLING_FREQUENCY as
 * power-down; if not, fall back to the lowest ODR or PM device actions. */
static int imu_set_odr(uint16_t hz)
{
	const struct sensor_value v = { .val1 = hz, .val2 = 0 };
	int err;

	err = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (!err) {
		err = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
				      SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	}
	return err;
}

/* The IMU sits on an always-on rail, so its state survives soft reboot and
 * the lsm6dsl driver's boot-time init fails on a non-fresh chip (Zephyr
 * issue #55892 class; observed as "Failed to initialize chip" -> -ENODEV on
 * every warm boot). The driver exposes no reset, so: raw-I2C SW_RESET the
 * chip, then re-run the driver's init with device_init(). */
#define LSM6_REG_CTRL3_C 0x12u
#define LSM6_SW_RESET    BIT(0)

static int imu_force_reset(void)
{
	const struct device *bus =
		DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6ds3tr_c)));
	const uint16_t addr = DT_REG_ADDR(DT_NODELABEL(lsm6ds3tr_c));
	uint8_t v;
	int err;

	if (!device_is_ready(bus)) {
		return -ENODEV;
	}

	/* A mid-transaction reset can leave the sensor holding SDA; clocking
	 * the bus out first makes the SW_RESET write reachable. */
	(void)i2c_recover_bus(bus);

	err = i2c_reg_write_byte(bus, addr, LSM6_REG_CTRL3_C, LSM6_SW_RESET);
	if (err) {
		LOG_WRN("IMU SW_RESET write failed: %d", err);
		return err;
	}

	/* SW_RESET self-clears in ~50 us */
	for (int i = 0; i < 100; i++) {
		k_busy_wait(50);
		err = i2c_reg_read_byte(bus, addr, LSM6_REG_CTRL3_C, &v);
		if (err == 0 && (v & LSM6_SW_RESET) == 0) {
			return 0;
		}
	}
	return -ETIMEDOUT;
}

static int imu_init(void)
{
	/* Driver init is deferred (see boards overlay): the chip must be
	 * SW_RESET to its cold-boot state BEFORE the driver's first init,
	 * because the always-on rail preserves sensor state across MCU
	 * resets and the driver chokes on a non-fresh chip. */
	for (int attempt = 0; !device_is_ready(imu) && attempt < 3; attempt++) {
		int err = imu_force_reset();

		if (err) {
			LOG_WRN("IMU reset failed (%d), try %d", err, attempt + 1);
		}
		k_sleep(K_MSEC(20));
		err = device_init(imu);
		if (err) {
			LOG_WRN("IMU driver init: %d (try %d)", err, attempt + 1);
			k_sleep(K_MSEC(50));
		}
	}
	if (!device_is_ready(imu)) {
		LOG_ERR("IMU not ready");
		return -ENODEV;
	}

	int err = imu_set_odr(IMU_ODR_HZ);

	if (err) {
		LOG_ERR("IMU ODR set: %d", err);
		return err;
	}

	const struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		/* VERIFY: lsm6dsl trigger registers DRDY on the accel
		 * channel; gyro data is read in the same fetch. */
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	err = sensor_trigger_set(imu, &trig, imu_drdy);
	if (err) {
		LOG_ERR("IMU trigger set: %d", err);
	}
	return err;
}

/* --- Fusion + TX thread ------------------------------------------------ */

static void fusion_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct imu_sample s;
	float q[4];
	uint32_t decim = 0;
	struct htk_tapdet tapdet;
	int64_t tap_until_ms = 0;

	/* True-ODR measurement: the LSM6DS3TR-C clocks its ODR from an internal
	 * oscillator, so "416 Hz" can be off by a few percent — a systematic
	 * fusion scale error (a 90 deg turn reading 87 deg, forever). Log-only
	 * for now; the number decides whether a rate-corrected VQF is worth it. */
	uint32_t odr_prev_t = 0;
	uint64_t odr_sum_us = 0;
	uint32_t odr_n = 0;
	bool odr_have_prev = false;

	htk_tapdet_init(&tapdet, (float)IMU_ODR_HZ, NULL);

	for (;;) {
		k_msgq_get(&imu_msgq, &s, K_FOREVER);

		if (odr_have_prev) {
			odr_sum_us += (uint32_t)(s.t_us - odr_prev_t);
			if (++odr_n == 4096) {
				/* centi-Hz without floats in the log path */
				uint32_t chz = (uint32_t)(4096ull * 100000000ull /
							  odr_sum_us);
				LOG_INF("measured ODR %u.%02u Hz, drops %u",
					chz / 100, chz % 100, imu_drops);

				/* One-time rate correction: internal-oscillator
				 * ODR error is a systematic fusion scale error
				 * (measured 431.56 Hz on unit 1 = 3.7 %). */
				static bool rate_corrected;

				if (!rate_corrected &&
				    (chz < 41392u || chz > 41808u)) {
					htk_fusion_set_sample_rate(
						(float)chz / 100.0f);
					rate_corrected = true;
					LOG_INF("fusion rate corrected to "
						"%u.%02u Hz", chz / 100,
						chz % 100);
				}
				odr_sum_us = 0;
				odr_n = 0;

				/* rest-gate diagnostics each window (~10 s):
				 * which condition blocks arming, in milli-units
				 * to keep floats out of the log path */
				struct htk_fusion_debug fd;

				htk_fusion_get_debug(&fd);
				LOG_INF("fusion: sigma=%d bcmax=%d vert=%d "
					"accdev=%d mdps/m; vqfrest=%u%% fail=%u "
					"spk1=%u spk8=%u",
					(int)(fd.sigma_dps * 1000.0f),
					(int)(fd.bc_norm_max_dps * 1000.0f),
					(int)(fd.vert_max_dps * 1000.0f),
					(int)(fd.acc_dev_max * 1000.0f),
					fd.vqf_rest_pct, fd.gate_fail_n,
					fd.spikes_1dps, fd.spikes_8dps);
				LOG_INF("coarse offset mdps: %d %d %d",
					(int)(fd.coarse_dps[0] * 1000.0f),
					(int)(fd.coarse_dps[1] * 1000.0f),
					(int)(fd.coarse_dps[2] * 1000.0f));
			}
		}
		odr_prev_t = s.t_us;
		odr_have_prev = true;

		/* Tap first: the gesture must be seen even if fusion is
		 * mid-convergence, and it only reads |acc|. */
		if (htk_tapdet_feed(&tapdet, s.t_us, s.acc)) {
			tap_until_ms = k_uptime_get() + 250;
			leds_tap();
			LOG_INF("double tap");
		}

		/* Fuse every sample at full ODR — decimation applies only
		 * to transmission, so the filter sees all the data. */
		htk_fusion_update(s.gyr, s.acc, s.t_us, q);

		if (!atomic_get(&g_streaming)) {
			decim = 0;
			continue;
		}

		if (++decim < (uint32_t)atomic_get(&g_tx_div)) {
			continue;
		}
		decim = 0;

		uint8_t mode = (uint8_t)atomic_get(&g_mode);
		uint8_t flags = 0; /* HW_FUSION=0: software VQF */
		const bool rest = htk_fusion_rest();

		if (htk_fusion_bias_ok()) {
			flags |= HTK_ORIENT_BIAS_OK;
		}
		if (rest) {
			flags |= HTK_ORIENT_REST;
		}
		leds_set_rest(rest); /* streaming blink: green moving, cyan held */
		if (k_uptime_get() < tap_until_ms) {
			flags |= HTK_ORIENT_TAP;
		}

		if (mode == HTK_MODE_QUAT || mode == HTK_MODE_BOTH) {
			struct htk_orient o = {
				.type = HTK_PKT_ORIENT,
				.id = my_id,
				.seq = seq++,
				.t_us = s.t_us,
				.q_w = q[0], .q_x = q[1],
				.q_y = q[2], .q_z = q[3],
				.flags = flags,
			};

			(void)htk_radio_send(&o, sizeof(o));
		}

		if (mode == HTK_MODE_RAW || mode == HTK_MODE_BOTH) {
			struct htk_raw r = {
				.type = HTK_PKT_RAW,
				.id = my_id,
				.seq = seq++,
				.t_us = s.t_us,
				.fs = RAW_FS_BYTE,
			};

			memcpy(r.gyr, s.rgyr, sizeof(r.gyr));
			memcpy(r.acc, s.racc, sizeof(r.acc));
			(void)htk_radio_send(&r, sizeof(r));
		}
	}
}

/* Below the IMU trigger thread (prio 4, prj.conf) so DRDY always lands in
 * the queue; above nothing else that matters — main mostly sleeps. */
K_THREAD_DEFINE(fusion_tid, 3072, fusion_thread, NULL, NULL, NULL, 6, 0, 0);

/* --- Command handling (from beacon payloads) --------------------------- */

static void set_rate(uint16_t hz)
{
	/* Clamp to nearest supported divider of the 416 Hz ODR. */
	static const uint16_t rates[] = { 52, 104, 208, 416 };
	uint16_t best = rates[0];
	int best_d = INT32_MAX;

	for (size_t i = 0; i < ARRAY_SIZE(rates); i++) {
		int d = (hz > rates[i]) ? (hz - rates[i]) : (rates[i] - hz);

		if (d < best_d) {
			best_d = d;
			best = rates[i];
		}
	}

	atomic_set(&g_rate_hz, best);
	atomic_set(&g_tx_div, IMU_ODR_HZ / best);
	LOG_INF("rate -> %u Hz", best);
}

/* One beacon: htk_beacon_hdr, optionally followed by exactly one embedded
 * command payload (type + body, §2.3). Commands are idempotent and repeated
 * across beacons by design — no dedup needed. Never broadcast: anything not
 * addressed to my_id is ignored. Unknown types skipped (forward compat). */
static void handle_beacon(const struct htk_beacon_rx *b)
{
	const uint8_t *cmd = b->data + sizeof(struct htk_beacon_hdr);
	size_t len = b->len - sizeof(struct htk_beacon_hdr);

	if (len == 0) {
		return; /* presence-only beacon */
	}

	/* memcpy into packed locals: unaligned-safe, and the wire is
	 * little-endian like the CPU. */
	switch (cmd[0]) {
	case HTK_PKT_RESET_FUSION: {
		struct htk_cmd_id c;

		if (len < sizeof(c)) {
			return;
		}
		memcpy(&c, cmd, sizeof(c));
		if (c.id != my_id) {
			return;
		}
		htk_fusion_reset();
		LOG_INF("fusion reset");
		break;
	}
	case HTK_PKT_SET_RATE: {
		struct htk_set_rate c;

		if (len < sizeof(c)) {
			return;
		}
		memcpy(&c, cmd, sizeof(c));
		if (c.id != my_id) {
			return;
		}
		set_rate(c.hz);
		break;
	}
	case HTK_PKT_SET_MODE: {
		struct htk_set_mode c;

		if (len < sizeof(c)) {
			return;
		}
		memcpy(&c, cmd, sizeof(c));
		if (c.id != my_id || c.mode > HTK_MODE_BOTH) {
			return;
		}
		atomic_set(&g_mode, c.mode);
		LOG_INF("mode -> %u", c.mode);
		break;
	}
	case HTK_PKT_IDENTIFY: {
		struct htk_cmd_id c;

		if (len < sizeof(c)) {
			return;
		}
		memcpy(&c, cmd, sizeof(c));
		if (c.id != my_id) {
			return;
		}
		leds_identify();
		break;
	}
	default:
		break;
	}
}

/* --- 1 Hz self-report -------------------------------------------------- */

static void send_tstatus(void)
{
	/* BRING-UP DEBUG: vbat_mV temporarily carries radio RX diagnostics
	 * (no battery ADC yet, field reads 0 = unknown otherwise). Encoding:
	 * high byte = listen-window count (bit7 forced if any cfg failure),
	 * low byte = RX events seen. Revert once the beacon path is proven. */
	struct htk_radio_debug rd;

	htk_radio_get_debug(&rd);

	struct htk_tstatus ts = {
		.type = HTK_PKT_TSTATUS,
		.id = my_id,
		.vbat_mV = (uint16_t)((((rd.cfg_fail ? 0x80u : 0u) |
					(rd.listens & 0x7Fu)) << 8) |
				      (rd.rx_events & 0xFFu)),
		.fw_major = HTK_FW_MAJOR,
		.fw_minor = HTK_FW_MINOR,
		.fw_patch = HTK_FW_PATCH,
		.mode = (uint8_t)atomic_get(&g_mode),
		.rate = (uint16_t)atomic_get(&g_rate_hz),
		.uptime_s = (uint16_t)(k_uptime_get() / 1000),
	};

	(void)htk_radio_send(&ts, sizeof(ts));
}

/* --- Power-state machine (control loop, PROTOCOL.md §2.4) -------------- */

enum run_state {
	ST_ACTIVE,  /* beacon within T_hold: stream, listen every ~1 s */
	ST_STANDBY, /* no receiver: IMU down, listen every ~5 s */
};

static void enter_active(void)
{
	(void)imu_set_odr(IMU_ODR_HZ);
	htk_fusion_resume(); /* no-op at boot; restores bias after standby */
	atomic_set(&g_streaming, 1);
	leds_set_link(LEDS_LINK_STREAMING);
	LOG_INF("ACTIVE");
}

static void enter_standby(void)
{
	atomic_set(&g_streaming, 0);
	htk_fusion_suspend(); /* save the converged gyro bias */
	(void)imu_set_odr(0);
	leds_set_link(LEDS_LINK_SEARCHING);
	LOG_INF("STANDBY");
	/* TODO(DEEP, optional): arm LSM6DS3TR-C wake-on-motion and stretch
	 * the listen interval further when also motionless (§2.4). */
}

int main(void)
{
	enum run_state state = ST_ACTIVE;
	/* Optimistic start: stream from boot so a live receiver picks us up
	 * immediately; drops to STANDBY after T_hold if nobody is there. */
	int64_t last_beacon = k_uptime_get();
	int64_t last_tstatus = 0;
	int err;

	my_id = tracker_id();
	LOG_INF("head unit id=0x%04x fw=%u.%u.%u", my_id,
		HTK_FW_MAJOR, HTK_FW_MINOR, HTK_FW_PATCH);

	err = leds_init();
	if (err) {
		LOG_ERR("leds_init: %d", err);
	}

	err = htk_radio_init();
	if (err) {
		LOG_ERR("radio init failed: %d — halting", err);
		return err; /* no radio, no product; LED stays searching */
	}

	/* Fusion must be ready before the first DRDY fires. */
	htk_fusion_init((float)IMU_ODR_HZ);

	err = imu_init();
	if (err) {
		LOG_ERR("IMU init failed: %d — halting", err);
		return err;
	}

	enter_active();

	for (;;) {
		uint32_t period_ms = (state == ST_ACTIVE) ?
			HTK_LISTEN_PERIOD_ACTIVE_MS : HTK_LISTEN_PERIOD_STANDBY_MS;

		k_sleep(K_MSEC(period_ms));

		/* Radio lock is held for the whole window; concurrent
		 * htk_radio_send() calls drop their sample (by design). */
		htk_radio_listen(HTK_LISTEN_WINDOW_MS);

		bool heard = false;
		struct htk_beacon_rx b;

		while (htk_radio_beacon_get(&b)) {
			heard = true; /* any beacon = a receiver is present */
			handle_beacon(&b);
		}

		/* Bring-up diagnostics (visible on the debug_usb console build). */
		struct htk_radio_debug rd;

		htk_radio_get_debug(&rd);
		LOG_INF("listen #%u: rx_events=%u beacons=%u cfg_fail=%u heard=%d",
			rd.listens, rd.rx_events, rd.beacons_queued, rd.cfg_fail,
			(int)heard);

		int64_t now = k_uptime_get();

		if (heard) {
			last_beacon = now;
		}

		if (state == ST_ACTIVE &&
		    (now - last_beacon) >= HTK_STANDBY_AFTER_MS) {
			state = ST_STANDBY;
			enter_standby();
		} else if (state == ST_STANDBY && heard) {
			state = ST_ACTIVE;
			enter_active();
		}

		/* ACTIVE loop period is ~1.1 s, so this lands at ~1 Hz. */
		if (state == ST_ACTIVE && (now - last_tstatus) >= 1000) {
			send_tstatus();
			last_tstatus = now;
		}
	}

	return 0;
}
