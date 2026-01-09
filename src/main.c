#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* BLE UUID: 00002222-0000-1000-8000-00805f9b34fb (LSB order for advertising) */
static const uint8_t uuid_custom_service[] = {
	0xfb, 0x34, 0x9b, 0x5f,
	0x80, 0x00, 0x00, 0x80,
	0x00, 0x10, 0x00, 0x00,
	0x22, 0x22, 0x00, 0x00
};

/* Advertising data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_UUID128_ALL, uuid_custom_service, sizeof(uuid_custom_service)),
};

/* Scan response data: include name so Android shows it */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* LEDs (these aliases exist on Nordic DKs) */
static const struct gpio_dt_spec leds[] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios),
};
#define NUM_LEDS ARRAY_SIZE(leds)

/* Buttons: sw0/sw1/sw2 aliases (portable across Nordic DKs) */
static const struct gpio_dt_spec btn_start  = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios); /* SW0 */
static const struct gpio_dt_spec btn_stop   = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios); /* SW1 */
static const struct gpio_dt_spec btn_toggle = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios); /* SW2 */

static struct gpio_callback btn_start_cb;
static struct gpio_callback btn_stop_cb;
static struct gpio_callback btn_toggle_cb;

static struct bt_conn *current_conn;

static volatile bool start_pressed;
static volatile bool stop_pressed;
static volatile bool toggle_pressed;

static bool want_advertising;
static bool adv_is_running;

/* Address mode:
 * true  -> RPA rotating (privacy)
 * false -> stable identity address (stable “MAC”)
 */
static bool use_rotating_rpa = true;

static void addr_to_str(const bt_addr_le_t *addr, char *out, size_t out_len)
{
	if (!addr || !out || out_len == 0) {
		return;
	}
	bt_addr_le_to_str(addr, out, out_len);
}

static void leds_all_off(void)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		gpio_pin_set_dt(&leds[i], 0);
	}
}

/* LED0: advertising, LED1: connected, LED2: RPA mode, LED3: passkey indicator */
static void leds_update(void)
{
	gpio_pin_set_dt(&leds[0], adv_is_running ? 1 : 0);
	gpio_pin_set_dt(&leds[1], current_conn ? 1 : 0);
	gpio_pin_set_dt(&leds[2], use_rotating_rpa ? 1 : 0);
	/* LED3 controlled by passkey / auth callbacks */
}

/* ---- Button ISRs ---- */
static void isr_start(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
	start_pressed = true;
}

static void isr_stop(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
	stop_pressed = true;
}

static void isr_toggle(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
	toggle_pressed = true;
}

/* ---- Advertising ---- */
static int adv_stop(void)
{
	int err = bt_le_adv_stop();
	if (err) {
		LOG_WRN("bt_le_adv_stop err %d", err);
	} else {
		LOG_INF("Advertising stopped");
	}
	adv_is_running = false;
	leds_update();
	return err;
}

static int adv_start(bool rotating_rpa)
{
	if (current_conn) {
		LOG_INF("Already connected; not starting advertising");
		return 0;
	}

	if (adv_is_running) {
		LOG_INF("Already advertising; not restarting");
		return 0;
	}

	struct bt_le_adv_param param = {
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.secondary_max_skip = 0,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};

	if (!rotating_rpa) {
		param.options |= BT_LE_ADV_OPT_USE_IDENTITY;
	}

	int err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		if (err == -EALREADY) {
			LOG_INF("Advertising already running (EALREADY)");
			adv_is_running = true;
			leds_update();
			return 0;
		}
		LOG_ERR("Advertising start failed (err %d)", err);
		return err;
	}

	adv_is_running = true;
	LOG_INF("Advertising started (%s), name=%s",
		rotating_rpa ? "RPA rotating" : "Stable identity",
		CONFIG_BT_DEVICE_NAME);

	leds_update();
	return 0;
}

/* ---- Connection callbacks ---- */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));
	LOG_INF("Connected: %s", peer);

	current_conn = bt_conn_ref(conn);

	/* For connectable advertising, controller stops advertising when connected */
	adv_is_running = false;
	leds_update();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};

	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));
	LOG_INF("Disconnected: %s (reason %u)", peer, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	gpio_pin_set_dt(&leds[3], 0); /* Clear passkey indicator */
	leds_update();

	if (want_advertising) {
		LOG_INF("Resuming advertising (user requested)");
		adv_start(use_rotating_rpa);
	}
}

static void security_changed(struct bt_conn *conn,
			     bt_security_t level,
			     enum bt_security_err err)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};
	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));

	/* level: L1..L4, err: 0 == success */
	LOG_INF("Security changed: %s level=%u err=%u", peer, level, err);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

/* ---- Pairing/Bonding ----
 * Key change: implement passkey_display to allow MITM-capable pairing.
 * Android will show a passkey entry UI; you enter the printed passkey on the phone.
 */

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};
	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));

	/* This is the critical line for debugging + user entry on Android */
	LOG_INF("Passkey for %s: %06u", peer, passkey);

	/* Visual indicator: LED3 on while passkey is relevant */
	gpio_pin_set_dt(&leds[3], 1);
}

/* For some methods, Zephyr will ask you to confirm “Just Works” / numeric comparison.
 * We keep pairing_confirm to allow “Just Works” when MITM isn't required.
 */
static void pairing_confirm(struct bt_conn *conn)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};
	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));

	LOG_INF("Pairing confirm for %s -> accepting", peer);
	bt_conn_auth_pairing_confirm(conn);
}

static void auth_cancel(struct bt_conn *conn)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};
	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));

	LOG_WRN("Pairing cancelled: %s", peer);
	gpio_pin_set_dt(&leds[3], 0);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};
	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));

	LOG_INF("Pairing complete: %s (bonded=%d)", peer, bonded);
	gpio_pin_set_dt(&leds[3], 0);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char peer[BT_ADDR_LE_STR_LEN] = {0};
	addr_to_str(bt_conn_get_dst(conn), peer, sizeof(peer));

	LOG_ERR("Pairing failed: %s (reason %d)", peer, reason);
	gpio_pin_set_dt(&leds[3], 0);
}

static struct bt_conn_auth_cb auth_cb = {
	.passkey_display = auth_passkey_display,
	.pairing_confirm = pairing_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* ---- Init ---- */
static int init_leds(void)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		if (!device_is_ready(leds[i].port)) {
			LOG_ERR("LED %d not ready", i);
			return -ENODEV;
		}
		gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
	}
	return 0;
}

static int init_buttons(void)
{
	if (!device_is_ready(btn_start.port) ||
	    !device_is_ready(btn_stop.port) ||
	    !device_is_ready(btn_toggle.port)) {
		LOG_ERR("Button port not ready");
		return -ENODEV;
	}

	/* Pull-ups are important on DK buttons */
	gpio_pin_configure_dt(&btn_start,  GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_configure_dt(&btn_stop,   GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_configure_dt(&btn_toggle, GPIO_INPUT | GPIO_PULL_UP);

	gpio_pin_interrupt_configure_dt(&btn_start,  GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&btn_stop,   GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&btn_toggle, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&btn_start_cb, isr_start, BIT(btn_start.pin));
	gpio_add_callback(btn_start.port, &btn_start_cb);

	gpio_init_callback(&btn_stop_cb, isr_stop, BIT(btn_stop.pin));
	gpio_add_callback(btn_stop.port, &btn_stop_cb);

	gpio_init_callback(&btn_toggle_cb, isr_toggle, BIT(btn_toggle.pin));
	gpio_add_callback(btn_toggle.port, &btn_toggle_cb);

	LOG_INF("Buttons initialized (sw0/sw1/sw2, pull-ups enabled)");
	return 0;
}

int main(void)
{
	int err;

	LOG_INF("Booting...");
	LOG_INF("=== FW: SW0=start adv, SW1=stop/disconnect, SW2=toggle RPA ===");
	LOG_INF("=== Pairing: if Android requests MITM, enter passkey printed in logs ===");

	err = init_leds();
	if (err) {
		return 0;
	}

	err = init_buttons();
	if (err) {
		return 0;
	}

	leds_all_off();
	leds_update();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	bt_conn_cb_register(&conn_callbacks);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
		LOG_INF("Settings loaded");
	}

	bt_conn_auth_cb_register(&auth_cb);
	bt_conn_auth_info_cb_register(&auth_info_cb);

	/* Bondable so Android can store keys */
	bt_set_bondable(true);

	LOG_INF("Ready. Device name=%s", CONFIG_BT_DEVICE_NAME);
	leds_update();

	while (1) {
		if (start_pressed) {
			start_pressed = false;
			LOG_INF("SW0 pressed -> start advertising");
			want_advertising = true;
			adv_start(use_rotating_rpa);
		}

		if (stop_pressed) {
			stop_pressed = false;
			LOG_INF("SW1 pressed -> stop/disconnect");
			want_advertising = false;

			if (current_conn) {
				bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			} else {
				adv_stop();
			}
		}

		if (toggle_pressed) {
			toggle_pressed = false;
			use_rotating_rpa = !use_rotating_rpa;
			LOG_INF("SW2 pressed -> mode=%s",
				use_rotating_rpa ? "RPA rotating" : "Stable identity");

			/* If currently advertising, restart to apply new mode */
			if (adv_is_running) {
				adv_stop();
				adv_start(use_rotating_rpa);
			}
			leds_update();
		}

		k_msleep(20);
	}

	return 0;
}
