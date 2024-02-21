/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

static void start_scan(void);

static struct bt_conn *default_conn;

#define UUID128_BYTE_LEN  (128/8)

static struct bt_uuid_128 discover_uuid = BT_UUID_INIT_128(0);

#define bmsServiceUuid BT_UUID_DECLARE_16(0xFF00)
#define bmsCharRdUuid BT_UUID_DECLARE_16(0xFF01)
#define bmsCharWrUuid BT_UUID_DECLARE_16(0xFF02)

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params[2];

static uint8_t buf[130];
static unsigned bufIdx;

static K_SEM_DEFINE(sem_subs, 0, 1);

static void hexPrint(uint8_t const * data, unsigned length) {
	while (length) {
		printk(" %02x", *data);
		++data;
		--length;
	}
	printk("\n");
}

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u:", data, length);
	hexPrint(data, length);

	uint8_t const* dat = data;
	if ((*dat == 0xDD) && (bufIdx == 0)) { /* probably start */
		memcpy(buf, data, length);
		bufIdx = length;
	} else if ( bufIdx != 0 ) { /* Probably second part */
		memcpy(&buf[bufIdx], data, length);
		bufIdx += length;
		k_sem_give(&sem_subs);
	} // else ignore
	return BT_GATT_ITER_CONTINUE;
}
static struct {
	uint8_t  cnt;
	uint16_t handles[2];
	uint16_t valueHandles[2];
} subscHandles = {0};

static uint16_t writeHandle;

static void dumpParams(const struct bt_gatt_discover_params* params)
{
  char uuidStr[50];
  bt_uuid_to_str(params->uuid, uuidStr, 50);


  printk ("Params: start: %u, end: %u, cb: %p, uuid: %s, type: %d\n",
		  params->start_handle,
		  params->end_handle,
		  params->func,
		  uuidStr,
		  params->type);
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_GATT)) {
		char uuidStr[32];

		bt_uuid_to_str(discover_params.uuid, uuidStr, 32);
		printk("-> %s (service)\n", uuidStr);

		memcpy(&discover_uuid, BT_UUID_GATT_SC, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		dumpParams(&discover_params);
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}

	} else if (!bt_uuid_cmp(discover_params.uuid,  BT_UUID_GATT_SC)) {
		char uuidStr[32];

		bt_uuid_to_str(discover_params.uuid, uuidStr, 32);
		printk("-> %s (characteristic)\n", uuidStr);

		memcpy(&discover_uuid, BT_UUID_GATT_CCC, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

		struct bt_gatt_chrc* chrc = (struct bt_gatt_chrc*)attr->user_data;
		printk("properties: 0x%02x, value_hdl: %u\n", chrc->properties, chrc-> value_handle);
		subscHandles.valueHandles[subscHandles.cnt] = bt_gatt_attr_value_handle(attr);
//		subscHandles.handles[subscHandles.cnt] = attr->handle;
//		subscHandles.cnt++;

		dumpParams(&discover_params);
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,  BT_UUID_GATT_CCC)) {
		char uuidStr[32];

		subscHandles.handles[subscHandles.cnt] = attr->handle;
		subscHandles.cnt++;

		bt_uuid_to_str(discover_params.uuid, uuidStr, 32);
		printk("-> %s (descriptor)\n", uuidStr);

		if (subscHandles.cnt == 1) {
			memcpy(&discover_uuid, bmsServiceUuid, sizeof(discover_uuid));
			discover_params.uuid = &discover_uuid.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_PRIMARY;

			dumpParams(&discover_params);
			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
			}
		} else {
			memcpy(&discover_uuid, bmsCharWrUuid, sizeof(discover_uuid));
			discover_params.uuid = &discover_uuid.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			dumpParams(&discover_params);
			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
			}

		}
	} else if (!bt_uuid_cmp(discover_params.uuid, bmsServiceUuid)) {

		char uuidStr[32];

		bt_uuid_to_str(discover_params.uuid, uuidStr, 32);
		printk("-> %s (service)\n", uuidStr);

		memcpy(&discover_uuid, bmsCharRdUuid, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		dumpParams(&discover_params);
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, bmsCharRdUuid)) {
		char uuidStr[32];

		bt_uuid_to_str(discover_params.uuid, uuidStr, 32);
		printk("-> %s (characteristic)\n", uuidStr);

		memcpy(&discover_uuid, BT_UUID_GATT_CCC, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

		struct bt_gatt_chrc* chrc = (struct bt_gatt_chrc*)attr->user_data;
		printk("properties: 0x%02x, value_hdl: %u\n", chrc->properties, chrc-> value_handle);
		subscHandles.valueHandles[subscHandles.cnt] = bt_gatt_attr_value_handle(attr);
		//subscHandles.handles[subscHandles.cnt] = attr->handle;
		//subscHandles.cnt++;

		dumpParams(&discover_params);
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, bmsCharWrUuid)) {
		char uuidStr[32];

		bt_uuid_to_str(discover_params.uuid, uuidStr, 32);
		printk("-> %s (characteristic)\n", uuidStr);

		memcpy(&discover_uuid, BT_UUID_GATT_CUD, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

		struct bt_gatt_chrc* chrc = (struct bt_gatt_chrc*)attr->user_data;
		printk("properties: 0x%02x, value_hdl: %u\n", chrc->properties, chrc-> value_handle);
		writeHandle = bt_gatt_attr_value_handle(attr);

		dumpParams(&discover_params);
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}

	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_GATT_CUD)) {
		char uuidStr[32];
		bt_uuid_to_str(discover_params.uuid, uuidStr, 32);
		printk("-> %s (characteristics)\n", uuidStr);
	}

	if (subscHandles.cnt == 2) {
		for (unsigned i = 0; i < subscHandles.cnt; i++) {
			printk(" Start subscription handle handle: %d, value_handle %d\n", 
					subscHandles.handles[i], 
					subscHandles.valueHandles[i]);
			//subscribe_params[i].end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;

			subscribe_params[i].notify = notify_func;
			subscribe_params[i].value = BT_GATT_CCC_NOTIFY; //BT_GATT_CCC_INDICATE; // What's the difference
			subscribe_params[i].ccc_handle = subscHandles.handles[i];
			subscribe_params[i].value_handle = subscHandles.valueHandles[i];
			err = bt_gatt_subscribe(conn, &subscribe_params[i]);
			if (err && err != -EALREADY) {
				printk("Subscribe failed (err %d)\n", err);
			} else {
				printk("[SUBSCRIBED handle %d] \n", subscribe_params[i].value_handle);
			}
		}
		subscHandles.cnt = 0;
	}

	return BT_GATT_ITER_STOP;
}

static struct {
	enum {
		STATUS_TESTER,
		STATUS_CANDIDATE,
		STATUS_CHOSEN
	} status;
	bt_addr_le_t addr;
} advDev = { .status = STATUS_TESTER };

static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;
	bool candidate = false;

	printk("[AD]: type %u, data_len %u:", data->type, data->data_len);
	hexPrint(data->data, data->data_len);

	switch (data->type) {
	case BT_DATA_FLAGS:
		break;

	case BT_DATA_TX_POWER:
		break;

	case BT_DATA_NAME_COMPLETE:
		if (data->data_len == 0) {
			printk("Bad name len\n");
		} else {
			unsigned dataLen = data->data_len;
			uint8_t* dat = data->data;
			printk("name: ");
			while (dataLen) {
				printk("%c", *dat);
				--dataLen;
				++dat;
			}
			printk("\n");
			if (0 == strncmp("xiaoxiang BMS", data->data, data->data_len)) {
				printk("\n\nFound candidate\n");
				candidate = true;
			} 
		}

		break;

	case BT_DATA_MANUFACTURER_DATA:
		{
			printk("Manufacturer data. Len %02x\n", data->data_len);
			unsigned dataLen = data->data_len;
			uint8_t* dat = data->data;
			while (dataLen) {
				printk(" %02x", *dat);
				--dataLen;
				++dat;
			}
			printk("\n");

			if ((data->data[0] == 0x8e) && (data->data[1] == 0xc2)) {
				candidate = true;
			}
		}
		break;
	case BT_DATA_SVC_DATA16:
	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		if (data->data_len % sizeof(uint16_t) != 0U) {
			printk("AD (16) malformed\n");
			return true;
		}

		break;
	case BT_DATA_SVC_DATA128:
	case BT_DATA_UUID128_SOME:
	case BT_DATA_UUID128_ALL:
		if (data->data_len % UUID128_BYTE_LEN != 0U) {
			printk("AD (128) malformed\n");
			return true;
		}

		return true;

	default:
		printk("Unknown BT_DATA type: %x\n", data->type);
		break;
	}

	if (candidate) {
		switch (advDev.status) {
		case STATUS_TESTER:
			printk("State Tester\n");
			if (data->type != BT_GAP_ADV_TYPE_SCAN_RSP) {
				bt_addr_le_copy(&advDev.addr, addr);
				advDev.status = STATUS_CANDIDATE;
				printk("-> Candidate\n");
				return false;
			}
			break;

		case STATUS_CANDIDATE:
			printk("State Candidate\n");
			if (bt_addr_le_eq(&advDev.addr, addr)) {
				advDev.status = STATUS_CHOSEN;
				printk("-> Chosen\n");
				return false;
			} else {
				advDev.status = STATUS_TESTER;
				printk("-> Tester\n");
				bt_addr_le_copy(&advDev.addr, &bt_addr_le_none);
			}
			break;

		case STATUS_CHOSEN:
			advDev.status = STATUS_TESTER;
			printk("Chosen -> Tester \n");

			bt_addr_le_copy(&advDev.addr, &bt_addr_le_none);
			break;

		default:
			printk("Error in status\n");
			advDev.status = STATUS_TESTER;
			bt_addr_le_copy(&advDev.addr, &bt_addr_le_none);
			break;
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i, data: ",
	       dev, type, ad->len, rssi);
	hexPrint(ad->data, ad->len);

	/* We're only interested in connectable events */
	if (type == BT_GAP_ADV_TYPE_ADV_IND ||
	    type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND ||
		type == BT_GAP_ADV_TYPE_SCAN_RSP) {
		bt_data_parse(ad, eir_found, (void *)addr);
	}

	if (advDev.status == STATUS_CHOSEN) {
		printk("Stopping scan\n");
		int err;

		err = bt_le_scan_stop();
		if (err) {
			printk("Stop LE scan failed (err %d)\n", err);
			return;
		}
		struct bt_le_conn_param *param;
		param = BT_LE_CONN_PARAM_DEFAULT;
		err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
					param, &default_conn);
		if (err) {
			printk("Create conn failed (err %d) - restart scanning\n", err);
			start_scan();
		}
	}
}

static void start_scan(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = /* BT_LE_SCAN_OPT_FILTER_DUPLICATE,*/ BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	printk("Connected: %s\n", addr);

	if (conn == default_conn) {
		memcpy(&discover_uuid, /*bmsServiceUuid*/BT_UUID_GATT, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;
	bufIdx = 0;
	subscHandles.cnt = 0;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void initiateSend()
{
	// header status command length data checksum footer
    //   DD     A5      03     00    FF     FD      77
	static const uint8_t data[7] = {0xdd, 0xa5, 0x3, 0x0, 0xff, 0xfd, 0x77};
	static uint16_t data_len;

	struct bt_conn* conn = NULL;;
	if (default_conn) {
		/* Get a connection reference to ensure that a
		 * reference is maintained in case disconnected
		 * callback is called while we perform GATT Write
		 * command.
		 */
		conn = bt_conn_ref(default_conn);
	}

	if (conn) {
		data_len = sizeof(data);
		printk("Sending data(hdl %u): ", writeHandle);
		hexPrint(data, sizeof(data));
		int err = bt_gatt_write_without_response(conn, writeHandle, data, sizeof(data), false);
		if (err) {
			printk("Failed writing data");
		}
	} else {
		printk("No tx. Conn is NULL");
	}

	bt_conn_unref(conn);
}

int main(void)
{
	int err;
	err = bt_enable(NULL);

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	start_scan();

//	int count = 2;

	while (true) {
		while (default_conn) {
			//if (count) {
				initiateSend();
			//	--count;
			//}
			err = k_sem_take(&sem_subs, K_MSEC(1000));
			if (err != 0) {
				printk("Could not take sem_conn (err %d)\n", err);
				continue;
			}

			hexPrint(buf, bufIdx);
			bufIdx = 0;
			k_sleep(K_MSEC(1000));

		}
		k_sleep(K_MSEC(5000));
	}
	return 0;
}
