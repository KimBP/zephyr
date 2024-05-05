/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* BMS UUIDs*/
static const struct bt_uuid_16 bmsUuid = BT_UUID_INIT_16(0xFF00);

static const struct bt_uuid_16 bmsDataUuid = BT_UUID_INIT_16(0xff01);

static const struct bt_uuid_16 bmsSettingUuid = BT_UUID_INIT_16(0xff02);

static bool notifySent = true;

#define BMS_DATA_BUF_LEN 256
static uint8_t bmsDataBuf[BMS_DATA_BUF_LEN] = {0};
static uint16_t bmsDataBufLen = 0;
static uint16_t bmsDataOffset = 0;

static bool dataRequested = false;

#define BMS_SETTING_BUF_LEN 128
static uint8_t bmsSettingBuf[BMS_SETTING_BUF_LEN] = {0};
static uint16_t bmsSettingBufLen = 0;

static ssize_t readDataBms(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	LOG_INF("ReadDataBms: data: %p, len: %u, offset: %u", (void*)value, len, offset);
	LOG_HEXDUMP_INF(value, len, "data");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t writeDataBms(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	LOG_INF("WriteDataBms: data: %p, len: %u, offset: %u", (void*)value, len, offset);
	LOG_HEXDUMP_INF(buf, len, "data");
	if (offset + len > BMS_DATA_BUF_LEN) { // or Write len?
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	value[offset + len] = 0;
	bmsDataBufLen = offset + len;

	return len;
}

static bool notifyEnable;

static void bmsCccCfgChanged(const struct bt_gatt_attr *attr, uint16_t value)
{
	notifyEnable = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification %s", notifyEnable ? "enabled" : "disabled");
}


static ssize_t readSettingBms(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	LOG_INF("ReadSettingBms: data: %p, len: %u, offset: %u", (void*)value, len, offset);
	LOG_HEXDUMP_INF(value, len, "data");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}


static ssize_t writeWithoutRspBms(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     const void *buf, uint16_t len, uint16_t offset,
				     uint8_t flags)
{
	uint8_t *value = attr->user_data;

	LOG_DBG("WriteWithoutRspBms: data: %p, len: %u, offset: %u", (void*)value, len, offset);
	LOG_HEXDUMP_INF(buf, len, "data");

	if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
		/* Write Request received. Reject it since this Characteristic
		 * only accepts Write Without Response.
		 */
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	}

	if (offset + len > BMS_SETTING_BUF_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint8_t* bufPtr = (uint8_t*)buf;

	if (bufPtr[0] != 0xdd) {
		return 0;
	}

	dataRequested = true;

	if (bufPtr[1] == 0xa5) {
		LOG_INF("Read msg: %u", bufPtr[2]);
	} else {
		LOG_INF("Write msg: %u", bufPtr[2]);
	}

	memcpy(value + offset, buf, len);
	value[offset + len] = 0;

	bmsSettingBufLen = offset + len;

	return len;
}

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(bmsSvc,
	BT_GATT_PRIMARY_SERVICE(&bmsUuid),
		BT_GATT_CHARACTERISTIC(&bmsDataUuid.uuid,
					   	   	   BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ ,
							   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							   readDataBms, writeDataBms, &bmsDataBuf),
			BT_GATT_CCC(bmsCccCfgChanged, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
			BT_GATT_CUD("Telink SPP: Module->Phone", BT_GATT_PERM_READ),
		BT_GATT_CHARACTERISTIC(&bmsSettingUuid.uuid,
					   	   	   BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							   readSettingBms, writeWithoutRspBms, &bmsSettingBuf),
			BT_GATT_CUD("Telink SPP: Phone->Module", BT_GATT_PERM_READ),
);


#define LF_ID_MSB ((BT_COMP_ID_LF >> 8) & 0xff)
#define LF_ID_LSB ((BT_COMP_ID_LF) & 0xff)

static uint8_t compId[] = {LF_ID_MSB, LF_ID_LSB, 0x12, 0x38, 0xc1, 0xa4};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_SOME,
		      BT_UUID_16_ENCODE(0xFF00),
		     ),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, compId, sizeof(compId)),
};

static uint16_t mtuTx = 200;

void mtuUpdated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	mtuTx = tx-3;
	LOG_INF("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gattCallbacks = {
	.att_mtu_updated = mtuUpdated
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)", err);
	} else {
		LOG_INF("Connected");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_WRN("Disconnected (reason 0x%02x)", reason);
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	LOG_INF("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void notifyData(struct bt_gatt_attr* notifyAttr)
{
	static int count = 4;

	static bool doSend = false;

	count++;
	if (count == 5) {
		count = 0;
		doSend = true;
	}
	if (bmsDataOffset) {
		doSend = true;
	}


	LOG_DBG("NotifyData: bmsDataOffset: %u", bmsDataOffset);
	if (doSend)
	{
		uint16_t len = bmsDataBufLen - bmsDataOffset;
		if (len > mtuTx) {
			len = mtuTx;
		}
		LOG_HEXDUMP_INF(bmsDataBuf+bmsDataOffset, len, "Notifying");
		int err = bt_gatt_notify(NULL, notifyAttr, bmsDataBuf+bmsDataOffset, len);
		if (err) {
			LOG_ERR("Notify error: %d", err);
		} else {
			LOG_DBG("Send notify, %u bytes", len);
			bmsDataOffset += len;
			if (bmsDataOffset == bmsDataBufLen) {
				// All sent
				bmsDataOffset = 0;
				notifySent = true;
			}
		}
	}
}


static void writeUint16(uint8_t* buf, uint16_t offset, uint16_t val)
{
	buf += offset;

	buf[0] = (uint8_t)(val >> 8);
	buf[1] = val & 0xFF;
}

static uint16_t checksum(uint8_t* data)
{
	data += 2;
	uint16_t crc = *data; // status is first value ??
	data++;
	uint8_t len = *data;
	crc += len; // Add len
	while(len--) {
		data++;
		crc += *data;
	}
	crc ^= 0xFFFF;
	crc++;
	LOG_DBG("CRC: %04x", crc);
	return crc;
}

void writeCrc(char* buf)
{
	uint16_t crc = checksum(buf);
	uint8_t len = buf[3];
	writeUint16(buf, 4 + len, crc);
}

uint8_t writeTail(char* buf)
{
	uint8_t len = buf[3];
	len += 4 + 2; // Header + CRC
	buf[len] = 0x77;

	return len+1;
}

uint8_t finishMessage(char* buf)
{
	writeCrc(buf);
	return writeTail(buf);
}


static int16_t random(int16_t min, int16_t max)
{
	uint32_t span = max - min + 1;

	uint32_t rand = sys_rand32_get();
	rand %= span;

	return min + rand;
}


static void bmsSimulate(uint8_t msgId)
{
	static struct BmsData4 {
		uint16_t cellVoltage[4]; // Unit 1mV: 3332 -> 3.332V
	} bmsData4 = { .cellVoltage = { 3332, 3332, 3332, 3332 } };

	bmsData4.cellVoltage[0] += random(-2,2);
	bmsData4.cellVoltage[1] += random(-2,2);
	bmsData4.cellVoltage[2] += random(-2,2);
	bmsData4.cellVoltage[3] += random(-6,6);

	static struct BmsData3 {
		uint16_t volt;	// Unit 10mV: 1332 -> 13,32V
		int16_t amp; // Unit 10mA: 7550 -> 75.5A
		uint16_t capRemaining; // Unit 10mAh: 7550 -> 75.5Ah
		uint16_t capNominal; // Unit 10mAh: 7550 -> 75.5Ah
		uint16_t cycles;
		uint16_t prodDate;  // bitfield: yyyyyyyymmmmddddd    (year + 2000)
							// example 0x2068: 0010111001101000 -> y:0010111 m: 0011  d: 01000
							// y = 23 aka 2023, m = 3 aka March, d = 8
		uint16_t balance[2];
		uint16_t protectionStatus; // bitfield representing different protections (bit 0..12)
		uint8_t swVersion; // 0x23 -> version 2.3
		uint8_t capacityPercent;
		uint8_t mosFet; // bit 0: charge FET, bit 1: discharge. 0=off, 1=o
		uint8_t cells; // Number of battery strings (cells?)
		uint8_t tempCnt; // = 3 here
		uint16_t temp[3]; // Unit 0.1 Degrees Kelvin (aka Celcius + 273.1)
	} bmsData3 = {
		.volt = 0,
		.amp = 0,
		.capRemaining = 9800,
		.capNominal = 10000,
		.cycles = 14,
		.prodDate = 0x2068,
		.balance = {0, 0 },
		.protectionStatus = 0,
		.swVersion = 0x20,
		.capacityPercent = 98,
		.mosFet = 0x3,
		.cells = 4,
		.tempCnt = 3,
		.temp = {2731 + 100, 2731 + 150, 2731 + 200},
	};

	bmsData3.volt = (bmsData4.cellVoltage[0] + bmsData4.cellVoltage[1] +
		            bmsData4.cellVoltage[2] + bmsData4.cellVoltage[3]) / 10;
	bmsData3.amp += random(-100, 100);
	bmsData3.capRemaining += random(-100,100);
	bmsData3.cycles = 14 + random(0,99)/99;
	bmsData3.capacityPercent = bmsData3.capRemaining / 100;
	bmsData3.temp[0] += random(-10, +10);
	bmsData3.temp[1] += random(-10, +10);
	bmsData3.temp[2] += random(-10, +10);

	if (notifySent) {
		notifySent=false;
		bmsDataBuf[0] = 0xdd; // msg start
		bmsDataBuf[1] = msgId; // msg type
		bmsDataBuf[2] = 0x00; // status

		switch (msgId) {
		case 3: {
			bmsDataBuf[3] = 0x1d; // Length of data
			writeUint16(bmsDataBuf, 4, bmsData3.volt);
			writeUint16(bmsDataBuf, 6, bmsData3.amp);
			writeUint16(bmsDataBuf, 8, bmsData3.capRemaining);
			writeUint16(bmsDataBuf, 10, bmsData3.capNominal);
			writeUint16(bmsDataBuf, 12, bmsData3.cycles);
			writeUint16(bmsDataBuf, 14, bmsData3.prodDate);
			writeUint16(bmsDataBuf, 16, bmsData3.balance[0]);
			writeUint16(bmsDataBuf, 18, bmsData3.balance[1]);
			writeUint16(bmsDataBuf, 20, bmsData3.protectionStatus);
			bmsDataBuf[22] = bmsData3.swVersion;
			bmsDataBuf[23] = bmsData3.capacityPercent;
			bmsDataBuf[24] = bmsData3.mosFet;
			bmsDataBuf[25] = bmsData3.cells;
			bmsDataBuf[26] = bmsData3.tempCnt;
			writeUint16(bmsDataBuf, 27, bmsData3.temp[0]);
			writeUint16(bmsDataBuf, 29, bmsData3.temp[1]);
			writeUint16(bmsDataBuf, 31, bmsData3.temp[2]);

			bmsDataBufLen = finishMessage(bmsDataBuf);

			LOG_DBG("Setting up msg %u (len %u)", bmsDataBuf[1], bmsDataBufLen);
			break;
		}

		case 4: {
			bmsDataBuf[3] = 0x08;
			writeUint16(bmsDataBuf, 4, bmsData4.cellVoltage[0]);
			writeUint16(bmsDataBuf, 6, bmsData4.cellVoltage[1]);
			writeUint16(bmsDataBuf, 8, bmsData4.cellVoltage[2]);
			writeUint16(bmsDataBuf, 10, bmsData4.cellVoltage[3]);

			bmsDataBufLen = finishMessage(bmsDataBuf);

			LOG_DBG("Setting up msg %u (len %u)", bmsDataBuf[3], bmsDataBufLen);
			break;
		}

		case 0xa1:
		case 5: {
			const char* devName = "Kims-Sim-100A";
			const uint8_t devNameLen = 13;
			bmsDataBuf[3] = devNameLen;
			memcpy(&bmsDataBuf[4], devName, devNameLen);

			bmsDataBufLen = finishMessage(bmsDataBuf);

			LOG_DBG("Setting up msg %u (len %u)", bmsDataBuf[3], bmsDataBufLen);
			break;
		}
		case 0x2e: {
			bmsDataBuf[3] = 2;
			writeUint16(bmsDataBuf, 4, 0x0007);

			bmsDataBufLen = finishMessage(bmsDataBuf);

			LOG_DBG("Setting up msg %u (len %u)", bmsDataBuf[3], bmsDataBufLen);

			break;
		}

		case 0xa0: {
			const char* manufacturer = "Sundown'r Development";
			const uint8_t manufacturerLen = 21;

			bmsDataBuf[3] = manufacturerLen + 1;
			bmsDataBuf[4] = manufacturerLen;
			memcpy(&bmsDataBuf[5], manufacturer, manufacturerLen);

			bmsDataBufLen = finishMessage(bmsDataBuf);

			LOG_DBG("Setting up msg %u (len %u)", bmsDataBuf[3], bmsDataBufLen);

			break;
		}

		case 0xAA: {
			bmsDataBuf[3] = 2 * 12;
			for(uint8_t i = 0; i < 12; i++) {
				writeUint16(bmsDataBuf, 4 + i*2, i+1);
			}
			bmsDataBufLen = finishMessage(bmsDataBuf);

			break;
		}
		default: {
			LOG_WRN("Unsupported msgId: %u", msgId);
			notifySent = true; // To ensure we get in here again
			break;
		}
		}
	}
}

int main(void)
{
	char str[BT_UUID_STR_LEN];
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	bt_ready();

	bt_gatt_cb_register(&gattCallbacks);

	struct bt_gatt_attr* bmsNotifyAttr = bt_gatt_find_by_uuid(bmsSvc.attrs, bmsSvc.attr_count,
					    &bmsDataUuid.uuid);

	bt_uuid_to_str(&bmsDataUuid.uuid, str, sizeof(str));
	LOG_DBG("BMS Data attr %p (UUID %s)", bmsNotifyAttr, str);

	while (1) {
		k_sleep(K_MSEC(100));

		if (notifyEnable ) {
			static uint8_t msgId;
			if (dataRequested) {
				msgId = bmsSettingBuf[2];
				dataRequested = false;
			} else {
				if (notifySent) {

					static uint8_t idx = 0;
					const uint8_t msgArray[] = {3,4,5,0xA0, 0x2e, 0xA1, 0xAA};

					msgId = msgArray[idx];
					idx++;
					if (idx == sizeof(msgArray)) {
						idx = 0;
					}
				}
//				bmsSimulate(msgId);
			}

			bmsSimulate(msgId);

			notifyData(bmsNotifyAttr);
		}
	}
	return 0;
}
