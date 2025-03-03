/*
 * Copyright 2025, Kim Bøndergaard <kim@fam-boendergaard.dk>
 */

#include "NMEA2000_zephyr.h"

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Z_NMEA, CONFIG_APP_BMS_LOG_LEVEL);

bool tNMEA2000_zephyr::CANSendFrame(unsigned long id, unsigned char len, 
      const unsigned char *buf, bool wait_sent)
{
  if (idleQueue.empty()) {
    LOG_WRN("No frames available for for transmit");
    return false;
  }

  auto frame = idleQueue.dequeue();

  frame->id = id;
  frame->dlc = (len > 8 ? 8 : len);
  frame->flags = CAN_FRAME_IDE;
  memcpy(frame->data, buf, frame->dlc);

  sendQueue.enqueue(frame);

  sendSem.release();

  return true;
}

bool tNMEA2000_zephyr::canSend()
{
  if (sendQueue.empty()) {
    return true;
  }

  auto frame = sendQueue.dequeue();

  int err;
  do {
    err = can_send(dev, frame, K_NO_WAIT, sendCbHandler, this);
  } while (err == -EAGAIN);

  idleQueue.enqueue(frame);

  if (err == 0) {
    // frame was sent 
    return true;
  }

  LOG_WRN("Failed sending frame, err: %d", err);

  return false;
}

bool tNMEA2000_zephyr::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
  struct can_frame frame;

  if (0 != k_msgq_get(&recvMsgQueue, &frame, K_NO_WAIT)) {
    return false;
  }

  id = frame.id;
  len = frame.dlc;
  memcpy(buf, frame.data, len);

  LOG_INF("A frame received");
  return true;
}

void tNMEA2000_zephyr::InitCANFrameBuffers()
{
  MaxCANReceiveFrames = maxRecvMsg;
  MaxCANSendFrames = framePoolSize;
}

tNMEA2000_zephyr::tNMEA2000_zephyr() 
: th(cpputil::thread(std::bind(&tNMEA2000_zephyr::runner, this)))
, dev(DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)))
{
  if (!device_is_ready(dev)) {
    printk("CAN device not ready");
    return;
  }
  
  can_set_bitrate(dev, canBitRate);
  can_set_mode(dev, CAN_MODE_ONE_SHOT);

  int err = can_start(dev);
  if (err != 0) {
    LOG_ERR("Error starting CAN controller (err %d)", err);
    return;
  }

  for(auto& frame : framePool) {
    idleQueue.enqueue(&frame);
  }

  // Setup filter receiving any packet. 
  const struct can_filter recvAllFilter = {
    .id = 0,
    .mask = 0,
    .flags = CAN_FRAME_IDE,
  };

  k_msgq_init(&recvMsgQueue, recvBuffer.data(), sizeof(struct can_frame), maxRecvMsg);
  err = can_add_rx_filter_msgq(dev, &recvMsgQueue, &recvAllFilter);
  if (err == -ENOSPC)
  {
    LOG_WRN("No Input filter slots available");
  }
}

tNMEA2000_zephyr::~tNMEA2000_zephyr()
{
  threadStop = true;
  th.join();
}

void tNMEA2000_zephyr::runner() 
{
  while (!threadStop) {

    if (sendSem.try_acquire()) {
      while (not sendQueue.empty()) {
        canSend();
      }
    }

    ParseMessages(); // This processes incoming messages
    k_sleep(K_MSEC(40));       
  }
}

void tNMEA2000_zephyr::sendCbHandler(const struct device *dev, int error, void *user_data)
{
  if (not user_data) {
    LOG_ERR("user_data expected");
    return;
  }

  auto inst = static_cast<tNMEA2000_zephyr*>(user_data);

  inst->sendSem.release();
}
