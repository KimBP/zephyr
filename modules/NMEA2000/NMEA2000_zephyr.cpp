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
  struct can_frame *frame;

  if (!idleQueue.try_dequeue(frame)) {
    LOG_WRN("No frames available for for transmit");
    return false;
  }

  frame->id = id;
  frame->dlc = (len > 8 ? 8 : len);
  frame->flags = CAN_FRAME_IDE;
  memcpy(frame->data, buf, frame->dlc);

  sendQueue.enqueue(frame);

  return true;
}

bool tNMEA2000_zephyr::canSend()
{
  if (sendQueue.empty()) {
    return true;
  }

  auto frame = sendQueue.dequeue();

  int err = can_send(dev, frame, K_MSEC(sendTimeoutMs), sendCbHandler, this);

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

  if (hasPendingFrame) {
    frame = pendingFrame;
    hasPendingFrame = false;
  } else if (0 != k_msgq_get(&recvMsgQueue, &frame, K_NO_WAIT)) {
    return false;
  }

  id = frame.id;
  len = frame.dlc;
  memcpy(buf, frame.data, len);

  LOG_DBG("Frame received, id 0x%08x", (unsigned int)frame.id);
  return true;
}

void tNMEA2000_zephyr::InitCANFrameBuffers()
{
  MaxCANReceiveFrames = maxRecvMsg;
  MaxCANSendFrames = framePoolSize;
}

tNMEA2000_zephyr::tNMEA2000_zephyr()
: dev(DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)))
{
  for(auto& frame : framePool) {
    idleQueue.enqueue(&frame);
  }

  k_msgq_init(&recvMsgQueue, recvBuffer.data(), sizeof(struct can_frame), maxRecvMsg);
}

bool tNMEA2000_zephyr::CANOpen()
{
  if (canStarted) {
    return true;
  }

  if (!device_is_ready(dev)) {
    LOG_ERR("CAN device not ready");
    return false;
  }

  if (rxFilterId < 0) {
    // Setup filter receiving any packet.
    const struct can_filter recvAllFilter = {
      .id = 0,
      .mask = 0,
      .flags = CAN_FRAME_IDE,
    };

    rxFilterId = can_add_rx_filter_msgq(dev, &recvMsgQueue, &recvAllFilter);
    if (rxFilterId < 0) {
      LOG_ERR("Failed adding rx filter (err %d)", rxFilterId);
      return false;
    }
  }

  int err = can_set_bitrate(dev, canBitRate);
  if (err != 0) {
    LOG_ERR("Error setting CAN bitrate %u (err %d)", (unsigned int)canBitRate, err);
    return false;
  }

  err = can_set_mode(dev, CAN_MODE_NORMAL);
  if (err != 0) {
    LOG_ERR("Error setting CAN mode (err %d)", err);
    return false;
  }

  err = can_start(dev);
  if (err != 0) {
    LOG_ERR("Error starting CAN controller (err %d)", err);
    return false;
  }
  canStarted = true;

  return true;
}

tNMEA2000_zephyr::~tNMEA2000_zephyr()
{
  Stop();

  if (canStarted) {
    int err = can_stop(dev);
    if (err != 0) {
      LOG_ERR("Error stopping CAN controller (err %d)", err);
    }
    canStarted = false;
  }

  if (rxFilterId >= 0) {
    can_remove_rx_filter(dev, rxFilterId);
    rxFilterId = -1;
  }
}

void tNMEA2000_zephyr::Start()
{
  if (th.joinable()) {
    return;
  }

  threadStop = false;
  th = cpputil::thread(std::bind(&tNMEA2000_zephyr::runner, this));
  th.setName("NMEA2000 wrapper");
}

void tNMEA2000_zephyr::Stop()
{
  threadStop = true;
  if (th.joinable()) {
    th.join();
  }
}

void tNMEA2000_zephyr::runner() 
{
  while (!threadStop) {


    ParseMessages(); // This processes incoming messages

    if (periodicHook) {
      periodicHook();
    }

    // Everything queued by ParseMessages() and the hook goes out now
    while (not sendQueue.empty()) {
      canSend();
    }

    // Sleep until a frame arrives, or the timeout lets library timers run
    if (!hasPendingFrame &&
        k_msgq_get(&recvMsgQueue, &pendingFrame, K_MSEC(recvWaitMs)) == 0) {
      hasPendingFrame = true;
    }
  }
}

void tNMEA2000_zephyr::sendCbHandler(const struct device *dev, int error, void *user_data)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(user_data);

  if (error != 0) {
    LOG_WRN("Frame transmit failed (err %d)", error);
  }
}
