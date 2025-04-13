#pragma once
/*
 * Copyright 2025, Kim Bøndergaard <kim@fam-boendergaard.dk>
 */

#include <zephyr/kernel.h>

#include <NMEA2000.h>

#include <Queue/SafeQueue.h>

#include <cpputil/semaphore.h>
#include <cpputil/mutex.h>
#include <cpputil/thread.h>

#include <zephyr/drivers/can.h>
#include <zephyr/devicetree.h>

#include <array>

class tNMEA2000_zephyr : public tNMEA2000
{

protected:
  bool CANSendFrame(unsigned long id, unsigned char len, 
		    const unsigned char *buf, bool wait_sent=true) override;
  bool CANOpen() override { return true; } // Done using normal zephyr device functions
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;
  void InitCANFrameBuffers() override;

  
  SafeQueue<struct can_frame*> sendQueue;
  SafeQueue<struct can_frame*> idleQueue;

  static const uint8_t framePoolSize{50};

  std::array<struct can_frame, framePoolSize> framePool;

private:
  cpputil::thread th;
  void runner();
  bool threadStop{false};
  cpputil::binary_semaphore sendSem;

private:
  uint32_t canBitRate{ KHZ(250) };

  /* can_tx_callback_t - runs in ISR context */
  static void sendCbHandler(const struct device *dev, int error, void *user_data);
  
private: 
  bool canSend();

public:
  tNMEA2000_zephyr();
  ~tNMEA2000_zephyr();

private:
  const struct device *dev;

  const static uint8_t maxRecvMsg{10};
  std::array<char, maxRecvMsg*sizeof(struct can_frame)> recvBuffer;
  struct k_msgq recvMsgQueue;
};

