#pragma once
/*
 * Copyright 2025, Kim Bøndergaard <kim@fam-boendergaard.dk>
 */

#include <zephyr/kernel.h>

#include <NMEA2000.h>

#include <Queue/SafeQueue.h>

#include <cpputil/mutex.h>
#include <cpputil/thread.h>

#include <zephyr/drivers/can.h>
#include <zephyr/devicetree.h>

#include <array>
#include <atomic>
#include <functional>

class tNMEA2000_zephyr : public tNMEA2000
{

protected:
  bool CANSendFrame(unsigned long id, unsigned char len, 
		    const unsigned char *buf, bool wait_sent=true) override;
  bool CANOpen() override;
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;
  void InitCANFrameBuffers() override;

  
  SafeQueue<struct can_frame*> sendQueue;
  SafeQueue<struct can_frame*> idleQueue;

  static const uint8_t framePoolSize{50};

  std::array<struct can_frame, framePoolSize> framePool;

private:
  cpputil::thread th;
  void runner();
  std::atomic<bool> threadStop{false};
  std::function<void()> periodicHook;

private:
  uint32_t canBitRate{ KHZ(250) };
  static constexpr int sendTimeoutMs{10};

  /* can_tx_callback_t - runs in ISR context */
  static void sendCbHandler(const struct device *dev, int error, void *user_data);
  
private: 
  bool canSend();

public:
  tNMEA2000_zephyr();
  ~tNMEA2000_zephyr();

  /*
   * The runner thread started by Start() is the only thread allowed to call
   * the tNMEA2000 API afterwards. Configure the library and set the hook
   * before calling Start().
   */

  /* Called from the runner thread on every loop iteration */
  void SetPeriodicHook(std::function<void()> hook) { periodicHook = std::move(hook); }

  /* Start the runner thread. It opens the library and keeps it running */
  void Start();

  /* Stop and join the runner thread */
  void Stop();

private:
  const struct device *dev;

  static constexpr uint8_t maxRecvMsg{64};
  static constexpr int recvWaitMs{10};
  alignas(struct can_frame) std::array<char, maxRecvMsg*sizeof(struct can_frame)> recvBuffer;
  struct k_msgq recvMsgQueue;

  // Frame taken from recvMsgQueue while waiting for data, returned first by CANGetFrame()
  struct can_frame pendingFrame;
  bool hasPendingFrame{false};

  // Rate limit for the rx queue full warning
  static constexpr int64_t recvFullWarnIntervalMs{10000};
  int64_t lastRecvFullWarnMs{-recvFullWarnIntervalMs};

  void checkRecvQueueFull();

  bool canStarted{false};
  int rxFilterId{-1};
};

