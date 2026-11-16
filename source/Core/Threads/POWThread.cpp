/*
 * POWThread.cpp
 *
 *  Created on: 16 Jan 2021
 *      Author: Ralim
 */

#include "BSP.h"
#include "FS2711.hpp"
#include "FreeRTOS.h"
#include "HUB238.hpp"
#include "QC3.h"
#include "Settings.h"
#include "USBPD.h"
#include "cmsis_os.h"
#include "configuration.h"
#include "main.hpp"
#include "stdbool.h"
#include "stdlib.h"
#include "task.h"

// Small worker thread to handle power (PD + QC) related steps

void startPOWTask(void const *argument __unused) {

  // Init any other misc sensors
  postRToSInit();
#if !defined(MODEL_TS101)
  while (preStartChecksDone() == 0) {
    osDelay(3);
  }
#endif
  // TS101: start USB-PD straight away rather than after the ~1.6s tip-resistance
  // measurement; some sources power-cycle VBUS if their Source_Capabilities go
  // unanswered for ~2s. Until the tip is measured getTipResistanceX10() returns a
  // conservative value, and we renegotiate once the measurement is done.
#ifdef MODEL_TS101
  bool renegotiatedAfterTipCheck = false;
#endif
  // You have to run this once we are willing to answer PD messages
  // Setting up too early can mean that we miss the ~20ms window to respond on some chargers
#ifdef POW_PD
  USBPowerDelivery::start();
  // Crank the handle at boot until we are stable and waiting for IRQ
  USBPowerDelivery::step();
#endif
#if POW_PD_EXT == 2
  FS2711::start();
  FS2711::negotiate();
#endif

  BaseType_t res;
  for (;;) {
    res = pdFALSE;
    // While the interrupt is low, dont delay
    /*This is due to a possible race condition, where:
     * IRQ fires
     * We read interrupt register but dont see the Good CRC
     * Then Good CRC is set while reading it out (racing on I2C read)
     * Then we would sleep as nothing to do, but 100ms> 20ms power supply typical timeout
     */
    if (!getFUS302IRQLow()) {
      res = xTaskNotifyWait(0x0, 0xFFFFFF, NULL, TICKS_100MS / 2);
    }

#ifdef POW_PD
    if (res != pdFALSE || getFUS302IRQLow()) {
      USBPowerDelivery::IRQOccured();
    }
    USBPowerDelivery::PPSTimerCallback();
    USBPowerDelivery::step();
#ifdef MODEL_TS101
    if (!renegotiatedAfterTipCheck && preStartChecksDone() && USBPowerDelivery::negotiationHasWorked()) {
      renegotiatedAfterTipCheck = true;
      USBPowerDelivery::renegotiate();
    }
#endif

#else
    (void)res;
#endif
#if POW_PD_EXT == 1
    hub238_check_negotiation();
#endif
#if POW_PD_EXT == 2
    FS2711::negotiate();
#endif
    power_check();
  }
}
