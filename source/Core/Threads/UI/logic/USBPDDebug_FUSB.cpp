#include "OperatingModes.h"
#include "ui_drawing.hpp"
#ifdef POW_PD
#include "pd.h"
#ifdef MODEL_TS101
#include "fusb302b.h"
extern "C" uint8_t diagLogReadBack(int16_t back);
#endif
#ifdef HAS_POWER_DEBUG_MENU
OperatingMode showPDDebug(const ButtonState buttons, guiContext *cxt) {
  // Print out the USB-PD state
  // Basically this is like the Debug menu, but instead we want to print out the PD status
  uint16_t *screen = &(cxt->scratch_state.state1);

#ifdef MODEL_TS101
  // Temporary: show the flash event log from before this boot, 10 entries per page,
  // oldest at top-left, newest at bottom-right. Front button = one page further back.
  {
    const int16_t page = (int16_t)(*screen);
    for (uint8_t i = 0; i < 10; i++) {
      // i = 0 is the oldest entry on this page
      uint8_t value = diagLogReadBack((int16_t)((page * 10) + (10 - i)));
      OLED::setCursor((i % 5) * 24, (i < 5) ? 0 : 16);
      OLED::drawHex(((uint32_t)value) << 24, FontStyle::SMALL, 2);
    }
    OLED::setCursor(120, 0);
    OLED::drawHex(((uint32_t)page) << 28, FontStyle::SMALL, 1);
    if (page > 14) {
      (*screen) = 0;
    }
  }
#else
  if ((*screen) == 0) {
    // Print the PD state machine
    uint8_t vbusState = 0;
    if (USBPowerDelivery::fusbPresent()) {
      if (USBPowerDelivery::negotiationComplete() || (xTaskGetTickCount() > (TICKS_SECOND * 10))) {
        if (!USBPowerDelivery::isVBUSConnected()) {
          vbusState = 2;
        } else {
          vbusState = 1;
        }
      }
    }
#ifdef MODEL_TS101
    // Diagnostics screen (temporary), all values in hex, 16 chars per line:
    //  line 1: <listen> <devId> <cc1><cc2> <irqA> <irqB> <irq>
    //  line 2: <state> <rxGood> <rxBad> <statusFail> <lastHdr>
    // drawHex prints the top nibbles of a 32-bit value, so shift the value up first
    (void)vbusState;
    OLED::setCursor(0, 0);
    OLED::drawHex(((uint32_t)fusbDiag.listenOnly) << 28, FontStyle::SMALL, 1);
    OLED::setCursor(16, 0);
    OLED::drawHex(((uint32_t)fusbDiag.devId) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(40, 0);
    OLED::drawHex(((uint32_t)fusbDiag.cc1) << 28, FontStyle::SMALL, 1);
    OLED::drawHex(((uint32_t)fusbDiag.cc2) << 28, FontStyle::SMALL, 1);
    OLED::setCursor(64, 0);
    OLED::drawHex(((uint32_t)fusbDiag.irqA) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(88, 0);
    OLED::drawHex(((uint32_t)fusbDiag.irqB) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(112, 0);
    OLED::drawHex(((uint32_t)fusbDiag.irq) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(0, 16);
    OLED::drawHex(((uint32_t)USBPowerDelivery::getStateNumber()) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(24, 16);
    OLED::drawHex(((uint32_t)fusbDiag.rxGood) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(48, 16);
    OLED::drawHex(((uint32_t)fusbDiag.rxBad) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(72, 16);
    OLED::drawHex(((uint32_t)fusbDiag.statusFail) << 24, FontStyle::SMALL, 2);
    OLED::setCursor(96, 16);
    OLED::drawHex(((uint32_t)fusbDiag.lastHdr) << 16, FontStyle::SMALL, 4);
#else
    ui_draw_usb_pd_debug_state(vbusState, USBPowerDelivery::getStateNumber());
#endif
  } else {
    // Print out the Proposed power options one by one
    auto lastCaps           = USBPowerDelivery::getLastSeenCapabilities();
    bool sourceIsEPRCapable = lastCaps[0] & PD_PDO_SRC_FIXED_EPR_CAPABLE;
    if (((*screen) - 1) < 11) {
      int voltage_mv     = 0;
      int min_voltage    = 0;
      int current_a_x100 = 0;
      int wattage        = 0;

      if ((lastCaps[(*screen) - 1] & PD_PDO_TYPE) == PD_PDO_TYPE_FIXED) {
        voltage_mv     = PD_PDV2MV(PD_PDO_SRC_FIXED_VOLTAGE_GET(lastCaps[(*screen) - 1])); // voltage in mV units
        current_a_x100 = PD_PDO_SRC_FIXED_CURRENT_GET(lastCaps[(*screen) - 1]);            // current in 10mA units
      } else if ((lastCaps[(*screen) - 1] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED) {
        if (sourceIsEPRCapable) {
          if ((lastCaps[(*screen) - 1] & PD_APDO_TYPE) == PD_APDO_TYPE_AVS) {
            voltage_mv  = PD_PAV2MV(PD_APDO_AVS_MAX_VOLTAGE_GET(lastCaps[(*screen) - 1]));
            min_voltage = PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(lastCaps[(*screen) - 1]));
            // Last value is wattage
            wattage = PD_APDO_AVS_MAX_POWER_GET(lastCaps[(*screen) - 1]);
          } else if (((lastCaps[(*screen) - 1] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS)) {
            voltage_mv     = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(lastCaps[(*screen) - 1]));
            min_voltage    = PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(lastCaps[(*screen) - 1]));
            current_a_x100 = PD_PAI2CA(PD_APDO_PPS_CURRENT_GET(lastCaps[(*screen) - 1])); // max current in 10mA units
          }
        } else {
          // Doesn't have EPR support. So treat as PPS
          // https://github.com/Ralim/IronOS/issues/1906
          voltage_mv     = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(lastCaps[(*screen) - 1]));
          min_voltage    = PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(lastCaps[(*screen) - 1]));
          current_a_x100 = PD_PAI2CA(PD_APDO_PPS_CURRENT_GET(lastCaps[(*screen) - 1])); // max current in 10mA units
        }
      }
      // Skip not used entries
      if (voltage_mv == 0) {
        (*screen) += 1;
      } else {
        ui_draw_usb_pd_debug_pdo(*screen, min_voltage / 1000, voltage_mv / 1000, current_a_x100, wattage);
      }
    } else {
      (*screen) = 0;
    }
  }
#endif
  if (buttons == BUTTON_B_SHORT) {
    return OperatingMode::InitialisationDone;
  } else if (buttons == BUTTON_F_SHORT) {
    (*screen) += 1;
  }
  return OperatingMode::UsbPDDebug;
}
#endif
#endif
