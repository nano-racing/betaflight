
#pragma once

#define RX_POWER_DISABLE        digitalLo(FPF132X_GPIO, FPF132X_EN_PIN)
#define RX_POWER_ENABLE         digitalHi(FPF132X_GPIO, FPF132X_EN_PIN)
#define RX_POWER_CYCLE_DELAY	200
#define RX_POWER_CYCLE			digitalLo(FPF132X_GPIO, FPF132X_EN_PIN); delay(RX_CYCLE_POWER_DELAY); digitalHi(FPF132X_GPIO, FPF132X_EN_PIN)
#define RX_POWER_3_3V           digitalLo(FPF132X_GPIO, FPF132X_SEL_PIN)
#define RX_POWER_5V             digitalHi(FPF132X_GPIO, FPF132X_SEL_PIN)

typedef struct rxPowerConfig_s {
    uint8_t rxPower;
    float rxVoltage;
} rxPowerConfig_t;

void fpf132xInit(rxPowerConfig_t *rxPowerConfig);
void fpf132xEnable5V();
void fpf132xTurnOffRx();
void fpf132xTurnOnRx();