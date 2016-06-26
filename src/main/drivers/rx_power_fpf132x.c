/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Driver for the FPF132X IC
 * This is a dual-input single-output power switch, allowing to select the voltage for the receiver.
 * It is also used to turn off the receiver, if it needs to be power cycled to finish the bind process.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>
#include "gpio.h"

#include "rx_power_fpf132x.h"

// security measure - not implemented
static bool enable5V = false;

void fpf132xInit(rxPowerConfig_t *rxPowerConfig)
{
#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(FPF132X_GPIO_PERIPHERAL, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

    GPIO_InitStructure.GPIO_Pin = FPF132X_EN_PIN;
    GPIO_Init(FPF132X_GPIO, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = FPF132X_SEL_PIN;
    GPIO_Init(FPF132X_GPIO, &GPIO_InitStructure);
#endif

//Not used
/*
#ifdef STM32F10X
    RCC_APB2PeriphClockCmd(FPF132X_GPIO_PERIPHERAL, ENABLE);

    gpio_config_t gpio;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_50MHz;
    
    gpio.pin = FPF132X_EN_PIN;
    gpioInit(FPF132X_GPIO, &gpio);
    
    gpio.pin = FPF132X_SEL_PIN;
    gpioInit(FPF132X_GPIO, &gpio);
#endif
*/
    if(rxPowerConfig->rxVoltage >= 3.3 && rxPowerConfig->rxVoltage < 5)
        RX_POWER_3_3V;
    else if(rxPowerConfig->rxVoltage == 5 && enable5V)
        RX_POWER_5V;
    else
        RX_POWER_DISABLE;
    
    if(rxPowerConfig->rxPower)
        RX_POWER_ENABLE;
    else
        RX_POWER_DISABLE;
}

void fpf132xEnable5V()
{
	enable5V = true;
}

void fpf132xTurnOffRx()
{
	RX_POWER_DISABLE;
}

void fpf132xTurnOnRx()
{
	RX_POWER_ENABLE;
}

