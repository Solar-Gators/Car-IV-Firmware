#pragma once

#include "main.h"
#include "DACx311.hpp"

typedef enum {
    ECO,
    POWER
} Mitsuba_ModeTypeDef;

typedef enum {
    FORWARD,
    REVERSE
} Mitsuba_DirectionTypeDef;

class Mitsuba {
public:
    Mitsuba(DACx311 *throttle_dac, 
            DACx311 *regen_dac,
            GPIO_TypeDef *main_gpio_port,
            uint16_t main_gpio_pin,
            GPIO_TypeDef *pwr_eco_gpio_port,
            uint16_t pwr_eco_gpio_pin,
            GPIO_TypeDef *fwd_rev_gpio_port,
            uint16_t fwd_rev_gpio_pin);
    void Enable();
    void Disable();
    void SetMode(Mitsuba_ModeTypeDef mode);
    void SetDirection(Mitsuba_DirectionTypeDef direction);
    HAL_StatusTypeDef SetThrottle(uint16_t value);
    HAL_StatusTypeDef SetRegen(uint16_t value);
private:
    DACx311 *_throttle_dac;
    DACx311 *_regen_dac;
    GPIO_TypeDef *_main_gpio_port;
    uint16_t _main_gpio_pin;
    GPIO_TypeDef *_pwr_eco_gpio_port;
    uint16_t _pwr_eco_gpio_pin;
    GPIO_TypeDef *_fwd_rev_gpio_port;
    uint16_t _fwd_rev_gpio_pin;
};