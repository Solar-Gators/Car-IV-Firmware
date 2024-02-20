#include "Mitsuba.hpp"

Mitsuba::Mitsuba(DACx311 *throttle_dac, 
                 DACx311 *regen_dac,
                 GPIO_TypeDef *main_gpio_port,
                 uint16_t main_gpio_pin,
                 GPIO_TypeDef *pwr_eco_gpio_port,
                 uint16_t pwr_eco_gpio_pin,
                 GPIO_TypeDef *fwd_rev_gpio_port,
                 uint16_t fwd_rev_gpio_pin) :
                 _throttle_dac(throttle_dac),
                 _regen_dac(regen_dac),
                 _main_gpio_port(main_gpio_port),
                 _main_gpio_pin(main_gpio_pin),
                 _pwr_eco_gpio_port(pwr_eco_gpio_port),
                 _pwr_eco_gpio_pin(pwr_eco_gpio_pin),
                 _fwd_rev_gpio_port(fwd_rev_gpio_port),
                 _fwd_rev_gpio_pin(fwd_rev_gpio_pin)
{
    HAL_GPIO_WritePin(_main_gpio_port, _main_gpio_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_pwr_eco_gpio_port, _pwr_eco_gpio_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_fwd_rev_gpio_port, _fwd_rev_gpio_pin, GPIO_PIN_RESET);

}

void Mitsuba::Enable() {
    HAL_GPIO_WritePin(_main_gpio_port, _main_gpio_pin, GPIO_PIN_SET);
}

void Mitsuba::Disable() {
    HAL_GPIO_WritePin(_main_gpio_port, _main_gpio_pin, GPIO_PIN_RESET);
}

void Mitsuba::SetMode(Mitsuba_ModeTypeDef mode) {
    if (mode == ECO) {
        HAL_GPIO_WritePin(_pwr_eco_gpio_port, _pwr_eco_gpio_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(_pwr_eco_gpio_port, _pwr_eco_gpio_pin, GPIO_PIN_RESET);
    }
}

void Mitsuba::SetDirection(Mitsuba_DirectionTypeDef direction) {
    if (direction == FORWARD) {
        HAL_GPIO_WritePin(_fwd_rev_gpio_port, _fwd_rev_gpio_pin, GPIO_PIN_SET);
    } else {
         HAL_GPIO_WritePin(_fwd_rev_gpio_port, _fwd_rev_gpio_pin, GPIO_PIN_RESET);
    }
}

HAL_StatusTypeDef Mitsuba::SetThrottle(uint16_t value) {
    return _throttle_dac->SetValue(value);
}

HAL_StatusTypeDef Mitsuba::SetRegen(uint16_t value) {
    return _regen_dac->SetValue(value);
}