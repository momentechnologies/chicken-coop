#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>

// #define NRF_GPIO_PIN_MAP(port, pin)   ((port << 5) | (pin & 0x1F))

#define motor_step NRF_GPIO_PIN_MAP(1, 4)
#define motor_dir NRF_GPIO_PIN_MAP(1, 7)
#define motor_enable NRF_GPIO_PIN_MAP(1, 10)
#define steps_to_endstop 100
#define stepper_speed 800 // 800us between steps

void stepper_init();
void stepper_run(bool dir);

#endif