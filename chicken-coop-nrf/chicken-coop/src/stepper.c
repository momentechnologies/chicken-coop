#include "stepper.h"

#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>

void stepper_init(void)
{
    nrf_gpio_cfg_output(motor_step);
    nrf_gpio_cfg_output(motor_dir);
    nrf_gpio_cfg_output(motor_enable);

	// Set the direction
	nrf_gpio_pin_set(motor_dir);

	// Disable the motor
	nrf_gpio_pin_set(motor_enable);
}

void stepper_run(bool dir)
{
    // Enable the motor
    nrf_gpio_pin_clear(motor_enable);

    // Set the direction based on input
    if (dir)
    {
        nrf_gpio_pin_set(motor_dir);
    }
    else
    {
        nrf_gpio_pin_clear(motor_dir);
    }

    // Run the motors
    for (int i = 0; i < steps_to_endstop; i++)
    {
        nrf_gpio_pin_set(motor_step);
        k_usleep(stepper_speed);
        nrf_gpio_pin_clear(motor_step);
        k_usleep(stepper_speed);
    }

    // Disable motor
    nrf_gpio_pin_set(motor_enable);
}