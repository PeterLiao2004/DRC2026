#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <cstdlib>

// Motor driver pins
#define M1A 2
#define M1B 3
#define M2A 4
#define M2B 5

// PWM settings
#define PWM_WRAP 1000
#define MIN_PWM 350
#define MAX_PWM 1000

void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
}

void set_pwm(uint pin, int duty) {
    if (duty < 0) duty = 0;
    if (duty > PWM_WRAP) duty = PWM_WRAP;

    pwm_set_gpio_level(pin, duty);
}

int speed_to_pwm(int speed_percent) {
    // speed_percent range: 0 to 100
    if (speed_percent <= 0) return 0;
    if (speed_percent > 100) speed_percent = 100;

    return MIN_PWM + (speed_percent * (MAX_PWM - MIN_PWM)) / 100;
}

void motor1_set_percent(int speed_percent) {
    // Positive = forward, negative = reverse
    int pwm = speed_to_pwm(abs(speed_percent));

    if (speed_percent > 0) {
        set_pwm(M1A, pwm);
        set_pwm(M1B, 0);
    } else if (speed_percent < 0) {
        set_pwm(M1A, 0);
        set_pwm(M1B, pwm);
    } else {
        set_pwm(M1A, 0);
        set_pwm(M1B, 0);
    }
}

void motor2_set_percent(int speed_percent) {
    // Positive = forward, negative = reverse
    int pwm = speed_to_pwm(abs(speed_percent));

    if (speed_percent > 0) {
        set_pwm(M2A, pwm);
        set_pwm(M2B, 0);
    } else if (speed_percent < 0) {
        set_pwm(M2A, 0);
        set_pwm(M2B, pwm);
    } else {
        set_pwm(M2A, 0);
        set_pwm(M2B, 0);
    }
}

void stop_all() {
    motor1_set_percent(0);
    motor2_set_percent(0);
}

void drive_forward(int speed_percent) {
    motor1_set_percent(speed_percent);
    motor2_set_percent(-speed_percent);
}

void drive_backward(int speed_percent) {
    motor1_set_percent(-speed_percent);
    motor2_set_percent(speed_percent);
}

void turn_left(int speed_percent) {
    // Left turn: one motor backwards, one motor forwards
    motor1_set_percent(-speed_percent);
    motor2_set_percent(-speed_percent);
}

void turn_right(int speed_percent) {
    // Right turn: one motor forwards, one motor backwards
    motor1_set_percent(speed_percent);
    motor2_set_percent(speed_percent);
}

int main() {
    stdio_init_all();

    setup_pwm_pin(M1A);
    setup_pwm_pin(M1B);
    setup_pwm_pin(M2A);
    setup_pwm_pin(M2B);

    stop_all();
    sleep_ms(3000);

    while (true) {
        printf("Driving slowly forward\n");
        drive_forward(20);   // slow forward
        sleep_ms(2000);

        printf("Stop\n");
        stop_all();
        sleep_ms(1000);

        printf("Turning left\n");
        turn_left(25);
        sleep_ms(800);

        printf("Stop\n");
        stop_all();
        sleep_ms(1000);

        printf("Turning right\n");
        turn_right(25);
        sleep_ms(800);

        printf("Stop\n");
        stop_all();
        sleep_ms(2000);
    }
}