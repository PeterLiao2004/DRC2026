#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Motor pins
#define M1A 2
#define M1B 3
#define M2A 4
#define M2B 5

// PWM range: 0 to 1000
#define PWM_WRAP 1000

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

void stop_all() {
    set_pwm(M1A, 0);
    set_pwm(M1B, 0);
    set_pwm(M2A, 0);
    set_pwm(M2B, 0);
}

void motor1_set_speed(int speed) {
    // speed range: -1000 to +1000
    if (speed > 0) {
        set_pwm(M1A, speed);
        set_pwm(M1B, 0);
    } else if (speed < 0) {
        set_pwm(M1A, 0);
        set_pwm(M1B, -speed);
    } else {
        set_pwm(M1A, 0);
        set_pwm(M1B, 0);
    }
}

void motor2_set_speed(int speed) {
    // speed range: -1000 to +1000
    if (speed > 0) {
        set_pwm(M2A, speed);
        set_pwm(M2B, 0);
    } else if (speed < 0) {
        set_pwm(M2A, 0);
        set_pwm(M2B, -speed);
    } else {
        set_pwm(M2A, 0);
        set_pwm(M2B, 0);
    }
}

void test_motor1_forward() {
    printf("Testing Motor 1 forward\n");

    for (int pwm = 100; pwm <= 1000; pwm += 100) {
        printf("Motor 1 PWM: %d\n", pwm);
        motor1_set_speed(pwm);
        sleep_ms(2000);
    }

    stop_all();
    sleep_ms(2000);
}

void test_motor2_forward() {
    printf("Testing Motor 2 forward\n");

    for (int pwm = 100; pwm <= 1000; pwm += 100) {
        printf("Motor 2 PWM: %d\n", pwm);
        motor2_set_speed(pwm);
        sleep_ms(2000);
    }

    stop_all();
    sleep_ms(2000);
}

void test_both_forward() {
    printf("Testing both motors forward\n");

    for (int pwm = 100; pwm <= 1000; pwm += 100) {
        printf("Both motors PWM: %d\n", pwm);
        motor1_set_speed(pwm);
        motor2_set_speed(pwm);
        sleep_ms(2000);
    }

    stop_all();
    sleep_ms(2000);
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
        test_motor1_forward();
        test_motor2_forward();
        test_both_forward();

        printf("PWM ramp test complete. Restarting in 5 seconds...\n");
        stop_all();
        sleep_ms(5000);
    }
}