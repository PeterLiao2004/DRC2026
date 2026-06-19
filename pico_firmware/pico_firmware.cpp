#include <stdio.h>
#include "pico/stdlib.h"

// Motor driver pins
#define M1A 2   // GP2
#define M1B 3   // GP3
#define M2A 4   // GP4
#define M2B 5   // GP5

void stop_all() {
    gpio_put(M1A, 0);
    gpio_put(M1B, 0);
    gpio_put(M2A, 0);
    gpio_put(M2B, 0);
}

void motor1_forward() {
    gpio_put(M1A, 1);
    gpio_put(M1B, 0);
}

void motor1_reverse() {
    gpio_put(M1A, 0);
    gpio_put(M1B, 1);
}

void motor2_forward() {
    gpio_put(M2A, 1);
    gpio_put(M2B, 0);
}

void motor2_reverse() {
    gpio_put(M2A, 0);
    gpio_put(M2B, 1);
}

int main() {
    stdio_init_all();

    gpio_init(M1A);
    gpio_init(M1B);
    gpio_init(M2A);
    gpio_init(M2B);

    gpio_set_dir(M1A, GPIO_OUT);
    gpio_set_dir(M1B, GPIO_OUT);
    gpio_set_dir(M2A, GPIO_OUT);
    gpio_set_dir(M2B, GPIO_OUT);

    // Safety: start stopped
    stop_all();
    sleep_ms(2000);

    while (true) {
        printf("Motor 1 forward\n");
        motor1_forward();
        sleep_ms(2000);

        printf("Stop\n");
        stop_all();
        sleep_ms(1000);

        printf("Motor 1 reverse\n");
        motor1_reverse();
        sleep_ms(2000);

        printf("Stop\n");
        stop_all();
        sleep_ms(1000);

        printf("Motor 2 forward\n");
        motor2_forward();
        sleep_ms(2000);

        printf("Stop\n");
        stop_all();
        sleep_ms(1000);

        printf("Motor 2 reverse\n");
        motor2_reverse();
        sleep_ms(2000);

        printf("Stop\n");
        stop_all();
        sleep_ms(3000);
    }
}