#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <cstdlib>
#include <cstdint>

// Motor driver pins
#define M1A 2
#define M1B 3
#define M2A 4
#define M2B 5

// Motor Encoder Pins
#define M1_ENCODER_A 6
#define M1_ENCODER_B 7
#define M2_ENCODER_A 8
#define M2_ENCODER_B 9

// Signed x4 quadrature counts (every A/B edge is counted).
volatile int32_t m1_encoder_count = 0;
volatile int32_t m2_encoder_count = 0;
volatile uint8_t m1_encoder_state = 0;
volatile uint8_t m2_encoder_state = 0;

// Previous AB state in the upper two bits, current AB state in the lower two.
// Invalid transitions (usually contact bounce or missed edges) contribute zero.
static const int8_t QUADRATURE_DELTA[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

uint8_t read_encoder_state(uint a_pin, uint b_pin) {
    return static_cast<uint8_t>((gpio_get(a_pin) << 1) | gpio_get(b_pin));
}

void encoder_gpio_callback(uint gpio, uint32_t events) {
    (void)events;

    if (gpio == M1_ENCODER_A || gpio == M1_ENCODER_B) {
        uint8_t current = read_encoder_state(M1_ENCODER_A, M1_ENCODER_B);
        m1_encoder_count += QUADRATURE_DELTA[(m1_encoder_state << 2) | current];
        m1_encoder_state = current;
    } else if (gpio == M2_ENCODER_A || gpio == M2_ENCODER_B) {
        uint8_t current = read_encoder_state(M2_ENCODER_A, M2_ENCODER_B);
        m2_encoder_count += QUADRATURE_DELTA[(m2_encoder_state << 2) | current];
        m2_encoder_state = current;
    }
}

void setup_encoders() {
    const uint pins[] = {M1_ENCODER_A, M1_ENCODER_B, M2_ENCODER_A, M2_ENCODER_B};

    for (uint pin : pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    m1_encoder_state = read_encoder_state(M1_ENCODER_A, M1_ENCODER_B);
    m2_encoder_state = read_encoder_state(M2_ENCODER_A, M2_ENCODER_B);

    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(M1_ENCODER_A, edges, true, &encoder_gpio_callback);
    gpio_set_irq_enabled(M1_ENCODER_B, edges, true);
    gpio_set_irq_enabled(M2_ENCODER_A, edges, true);
    gpio_set_irq_enabled(M2_ENCODER_B, edges, true);
}

// PWM se#define ENCODER_COUNTS_PER_REV 893
#define RPM_SAMPLE_MS 500
#define SAMPLES_PER_LEVEL 8

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

void drive_forward_pwm(int pwm) {
    // The motors are mounted in opposite directions on the drivetrain.
    set_pwm(M1A, pwm);
    set_pwm(M1B, 0);
    set_pwm(M2A, 0);
    set_pwm(M2B, pwm);
}

void read_encoder_counts(int32_t &m1, int32_t &m2) {
    uint32_t irq_state = save_and_disable_interrupts();
    m1 = m1_encoder_count;
    m2 = m2_encoder_count;
    restore_interrupts(irq_state);
}

int32_t counts_to_rpm_x10(int32_t delta_counts, uint32_t sample_ms) {
    // RPM x10 avoids relying on floating-point printf support.
    int64_t magnitude = delta_counts < 0 ? -(int64_t)delta_counts : delta_counts;
    return static_cast<int32_t>((magnitude * 600000) /
                                (ENCODER_COUNTS_PER_REV * sample_ms));
}

void print_rpm(int motor, int32_t delta_counts) {
    int32_t rpm_x10 = counts_to_rpm_x10(delta_counts, RPM_SAMPLE_MS);
    printf("M%d: %ld counts, %ld.%01ld RPM",
           motor,
           static_cast<long>(delta_counts),
           static_cast<long>(rpm_x10 / 10),
           static_cast<long>(rpm_x10 % 10));
}

void run_motor_speed_test() {
    // Raw PWM levels include the measured start threshold and span the range.
    const int pwm_levels[] = {350, 400, 500, 600, 700, 850, 1000};

    printf("\nMotor speed sweep: %d encoder counts/rev, %d ms samples\n",
           ENCODER_COUNTS_PER_REV, RPM_SAMPLE_MS);
    printf("Keep the wheels safely clear. Starting in 2 seconds...\n");
    sleep_ms(2000);

    for (int pwm : pwm_levels) {
        int percent = (pwm * 100) / PWM_WRAP;
        printf("\n--- PWM %d/%d (%d%% duty) ---\n", pwm, PWM_WRAP, percent);

        int32_t previous_m1;
        int32_t previous_m2;
        read_encoder_counts(previous_m1, previous_m2);
        drive_forward_pwm(pwm);

        for (int sample = 1; sample <= SAMPLES_PER_LEVEL; ++sample) {
            sleep_ms(RPM_SAMPLE_MS);

            int32_t current_m1;
            int32_t current_m2;
            read_encoder_counts(current_m1, current_m2);

            printf("%d.%01d s\t", sample / 2, (sample % 2) * 5);
            print_rpm(1, current_m1 - previous_m1);
            printf("\t");
            print_rpm(2, current_m2 - previous_m2);
            printf("\n");

            previous_m1 = current_m1;
            previous_m2 = current_m2;
        }

        stop_all();
        printf("Stopped\n");
        sleep_ms(1500);
    }

    stop_all();
    printf("\nSweep complete. Motors stopped. Press R to run it again.\n");
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
    setup_encoders();

    stop_all();
    sleep_ms(3000);

    run_motor_speed_test();

    while (true) {
        int input = getchar_timeout_us(0);
        if (input == 'r' || input == 'R') run_motor_speed_test();
     if (m1 != last_m1 || m2 != last_m2) {
            printf("M1: %ld\tM2: %ld\n", static_cast<long>(m1), static_cast<long>(m2));
            last_m1 = m1;
            last_m2 = m2;
        }

        sleep_ms(20);
    }
}
