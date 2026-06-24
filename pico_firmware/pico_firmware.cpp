#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <cstdlib>
#include <cstdint>

//---------------------Defines------------------//
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

// PWM settings
#define PWM_WRAP 1000
#define MIN_PWM 350
#define MAX_PWM 1000
#define ENCODER_COUNTS_PER_REV 893
#define RPM_SAMPLE_MS 500
#define SAMPLES_PER_LEVEL 8

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

//----------------------------------------------------//


//-------------------Encoder Handling------------------//
// This reads the two encoder pins and turns them into one number from 0 to 3.
uint8_t read_encoder_state(uint a_pin, uint b_pin) {
    return static_cast<uint8_t>((gpio_get(a_pin) << 1) | gpio_get(b_pin));
}
// Encoder GPIO ISR: automatically runs whenever an encoder pin changes. Updates encoder counts and state.
void encoder_gpio_callback(uint gpio, uint32_t events) {
    (void)events;

    if (gpio == M1_ENCODER_A || gpio == M1_ENCODER_B) {
        uint8_t current = read_encoder_state(M1_ENCODER_A, M1_ENCODER_B);
        m1_encoder_count += QUADRATURE_DELTA[(m1_encoder_state << 2) | current];
        m1_encoder_state = current;
    } else if (gpio == M2_ENCODER_A || gpio == M2_ENCODER_B) {
        uint8_t current = read_encoder_state(M2_ENCODER_A, M2_ENCODER_B);
        // Motor 2 is mounted/wired in the opposite direction, so invert its
        // encoder delta to keep positive counts consistent between motors.
        m2_encoder_count -= QUADRATURE_DELTA[(m2_encoder_state << 2) | current];
        m2_encoder_state = current;
    }
}

// Initializes encoder GPIOs and sets up interrupts to track encoder counts.
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

// Atomically reads the encoder counts. Should be called at a regular interval to track speed.
void read_encoder_counts(int32_t &m1, int32_t &m2) {
    uint32_t irq_state = save_and_disable_interrupts();
    m1 = m1_encoder_count;
    m2 = m2_encoder_count;
    restore_interrupts(irq_state);
}

void reset_encoder_counts() {
    uint32_t irq_state = save_and_disable_interrupts();
    m1_encoder_count = 0;
    m2_encoder_count = 0;
    restore_interrupts(irq_state);
}

// Converts change in encoder counts over a sample period to RPM x10 (to avoid floating-point).
int32_t counts_to_rpm_x10(int32_t delta_counts, uint32_t sample_ms) {
    // Keep the sign so negative RPM continues to indicate reverse movement.
    return static_cast<int32_t>(
        (static_cast<int64_t>(delta_counts) * 600000) /
        (static_cast<int64_t>(ENCODER_COUNTS_PER_REV) * sample_ms));
}
//----------------------------------------------------//
//-------------------PWM/Motor Control------------------//

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
//-------------------High-Level Motor Control-----------------//

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

//------------------------Testing/Helpers -------------------------//
// Prints signed RPM to one decimal place without floating-point printf support.
void print_rpm(const char *motor, int32_t delta_counts, uint32_t sample_ms) {
    int32_t rpm_x10 = counts_to_rpm_x10(delta_counts, sample_ms);
    int64_t magnitude = rpm_x10 < 0
        ? -static_cast<int64_t>(rpm_x10)
        : static_cast<int64_t>(rpm_x10);

    printf("%s RPM: %s%lld.%01lld",
           motor,
           rpm_x10 < 0 ? "-" : "",
           static_cast<long long>(magnitude / 10),
           static_cast<long long>(magnitude % 10));
}

// Prints cumulative wheel rotations to three decimal places without requiring
// floating-point printf support.
void print_rotations(const char *motor, int32_t counts) {
    int64_t rotations_x1000 =
        (static_cast<int64_t>(counts) * 1000) / ENCODER_COUNTS_PER_REV;
    uint64_t magnitude = rotations_x1000 < 0
        ? static_cast<uint64_t>(-rotations_x1000)
        : static_cast<uint64_t>(rotations_x1000);

    printf("%s: %s%llu.%03llu rotations",
           motor,
           rotations_x1000 < 0 ? "-" : "",
           static_cast<unsigned long long>(magnitude / 1000),
           static_cast<unsigned long long>(magnitude % 1000));
}
//------------------------Main Loop-------------------------//
int main() {
    // Initialize stdio for printf debugging (over USB).
    stdio_init_all();

    // Set up PWM pins and encoder GPIOs with interrupts.
    setup_pwm_pin(M1A);
    setup_pwm_pin(M1B);
    setup_pwm_pin(M2A);
    setup_pwm_pin(M2B);
    setup_encoders();

    // Initialize all motors to stopped
    stop_all();
    sleep_ms(3000);

    reset_encoder_counts();
    int32_t last_m1 = 0;
    int32_t last_m2 = 0;
    int32_t rpm_sample_m1 = 0;
    int32_t rpm_sample_m2 = 0;
    uint64_t last_rpm_sample_us = time_us_64();

    printf("\nManual wheel rotation test\n");
    printf("Rotate either wheel by hand. RPM updates every %d ms. Press R to reset.\n",
           RPM_SAMPLE_MS);

    while (true) {
        int input = getchar_timeout_us(0);
        if (input == 'r' || input == 'R') {
            reset_encoder_counts();
            last_m1 = 0;
            last_m2 = 0;
            rpm_sample_m1 = 0;
            rpm_sample_m2 = 0;
            last_rpm_sample_us = time_us_64();
            printf("Rotations reset: M1 = 0.000, M2 = 0.000\n");
        }

        int32_t m1;
        int32_t m2;
        read_encoder_counts(m1, m2);

        if (m1 != last_m1 || m2 != last_m2) {
            // print_rotations("M1", m1);
            // printf("\t");
            // print_rotations("M2", m2);
            // printf("\n");
            last_m1 = m1;
            last_m2 = m2;
        }

        uint64_t now_us = time_us_64();
        uint64_t elapsed_us = now_us - last_rpm_sample_us;
        if (elapsed_us >= static_cast<uint64_t>(RPM_SAMPLE_MS) * 1000) {
            uint32_t elapsed_ms = static_cast<uint32_t>(elapsed_us / 1000);

            print_rpm("M1", m1 - rpm_sample_m1, elapsed_ms);
            printf("\t");
            print_rpm("M2", m2 - rpm_sample_m2, elapsed_ms);
            printf("\n");

            rpm_sample_m1 = m1;
            rpm_sample_m2 = m2;
            last_rpm_sample_us = now_us;
        }

        sleep_ms(20);
    }
}
