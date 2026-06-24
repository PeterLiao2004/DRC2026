#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>

//----------------Hardware Definitions------------------//
// Wheel Diameter
#define WHEEL_DIAMETER_MM 65
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159)
#define MM_PER_REV WHEEL_CIRCUMFERENCE_MM

// Wheelbase (distance between the two wheels)
#define WHEELBASE_MM 240

//--------------------GPIO Pin Definitions------------------//
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

//Buttons
#define BUTTON1 10
#define BUTTON2 11

//LED Pins
#define LED1 12
#define LED2 13
#define LED3 14
#define LED4 15
#define STATUS_LED_PIN PICO_DEFAULT_LED_PIN

//LED Strip Pins
#define LED_STRIP1 16
#define LED_STRIP2 17

//Custom Pins (for future use, e.g. sensors)
#define CUSTOM1 18
#define CUSTOM2 19
#define CUSTOM3 20
//--------------------Control Parameters------------------//
// PWM settings
#define PWM_WRAP 1000
#define MIN_PWM 350
#define MAX_PWM 1000
#define ENCODER_COUNTS_PER_REV 893
#define RPM_SAMPLE_MS 500
#define SAMPLES_PER_LEVEL 8

// Speed/PID control settings
#define DERIVATIVE_FILTER_ALPHA 0.2f
#define INTEGRAL_LIMIT 200.0f
#define MAX_PID_GAIN 10.0f

float line_kp = 0.2f;
float line_ki = 0.0f;
float line_kd = 0.1f;

float last_error = 0.0f;
float integral = 0.0f;
float filtered_derivative = 0.0f;
uint64_t last_control_time_us = 0;
bool derivative_initialized = false;

#define BASE_SPEED_PERCENT 30
#define MIN_SPEED_PERCENT 10
#define MAX_SPEED_PERCENT 100

// How much to reduce base speed at maximum error (100% error = 40% of base speed). This allows more time for correction when the error is large.
float min_factor = 1.0f;

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

//---------------------Serial communication------------------//
#define SERIAL_BUFFER_SIZE 32


//=====================Functions============================//
//--------------------Setup/Initialization------------------//
// Forward declarations for the hardware-specific setup functions below.
void setup_pwm_pin(uint pin);
void setup_encoders();
void stop_all();

void setup_gpio() {
    // Motor driver outputs use the PWM peripheral.
    const uint motor_pins[] = {M1A, M1B, M2A, M2B};
    for (uint pin : motor_pins) {
        setup_pwm_pin(pin);
    }

    // Encoder inputs and their edge interrupts.
    setup_encoders();

    // Buttons are active-low: pressed reads 0, released reads 1.
    const uint button_pins[] = {BUTTON1, BUTTON2};
    for (uint pin : button_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    // Status LEDs start switched off.
    const uint led_pins[] = {LED1, LED2, LED3, LED4, STATUS_LED_PIN};
    for (uint pin : led_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, false);
    }

    // Hold the LED-strip data pins low until strip control is implemented.
    const uint led_strip_pins[] = {LED_STRIP1, LED_STRIP2};
    for (uint pin : led_strip_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, false);
    }

    // Leave future-use pins as high-impedance inputs.
    const uint custom_pins[] = {CUSTOM1, CUSTOM2, CUSTOM3};
    for (uint pin : custom_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    }

    // Ensure the robot cannot move as initialization completes.
    stop_all();
}

//---------------------LED Control------------------//
void set_led(uint led_pin, bool on) {
    gpio_put(led_pin, on);
}

void toggle_led(uint led_pin) {
    set_led(led_pin, !gpio_get_out_level(led_pin));
}

bool is_led_on(uint led_pin) {
    return gpio_get_out_level(led_pin);
}

void set_status_led(bool on) {
    set_led(STATUS_LED_PIN, on);
}

//---------------------Button Control------------------//
// Buttons use pull-ups, so a LOW input means the button is pressed.
bool is_button_pressed(uint button_pin) {
    return gpio_get(button_pin) == 0;
}

//---------------------Utility Functions------------------//
int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

float clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

float map_range(float value,
                float input_min,
                float input_max,
                float output_min,
                float output_max) {
    const float input_range = input_max - input_min;

    if (input_range == 0.0f) {
        return output_min;
    }

    const float normalized_value = (value - input_min) / input_range;
    const float output_range = output_max - output_min;

    return output_min + normalized_value * output_range;
}

//--------------------Serial Communication------------------//

bool read_serial_line(char *buffer, int buffer_size) {
    static int index = 0;

    int ch = getchar_timeout_us(0);

    while (ch != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            if (index > 0) {
                buffer[index] = '\0';
                index = 0;
                return true;
            }
        } else {
            if (index < buffer_size - 1) {
                buffer[index++] = (char)ch;
            } else {
                // Buffer overflow protection
                index = 0;
            }
        }

        ch = getchar_timeout_us(0);
    }

    return false;
}

bool parse_drive_command(const char *line, float &error, int &base_speed) {
    if ((line[0] != 'D' && line[0] != 'd') || line[1] != ',') {
        return false;
    }

    char *end = nullptr;
    const char *error_start = line + 2;
    error = std::strtof(error_start, &end);
    if (end == error_start || !std::isfinite(error)) {
        return false;
    }

    while (*end == ' ' || *end == '\t') ++end;
    if (*end != ',') return false;

    const char *speed_start = end + 1;
    long parsed_speed = std::strtol(speed_start, &end, 10);
    if (end == speed_start) return false;

    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return false;

    if (error < -100.0f || error > 100.0f ||
        parsed_speed < 0 || parsed_speed > 100) {
        return false;
    }

    base_speed = static_cast<int>(parsed_speed);
    return true;
}

bool parse_float_field(const char *&cursor, float &value, bool final_field) {
    char *end = nullptr;
    value = std::strtof(cursor, &end);

    if (end == cursor || !std::isfinite(value)) {
        return false;
    }

    while (*end == ' ' || *end == '\t') ++end;

    if (final_field) {
        return *end == '\0';
    }

    if (*end != ',') {
        return false;
    }

    cursor = end + 1;
    return true;
}

bool parse_pid_command(const char *line, float &kp, float &ki, float &kd) {
    bool valid_prefix =
        (line[0] == 'P' && line[1] == 'I' && line[2] == 'D' && line[3] == ',') ||
        (line[0] == 'p' && line[1] == 'i' && line[2] == 'd' && line[3] == ',');

    if (!valid_prefix) {
        return false;
    }

    const char *cursor = line + 4;
    if (!parse_float_field(cursor, kp, false) ||
        !parse_float_field(cursor, ki, false) ||
        !parse_float_field(cursor, kd, true)) {
        return false;
    }

    return kp >= 0.0f && kp <= MAX_PID_GAIN &&
           ki >= 0.0f && ki <= MAX_PID_GAIN &&
           kd >= 0.0f && kd <= MAX_PID_GAIN;
}

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
        // Invert M2 so forward wheel movement has the same sign as M1.
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

// Converts change in encoder counts over a sample period to RPM x10 (to avoid floating-point).
int32_t counts_to_rpm_x10(int32_t delta_counts, uint32_t sample_ms) {
    // RPM x10 avoids relying on floating-point printf support while retaining
    // the direction of travel.
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

void set_turn_indicators(bool left_on, bool right_on) {
    set_led(LED1, left_on);
    set_led(LED4, right_on);
}

void set_motion_indicators(bool forward_on, bool reverse_on) {
    set_led(LED2, forward_on);
    set_led(LED3, reverse_on);
}

void stop_all() {
    motor1_set_percent(0);
    motor2_set_percent(0);
    set_turn_indicators(false, false);
    set_motion_indicators(false, false);

    // Do not carry stale PID history into the next drive command.
    last_error = 0.0f;
    integral = 0.0f;
    filtered_derivative = 0.0f;
    last_control_time_us = 0;
    derivative_initialized = false;
}

void drive_forward(int speed_percent) {
    set_turn_indicators(false, false);
    set_motion_indicators(true, false);
    motor1_set_percent(speed_percent);
    motor2_set_percent(speed_percent);
}

void drive_backward(int speed_percent) {
    set_turn_indicators(false, false);
    set_motion_indicators(false, true);
    motor1_set_percent(-speed_percent);
    motor2_set_percent(-speed_percent);
}

void turn_left(int speed_percent) {
    // Left turn: one motor backwards, one motor forwards
    set_turn_indicators(true, false);
    set_motion_indicators(false, false);
    motor1_set_percent(-speed_percent);
    motor2_set_percent(speed_percent);
}

void turn_right(int speed_percent) {
    // Right turn: one motor forwards, one motor backwards
    set_turn_indicators(false, true);
    set_motion_indicators(false, false);
    motor1_set_percent(speed_percent);
    motor2_set_percent(-speed_percent);
}

//-------------------Speed/PID Control ---------------------------------//
// Speed based on error magnitude: higher error = slower base speed, to allow more correction time.
int calculate_scaled_speed(int error, int base_speed) {
    // error: -100 to +100
    // pi_base_speed: 0 to 100

    error = clamp_int(error, -100, 100);
    base_speed = clamp_int(base_speed, 0, 100);

    if (base_speed == 0) return 0;

    int abs_error = abs(error);

    // Reduce speed as error increases.
    float error_factor = 1.0f - (abs_error / 100.0f) * (1.0f - min_factor);
    error_factor = clamp_float(error_factor, min_factor, 1.0f);

    int adjusted_base_speed = static_cast<int>(base_speed * error_factor);

    return clamp_int(adjusted_base_speed, MIN_SPEED_PERCENT, MAX_SPEED_PERCENT);
}

// Proportional-integral-derivative steering controller.
void pid_drive(float error, int base_speed) {
    // Stop if base speed is zero
    if (base_speed <= 0) {
        stop_all();
        return;
    }

    int scaled_speed = calculate_scaled_speed(static_cast<int>(error), base_speed);

    uint64_t now_us = to_us_since_boot(get_absolute_time());
    float derivative = 0.0f;
    float dt_seconds = 0.0f;

    if (derivative_initialized) {
        dt_seconds = (now_us - last_control_time_us) / 1000000.0f;

        // Ignore samples that are too close together or follow a long pause.
        if (dt_seconds >= 0.005f && dt_seconds <= 0.25f) {
            float raw_derivative = (error - last_error) / dt_seconds;
            filtered_derivative =
                DERIVATIVE_FILTER_ALPHA * raw_derivative +
                (1.0f - DERIVATIVE_FILTER_ALPHA) * filtered_derivative;
            derivative = filtered_derivative;

            // Trapezoidal integration with a limit to prevent windup.
            integral += 0.5f * (error + last_error) * dt_seconds;
            integral = clamp_float(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        } else {
            filtered_derivative = 0.0f;
        }
    }

    last_error = error;
    last_control_time_us = now_us;
    derivative_initialized = true;

    float p_term = line_kp * error;
    float i_term = line_ki * integral;
    float d_term = line_kd * derivative;
    int correction = static_cast<int>(std::round(p_term + i_term + d_term));
    correction = clamp_int(correction, -100, 100);

    int left_speed = scaled_speed + correction;
    int right_speed = scaled_speed - correction;

    // Clamp speeds to -100 to 100
    left_speed = clamp_int(left_speed, -100, 100);
    right_speed = clamp_int(right_speed, -100, 100);

    if (correction > 0) {
        set_turn_indicators(false, true);
    } else if (correction < 0) {
        set_turn_indicators(true, false);
    } else {
        set_turn_indicators(false, false);
    }

    if (scaled_speed > 0) {
        set_motion_indicators(true, false);
    } else if (scaled_speed < 0) {
        set_motion_indicators(false, true);
    } else {
        set_motion_indicators(false, false);
    }

    // Your motors are mounted opposite directions
    motor1_set_percent(left_speed);
    motor2_set_percent(right_speed);

    printf("error: %.2f, P: %.2f, I: %.2f, D: %.2f, base: %d, left: %d, right: %d\n",
           error, p_term, i_term, d_term, scaled_speed, left_speed, right_speed);
}

//------------------------Testing/Helpers -------------------------//
// Prints signed RPM to one decimal place without floating-point printf support.
void print_rpm(int motor, int32_t delta_counts, uint32_t sample_ms) {
    int32_t rpm_x10 = counts_to_rpm_x10(delta_counts, sample_ms);
    int64_t magnitude = rpm_x10 < 0
        ? -static_cast<int64_t>(rpm_x10)
        : static_cast<int64_t>(rpm_x10);

    printf("M%d RPM: %s%lld.%01lld",
           motor,
           rpm_x10 < 0 ? "-" : "",
           static_cast<long long>(magnitude / 10),
           static_cast<long long>(magnitude % 10));
}

struct EncoderRpmState {
    int32_t previous_m1 = 0;
    int32_t previous_m2 = 0;
    uint64_t previous_sample_us = 0;
};

void initialize_encoder_rpm(EncoderRpmState &state) {
    read_encoder_counts(state.previous_m1, state.previous_m2);
    state.previous_sample_us = time_us_64();
}

void print_encoder_rpm_if_due(EncoderRpmState &state) {
    uint64_t now_us = time_us_64();
    uint64_t elapsed_us = now_us - state.previous_sample_us;
    if (elapsed_us < static_cast<uint64_t>(RPM_SAMPLE_MS) * 1000) {
        return;
    }

    int32_t current_m1;
    int32_t current_m2;
    read_encoder_counts(current_m1, current_m2);

    uint32_t elapsed_ms = static_cast<uint32_t>(elapsed_us / 1000);
    print_rpm(1, current_m1 - state.previous_m1, elapsed_ms);
    printf("\t");
    print_rpm(2, current_m2 - state.previous_m2, elapsed_ms);
    printf("\n");

    state.previous_m1 = current_m1;
    state.previous_m2 = current_m2;
    state.previous_sample_us = now_us;
}

void run_drive_step(const char *name,
                    void (*movement)(int),
                    int speed_percent,
                    uint led_pin,
                    uint32_t duration_ms) {
    int32_t start_m1;
    int32_t start_m2;
    read_encoder_counts(start_m1, start_m2);

    printf("%s at %d%%\n", name, speed_percent);
    set_led(led_pin, true);
    movement(speed_percent);
    sleep_ms(duration_ms);
    stop_all();
    set_led(led_pin, false);

    int32_t end_m1;
    int32_t end_m2;
    read_encoder_counts(end_m1, end_m2);
    printf("Stopped - encoder change: M1 = %ld, M2 = %ld\n",
           static_cast<long>(end_m1 - start_m1),
           static_cast<long>(end_m2 - start_m2));

    sleep_ms(1000);
}

void run_drive_test() {
    constexpr int TEST_SPEED_PERCENT = 25;

    printf("\nDrive test starting. Keep the area clear.\n");
    sleep_ms(2000);

    run_drive_step("Forward", drive_forward, TEST_SPEED_PERCENT, LED2, 1500);
    run_drive_step("Backward", drive_backward, TEST_SPEED_PERCENT, LED3, 1500);
    run_drive_step("Turn left", turn_left, TEST_SPEED_PERCENT, LED1, 1000);
    run_drive_step("Turn right", turn_right, TEST_SPEED_PERCENT, LED4, 1000);

    stop_all();
    printf("Drive test complete.\n");
}

void set_all_leds(bool on) {
    const uint led_pins[] = {LED1, LED2, LED3, LED4, STATUS_LED_PIN};
    for (uint pin : led_pins) {
        set_led(pin, on);
    }
}

void run_led_test() {
    const uint led_pins[] = {LED1, LED2, LED3, LED4, STATUS_LED_PIN};

    printf("\nLED test: each LED should light in sequence.\n");
    set_all_leds(false);

    for (uint pin : led_pins) {
        set_led(pin, true);
        sleep_ms(400);
        set_led(pin, false);
    }

    printf("LED test: all LEDs on.\n");
    set_all_leds(true);
    sleep_ms(1000);
    set_all_leds(false);
    printf("LED test complete.\n");
}

void run_button_test() {
    constexpr uint32_t BUTTON_TEST_MS = 10000;
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    bool previous_button1 = false;
    bool previous_button2 = false;

    printf("\nButton test: press BUTTON1 and BUTTON2 within 10 seconds.\n");
    printf("LED1 follows BUTTON1; LED2 follows BUTTON2.\n");

    while (to_ms_since_boot(get_absolute_time()) - start_ms < BUTTON_TEST_MS) {
        bool button1_pressed = is_button_pressed(BUTTON1);
        bool button2_pressed = is_button_pressed(BUTTON2);

        set_led(LED1, button1_pressed);
        set_led(LED2, button2_pressed);

        if (button1_pressed != previous_button1) {
            printf("BUTTON1 %s\n", button1_pressed ? "pressed" : "released");
            previous_button1 = button1_pressed;
        }

        if (button2_pressed != previous_button2) {
            printf("BUTTON2 %s\n", button2_pressed ? "pressed" : "released");
            previous_button2 = button2_pressed;
        }

        sleep_ms(20);
    }

    set_led(LED1, false);
    set_led(LED2, false);
    printf("Button test complete.\n");
}

void run_hardware_test() {
    printf("\n=== Hardware self-test ===\n");
    run_led_test();
    run_button_test();

    printf("\nMotor test is next. Keep the wheels raised and area clear.\n");
    sleep_ms(3000);
    run_drive_test();

    set_all_leds(false);
    set_status_led(true);
    printf("=== Hardware self-test complete ===\n\n");
}

// Dummy error values for testing P Control
const float dummy_errors[] = {
    0.0f,     // straight
    -20.0f,   // slight correction left
    -50.0f,   // stronger correction left
    0.0f,     // straight again
    20.0f,    // slight correction right
    50.0f,    // stronger correction right
    0.0f      // straight
};

const int num_errors = sizeof(dummy_errors) / sizeof(dummy_errors[0]);

void run_dummy_pid_test() {
    printf("\nDummy P-control test starting. Keep the wheels raised.\n");
    sleep_ms(2000);

    for (int i = 0; i < num_errors; ++i) {
        float error = dummy_errors[i];
        printf("Dummy error: %.1f\n", error);

        pid_drive(error, BASE_SPEED_PERCENT);
        sleep_ms(1500);

        stop_all();
        sleep_ms(500);
    }

    stop_all();
    printf("Dummy P-control test complete.\n");
}

struct SerialDriveState {
    bool drive_command_active = false;
    bool timeout_active = false;
    uint32_t last_drive_command_ms = 0;
};

bool is_stop_command(const char *line) {
    return strcmp(line, "S") == 0 ||
           strcmp(line, "s") == 0 ||
           strcmp(line, "STOP") == 0 ||
           strcmp(line, "stop") == 0;
}

void handle_serial_command(const char *line, SerialDriveState &state) {
    state.timeout_active = false;
    set_all_leds(false);
    set_status_led(true);
    printf("Received from Pi: %s\n", line);

    if (is_stop_command(line)) {
        stop_all();
        state.drive_command_active = false;
        printf("Stopped\n");
        return;
    }

    float new_kp;
    float new_ki;
    float new_kd;
    if (parse_pid_command(line, new_kp, new_ki, new_kd)) {
        stop_all();
        state.drive_command_active = false;
        line_kp = new_kp;
        line_ki = new_ki;
        line_kd = new_kd;
        printf("PID gains updated: Kp=%.4f, Ki=%.4f, Kd=%.4f\n",
               line_kp, line_ki, line_kd);
        return;
    }

    float error;
    int base_speed;
    if (parse_drive_command(line, error, base_speed)) {
        pid_drive(error, base_speed);
        state.drive_command_active = base_speed > 0;
        state.last_drive_command_ms = to_ms_since_boot(get_absolute_time());
        printf("Drive command: error %.2f, base speed %d%%\n",
               error, base_speed);
        return;
    }

    stop_all();
    state.drive_command_active = false;
    printf("Invalid command. Expected D,error,base_speed, PID,kp,ki,kd, or S\n");
}

void check_serial_timeout(SerialDriveState &state, uint32_t timeout_ms) {
    if (!state.drive_command_active) {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - state.last_drive_command_ms < timeout_ms) {
        return;
    }

    stop_all();
    state.drive_command_active = false;
    state.timeout_active = true;
    set_all_leds(true);
    printf("Stopped: serial command timeout\n");
}

//------------------------Main Loop-------------------------//
int main() {
    constexpr uint32_t SERIAL_COMMAND_TIMEOUT_MS = 500;

    stdio_init_all();
    setup_gpio();
    sleep_ms(3000);

    printf("Pico ready. Send D,error,base_speed (example D,-25.5,30).\n");
    printf("Send PID,kp,ki,kd to tune gains (example PID,0.2,0.0,0.1).\n");
    printf("Send S to stop.\n");

    char line[SERIAL_BUFFER_SIZE];
    SerialDriveState state;
    EncoderRpmState rpm_state;
    initialize_encoder_rpm(rpm_state);

    while (true) {
        if (read_serial_line(line, SERIAL_BUFFER_SIZE)) {
            handle_serial_command(line, state);
        }

        check_serial_timeout(state, SERIAL_COMMAND_TIMEOUT_MS);
        print_encoder_rpm_if_due(rpm_state);
        set_status_led(state.timeout_active);
        sleep_ms(5);
    }
}
