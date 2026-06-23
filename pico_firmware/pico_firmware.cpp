#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include <cstring>

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

// PID control settings
#define KP 0.5f
#define BASE_SPEED_PERCENT 30

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


//--------------------Serial Communication------------------//
int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

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
        m2_encoder_count += QUADRATURE_DELTA[(m2_encoder_state << 2) | current];
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
    // RPM x10 avoids relying on floating-point printf support.
    int64_t magnitude = delta_counts < 0 ? -(int64_t)delta_counts : delta_counts;
    return static_cast<int32_t>((magnitude * 600000) /
                                (ENCODER_COUNTS_PER_REV * sample_ms));
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
    motor2_set_percent(speed_percent);
}

void drive_backward(int speed_percent) {
    motor1_set_percent(-speed_percent);
    motor2_set_percent(-speed_percent);
}

void turn_left(int speed_percent) {
    // Left turn: one motor backwards, one motor forwards
    motor1_set_percent(-speed_percent);
    motor2_set_percent(speed_percent);
}

void turn_right(int speed_percent) {
    // Right turn: one motor forwards, one motor backwards
    motor1_set_percent(speed_percent);
    motor2_set_percent(-speed_percent);
}

//-------------------PID Control ---------------------------------//
// This is a simple proportional controller that adjusts motor speeds based on an error value.
void pid_drive(float error, int base_speed) {
    float Kp = KP;  // steering strength, tune this later

    int correction = static_cast<int>(Kp * error);

    int left_speed = base_speed - correction;
    int right_speed = base_speed + correction;

    // Clamp speeds to -100 to 100
    left_speed = clamp_int(left_speed, -100, 100);
    right_speed = clamp_int(right_speed, -100, 100);

    // Your motors are mounted opposite directions
    motor1_set_percent(left_speed);
    motor2_set_percent(right_speed);

    printf("error: %.2f, base: %d, left: %d, right: %d\n",
           error, base_speed, left_speed, right_speed);
}

//------------------------Testing/Helpers -------------------------//
// Helper function to print RPM in a human-friendly format.
void print_rpm(int motor, int32_t delta_counts) {
    int32_t rpm_x10 = counts_to_rpm_x10(delta_counts, RPM_SAMPLE_MS);
    printf("M%d: %ld counts, %ld.%01ld RPM",
           motor,
           static_cast<long>(delta_counts),
           static_cast<long>(rpm_x10 / 10),
           static_cast<long>(rpm_x10 % 10));
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

    run_drive_step("Forward", drive_forward, TEST_SPEED_PERCENT, LED1, 1500);
    run_drive_step("Backward", drive_backward, TEST_SPEED_PERCENT, LED2, 1500);
    run_drive_step("Turn left", turn_left, TEST_SPEED_PERCENT, LED3, 1000);
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
    0.0f,    // straight
    0.2f,    // slight correction one way
    0.5f,    // stronger correction one way
    0.0f,    // straight again
    -0.2f,   // slight correction other way
    -0.5f,   // stronger correction other way
    0.0f     // straight
};

const int num_errors = sizeof(dummy_errors) / sizeof(dummy_errors[0]);
//------------------------Main Loop-------------------------//
int main() {
    // Initialize stdio for printf debugging (over USB).
    stdio_init_all();

    // Configure every GPIO and leave all outputs in a safe state.
    setup_gpio();
    sleep_ms(3000);

    // Testing code
    //run_led_test();
    run_hardware_test();

    printf("Pico ready. Send values from -100 to 100.\n");

    char line[SERIAL_BUFFER_SIZE];

    while (true) {
        if (read_serial_line(line, SERIAL_BUFFER_SIZE)) {
            set_status_led(true);
            printf("Received from Pi: %s\n", line);

            int value = atoi(line);
            value = clamp_int(value, -100, 100);


            pid_drive(value, BASE_SPEED_PERCENT);
        }

        sleep_ms(5);
        set_status_led(false);
    }
}
