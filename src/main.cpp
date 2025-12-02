#include <Arduino.h>
#include <i2c.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "uart.h"
#include "us_sensor.h"
#include "ir_sensor.h"
#include "imu.h"
#include "servo.h"
#include "fan.h"

// ---------- EXTERNS ----------
extern uint16_t front_cm;
extern uint16_t right_cm;

// ---------- CONSTANTS ----------
#define FRONT_OBS_CM            70.0f
#define TURN_ANGLE              180.0f
#define TURN_DISTANCE           50.0f
#define YAW_TOL_DEG             3.0f

#define SERVO_LEFT_LIMIT        (-180.0f)
#define SERVO_CENTER_ANGLE      0.0f
#define SERVO_RIGHT_LIMIT       (+180.0f)

#define KP_HEADING              3.0f

#define THRUST_FORWARD_PWM      220
#define THRUST_TURN_PWM_MED     200
#define THRUST_STOP_PWM         0

#define TURN_TIMEOUT_TICKS      200
#define TURN_MIN_TICKS          20

#define US_MIN_VALID_CM         10
#define US_MAX_VALID_CM         500

#define SERVO_DEADZONE_DEG      0.6f
#define SERVO_MAX_STEP_DEG      20.0f   // max servo movement per update
#define ERR_ALPHA               0.15f


// ---------- STATES ----------
typedef enum {
    STATE_FORWARD = 0,
    STATE_PRE_TURN_STOP,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_STOP
} HoverState;


// ---------- GLOBALS ----------
static float heading_target_deg = 0.0f;
static float turn_target_deg = 0.0f;
static uint16_t turn_ticks = 0;
static uint16_t turn_timer = 0;
static uint32_t last_servo_update = 0;

// ---- STUCK DETECTION ----
static uint32_t stuck_timer = 0;
static float last_front = 0;
static uint8_t stuck_active = 0;
#define STUCK_TIME_MS        2000   // time without movement before stuck
#define STUCK_DIST_MIN_DELTA 5.0f   // minimum distance change considered "movement"


// ---------- HELPERS ----------
static float normalize_angle(float a) {
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static float angle_error(float target, float current) {
    return normalize_angle(target - current);
}

static uint8_t us_valid(uint16_t cm) {
    return (cm >= US_MIN_VALID_CM && cm <= US_MAX_VALID_CM);
}

static float clampf(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}



// ---------- SETUP ----------
static void setup(void)
{
    init();

    uart_init();
    uart_print("Hovercraft init\r\n");

    us_init();
    uart_print("US init\r\n");
    ir_init();
    uart_print("IR init\r\n");
    servo_init();
    uart_print("Servo init\r\n");
    fan_init();
    uart_print("Fan init\r\n");
    i2c_init();
    uart_print("I2C init\r\n");
    imu_init();
    uart_print("IMU init\r\n");
    _delay_ms(100);
    imu_calibrate();

    setServoAngle(SERVO_CENTER_ANGLE);

    setLiftFanOn();
    setThrustFan(THRUST_STOP_PWM);

    sei();

    uart_print("READY\r\n");
}


// ---------- MAIN ----------
int main(void)
{
    setup();
    uart_print("Hovercraft Initialized!\r\n");

    HoverState state = STATE_FORWARD;

    while (1) {
        imu_update();
        float yaw = normalize_angle(imu_get_yaw());

        ir_update();
        float top = ir_get_cm();

        us_update();
        float front = front_cm;
        float right = right_cm;

        setLiftFanOn();

        uart_print("Front: ");
        uart_print_float(front);
        uart_print("     Right : ");
        uart_print_float(right);
        uart_print("\r\n");



        switch (state) {
            // ================= FORWARD =================
            case STATE_FORWARD: {
                setLiftFanOn();
                setThrustFan(THRUST_FORWARD_PWM);

                float err = angle_error(heading_target_deg, yaw);

                // P-control ONLY (like test1)
                float servo_correction = -KP_HEADING * err;
                uint32_t now = millis();

                if (now - last_servo_update >= 20) {
                    setServoAngle(servo_correction);
                    last_servo_update = now;
                }
                servo_correction = clampf(servo_correction, -45.0f, 45.0f);
                setServoAngle(servo_correction);

                // ---------- STOP ----------
                if (ir_detect()) {
                    state = STATE_STOP;
                    break;
                }

                // ---- STUCK DETECTION ----
                uint32_t now_ms = millis();

                if (front < 10 && front != 9999) {
                    if (!stuck_active) {
                        stuck_active = true;
                        stuck_timer = now_ms;
                    } else if (now_ms - stuck_timer >= STUCK_TIME_MS)
                            stuck_active = 1;
                } else {
                        stuck_timer = 0;
                }
                last_front = front;


                if (stuck_active) {
                    // STOP THRUST IMMEDIATELY
                    setThrustFan(0);

                    // FORCE FULL LOCK SERVO
                    if (us_valid(right) && right > front) {
                        setServoAngle(+90);   // ESCAPE RIGHT HARD
                        turn_target_deg = normalize_angle(heading_target_deg - TURN_ANGLE);
                        state = STATE_TURN_RIGHT;
                    }
                    else {
                        setServoAngle(-90);             // ESCAPE LEFT HARD
                        turn_target_deg = normalize_angle(heading_target_deg + TURN_ANGLE);
                        state = STATE_TURN_LEFT;
                    }

                    // Reset stuck detection
                    stuck_active = 0;
                    stuck_timer  = 0;

                    break;
                }


                // ---------- FRONT CHECK ----------
                if (us_valid(front) && front < FRONT_OBS_CM) {
                    setThrustFan(120);
                    if (++turn_ticks >= 10) {
                        turn_ticks = 0;
                        setServoAngle(0);
                        setThrustFan(0);
                        state = STATE_PRE_TURN_STOP;
                    }
                } else {
                    turn_ticks = 0;
                }

                break;
            }

            // ================= Pre Stop =================
            case STATE_PRE_TURN_STOP:
            {
                static uint16_t stop_ticks = 0;

                setServoAngle(0);
                setThrustFan(0);
                setLiftFanOff();

                if (++stop_ticks >= 20) {
                    stop_ticks = 0;

                    // uart_print("DECISION: R=");
                    // uart_print_int(right);
                    // uart_print(" FRONT=");
                    // uart_print_int(front);
                    // uart_print("\r\n");


                    if (us_valid(right) && right >= 35 && right != 9999) {
                        turn_target_deg = normalize_angle(heading_target_deg - TURN_ANGLE);
                        state = STATE_TURN_RIGHT;
                    } else {
                        turn_target_deg = normalize_angle(heading_target_deg + TURN_ANGLE);
                        state = STATE_TURN_LEFT;
                    }
                    turn_timer = 0;
                }
                break;
            }

            // ================= TURN RIGHT =================
            case STATE_TURN_RIGHT:
            {
                setLiftFanOn();
                setThrustFan(THRUST_TURN_PWM_MED);

                float remaining = angle_error(turn_target_deg, yaw);

                if (fabs(remaining) <= YAW_TOL_DEG || ++turn_timer > TURN_TIMEOUT_TICKS) {
                    setServoAngle(0);
                    heading_target_deg = imu_get_yaw();

                    stuck_active = 0;
                    stuck_timer  = 0;
                    last_front   = front;

                    state = STATE_FORWARD;
                    break;
                }

                uint32_t now_ms = millis();
                if (now_ms - last_servo_update >= 20) {
                    setServoAngle(+90);
                    last_servo_update = now_ms;
                }

                break;
            }

            // ================= TURN LEFT =================
            case STATE_TURN_LEFT:
            {
                setLiftFanOn();
                setThrustFan(THRUST_TURN_PWM_MED);

                float remaining = angle_error(turn_target_deg, yaw);

                if (fabs(remaining) <= YAW_TOL_DEG || ++turn_timer > TURN_TIMEOUT_TICKS) {
                    setServoAngle(0);
                    heading_target_deg = imu_get_yaw();

                    stuck_active = 0;
                    stuck_timer  = 0;
                    last_front   = front;

                    state = STATE_FORWARD;
                    break;
                }

                uint32_t now_ms = millis();
                if (now_ms - last_servo_update >= 20)
                {
                    setServoAngle(-90);
                    last_servo_update = now_ms;
                }

                break;
            }

            // ================= STOP =================
            case STATE_STOP: {
                setServoAngle(SERVO_CENTER_ANGLE);
                setThrustFan(THRUST_STOP_PWM);
                setLiftFanOff();
                break;
            }

            default:
                break;
        }
    }
    return 0;
}