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
#define FRONT_OBS_CM            40.0f
#define TURN_ANGLE              90.0f
#define YAW_TOL_DEG             2.0f

#define SERVO_CENTER_ANGLE      0.0f
#define KP_HEADING              3.0f

#define THRUST_FORWARD_PWM      255
#define THRUST_TURN_PWM_MED     200
#define THRUST_STOP_PWM         0

#define TURN_TIMEOUT_MS         2000

#define US_MIN_VALID_CM         5
#define US_MAX_VALID_CM         500

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
static uint32_t last_servo_update = 0;
static uint32_t turn_start_ms = 0;

// ---- STUCK DETECTION ----
static uint32_t stuck_timer = 0;
static uint8_t stuck_active = 0;

#define STUCK_TIME_MS 2000

static uint8_t sensors_enabled = 1;

// ---------- HELPERS ----------
static float normalize_angle(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
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
static void setup(void) {
    init();

    uart_init();
    us_init();
    ir_init();
    servo_init();
    fan_init();
    i2c_init();
    imu_init();
    _delay_ms(100);
    imu_calibrate();

    setServoAngle(0);
    setLiftFanOn();
    setThrustFan(0);
    sei();
}


// ---------- MAIN ----------
int main(void)
{
    setup();

    HoverState state = STATE_FORWARD;

    while (1)
    {
        imu_update();
        float yaw = normalize_angle(imu_get_yaw());

        if (sensors_enabled) {
            ir_update();
            us_update();
        }

        float front = front_cm;
        float right = right_cm;

        switch (state)
        {
            case STATE_FORWARD:
            {
                sensors_enabled = 1;

                setLiftFanOn();
                setThrustFan(THRUST_FORWARD_PWM);


                float err = angle_error(heading_target_deg, yaw);
                float servo = clampf(-KP_HEADING * err, -45, 45);

                uint32_t now = millis();
                if (now - last_servo_update > 20)
                {
                    setServoAngle(servo + 20.0f);
                    last_servo_update = now;
                }


                if (ir_detect())
                {
                    state = STATE_STOP;
                    break;
                }

                uint32_t t = millis();
                if (front < 15 && front != 9999)
                {
                    if (!stuck_timer) stuck_timer = t;
                    else if (t - stuck_timer > STUCK_TIME_MS) stuck_active = 1;
                }
                else stuck_timer = 0;

                if (stuck_active)
                {
                    setThrustFan(0);

                    if (us_valid(right) && right > front)
                    {
                        setServoAngle(+150);
                        turn_target_deg = normalize_angle(yaw - TURN_ANGLE);
                        state = STATE_TURN_RIGHT;
                    }
                    else
                    {
                        setServoAngle(-150);
                        turn_target_deg = normalize_angle(yaw + TURN_ANGLE);
                        state = STATE_TURN_LEFT;
                    }

                    turn_start_ms = millis();
                    stuck_active = 0;
                    stuck_timer = 0;
                    break;
                }

                if (us_valid(front) && front < FRONT_OBS_CM)
                {
                    setThrustFan(120);
                    static uint8_t ticks = 0;

                    if (++ticks >= 8)
                    {
                        ticks = 0;
                        setServoAngle(0);
                        setThrustFan(0);
                        state = STATE_PRE_TURN_STOP;
                    }
                }
                else
                {
                    static uint8_t ticks = 0;
                    ticks = 0;
                }

                break;
            }

            case STATE_PRE_TURN_STOP: {
                sensors_enabled = 1;

                static uint32_t stop_start = 0;
                if (!stop_start) stop_start = millis();

                setServoAngle(0);
                setThrustFan(0);
                setLiftFanOff();

                if (millis() - stop_start > 400)
                {
                    stop_start = 0;

                    if (us_valid(right) && right >= 35)
                    {
                        turn_target_deg = normalize_angle(yaw - TURN_ANGLE);
                        state = STATE_TURN_RIGHT;
                    }
                    else
                    {
                        turn_target_deg = normalize_angle(yaw + TURN_ANGLE);
                        state = STATE_TURN_LEFT;
                    }

                    turn_start_ms = millis();
                }

                break;
            }

            case STATE_TURN_RIGHT: {
                sensors_enabled = 0;

                setLiftFanOn();
                setThrustFan(THRUST_TURN_PWM_MED);
                setServoAngle(+140);

                float remaining = angle_error(turn_target_deg, yaw);

                if (fabs(remaining) <= YAW_TOL_DEG ||
                    millis() - turn_start_ms > TURN_TIMEOUT_MS) {
                    setServoAngle(0);
                    heading_target_deg = yaw;
                    sensors_enabled = 1;
                    state = STATE_FORWARD;
                }

                break;
            }

            case STATE_TURN_LEFT: {
                sensors_enabled = 0;

                setLiftFanOn();
                setThrustFan(THRUST_TURN_PWM_MED);
                setServoAngle(-140);

                float remaining = angle_error(turn_target_deg, yaw);

                if (fabs(remaining) <= YAW_TOL_DEG ||
                    millis() - turn_start_ms > TURN_TIMEOUT_MS) {
                    setServoAngle(0);
                    heading_target_deg = yaw;
                    sensors_enabled = 1;
                    state = STATE_FORWARD;
                }

                break;
            }

            case STATE_STOP:
            {
                setServoAngle(0);
                setThrustFan(0);
                setLiftFanOff();
                break;
            }
        }
    }
}
