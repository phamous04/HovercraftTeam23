// Hovercraft.ino
#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "uart.h"
#include "us_sensor.h"
#include "ir_sensor.h"
#include "imu.h"
#include "servo.h"
#include "fan.h"


// ----------------------- Test Main Program -------------------------
void setup() {
    uart_init();
    //us_init();
    //ir_init();
    servo_init();
    fan_init();
    //imu_init();
    //imu_calibrate();

    sei();  // Enable global interrupts
}

void test_us_sensors() {
    float front_distance = read_front_sensor();
    float left_distance = read_left_sensor();
    float right_distance = read_right_sensor();

    uart_print("Front: ");
    uart_print_float(front_distance);
    uart_print(" cm, Left: ");
    uart_print_float(left_distance);
    uart_print(" cm, Right: ");
    uart_print_float(right_distance);
    uart_print(" cm\n");

    _delay_ms(500);
}

void test_ir_sensor() {
    uart_print("IR ADC Value: ");
    uart_print_int(ir_read());
    uart_print("\n");

    uart_print("IR Distance: ");
    uart_print_int(ir_get_distance());
    uart_print("\n");
}

void test_imu() {
    float gyro_z = imu_get_gyro_z();
    float yaw = imu_get_yaw();

    imu_read();
    uart_print("Gyro Z: ");
    uart_print_float(gyro_z);
    uart_print(" | Yaw Angle: ");
    uart_print_float(yaw);
    uart_print("\n");
}

void test_servo() {
    setServoAngle(0);
    uart_print("Servo set to 0 degrees\n");
    _delay_ms(3000);

    setServoAngle(90);
    uart_print("Servo set to 90 degrees\n");
    _delay_ms(3000);

    setServoAngle(180);
    uart_print("Servo set to 180 degrees\n");
    _delay_ms(3000);
}

void test_thrust_fan() {
    setThrustFan(255);
    uart_print("Thrust fan set to speed 255\n");
    _delay_ms(20000);

    setThrustFan(0);
    _delay_ms(2000);
}

void test_lift_fan() {
    setLiftFan(255);
    uart_print("Lift fan set to speed 255\n");
    _delay_ms(20000);

    setLiftFan(0);
    uart_print("Lift fan stopped\n");
    _delay_ms(2000);
}


int main(void) {
    setup();


    while (1) {
        //test_thrust_fan();
        //test_lift_fan();
        //test_servo();
        //ir_update();
        //test_ir_sensor();

        setLiftFan(255);
        setThrustFan(120);
        _delay_ms(20000);

        setLiftFan(0);
        setThrustFan(0);
        _delay_ms(3000);

    }

    return 0;
}
