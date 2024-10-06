#include <Arduino.h>
#include <DriveMaster.h>

#define BASE_SPEED 100

HBridge rightMotor(3, 4);
HBridge leftMotor(5, 6);

class PID
{
private:
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;

public:
    PID(double kp_, double ki_, double kd_) : kp(kp_), ki(ki_), kd(kd_), prev_error(0), integral(0) {}

    float calculate(float setpoint, float measured_value, float dt)
    {
        float error = setpoint - measured_value;
        float P = kp * error;
        integral += error * dt;
        float I = ki * integral;

        float derivative = (error - prev_error) / dt;
        float D = kd * derivative;
        return P + I + D;
    }
};

PID controller(1, 1, 1);

void setup()
{
    rightMotor.begin();
    leftMotor.begin();
}

void loop()
{
    static uint32_t prev_time;
    static float correction;
    uint32_t current_time = millis();
    uint32_t error = -2 * analogRead(A0) - analogRead(A1) + analogRead(A3) + 2 * analogRead(A4);

    if (current_time - prev_time >= 10)
    {
        float correction = controller.calculate(0, error, current_time - prev_time);
        prev_time = current_time;
    }
    leftMotor.write(BASE_SPEED + correction);
    rightMotor.write(BASE_SPEED - correction);
}