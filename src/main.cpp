#include "mbed.h"
#include "Pins.h"
#include "Motor.h"
#include "QEI.h"
#include "OneWire_Methods.h"
#include "ds2781.h"

Serial pc(USBTX, USBRX);
Serial HM10(PA_11, PA_12);

//Battery Monitor
DigitalInOut one_wire_pin(BAT_MON);
int VoltageReading;
float Voltage;
DigitalOut red_led(LED_1);
DigitalOut yellow_led(LED_2);
DigitalOut green_led(LED_3);

//Sesnors
AnalogIn sensor_in[] = {SENS_1, SENS_2, SENS_3, SENS_4, SENS_5, SENS_6};

//Encoders
QEI left_encoder(L_CHANNEL_A, L_CHANNEL_B, NC, 256, QEI::X4_ENCODING);
QEI right_encoder(R_CHANNEL_A, R_CHANNEL_B, NC, 256, QEI::X4_ENCODING);
float tick_left = 0.0f, tick_right = 0.0f;
const float SAMPLE_TIME = 0.01;

//Motors
Motor left_motor(L_MOTOR_PWM, L_MOTOR_DIR);
Motor right_motor(R_MOTOR_PWM, R_MOTOR_DIR);
DigitalOut enable_motors(ENABLE);
const float MIN = 0.225f;               //min PWM duty cycle
const float MAX_PWM = 0.65f;            //max PWM duty cycle
const float MAX_SPEED = MAX_PWM * 1023; //map max motor speed to int

//Tickers
Ticker encoder_ticks;
Ticker battery_mon;

//PID controller tuning
const int set_point = 2500.0f;  //Set point = the position that the controller is targeting
const float Kp = 0.65f; //Proportional term
const float Ki = 0.0f; //Integeral term
const float Kd = 6.5f; //Dervative term
/* 
Buggy tuning:
1. Set desired max speed by changing MAX_PWM
2. Put buggy down on track, see if it tracks the white line
3. Increase Kp by a small amount (0.05 to 0.1) if it fails to track white line
4. If increased Kp works, record it
5. If buggy oscillates too much, change Kd (usually x10 to x20 larger than Kp)
*/
void buggy_initialize()
{
    wait(0.1);
    enable_motors = 1;
    left_motor.motorDirection(0);
    right_motor.motorDirection(1);
    right_motor.motorSpeed(0.35);
    left_motor.motorSpeed(0.35);
    wait(0.2);
}

void calc_speed()
{
    int pulse_left = 0, pulse_right = 0;

    pulse_left = abs(left_encoder.getPulses());
    pulse_right = abs(right_encoder.getPulses());

    tick_left = pulse_left / SAMPLE_TIME;
    tick_right = pulse_right / SAMPLE_TIME;
    //pc.printf("Tick L: %f Tick R: %f\n", tick_left, tick_right);
    //Reset encoders for next iteration
    left_encoder.reset();
    right_encoder.reset();
}

void battery_monitor()  //15/4: need to fix battery monitor
{
    VoltageReading = ReadVoltage();
    Voltage = VoltageReading * 0.00976;
    //pc.printf("Voltage = %f", Voltage);

    if (Voltage > 9.6)
    {
        green_led = 1;
    }
    else
    {
        green_led = 0;
    }

    if (Voltage <= 9.6 && Voltage > 8.8)
    {
        yellow_led = 1;
    }
    else
    {
        yellow_led = 0;
    }

    if (Voltage < 8.8)
    {
        red_led = 1;
    }
    else
    {
        red_led = 0;
    }
}

int main()
{
    encoder_ticks.attach(&calc_speed, SAMPLE_TIME);
    //battery_mon.attach(&battery_monitor, 1);

    HM10.baud(9600);

    const int num_sensors = 6;
    int sensor_values[6];
    for (int i = 0; i < num_sensors; i++)
    {
        sensor_values[i] = 0;
    }

    //Position calc
    float average = 0.0f;
    int sum = 0;
    int position = 0;

    //PID
    float proportional = 0.0f;
    float last_proportional = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;
    float error_value = 0.0f;

    //Motor
    float left_speed = 0.0f;
    float right_speed = 0.0f;

    float l_modifier = 0.0f;
    float r_modifier = 0.0f;

    //BLE
    int end_of_line = 0;
    char c;
    int S3 = 0;
    int S4 = 0;

    buggy_initialize();

    while (1)
    {
        //pc.printf("Tick L: %f Tick R: %f\n", tick_left, tick_right);
    PIDLoop:
        end_of_line = 0;
        average = 0;
        sum = 0;
        position = 0;

        for (int i = 0; i < num_sensors; i++)
        {
            sensor_values[i] = sensor_in[i].read_u16() >> 6;

            if (sensor_values[i] < 300)
            {
                end_of_line++;
            }
            //pc.printf("Sensor %d reading: %d\n", i, sensor_values[i]);
            average += sensor_values[i] * i * 1000; // calc weighted mean
            sum += sensor_values[i];
        }

        if (end_of_line == 6) //if all sensors detect black, end of line
        {
            right_motor.motorSpeed(0);
            left_motor.motorSpeed(0);
            goto BLEListen;
            //goto Turnaround; //this is for fully automated line tracking
        }

        if (HM10.readable())
        {
            goto BLEListen;
        }

        position = average / sum;
        //pc.printf("Average: %f, Sum: %d, Position:%d\n", average, sum, position);

        //PID controller
        proportional = position - set_point;
        integral = integral + proportional;
        derivative = proportional - last_proportional;
        error_value = (proportional * Kp) + (integral * Ki) + (derivative * Kd);
        last_proportional = proportional;

        //Clamping error values
        if (error_value < -1024)
        {
            error_value = -1024;
        }

        if (error_value > 1024)
        {
            error_value = 1024;
        }
        //pc.printf("Error value: %f\n", error_value);

        if (error_value < 0)
        { //if buggy to right
            right_speed =
                MAX_SPEED + error_value; //add more right power to steer back
            left_speed = MAX_SPEED;
        }
        else
        { // if buggy to left
            right_speed = MAX_SPEED;
            left_speed = MAX_SPEED - error_value; //decrease power to left wheel to steer back
        }

        //pc.printf("1. R speed: %f, L speed: %f\n", right_speed, left_speed);
        //wait(0.5);

        right_speed = (right_speed / 1023.0);
        left_speed = (left_speed / 1023.0);
        if (right_speed < 0)
        {
            right_speed = right_speed * -1;
        }

        if (left_speed < 0)
        {
            left_speed = left_speed * -1;
        }

        //pc.printf("R speed: %f, L speed: %f\n", right_speed, left_speed);
        //wait(0.5);

        if (right_speed < MIN)
        {
            right_speed = MIN;
        }

        if (left_speed < MIN)
        {
            left_speed = MIN;
        }

        /*
        //Modifier to increase speed going up ramp
        if (tickR < 1000 && tickL < 1000)
        {
        modifier = 0.8f;
        }
        else
        {
        modifier = 0.0f;
        }*/  
    
        //Modifier to decrease speed going down ramp
        if (tick_left > 5000 || tick_right > 5000)
        {
            l_modifier = -left_speed;
            r_modifier = -right_speed;
        }
        else
        {
            l_modifier = 0.0f;
            r_modifier = 0.0f;
        }

        //Set motor speeds
        left_motor.motorSpeed(left_speed + l_modifier);
        right_motor.motorSpeed(right_speed + r_modifier);

        //pc.printf("2. R speed: %f, L speed: %f\n", right_speed, left_speed);
        //wait(0.5);
    }

    BLEListen:
        //pc.printf("Reached BLE\n");
        while (1)
        {
            c = HM10.getc();
            if (c == 'T' || c == 't') //lower case t quicker to type?
            {
                while (1)
                {
                    Turnaround:
                    S3 = sensor_in[4].read_u16() >> 6;
                    //pc.printf("S3 = %d\n", S3);
                    S4 = sensor_in[5].read_u16() >> 6;
                    //pc.printf("S4 = %d\n", S4);
                    if (S3 < 500 && S4 < 500)
                    {
                        left_motor.motorDirection(1);
                        right_motor.motorDirection(1);
                        left_motor.motorSpeed(0.3);
                        right_motor.motorSpeed(0.3);
                    }
                    else
                    {
                        left_motor.motorSpeed(0);
                        right_motor.motorSpeed(0);
                        wait(0.05);

                        left_motor.motorDirection(0);
                        right_motor.motorDirection(1);
                        goto PIDLoop;
                    }
                }
            }
        }
}