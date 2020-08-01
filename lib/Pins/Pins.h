#ifndef PINS_H
#define PINS_H

//Motors
#define L_MOTOR_PWM PB_14
#define L_MOTOR_DIR PB_13
#define R_MOTOR_PWM PB_1
#define R_MOTOR_DIR PB_15
#define ENABLE PA_14

//Quadrature encoder
#define L_CHANNEL_A PC_12
#define L_CHANNEL_B PC_10
#define R_CHANNEL_A PD_2
#define R_CHANNEL_B PC_11
 
//Line sensors
#define SENS_1 A0
#define SENS_2 A1
#define SENS_3 A2
#define SENS_4 A3
#define SENS_5 A4
#define SENS_6 A5
 
//Line sensor enable
#define EN_SENS_1 D7
#define EN_SENS_2 D6
#define EN_SENS_3 D5
#define EN_SENS_4 D4
#define EN_SENS_5 D3
#define EN_SENS_6 D2
 
//HM-10 BLE
#define BLE_TX PA_12
#define BLE_RX PA_11

//Battery monitor
#define BAT_MON PA_13

//LEDs
#define LED_1 PC_15
#define LED_2 PH_0
#define LED_3 PH_1

#endif