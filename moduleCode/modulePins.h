/*

  modulePins.h - Abstract pin numbers with names
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This file contains the pin defitions for 
  the boaboard arduinos.

*/


/************************** Arduino Defines **********************************/

#ifndef _MODULE_PINS_H_
#define _MODULE_PINS_H_

#include <Arduino.h>

// Arduino analog Pins
#define ADC0   A0
#define ADC1   A1
#define ADC2   A2
#define ADC3   A3
#define ADC4   A4
#define ADC5   A5
#define ADC6   A6 
#define ADC7   A7
#define ADC8   A8
#define ADC9   A9
#define ADC10 A10
#define ADC11 A11
#define ADC12 A12
#define ADC13 A13
#define ADC14 A14
#define ADC15 A15


// Arduino digital Pins
#define  ARD0  0
#define  ARD1  1
#define  ARD2  2
#define  ARD3  3
#define  ARD4  4
#define  ARD5  5
#define  ARD6  6
#define  ARD7  7
#define  ARD8  8
#define  ARD9  9
#define ARD10 10
#define ARD11 11
#define ARD12 12
#define ARD13 13
#define ARD14 14
#define ARD15 15
#define ARD16 16
#define ARD17 17
#define ARD18 18
#define ARD19 19
#define ARD20 20
#define ARD21 21
#define ARD22 22
#define ARD23 23
#define ARD24 24
#define ARD25 25
#define ARD26 26
#define ARD27 27
#define ARD28 28
#define ARD29 29
#define ARD30 30
#define ARD31 31
#define ARD32 32
#define ARD33 33
#define ARD34 34
#define ARD35 35
#define ARD36 36
#define ARD37 37
#define ARD38 38
#define ARD39 39
#define ARD40 40
#define ARD41 41
#define ARD42 42
#define ARD43 43
#define ARD44 44
#define ARD45 45
#define ARD46 46
#define ARD47 47
#define ARD48 48
#define ARD49 49
#define ARD50 50
#define ARD51 51
#define ARD52 52
#define ARD53 53


// Arduino PWM pins
#define  PWM2   ARD2
#define  PWM3   ARD3
#define  PWM4   ARD4
#define  PWM5   ARD5
#define  PWM6   ARD6
#define  PWM7   ARD7
#define  PWM8   ARD8
#define  PWM9   ARD9
#define PWM10  ARD10
#define PWM11  ARD11
#define PWM12  ARD12
#define	PWM13  ARD13


// Arduino serial lines
#define RX0   ARD0
#define TX0   ARD1
#define RX1  ARD19
#define TX1  ARD18
#define RX2  ARD17
#define TX2  ARD16
#define RX3  ARD15
#define TX3  ARD14


/************************** Titanoboa BoaShield Defines **********************************/

// RS485 Enable Line
#define RS485_TX_ENABLE ARD53

// Battery
#define BAT_LEVEL_24V  ADC1  //input

// Motor
#define PRESSURE_SENSOR  ADC5  //input
#define MOTOR_CONTROL    PWM5  //output


// Kill circuit
#define KILL_CIRCUIT_1  ARD0  //output
#define KILL_CIRCUIT_2  ARD1  //output


// Position Sensors
// Horizontal position sensors
#define HORZ_POS_SENSOR_1   ADC6
#define HORZ_POS_SENSOR_2   ADC7
#define HORZ_POS_SENSOR_3   ADC8
#define HORZ_POS_SENSOR_4   ADC9
#define HORZ_POS_SENSOR_5  ADC10

// Vertical position sensors
#define VERT_POS_SENSOR_1  ADC11
#define VERT_POS_SENSOR_2  ADC12
#define VERT_POS_SENSOR_3  ADC13
#define VERT_POS_SENSOR_4  ADC14
#define VERT_POS_SENSOR_5  ADC15


// Horizontal limit switches
#define HORZ_LIMIT_SWITCH_1  ARD40
#define HORZ_LIMIT_SWITCH_2  ARD41
#define HORZ_LIMIT_SWITCH_3  ARD42
#define HORZ_LIMIT_SWITCH_4  ARD43
#define HORZ_LIMIT_SWITCH_5  ARD44

// Vertical limit switches
#define VERT_LIMIT_SWITCH_1  ARD45
#define VERT_LIMIT_SWITCH_2  ARD46
#define VERT_LIMIT_SWITCH_3  ARD47
#define VERT_LIMIT_SWITCH_4  ARD48
#define VERT_LIMIT_SWITCH_5  ARD49


// Actuators
#define DITHER         PWM6

// Horizontal actuators
#define HORZ_ACTUATOR_1  PWM13
#define HORZ_ACTUATOR_2  PWM12
#define HORZ_ACTUATOR_3  PWM11
#define HORZ_ACTUATOR_4  PWM10
#define HORZ_ACTUATOR_5   PWM9

// Vertical actuators
#define VERT_ACTUATOR_1   PWM8
#define VERT_ACTUATOR_2   PWM7
#define VERT_ACTUATOR_3   PWM4
#define VERT_ACTUATOR_4   PWM3
#define VERT_ACTUATOR_5   PWM2

// Actuator control signals
// Horizontal
#define HORZ_ACTUATOR_CTRL_1  ARD31
#define HORZ_ACTUATOR_CTRL_2  ARD30
#define HORZ_ACTUATOR_CTRL_3  ARD29
#define HORZ_ACTUATOR_CTRL_4  ARD28
#define HORZ_ACTUATOR_CTRL_5  ARD27

// Vertical
#define VERT_ACTUATOR_CTRL_1  ARD26
#define VERT_ACTUATOR_CTRL_2  ARD25
#define VERT_ACTUATOR_CTRL_3  ARD24
#define VERT_ACTUATOR_CTRL_4  ARD23
#define VERT_ACTUATOR_CTRL_5  ARD22


// Communications
// Serial
#define RX_UPSTREAM    RX1
#define TX_UPSTREAM    TX1
#define RX_DOWNSTREAM  RX2
#define TX_DOWNSTREAM  TX2

// LEDs
#define LED_8  ARD32
#define LED_7  ARD33
#define LED_6  ARD34
#define LED_5  ARD35
#define LED_4  ARD36
#define LED_3  ARD37
#define LED_2  ARD38
#define LED_1  ARD39


// Position sensor pin arrays for easy looping
const int HORZ_POS_SENSOR[] = {
  HORZ_POS_SENSOR_1,
  HORZ_POS_SENSOR_2,
  HORZ_POS_SENSOR_3,
  HORZ_POS_SENSOR_4,
  HORZ_POS_SENSOR_5
};

const int VERT_POS_SENSOR[] = {
  VERT_POS_SENSOR_1,
  VERT_POS_SENSOR_2,
  VERT_POS_SENSOR_3,
  VERT_POS_SENSOR_4,
  VERT_POS_SENSOR_5
};

// Actuator pin arrays for easy looping
const int HORZ_ACTUATOR_CTRL[] = {
  HORZ_ACTUATOR_CTRL_1,
  HORZ_ACTUATOR_CTRL_2,
  HORZ_ACTUATOR_CTRL_3,
  HORZ_ACTUATOR_CTRL_4,
  HORZ_ACTUATOR_CTRL_5
};
const int HORZ_ACTUATOR[] = {
  HORZ_ACTUATOR_1,
  HORZ_ACTUATOR_2,
  HORZ_ACTUATOR_3,
  HORZ_ACTUATOR_4,
  HORZ_ACTUATOR_5
};

const int VERT_ACTUATOR_CTRL[] = {
  VERT_ACTUATOR_CTRL_1,
  VERT_ACTUATOR_CTRL_2,
  VERT_ACTUATOR_CTRL_3,
  VERT_ACTUATOR_CTRL_4,
  VERT_ACTUATOR_CTRL_5
};
const int VERT_ACTUATOR[] = {
  VERT_ACTUATOR_1,
  VERT_ACTUATOR_2,
  VERT_ACTUATOR_3,
  VERT_ACTUATOR_4,
  VERT_ACTUATOR_5
};

// LED pin arrays for easy looping
const int LED[] = {
  LED_1,
  LED_2,
  LED_3,
  LED_4,
  LED_5,
  LED_6,
  LED_7,
  LED_8  
};

#endif _MODULE_PINS_H_

