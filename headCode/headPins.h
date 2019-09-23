//////////////////////////////////////////////////////////////////////////
//
// Filename: BoaShield_pins.h
//
// Description: all the pin defines for the Titanoboa BoaShield on the
//              Arduino
//
//////////////////////////////////////////////////////////////////////////


/************************** Arduino Defines **********************************/

#ifndef _BOA_SHIELD_PINS_H_
#define _BOA_SHIELD_PINS_H_

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

// Actuators
#define DITHER         PWM6 //Maps to PV-DITHER in schematic, PIN 6

// Horizontal actuators
#define HEAD_ACTUATOR_1  PWM3 //Maps to PV-HEAD0 on schematic, PIN 3
#define HEAD_ACTUATOR_2  PWM8 //Maps to PV-HEAD1 on schematic, PIN 8
#define HEAD_ACTUATOR_3  PWM11 //Maps to PV-HEAD2 on schematic, PIN 11

// Actuator control signals
// Horizontal
#define HEAD_ACTUATOR_CTRL_1  ARD7 //Maps to PV-HEAD-CRTL0 on schematic
#define HEAD_ACTUATOR_CTRL_2  ARD9 //Maps to PV-HEAD-CRTL1 on schematic
#define HEAD_ACTUATOR_CTRL_3  ARD12 //Maps to PV-HEAD-CRTL2 on schematic

// Position Sensors
// Horizontal position sensors
#define HEAD_POS_SENSOR_1   ADC6 //Maps to POS-SENSE-HOR0 in schematic
#define HEAD_POS_SENSOR_2   ADC7 //Maps to POS-SENSE-HOR1 in schematic
#define HEAD_POS_SENSOR_3   ADC8 //Maps to POS-SENSE-HOR2 in schematic
#define HEAD_POS_SENSOR_4   ADC9 //Maps to POS-SENSE-HOR3 in schematic
#define HEAD_POS_SENSOR_5  ADC10 //Maps to POS-SENSE-HOR4 in schematic
#define HEAD_POS_SENSOR_6  ADC11 //Maps to POS-SENSE-HOR5 in schematic

// LEDs
#define LED_1  ARD38 //Maps to LED0 in schematic
#define LED_2  ARD39 //Maps to LED1 in schematic
#define LED_3  ARD40 //Maps to LED2 in schematic
#define LED_4  ARD41 //Maps to LED3 in schematic
#define LED_5  ARD42 //Maps to LED4 in schematic
#define LED_6  ARD43 //Maps to LED5 in schematic

const int HEAD_POS_SENSOR[] = {
  HEAD_POS_SENSOR_1,
  HEAD_POS_SENSOR_2,
  HEAD_POS_SENSOR_3,
  HEAD_POS_SENSOR_4,
  HEAD_POS_SENSOR_5,
  HEAD_POS_SENSOR_6
};

const int HEAD_ACTUATOR_CTRLS[] = {
  HEAD_ACTUATOR_CTRL_1,
  HEAD_ACTUATOR_CTRL_2,
  HEAD_ACTUATOR_CTRL_3
};
const int HEAD_ACTUATORS[] = {
  HEAD_ACTUATOR_1,
  HEAD_ACTUATOR_2,
  HEAD_ACTUATOR_3
};

// Battery
#define BAT_LEVEL_24V  ADC2  //A2, Mapped to BATT-LEVEL-24V on schematic

// Communications
// Serial
#define RX_UPSTREAM    RX1
#define TX_UPSTREAM    TX1
#define RX_DOWNSTREAM  RX2
#define TX_DOWNSTREAM  TX2

// Map actuators numbers to actual actuators
#define JAW_ACTUATOR	HEAD_ACTUATOR_3
#define JAW_CTRL	HEAD_ACTUATOR_CTRL_3
#define JAW_OPEN_CTRL_SELECT	1
#define JAW_CLOSE_CTRL_SELECT	0

#define HEAD_ACTUATOR	HEAD_ACTUATOR_2
#define HEAD_CTRL	HEAD_ACTUATOR_CTRL_2
#define HEAD_RAISE_CTRL_SELECT	0
#define HEAD_LOWER_CTRL_SELECT	1

#define AUX_ACTUATOR	HEAD_ACTUATOR_1
#define AUX_CTRL	HEAD_ACTUATOR_CTRL_1
#define AUX_EXTEND_CTRL_SELECT	0
#define AUX_RETRACT_CTRL_SELECT	1


#endif _BOA_SHIELD_PINS_H_



