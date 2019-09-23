/*

  moduleCode.ino - Controls 1 Titanaboa module of 5 vertebrae  
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This code runs on an Arduino MEGA with a BoaShield to
  control 5 vertical and 5 horizontal actuators of a 5 vertibrae module.
  Titanaboa moves by pushing a sequence of angles from head to tail at an 
  interval initiated by the head board.
  
  Vertebrae angles are controlled by a hydrualic system in a control
  loop with angular position sensors. Communication between the 
  modules is done over a serial daisy chain. 
*/

// We are using an interrupt for the PID. this interrupt could come in any time, possibly in the middle
// of communication. To ensure that we don't lose data, make the serial buffer size larger than the packets.
#include "HardwareSerial.cpp"
#if SERIAL_BUFFER_SIZE != 128
  #error "ERROR: Serial buffer must be 128 bytes. Please modify your arduino program files."
  // Arduino Folder\hardware\arduino\cores\arduino
#endif 

#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "modulePins.h"
#include "PIDcontrol.h"
#include "OneWireNXP.h"
#include "MemoryFree.h"


// Defines and constants
#define HEAD_SERIAL Serial1             // Serial to the upstream module
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define RS485_SERIAL Serial3            // Serial bus over the entire snake
#define USB_COM_PORT Serial             // Serial for debugging

// Enable serial stream writing
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

// Memory locations for items stored to EEPROM
#define MY_MODULE_NUMBER_ADDRESS  0
#define HORIZONTAL_CALIBRATION_ADDRESS  2
#define VERTICAL_CALIBRATION_ADDRESS  (HORIZONTAL_CALIBRATION_ADDRESS + 25)
#define VERTICAL_STRAIGHT_ADDRESS  (VERTICAL_CALIBRATION_ADDRESS + 25)

// The master setpoints for all actuators. 
byte horzAngleSetpoints[] = {255,255,255,255,255};  // Target horizontal setpoints in angle space. (Range 0 - 254)
byte vertAngleSetpoints[] = {255,255,255,255,255};  // Target vertical setpoints int angle space. (255 = disabled)
int horzSensorSetpoints[] = {-1,-1,-1,-1,-1};      // Target horizontal setpoints in sensor space. (Range 0 to 1023)
int vertSensorSetpoints[] = {-1,-1,-1,-1,-1};      // Target vertical setpoints in sensor space. (-1 = disabled)

// Current valve outputs
byte horzValveOutputs[] = {0,0,0,0,0};
byte vertValveOutputs[] = {0,0,0,0,0};

// Incrimental setpoints moving torwards the master setpoints. 
// They are determined by the current position, the master setpoint and the slew rate 
int horzSensorIncrementalSetpoints[] = {-1,-1,-1,-1,-1};
int vertSensorIncrementalSetpoints[] = {-1,-1,-1,-1,-1};

// Array of sensor values we recorded as straight for the verticals
int vertStraightArray[] = {-1, -1, -1, -1, -1};

// Timer arrays used to timeout when waiting for an actuator to reach its set point
unsigned long horzTimerArray[] = {0,0,0,0,0};                           
unsigned long vertTimerArray[] = {0,0,0,0,0};

// Calibration values. The values of the position sensors at the hard limits of actuation.
// Assume a linear sensor characteristic between these two points.
int horzHighCalibration[] = {1023, 1023, 1023, 1023, 1023};
int horzLowCalibration[] = {0, 0, 0, 0, 0};
int vertHighCalibration[] = {1023, 1023, 1023, 1023, 1023};
int vertLowCalibration[]  = {0, 0, 0, 0, 0};

// We configure the deadbands in angle space (0-254). These arrays are a mapping of these
// values to each sensor space. They are calculated by calling calculateDeadbands()
int horzSensorDeadbands[] = {-1,-1,-1,-1,-1};
int vertSensorDeadbands[] = {-1,-1,-1,-1,-1};

// When moving actuators . Number of angles to incriment each PID loop.
byte horzSensorSlewRates[] = {-1,-1,-1,-1,-1};
byte vertSensorSlewRates[]  = {-1,-1,-1,-1,-1};

byte myModuleNumber = 0;                // My position in the module chain (1,2,3...)
boolean iAmLastModule = false;          // If we are the last module don't read tail serial
byte killSwitchPressed = false;         // If the kill switch isn't pressed. Don't move.
boolean runPidLoop = false;             // Set to true if you want to run pid on interrupt interval.
byte runtimeMotorSpeed = 0;             // Runtime analog output motor speed. Set by joystick.
const byte horzAngleDeadband = 10;      // The horizontal deadband, 0 to 254 range
const byte vertAngleDeadband = 10;      // The vertical deadband, 0 to 254 range
const int actuatorTimeout = 5000;       // Actuators have this many ms to get to the master setpoint
const byte calibrateMotorSpeed = 150;   // Motor speed for vertical and horz calibration.
const byte jawMotorSpeed = 90;          // Motor speed for jaw open/close. (the jaw uses the module 1 motor)
const byte manualMotorSpeed = 200;      // Motor speed for manual mode operations.
const int pidLoopTime = 15;             // We execute the pid loop on this interval (ms)
byte horzAngleSlewRate = 2;             // When moving horizontals. Max number of angles to incriment each PID loop.
byte vertAngleSlewRate = 2;             // When moving verticals. Max number of angles to incriment each PID loop.
boolean horzControlSlewRate = false; 
boolean vertControlSlewRate = true; 


// Variables for manual mode
int manualActuationDelay = 200;
int manualVertibraeSelect = 0;

// PID Constants
byte horzAngleKp = 20;
byte horzAngleKi = 20;
byte horzAngleKd = 0;
byte vertAngleKp = 20;
byte vertAngleKi = 20;
byte vertAngleKd = 0;
int horzSensorKp[5] = {-1,-1,-1,-1,-1};
int horzSensorKi[5] = {-1,-1,-1,-1,-1};
int horzSensorKd[5] = {-1,-1,-1,-1,-1};
int vertSensorKp[5] = {-1,-1,-1,-1,-1};
int vertSensorKi[5] = {-1,-1,-1,-1,-1};
int vertSensorKd[5] = {-1,-1,-1,-1,-1};

// PID Controllers for the horizontal actuators
// Declaration of horizontal PID controllers, default even and odd values applied here
// but get updated during calibration
PIDcontrol PIDcontrollerHorizontal[] = {
  PIDcontrol(HORZ_POS_SENSOR[0], HORZ_ACTUATOR_CTRL[0], HORZ_ACTUATOR[0]),
  PIDcontrol(HORZ_POS_SENSOR[1], HORZ_ACTUATOR_CTRL[1], HORZ_ACTUATOR[1]),
  PIDcontrol(HORZ_POS_SENSOR[2], HORZ_ACTUATOR_CTRL[2], HORZ_ACTUATOR[2]),
  PIDcontrol(HORZ_POS_SENSOR[3], HORZ_ACTUATOR_CTRL[3], HORZ_ACTUATOR[3]),
  PIDcontrol(HORZ_POS_SENSOR[4], HORZ_ACTUATOR_CTRL[4], HORZ_ACTUATOR[4]),
};

// PID Controllers for the vertical actuators
// Declaration of vertical PID controllers, default even and odd values only apply to
// odd numbered modules
PIDcontrol PIDcontrollerVertical[] = {
  PIDcontrol(VERT_POS_SENSOR[0], VERT_ACTUATOR_CTRL[0], VERT_ACTUATOR[0]),
  PIDcontrol(VERT_POS_SENSOR[1], VERT_ACTUATOR_CTRL[1], VERT_ACTUATOR[1]),
  PIDcontrol(VERT_POS_SENSOR[2], VERT_ACTUATOR_CTRL[2], VERT_ACTUATOR[2]),
  PIDcontrol(VERT_POS_SENSOR[3], VERT_ACTUATOR_CTRL[3], VERT_ACTUATOR[3]),
  PIDcontrol(VERT_POS_SENSOR[4], VERT_ACTUATOR_CTRL[4], VERT_ACTUATOR[4]),
};


/****************************************************************************
 setup(): Initializes serial ports, pins and loads EEPROM calibration
**************************************************************************/
void setup()
{
  // Initialize serial ports
  TAIL_SERIAL.begin(115200); 
  HEAD_SERIAL.begin(115200); 
  USB_COM_PORT.begin(115200);
  RS485_SERIAL.begin(115200);

  // Setup the output that enables RS485 transmit  
  pinMode(RS485_TX_ENABLE, OUTPUT);
  digitalWrite(RS485_TX_ENABLE, LOW);

  // Turn off motor pin
  pinMode(MOTOR_CONTROL,OUTPUT);
  analogWrite(MOTOR_CONTROL,0);

  // Initialize solenoid valve control ouputs
  for(int i=0;i<5;i++)
  {
    pinMode(HORZ_ACTUATOR[i],OUTPUT);
    analogWrite(HORZ_ACTUATOR[i],0);
    pinMode(VERT_ACTUATOR[i],OUTPUT);
    analogWrite(VERT_ACTUATOR[i],0);
  }

  // Initialize solenoid valve selection ouputs
  // Each valve has two solenoids: 1 for extending
  // 1 for retracting. This signal selects which one 
  // we are controlling.
  for(int i=0;i<5;i++)
  {
    pinMode(HORZ_ACTUATOR_CTRL[i],OUTPUT);
    pinMode(VERT_ACTUATOR_CTRL[i],OUTPUT);
  }

  // Initialize position sensor inputs.
  for(int i=0;i<5;i++)
  {
    pinMode(HORZ_POS_SENSOR[i],INPUT);
    pinMode(VERT_POS_SENSOR[i],INPUT);
  }

  //initialize LED Pins
  for(int i=0;i<8;i++)
  {
    pinMode(LED[i],OUTPUT);
  }
  
  // Load previous calibration from EEPROM
  setModuleNumber(EEPROM.read(MY_MODULE_NUMBER_ADDRESS));
  
  for(int i=0;i<5;i++)
  { 
    horzHighCalibration[i] = EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS) + 
                            (EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS + 1) << 8);
    horzLowCalibration[i] =  EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS + 2) +
                            (EEPROM.read(i*5 + HORIZONTAL_CALIBRATION_ADDRESS + 3) << 8);

    vertStraightArray[i] = EEPROM.read(i*5 + VERTICAL_STRAIGHT_ADDRESS)
                           + (EEPROM.read(i*5 + VERTICAL_STRAIGHT_ADDRESS+1) << 8);
                           
    vertHighCalibration[i] = EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS) + 
                            (EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS + 1) << 8);
    vertLowCalibration[i] =  EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS + 2) +
                            (EEPROM.read(i*5 + VERTICAL_CALIBRATION_ADDRESS + 3) << 8);   
  }
  
  // We have values in 0-255 angle space that need to be mapped to sensor space.
  mapAngleConstantsToSensorSpace();

  // Initialize incrimental setpoints to current sensor positions
  for (int i = 0; i < 5; ++i)
  {
    horzSensorIncrementalSetpoints[i] = analogRead(HORZ_POS_SENSOR[i]);
    vertSensorIncrementalSetpoints[i] = analogRead(VERT_POS_SENSOR[i]);
  }

  // Print out the initialization information
  USB_COM_PORT.print("\nHi I'm Titanoboa, MODULE #: ");
  USB_COM_PORT.println(myModuleNumber, DEC);

  // Check if I'm the last module
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write(200);
  delay(10);
  boolean avail = (TAIL_SERIAL.available() == 1);
  int data = TAIL_SERIAL.read();
  if (avail && (data == 200))
  {
    iAmLastModule = true;
    USB_COM_PORT << "... and I'm the last module! (" << data << ")\n\n";
  }
  else
  {
    USB_COM_PORT << "Not last. (" << data << ")\n\n";    
  }
 
  printBatteryVoltage();
  printCalibrationValues();
  
  clearSerialBuffer(HEAD_SERIAL);
  clearSerialBuffer(TAIL_SERIAL);
  
  // AVR code to enable overflow interrupt on timer 1
  // Used for the pid interrupt
  TIMSK1 |= _BV(TOIE1);
  runPidLoop = true;
   
  USB_COM_PORT << "There are " << freeMemory() << " bytes of free memory.\n\n"; 
}

/***********************************************************************************
  loop(): Checks for Upstream Serial or USB Serial commands. Executes them.
 ***********************************************************************************/
 
void loop()
{  
  // Check the USB port for command to enter manual mode
  if (USB_COM_PORT.available() > 0)
  {
    if (USB_COM_PORT.read() == 'M')
    {
      manualControl();
    }
  }
  
  if (RS485_SERIAL.available() > 0)
  {
    if (RS485_SERIAL.read() == 't')
    {      
        processRS485CommunicationTestCommand();
    }
  }

  // Check upstream Serial for commands from the head
  if (HEAD_SERIAL.available() > 0)
  {
    //letters in use: p, s, c, h, g, v, t
    char command = HEAD_SERIAL.read();
    switch (command)
    {
      case 's':
        processNewSetpoints();
        acknowledgeCommand();
        break;
      case 'd':
        processDiagnosticsCommand();
        break;
      case 'c':
        processCalibrateCommand();
        acknowledgeCommand();
        break;
      case 'h':
        processJawMovingMotorPulseCommand();
        break;
      case 'g':
        processHeadManualModeMotorPulseCommand();
        break;
      case 't':
        processSerialChainCommunicationTestCommand();
        break;

      default:
        USB_COM_PORT.print("Invalid command from upstream serial = ");
        USB_COM_PORT.println(command);
        clearSerialBuffer(HEAD_SERIAL);
    }
  }
  
  while (iAmLastModule == false && TAIL_SERIAL.available() > 0)
  {
    byte data = TAIL_SERIAL.read();
    HEAD_SERIAL.write(data);
  }
}//end loop()

/************************************************************************************
  acknowledgeCommand(): Tells the head the command is finished on this module.
                        Only send acknowledgements if the head is programmed to wait
                        for them after that command.
 ***********************************************************************************/
void acknowledgeCommand()
{
  HEAD_SERIAL.write(myModuleNumber);
}

/************************************************************************************
  processCommmunicationTestCommand(): Tests up and down serial communication
 ***********************************************************************************/
void processSerialChainCommunicationTestCommand()
{
  USB_COM_PORT << "Running serial chain communication test. Sending up myModuleNumber = " << myModuleNumber <<"\n";
  
  // Respond with module number so the head knows I got the message.
  HEAD_SERIAL.write(myModuleNumber);
  
  // Echo back 't' if this is the last module.
  if (iAmLastModule)
  {
    HEAD_SERIAL.write('t');
    USB_COM_PORT << "I'm the last module. Sending up 't'.\n";
  }
  else
  {
    // If this is not the last module continue to send the command down.
    clearSerialBuffer(TAIL_SERIAL);
    TAIL_SERIAL.write('t');
  
    // Take 100ms to send up any other serial data.
    long startTime = millis();
    int i = 0;
    while ((millis() - startTime) < 100)
    {
      if (TAIL_SERIAL.available() > 0)
      {
        byte data = TAIL_SERIAL.read();
        USB_COM_PORT << "Recieved " << data << " on tail. Sending it up. " << i++ << "\n";
        HEAD_SERIAL.write(data); 
      }
    }
  }
  USB_COM_PORT << "\n";
}


/************************************************************************************
  processCommmunicationTestCommand(): Tests up and down serial communication
 ***********************************************************************************/
void processRS485CommunicationTestCommand()
{
  USB_COM_PORT << "Running RS-485 communication test.\n";
  
  if (myModuleNumber != 1)
  {
    USB_COM_PORT << "Waiting for Module " << (myModuleNumber - 1) << " to respond.... ";
    
    // Wait for the previous module to respond on RS485 port.
    boolean previousModuleFound = false;
    long startTime = millis();
    while ((millis() - startTime) < 100)
    { 
      byte data = RS485_SERIAL.read();  
      if (data == (myModuleNumber - 1))
      {
        previousModuleFound = true;
        USB_COM_PORT << "Got it!\n";
        break;    
      }
    }
    
    // Print error and quit if we don't find previous module number.
    if (!previousModuleFound)
    {
      USB_COM_PORT << "\nNo response from Module " << (myModuleNumber - 1) << " Will not send out myModuleNumber. Quitting.\n";
      delay(100);
      clearSerialBuffer(RS485_SERIAL);
      return;       
    }
  }

  // Respond with module number to let everyone know im here.
  USB_COM_PORT << "It's now my turn. Sending out myModuleNumber on RS-485.\n";
  digitalWrite(RS485_TX_ENABLE, HIGH);  
  RS485_SERIAL.write(myModuleNumber); 
  RS485_SERIAL.flush();
  digitalWrite(RS485_TX_ENABLE, LOW);

  USB_COM_PORT << "\n";
  //delay(20);
  clearSerialBuffer(RS485_SERIAL);
  return;   
}

/************************************************************************************
  processNewSetpoints(): Recieves new settings and setpoints from the head
 ***********************************************************************************/
void processNewSetpoints()
{
  char chars[125];
  byte* settings;
  
  // Get data array from upstream. Store in an array.
  HEAD_SERIAL.setTimeout(30);
  if (HEAD_SERIAL.readBytes(chars, 125) < 125)
  {
    USB_COM_PORT.println("ERROR: Missing part of settings array.");
    clearSerialBuffer(HEAD_SERIAL);
    return;
  }
  settings = (byte*)&chars[0];
  
  // Ensure the data is valid. 
  byte checksum = calculateChecksum(settings, 124);
  if (settings[124] != checksum)
  {
    USB_COM_PORT << "ERROR: Settings array checksum failed (is: " << checksum << " should be: " << settings[124] << ")\n";
    clearSerialBuffer(HEAD_SERIAL);
    return;
  }
  
  // Send data array downstream.
  TAIL_SERIAL.write('s');
  TAIL_SERIAL.write(settings, 125);

  // Copy new setpoints [Setting bytes 0 to 59]
  for (int i = 0; i < 5; ++i)
  {
    byte newHorzAngle = settings[(myModuleNumber-1) * 5 + i];
    byte newVertAngle = settings[(myModuleNumber-1) * 5 + i + 30];
      
    setHorzAngleSetpoint(i, newHorzAngle);
    setVertAngleSetpoint(i, newVertAngle);
  }
  
  // Change the lights
  int lightByte = 60 + myModuleNumber - 1;
  digitalWrite(LED[0], (settings[lightByte] & B00000001) == B00000001);
  digitalWrite(LED[1], (settings[lightByte] & B00000010) == B00000010);
  digitalWrite(LED[2], (settings[lightByte] & B00000100) == B00000100);
  digitalWrite(LED[3], (settings[lightByte] & B00001000) == B00001000);
  digitalWrite(LED[4], (settings[lightByte] & B00010000) == B00010000);

  // Check the kill switch [Setting bit 70.0]
  boolean newKillSwitchPressed = (settings[70] & B00000001) > 0;
  if (newKillSwitchPressed == false)
  {
    // Stop all movement when the kill switch is released
    stopMovement();
  }
  if (newKillSwitchPressed == true && killSwitchPressed == false)
  {
    // Reset timeout timers when kill switch is pressed
    for (int i = 0; i < 5; ++i)
    {
      horzTimerArray[i] = millis();
      PIDcontrollerHorizontal[i].zeroIntegral();
      vertTimerArray[i] = millis();
      PIDcontrollerVertical[i].zeroIntegral();
    }
  }
  killSwitchPressed = newKillSwitchPressed;
     
  // Copy new motor speed [Setting byte 72]
  runtimeMotorSpeed = settings[72];

  // PID and slew rate settings to be converted to sensor space.
  byte hKp = settings[73];
  byte hKi = settings[74];
  byte hKd = settings[75];
  byte vKp = settings[76];
  byte vKi = settings[77];
  byte vKd = settings[78];
  byte hSlewRate = settings[79];
  byte vSlewRate = settings[80];
  
  if (hKp != horzAngleKp ||
      hKi != horzAngleKi ||
      hKd != horzAngleKd ||
      vKp != vertAngleKp ||
      vKi != vertAngleKi ||
      vKd != vertAngleKd ||      
      hSlewRate != horzAngleSlewRate ||
      vSlewRate != vertAngleSlewRate)
  {
    horzAngleKp = hKp;
    horzAngleKi = hKi;
    horzAngleKd = hKd;
    vertAngleKp = vKp;
    vertAngleKi = vKi;
    vertAngleKd = vKd; 
    horzAngleSlewRate = hSlewRate;
    vertAngleSlewRate = vSlewRate;
    mapAngleConstantsToSensorSpace();
  }
}

/**************************************************************************************
  calculateChecksum(): Calculates a simple XOR checksum from the provided array
 *************************************************************************************/
byte calculateChecksum(byte* array, int count)
{
  byte checksum = 0;
  for (int i = 0; i < count; ++i)
  {
    //USB_COM_PORT.println(array[i]);
    checksum = checksum ^ array[i];    
  }
  //USB_COM_PORT << "\n\n\n\n";
  return checksum;
}

/**************************************************************************************
  printProgMemString(): Print a string stored in program memory to USB_COM_PORT
 *************************************************************************************/
void printProgMemString(const prog_uchar *str)
{
  char c;
  while(c = pgm_read_byte(str++))
    USB_COM_PORT.write(c);
}

/**************************************************************************************
  clearSerialBuffer(): In Arduino 1.0, Serial.flush() no longer does what we want!
                       http://arduino.cc/en/Serial/Flush
 *************************************************************************************/
void clearSerialBuffer(HardwareSerial &serial)
{
  while (serial.available() > 0)
  {
    serial.read();
  }
}

/**************************************************************************************
  clearSerialBuffer(): Removes a given number of bytes from the serial buffer
 *************************************************************************************/
void clearSerialBuffer(HardwareSerial &serial, int bytesToClear)
{
  int bytesCleared = 0;
  while (bytesCleared < bytesToClear)
  {
    while (serial.available() == 0);
    serial.read();
    bytesCleared++;
  }
}

 /************************************************************************************
  processDiagnosticsCommand(): Responds to the head's request to get diagnsotic info.
 ***********************************************************************************/

void processDiagnosticsCommand()
{
  char packetType[1];

  // Get data array from upstream. Store in an array.
  HEAD_SERIAL.setTimeout(30);
  if (HEAD_SERIAL.readBytes(packetType, 1) < 1)
  {
    USB_COM_PORT.println("ERROR: Missing diagnostics packet type");
    return;
  }

  switch (packetType[0])
  {
  case 1:
    sendHeadAndModuleDiagnostics();
    break;
  case 2:
    sendHorzAngleDiagnostics();
    break;
  case 3:
    sendVertAngleDiagnostics();
    break;
  case 4:
    sendHorzCalibrationDiagnostics();
    break;
  case 5:
    sendVertCalibrationDiagnostics();
    break;
  default:
    USB_COM_PORT.println("ERROR: Invalid diagnostics packet type");
    return;
  }

  // Tell next module to send diagnostics
  TAIL_SERIAL.write('d'); 
  TAIL_SERIAL.write(packetType[0]);
}

/************************************************************************************
 * sendHeadAndModuleDiagnostics(): Sends general information about the module. Not vertibrae.
 ***********************************************************************************/
void sendHeadAndModuleDiagnostics()
{
  int batteryVoltage = map(analogRead(BAT_LEVEL_24V),0,1023,0,25000);
  int hydraulicPressure = map(analogRead(PRESSURE_SENSOR),102,921,0,2000);
  
  batteryVoltage = constrain(batteryVoltage, 0, 25000);
  hydraulicPressure = constrain(hydraulicPressure, 0, 2000);
  
  byte data[15];
  data[0] = highByte(batteryVoltage);
  data[1] = lowByte(batteryVoltage);
  data[2] = 0;
  data[3] = runtimeMotorSpeed;
  data[4] = highByte(hydraulicPressure);
  data[5] = lowByte(hydraulicPressure);
  //data[14] = lowByte(hydraulicPressure); Room for more bytes per module
  
  HEAD_SERIAL.write(data, 15);
}

/************************************************************************************
 * sendHorzAngleDiagnostics(): 
 ***********************************************************************************/
void sendHorzAngleDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    byte horzAngle = getCurrentHorizontalAngle(i);
    byte horzIncSetpoint = getCurrentHorizIncrimentalSetpointAngle(i);
       
    data[i * 4 + 0] = horzIncSetpoint;
    data[i * 4 + 1] = horzAngle;
    data[i * 4 + 2] = horzAngleSetpoints[i];
    data[i * 4 + 3] = horzValveOutputs[i];
  }
  HEAD_SERIAL.write(data, 20);
}

/************************************************************************************
 * sendVertAngleDiagnostics(): 
 ***********************************************************************************/
void sendVertAngleDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    byte vertAngle = getCurrentVerticalAngle(i);
    byte vertIncSetpoint = getCurrentVertIncrimentalSetpointAngle(i);
       
    data[i * 4 + 0] = vertIncSetpoint;
    data[i * 4 + 1] = vertAngle;
    data[i * 4 + 2] = vertAngleSetpoints[i];
    data[i * 4 + 3] = vertValveOutputs[i];
  }
  HEAD_SERIAL.write(data, 20);
}

/************************************************************************************
 * sendHorzCalibrationDiagnostics(): 
 ***********************************************************************************/
void sendHorzCalibrationDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    data[i * 4 + 0] = highByte(horzHighCalibration[i]);
    data[i * 4 + 1] = lowByte(horzHighCalibration[i]);
    data[i * 4 + 2] = highByte(horzLowCalibration[i]);
    data[i * 4 + 3] = lowByte(horzLowCalibration[i]);
  }
  HEAD_SERIAL.write(data, 20);
}

/************************************************************************************
 * sendVertCalibrationDiagnostics(): 
 ***********************************************************************************/
void sendVertCalibrationDiagnostics()
{
  byte data[20];
  for (int i = 0; i < 5; ++i)
  {
    // Note: we're sending the straight array for now
    // instead of the calibration. It's more useful.
    data[i * 4 + 0] = highByte(vertHighCalibration[i]);
    data[i * 4 + 1] = lowByte(vertHighCalibration[i]);
    data[i * 4 + 2] = highByte(vertLowCalibration[i]);
    data[i * 4 + 3] = lowByte(vertLowCalibration[i]);
  }
  HEAD_SERIAL.write(data, 20);
}


/************************************************************************************
  processJawMovingMotorCommand(): Turns the first motor on the off so the jaw can 
                                  open or close.
 ***********************************************************************************/

void processJawMovingMotorPulseCommand()
{
  runPidLoop = false;
  
  // This command is only for the first module
  if (myModuleNumber != 1)
    return;
    
  USB_COM_PORT << "Turning motor on for 2500ms (speed = " << jawMotorSpeed << ")\n";
  analogWrite(MOTOR_CONTROL, jawMotorSpeed);
  delay(2500);
  analogWrite(MOTOR_CONTROL, 0);
  USB_COM_PORT.print("Turning motor off\n");
  
  runPidLoop = true;
}

/************************************************************************************
  processShortMotorPulseCommand(): Turns the first module motor on and off so we can
                                   do manual mode on the head and jaw.
 ***********************************************************************************/
 
void processHeadManualModeMotorPulseCommand()
{ 
  runPidLoop = false;
  
  // This command is only for the first module
  if (myModuleNumber != 1)
    return;
    
  USB_COM_PORT.print("Turning motor on for 200ms\n");
  analogWrite(MOTOR_CONTROL, manualMotorSpeed);
  delay(400);
  analogWrite(MOTOR_CONTROL, 0);
  USB_COM_PORT.print("Turning motor off\n");
  
  runPidLoop = true;
}

/************************************************************************************
  mapAngleConstantsToSensorSpace(): Based on the calibration, map the deadbands from angle 
                                    space (0-254) to sensor space (low to high calibration)
 ***********************************************************************************/
void mapAngleConstantsToSensorSpace()
{
  for (int i = 0; i < 5; ++i)
  {
    horzSensorDeadbands[i] = map(horzAngleDeadband, 0, 254, 0, horzHighCalibration[i] - horzLowCalibration[i]);
    vertSensorDeadbands[i] = map(vertAngleDeadband, 0, 254, 0, vertHighCalibration[i] - vertLowCalibration[i]);
    
    horzSensorSlewRates[i] = map(horzAngleSlewRate, 0, 254, 0, horzHighCalibration[i] - horzLowCalibration[i]);
    vertSensorSlewRates[i] = map(vertAngleSlewRate, 0, 254, 0, vertHighCalibration[i] - vertLowCalibration[i]);
    
    horzSensorKp[i] = map(horzAngleKp, 0, 254, 0, horzHighCalibration[i] - horzLowCalibration[i]);
    horzSensorKi[i] = map(horzAngleKi, 0, 254, 0, horzHighCalibration[i] - horzLowCalibration[i]);
    horzSensorKd[i] = map(horzAngleKd, 0, 254, 0, horzHighCalibration[i] - horzLowCalibration[i]);
    
    vertSensorKp[i] = map(vertAngleKp, 0, 254, 0, vertHighCalibration[i] - vertLowCalibration[i]);
    vertSensorKi[i] = map(vertAngleKi, 0, 254, 0, vertHighCalibration[i] - vertLowCalibration[i]);
    vertSensorKd[i] = map(vertAngleKd, 0, 254, 0, vertHighCalibration[i] - vertLowCalibration[i]);   
    
    // Ensure we never get 0 slew rate due to a bad calibration. 
    if (horzSensorSlewRates[i] == 0) horzSensorSlewRates[i] = 1;
    if (vertSensorSlewRates[i] == 0) vertSensorSlewRates[i] = 1;  
  }
  
  // Update the constants in the PID controllers
  for(int i=0;i<5;i++)
  {
    PIDcontrollerHorizontal[i].setConstants(horzSensorKp[i], horzSensorKd[i], horzSensorKi[i]);
    PIDcontrollerVertical[i].setConstants(vertSensorKp[i], vertSensorKd[i], vertSensorKi[i]);
  }
}


/************************************************************************************
  This is an AVR interrupt. On the Arduino MEGA 2560 it triggers every 2ms (different on other Arduinos)
  We are using this interrupt to the run pid loop on a fixed, consistant interval. 
 ***********************************************************************************/
ISR(TIMER1_OVF_vect, ISR_NOBLOCK)
{
  if (runPidLoop == false)
    return;
    
  static int count = 0;
  ++count;
  
  // Wait for "pidLoopTime" milliseconds to occur
  if ((count << 1) > pidLoopTime) // bitshift left to multiple by 2 (i.e. 2 milliseconds per count)
  {      
    count = 0;
    if (killSwitchPressed)
    {
      executePID(runtimeMotorSpeed, true, true); 
    }
    else
    {
      stopMovement();
    }
  }
}

/************************************************************************************
  executePID(): Modulates the values to achieve position set points (Runs the PID)
 ***********************************************************************************/
void executePID(byte motorSpeed, boolean doHorizontal, boolean doVertical)
{
  //set if any segment moved, so we can turn off motor when setpoint reached
  boolean moved = false;

  for (int i=0; i < 5; i++)
  {
    //////////////////////////
    ////// HORIZONTAL ////////
    if (horzSensorSetpoints[i] >= 0 && doHorizontal)
    {
      int currentSensorValue = analogRead(HORZ_POS_SENSOR[i]);
      
      // If we are have not reached the deadband and haven't timed out
      if ((abs(currentSensorValue - horzSensorSetpoints[i]) > horzSensorDeadbands[i]) &&
          (millis() - horzTimerArray[i]) < actuatorTimeout)
      {
        // If enabled, control the slew rate of the actuator.       
        if (horzControlSlewRate)
        {
          signed int dir = (horzSensorIncrementalSetpoints[i] > horzSensorSetpoints[i]) ? -1 : 1;
          int setpoint = horzSensorIncrementalSetpoints[i] + dir * horzSensorSlewRates[i];
          
          if (dir > 0 && setpoint > horzSensorSetpoints[i]) setpoint = horzSensorSetpoints[i];
          if (dir < 0 && setpoint < horzSensorSetpoints[i]) setpoint = horzSensorSetpoints[i];    
          horzSensorIncrementalSetpoints[i] = setpoint;
          PIDcontrollerHorizontal[i].setSetPoint(setpoint);      
        }
        // Otherwise just run the PID directly to the final setpoint.
        else
        {
          horzSensorIncrementalSetpoints[i] = horzSensorSetpoints[i];
          PIDcontrollerHorizontal[i].setSetPoint(horzSensorSetpoints[i]);     
        }          
        
        analogWrite(MOTOR_CONTROL, motorSpeed);
        horzValveOutputs[i] = PIDcontrollerHorizontal[i].updateOutput();
        moved = true;
      }
      // We are in the deadband or we took too long to get there    
      else
      {
        analogWrite(HORZ_ACTUATOR[i],0);
        horzValveOutputs[i] = 0;
      }
    }
    // Setpoint is uninitialized (-1) turn actuator off
    else
    {
      analogWrite(HORZ_ACTUATOR[i], 0);
      horzValveOutputs[i] = 0;
    }

    ///////////////////////    
    ////// VERTICAL ///////
    if (vertSensorSetpoints[i] >= 0 && doVertical && !(i == 4 && iAmLastModule))
    {
      int currentSensorValue = analogRead(VERT_POS_SENSOR[i]);
      
      // If we are have not reached the deadzone and haven't timed out
      if ((abs(currentSensorValue - vertSensorSetpoints[i]) > vertSensorDeadbands[i]) &&
          (millis() - vertTimerArray[i]) < actuatorTimeout)
      {
        // If enabled, control the slew rate of the actuator.       
        if (vertControlSlewRate)
        {
          signed int dir = (vertSensorIncrementalSetpoints[i] > vertSensorSetpoints[i]) ? -1 : 1;
          int setpoint = vertSensorIncrementalSetpoints[i] + dir * vertSensorSlewRates[i];
          
          if (dir > 0 && setpoint > vertSensorSetpoints[i]) setpoint = vertSensorSetpoints[i];
          if (dir < 0 && setpoint < vertSensorSetpoints[i]) setpoint = vertSensorSetpoints[i];    
          vertSensorIncrementalSetpoints[i] = setpoint;          
          PIDcontrollerVertical[i].setSetPoint(setpoint);       
        }
        // Otherwise just run the PID directly to the final setpoint.
        else
        {
          vertSensorIncrementalSetpoints[i] = vertSensorSetpoints[i];          
          PIDcontrollerVertical[i].setSetPoint(vertSensorSetpoints[i]);    
        }
        
        analogWrite(MOTOR_CONTROL, motorSpeed);
        vertValveOutputs[i] = PIDcontrollerVertical[i].updateOutput();
        moved = true;
      }
      // We are in the deadband or we took too long to get there
      else
      {
        analogWrite(VERT_ACTUATOR[i], 0);
        vertValveOutputs[i] = 0;
      }
    }
    // Setpoint is uninitialized (-1) turn actuator off
    else
    {
      analogWrite(VERT_ACTUATOR[i], 0);
      vertValveOutputs[i] = 0;
    }
  }
  
  if(moved == false)
  {
    analogWrite(MOTOR_CONTROL, 0);
  }
}// end move()


/**************************************************************************************
  manualStraightenHorizontal(): Makes titanoboa vertically straight as an arrow horizontally.
                                Important! Make sure pid is disabled when running this function.
 *************************************************************************************/
void manualStraightenHorizontal()
{
  USB_COM_PORT << "Straightening horizontal. \n";
  
  // Set setpoints to halfway between low and high calibration values
  setHorzAngleSetpoint(0, 127);
  setHorzAngleSetpoint(1, 127);
  setHorzAngleSetpoint(2, 127);
  setHorzAngleSetpoint(3, 127);
  setHorzAngleSetpoint(4, 127);
  
  // reset timeout timers
  for (int i = 0; i < 5; ++i)
  {
    horzTimerArray[i] = millis();
  }
  
  // Do three seconds of movement
  unsigned long startTime = millis();
  while (millis() - startTime < actuatorTimeout)
  {
    executePID(manualMotorSpeed, true, false);
    delay(pidLoopTime);
  }
  stopMovement();
}

/**************************************************************************************
  manualStraightenVertical(): For manual mode only. Makes titanoboa vertically straight.
                              Important! Make sure pid is disabled when running this function.
 *************************************************************************************/
void manualStraightenVertical()
{
  USB_COM_PORT << "Straightening vertical.\n ";
  
  setVertAngleSetpoint(0, 127);
  setVertAngleSetpoint(1, 127);
  setVertAngleSetpoint(2, 127);
  setVertAngleSetpoint(3, 127);
  setVertAngleSetpoint(4, 127);  

  // reset timeout timers
  for (int i = 0; i < 5; ++i)
  {
    vertTimerArray[i] = millis();
  }
  
  // Do three seconds of movement
  unsigned long startTime = millis();
  while (millis() - startTime < actuatorTimeout)
  {
    executePID(manualMotorSpeed, false, true);
    delay(pidLoopTime);
  }
  stopMovement();
}

/**************************************************************************************
  processCalibrateCommand(): Straightens. Tells the next module to calibrate then calibrates this one.
 *************************************************************************************/
void processCalibrateCommand()
{
  runPidLoop = false;
  char vertOrHorz[1];

  // Get data array from upstream. This will tell us if we're doing vertical or horizontal
  HEAD_SERIAL.setTimeout(30);
  if (HEAD_SERIAL.readBytes(vertOrHorz, 1) < 1)
  {
    USB_COM_PORT.println("ERROR: Missing vertical or horizontal byte");
    return;
  }
  if (vertOrHorz[0] != 'h' && vertOrHorz[0] != 'v')
  {
    USB_COM_PORT << "ERROR: Expected 'h' or 'v'. Got '" << vertOrHorz[0] << "'";
    return;
  }
  
  TAIL_SERIAL.write('c');
  TAIL_SERIAL.write(vertOrHorz[0]);
  
  // Stop any current motion
  stopMovement();
  
  // Run calibration
  if (vertOrHorz[0] == 'h')
  {
    USB_COM_PORT << "h";
    calibrateHorizontal();
  }
  if (vertOrHorz[0] == 'v')
  {
    calibrateVertical();
  }
  runPidLoop = true;
} // end processHorzCalibrateCommand



/**************************************************************************************
  calibrateHorizontal(): Records horizontal sensor values at the hard limits of each vertebrae. Saves to EEPROM.
 *************************************************************************************/
void calibrateHorizontal()
{
  USB_COM_PORT << "\nCalibrating Horizontal\n"; 
  
  USB_COM_PORT << "Contracting All... ";
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);  
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], HIGH);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  long startTime = millis();
  while (millis() - startTime < 4000)
  {
    if (HEAD_SERIAL.available() > 0)
      {
        char command = HEAD_SERIAL.read();
        if (command == 's')
          {
            TAIL_SERIAL.print('s');
            stopMovement(); 
            return;
          }
      }
  }
  
  stopMovement();
  delay(1000);
  for(int i = 0; i < 5; i++)
  {
    horzHighCalibration[i] = analogRead(HORZ_POS_SENSOR[i]);
  }
  USB_COM_PORT << "Recorded\n";   

  USB_COM_PORT << "Extending All... ";
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[i], LOW);
    analogWrite(HORZ_ACTUATOR[i],255);
  }
  startTime = millis();
  while (millis() - startTime < 4000)
  {
    if (HEAD_SERIAL.available() > 0)
      {
        char command = HEAD_SERIAL.read();
        if (command == 's')
          {
            TAIL_SERIAL.print('s');
            stopMovement(); 
            return;
          }
      }
  }
  
  stopMovement(); 
  delay(1000);
  for(int i = 0; i < 5; i++)
  {
    horzLowCalibration[i] = analogRead(HORZ_POS_SENSOR[i]);
  }
  USB_COM_PORT << "Recorded\n";    

  // Adjust high and low range and account for alternating actuators
  for(int i=0;i<5;i++)
  {
    if(horzHighCalibration[i]>horzLowCalibration[i])
    {
    }
    else
    {
      int temp = horzHighCalibration[i];
      horzHighCalibration[i] = horzLowCalibration[i];
      horzLowCalibration[i] = temp;
    }
  }
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i = 0; i < 5; i++)
  { 
    EEPROM.write(i*5+2,lowByte(horzHighCalibration[i]));
    EEPROM.write(i*5+3,highByte(horzHighCalibration[i]));
    EEPROM.write(i*5+4,lowByte(horzLowCalibration[i]));
    EEPROM.write(i*5+5,highByte(horzLowCalibration[i]));
  }

  manualStraightenHorizontal();
  USB_COM_PORT << "Calibration Done\n";
  
  mapAngleConstantsToSensorSpace();
  printCalibrationValues();
  
  HEAD_SERIAL.print(myModuleNumber);
  
}//end calibrateHorizontal()


/**************************************************************************************
  calibrateVertical(): Records vertical sensor values at the hard limits of each 
                       vertebrae. 
                       Saves to EEPROM.
 *************************************************************************************/
void calibrateVertical()
{
  USB_COM_PORT << "Calibrating Vertical\n";
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);

  // for vertical calibration don't run actuators to their extremes all in the same 
  // direction otherwise will end up with a giant U that will tip over
  for (int i = 0; i < 5; i++)
  {
    if (myModuleNumber%2 == 0)
    {
      //even numbered module
      if (i%2 == 0)
      {
        //even numbered actuators       
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
    else
    {
      //odd numbered module
      if (i%2 == 0)
      {
        //even numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
  }

  //ensure there's enough time for actuators to reach their maximum
  //before turning off the motor and killing signal to actuators
  delay(4000);
  analogWrite(MOTOR_CONTROL, 0);
  for (int i=0;i<5;i++)
  {
    analogWrite(VERT_ACTUATOR[i], 0);
  }

  // Record 1st set of values
  for(int i=0; i<5; i++)
  {
    vertHighCalibration[i] = analogRead(VERT_POS_SENSOR[i]);
  }

  //Now go to the other extremes
  analogWrite(MOTOR_CONTROL, calibrateMotorSpeed);
  for (int i=0; i<5; i++)
  {
    if (myModuleNumber%2 == 0)
    {
      //even numbered module
      if (i%2 == 0)
      {
        //even numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
    else
    {
      //odd numbered module
      if (i%2 == 0)
      {
        //even numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], HIGH);
        analogWrite(VERT_ACTUATOR[i],255);
      }
      else
      {
        //odd numbered actuators
        digitalWrite(VERT_ACTUATOR_CTRL[i], LOW);
        analogWrite(VERT_ACTUATOR[i],255);
      }
    }
  }

  // ensure there's enough time for actuators to reach their maximum
  // before turning off the motor and killing signal to actuators
  delay(4000);
  analogWrite(MOTOR_CONTROL, 0);
  for (int i=0;i<5;i++)
  {
    analogWrite(VERT_ACTUATOR[i],0);
  }

  // Record 2nd set of values
  for (int i=0; i<5; i++)
  {
    vertLowCalibration[i] = analogRead(VERT_POS_SENSOR[i]);
  }
  
  // Adjust high and low range and account for alternating actuators
  for(int i=0;i<5;i++)
  {
    if(vertHighCalibration[i] < vertLowCalibration[i])
    {
      int temp = vertHighCalibration[i];
      vertHighCalibration[i] = vertLowCalibration[i];
      vertLowCalibration[i] = temp;
    }
  }
  
  // Save calibration to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i=0;i<5;i++)
  { 
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 0, lowByte(vertHighCalibration[i]));
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 1, highByte(vertHighCalibration[i]));
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 2, lowByte(vertLowCalibration[i]));
    EEPROM.write(i*5 + VERTICAL_CALIBRATION_ADDRESS + 3, highByte(vertLowCalibration[i]));
  }
  manualStraightenVertical();
  USB_COM_PORT << "Calibration Done\n";
  
  mapAngleConstantsToSensorSpace();
  printCalibrationValues();
  
}//end calibrateVertical()


/**************************************************************************************
  getCurrentHorizontalAngle(): Get horizontal vertibrae value in 0-254 angle space
 *************************************************************************************/
byte getCurrentHorizontalAngle(int i)
{
  int constrainedValue = constrain(analogRead(HORZ_POS_SENSOR[i]), horzLowCalibration[i], horzHighCalibration[i]);
  return map(constrainedValue, horzLowCalibration[i], horzHighCalibration[i], 0, 254);
}

/**************************************************************************************
  getCurrentVerticalAngle(): Get vertical vertibrae value in 0-254 angle space
 *************************************************************************************/
byte getCurrentVerticalAngle(int i)
{
  int constrainedValue = constrain(analogRead(VERT_POS_SENSOR[i]), vertLowCalibration[i], vertHighCalibration[i]);
  return map(constrainedValue, vertLowCalibration[i], vertHighCalibration[i], 0, 254);
}

/**************************************************************************************
  getCurrentHorizontalIncrimentalSetpoint(): 
 *************************************************************************************/
byte getCurrentHorizIncrimentalSetpointAngle(int i)
{
  if (horzSensorIncrementalSetpoints[i] == -1) return 255;
  int constrainedValue = constrain(horzSensorIncrementalSetpoints[i], horzLowCalibration[i], horzHighCalibration[i]); 
  return map(constrainedValue, horzLowCalibration[i], horzHighCalibration[i], 0, 254);
}

/**************************************************************************************
  getCurrentVerticalIncrimentalSetpoint(): 
 *************************************************************************************/
byte getCurrentVertIncrimentalSetpointAngle(int i)
{
  if (vertSensorIncrementalSetpoints[i] == -1) return 255;
  int constrainedValue = constrain(vertSensorIncrementalSetpoints[i], vertLowCalibration[i], vertHighCalibration[i]); 
  return map(constrainedValue, vertLowCalibration[i], vertHighCalibration[i], 0, 254);
}

/**************************************************************************************
  printSensorDiagnostics(): Print sensor values and setpoints to the USB Serial
 *************************************************************************************/
void printSensorDiagnostics()
{
  USB_COM_PORT.print("\nSensor Diagnostics\n");
  USB_COM_PORT.print("Angles Values are 0-254: 255 = disabled\n");
  USB_COM_PORT.print("Sensors Values are 0-1023: -1 = disabled\n");

  USB_COM_PORT.print("\nHorizontal\n");
  USB_COM_PORT.print("\nHORZ_ANGLE_SETPOINT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_ANGLE_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << getCurrentHorizontalAngle(i) << "\t";
  }
  USB_COM_PORT.print("\nHORZ_ANGLE_DEADBAND:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleDeadband << "\t";
  }
  USB_COM_PORT.print("\nHORZ_ANGLE_SLEW_RATE:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleSlewRate << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_P:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleKp << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_I:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleKi << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_D:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzAngleKd << "\t";
  }
  

  USB_COM_PORT.print("\n\nHORZ_SENSOR_CAL_HIGH: \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzHighCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_SENSOR_CAL_LOW:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzLowCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_SENSOR_CAL_RANGE:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzHighCalibration[i] - horzLowCalibration[i] << "\t";
  }
  
   
  USB_COM_PORT.print("\n\nHORZ_SENSOR_SETPOINT:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_SENSOR_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << analogRead(HORZ_POS_SENSOR[i]) << "\t";
  }
  USB_COM_PORT.print("\nHORZ_SENSOR_DEADBAND:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorDeadbands[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_SENSOR_SLEW_RATE:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorSlewRates[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_P:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorKp[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_I:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorKi[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_D:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzSensorKd[i] << "\t";
  }
  USB_COM_PORT << "\n";
  
  
  USB_COM_PORT.print("\nVertical\n");
  USB_COM_PORT.print("\nVERT_ANGLE_SETPOINT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << getCurrentVerticalAngle(i) << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_DEADBAND:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleDeadband << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_SLEW_RATE:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleSlewRate << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_P:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleKp << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_I:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleKi << "\t";
  }
  USB_COM_PORT.print("\nVERT_ANGLE_D:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertAngleKd << "\t";
  }
  

  USB_COM_PORT.print("\n\nVERT_SENSOR_CAL_HIGH:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertHighCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_CAL_LOW:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertLowCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_CAL_RANGE: \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertHighCalibration[i] - vertLowCalibration[i] << "\t";
  }  
  
  USB_COM_PORT.print("\n\nVERT_CAL_STRAIGHT:\t");  
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertStraightArray[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_SETPOINT:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorSetpoints[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_CURRENT:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << analogRead(VERT_POS_SENSOR[i]) << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_DEADBAND:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorDeadbands[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_SLEW_RATE:\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorSlewRates[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_P:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorKp[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_I:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorKi[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_SENSOR_D:\t\t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertSensorKd[i] << "\t";
  }
  USB_COM_PORT << "\n";

}

/**************************************************************************************
  printCalibrationValues(): Prints all calibration values.
 *************************************************************************************/
void printCalibrationValues()
{
  USB_COM_PORT.println("\nCalibration");
  USB_COM_PORT.print("HORZ_HIGH:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzHighCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_LOW:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzLowCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nHORZ_RANGE: \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << horzHighCalibration[i] - horzLowCalibration[i] << "\t";
  }
  
  USB_COM_PORT.print("\n\nVERT_HIGH:   \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertHighCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_LOW:     \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertLowCalibration[i] << "\t";
  }
  USB_COM_PORT.print("\nVERT_RANGE: \t");
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertHighCalibration[i] - vertLowCalibration[i] << "\t";
  }  
  USB_COM_PORT.print("\nVERT_STRAIGHT:\t");  
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT << vertStraightArray[i] << "\t";
  }  
  USB_COM_PORT.println('\n');
}

/**************************************************************************************
  printBatteryVoltage(): Prints the battery voltage to the USB Serial port.
 *************************************************************************************/
void printBatteryVoltage()
{
  float batteryVolts = ((float)map(analogRead(BAT_LEVEL_24V),0,1023,0,25000))/1000;
  USB_COM_PORT << "The battery is at " << batteryVolts << "V\n";
}

/**************************************************************************************
  printHydraulicPressure(): Prints the pressure to the USB Serial port.
 *************************************************************************************/
void printHydraulicPressure()
{
  int readout = analogRead(PRESSURE_SENSOR);
  float voltage = (float)map(readout, 0, 1023, 0, 500) / 100;
  
  if (voltage < 0.45 || voltage > 4.55)
  {
    USB_COM_PORT << "ERROR: Pressure sensor should be between 0.5V and 4.5V. (" << voltage << "V)\n";
  }
  else
  {
    int hydraulicPressure = map(readout,102,921,0,2000); // 0.5-4.5V > 0-2000psi
    hydraulicPressure = (hydraulicPressure < 0) ? 0 : hydraulicPressure;    
    USB_COM_PORT << "The pressure is at " << hydraulicPressure << "psi (" << voltage << "V)\n";
  }
}


/**************************************************************************************
  saveVerticalPosition(): Saves the current vertical sensor positions
 *************************************************************************************/
void saveVerticalPosition()
{
  for(int i=0;i<5;i++)
  {
    USB_COM_PORT.print("\nSaving current vertical position\n");
    
    // Get the current raw value
    vertStraightArray[i] = analogRead(VERT_POS_SENSOR[i]);
 
    USB_COM_PORT.print("Vertical sensor ");
    USB_COM_PORT.print(i+1);
    USB_COM_PORT.print(": current position ");
    USB_COM_PORT.print(vertStraightArray[i]);
    USB_COM_PORT.print(", low limit ");
    USB_COM_PORT.print(vertLowCalibration[i]);
    USB_COM_PORT.print(", high limit ");
    USB_COM_PORT.println(vertHighCalibration[i]);
  }

  // Save current vertical position to EEPROM
  // 10-bit values must be split into two bytes for storage.
  for(int i=0;i<5;i++)
  {
    // save current position as straight
    EEPROM.write(i*5+VERTICAL_STRAIGHT_ADDRESS, lowByte(vertStraightArray[i]));
    EEPROM.write(i*5+VERTICAL_STRAIGHT_ADDRESS+1, highByte(vertStraightArray[i]));
  }
}


/**************************************************************************************
  setManualActuationDelay(): sets the actuation delay for manual mode actuation
 *************************************************************************************/
void setManualActuationDelay()
{
  stopMovement();
  while (USB_COM_PORT.available() < 1);
  char delaySetting = USB_COM_PORT.read();
  switch (delaySetting)
  {
    case 's':
      manualActuationDelay = 150;
      USB_COM_PORT.print("Actuation delay set to smallest time (150ms)\n");
      break;
    default:
    case 'm':
      manualActuationDelay = 200;
      USB_COM_PORT.print("Actuation delay set to medium time (200ms)\n");
      break;
    case 'l':
      manualActuationDelay = 300;
      USB_COM_PORT.print("Actuation delay set to largest time (300ms)\n");
      break;
  }
}

/**************************************************************************************
  manualVerticalActuatorMove(): Pulses to extend or contract the selected vertical actuator
 *************************************************************************************/
void manualVerticalActuatorMove(char dir)
{
  if (dir != 'e' && dir != 'c')
    return;
    
  stopMovement();

  // Setup the valve direction
  digitalWrite(VERT_ACTUATOR_CTRL[manualVertibraeSelect], LOW);
  if (dir == 'c')
  {
    digitalWrite(VERT_ACTUATOR_CTRL[manualVertibraeSelect], HIGH);
  }
  
  // Run actuation
  analogWrite(MOTOR_CONTROL, manualMotorSpeed);
  analogWrite(VERT_ACTUATOR[manualVertibraeSelect], 255);  
  delay(manualActuationDelay);
  stopMovement(); 
   
  USB_COM_PORT << "Vertibrae " << (manualVertibraeSelect + 1) << " " <<
    ((dir == 'e') ? "Vertical Extend" : "Vertical Contract") << " (" << manualActuationDelay << "ms)\n";  
}


/**************************************************************************************
  manualHorizontalActuatorMove(): Pulses to extend or contract the selected horizontal actuator
 *************************************************************************************/
void manualHorizontalActuatorMove(char dir)
{
  if (dir != 'e' && dir != 'c')
    return;
    
  stopMovement();

  // Setup the valve direction
  boolean even = PIDcontrollerHorizontal[manualVertibraeSelect].getEven();
  digitalWrite(HORZ_ACTUATOR_CTRL[manualVertibraeSelect], LOW);
  if (dir == 'c')
  {
    digitalWrite(HORZ_ACTUATOR_CTRL[manualVertibraeSelect], HIGH);
  }
  
  // Run actuation
  analogWrite(MOTOR_CONTROL, manualMotorSpeed);
  analogWrite(HORZ_ACTUATOR[manualVertibraeSelect], 255);  
  delay(manualActuationDelay);
  stopMovement(); 
   
  USB_COM_PORT << "Vertibrae " << (manualVertibraeSelect + 1) << " " <<
    ((dir == 'e') ? "Horizontal Extend" : "Horizontal Contract") << " (" << manualActuationDelay << "ms)\n";  
}


/**************************************************************************************
  manualSetAllLEDs(): Turn all LEDs on or off
 *************************************************************************************/
void manualSetAllLEDs(boolean value)
{
  for (int i = 0; i < 5; i++)
  {
      digitalWrite(LED[i], value);
  }  
}

/**************************************************************************************
  manualSetModuleNumber(): manually set the module number from USB serial
 *************************************************************************************/
void manualSetModuleNumber()
{
  USB_COM_PORT << "Manually program myModuleNumber (0-9)...\n";
  while(USB_COM_PORT.available() < 1);
  int newNum = USB_COM_PORT.read() - 48;
  if (newNum < 0 || newNum > 6)
  {
    USB_COM_PORT << "Invalid\n";
    return;
  }
  USB_COM_PORT << "Now set to " << newNum << "\n";
  EEPROM.write(MY_MODULE_NUMBER_ADDRESS, newNum);
  setModuleNumber(newNum);
  return;    
}

/**************************************************************************************
  setModuleNumber(): Sets the module number and thus the even and odd vertibrae accordingly.
 *************************************************************************************/
void setModuleNumber(int value)
{
  myModuleNumber = value;
 
  // setup even/odd for even modules
  if (myModuleNumber % 2 == 0)
  {  
    PIDcontrollerVertical[0].setEven(false);
    PIDcontrollerVertical[1].setEven(true);
    PIDcontrollerVertical[2].setEven(false);
    PIDcontrollerVertical[3].setEven(true);
    PIDcontrollerVertical[4].setEven(false);
    
    PIDcontrollerHorizontal[0].setEven(false);
    PIDcontrollerHorizontal[1].setEven(true);
    PIDcontrollerHorizontal[2].setEven(false);
    PIDcontrollerHorizontal[3].setEven(true);
    PIDcontrollerHorizontal[4].setEven(false);
  }
  else
  {
    PIDcontrollerVertical[0].setEven(true);
    PIDcontrollerVertical[1].setEven(false);
    PIDcontrollerVertical[2].setEven(true);
    PIDcontrollerVertical[3].setEven(false);
    PIDcontrollerVertical[4].setEven(true);
    
    PIDcontrollerHorizontal[0].setEven(true);
    PIDcontrollerHorizontal[1].setEven(false);
    PIDcontrollerHorizontal[2].setEven(true);
    PIDcontrollerHorizontal[3].setEven(false);
    PIDcontrollerHorizontal[4].setEven(true);  
  }  
}


/**************************************************************************************
  runSerialChainCommunicationTest(): Tests the communication between this module and a downstream 
                                      module with the serial terminator. 
 *************************************************************************************/
boolean runSerialChainCommunicationTest()
{
  // Figure out what module we're running the test to.
  USB_COM_PORT << "\nI am module " << myModuleNumber << ". What is the last module number?\n";
  while (USB_COM_PORT.available() == 0);

  int lastModuleNumber = USB_COM_PORT.read() - 48;
  if (lastModuleNumber < myModuleNumber || lastModuleNumber > 6)
  {
    USB_COM_PORT << "INVALID: " << lastModuleNumber << " is not a valid module number. Can only communicate downstream up to module 6.\n";
    return false;
  }
  USB_COM_PORT << "Running Serial Chain Communication Test: Module " << myModuleNumber << " to Module " << lastModuleNumber << "\n\n";
  USB_COM_PORT << "Send any key to start...\n\n";
  while (USB_COM_PORT.read() == -1);
  
  // Run 20 communication tests
  clearSerialBuffer(TAIL_SERIAL);
  int numberOfErrors = 0;
  
  for (int i = 1; i <= 20; ++i)
  {
    USB_COM_PORT << "Running test " << i << " of 20.\n";
    TAIL_SERIAL.write('t');
    delay(100);
    
    // We should recieve modules numbers followed by ASCII 't' aka ASCII 116
    long startTime = millis();
    int num = myModuleNumber + 1;
    boolean gotLastModuleT = false;
    boolean error = false;
    int numberOfModulesInTest = (lastModuleNumber - myModuleNumber);
    
    while(millis() - startTime < 150)
    {
      // Wait for data
      if (!TAIL_SERIAL.available())
      {
        continue;
      }
      byte data = TAIL_SERIAL.read();
      
      // If we've heard from every module look for ASCII charater 't'
      if (num > lastModuleNumber)
      {
        if (data == 't')
        {
          USB_COM_PORT << "Module #" << num - 1 << " knows it's the last module!\n";
          gotLastModuleT = true;
        }
        else
        {
          USB_COM_PORT << "ERROR: Didn't recieve 't' from last module #" << num -1 << ". Recieved ASCII " << data << " instead.\n";
          error = true;
        }
        break;              
      }
      
      // Look for each module to respond with it's module number (in sequence)
      if (data == num)
      {
        USB_COM_PORT << "Module #" << num << " is talking!\n";
      }
      else
      {
        USB_COM_PORT << "ERROR: Expected Module # '" << num << "'. Recieved '" << data << "'\n";
        error = true;
      }
      ++num;
    }
    // If we didn't recieve data from all modules.
    if (!error && num <= lastModuleNumber)
    {
      USB_COM_PORT << "ERROR: Did not recieve data from all " << numberOfModulesInTest << " modules.\n";
      error = true;     
    }
    // If we didn't recieve 't' from the last module.
    if (!error && !gotLastModuleT)
    {
      static prog_uchar toPrint[] PROGMEM = "ERROR: Did not recieve 't' from the last module. Is the serial terminator plugged in?\n";
      printProgMemString(toPrint);
      error = true;
    }
    USB_COM_PORT << "\n";
    
    if(error) ++numberOfErrors;
  }
  
  // Give the result of the tests
  if (numberOfErrors > 0)
  {
    static prog_uchar toPrint[] PROGMEM = "RESULT: ERROR! Some serial chain communication tests failed.\n\n";
    printProgMemString(toPrint);
  }
  else
  {
    static prog_uchar toPrint[] PROGMEM = "RESULT: SUCCESS! All serial chain communication tests passed.\n\n";
    printProgMemString(toPrint);
  }
  delay(500);
  clearSerialBuffer(TAIL_SERIAL);
}

/**************************************************************************************
  runRS485CommunicationTest(): Makes sure downstream modules are responding over RS-485.
 *************************************************************************************/
boolean runRS485CommunicationTest()
{  
  // Figure out what module we're running the test to.
  USB_COM_PORT << "\nI am module " << myModuleNumber << ". What is the last module number?\n";
  while (USB_COM_PORT.available() == 0);

  int lastModuleNumber = USB_COM_PORT.read() - 48;
  if (lastModuleNumber < myModuleNumber || lastModuleNumber > 6)
  {
    USB_COM_PORT << "INVALID: " << lastModuleNumber << " is not a valid module number. Can only communicate downstream up to module 6.\n";
    return false;
  }
  USB_COM_PORT << "Running RS-485 Communication Test: Module " << myModuleNumber << " to Module " << lastModuleNumber << "\n\n";
  USB_COM_PORT << "Send any key to start...\n\n";
  while (USB_COM_PORT.read() == -1);  
  
  clearSerialBuffer(RS485_SERIAL);
  int numberOfErrors = 0;
  
  for (int i = 1; i <= 20; ++i)
  {
    USB_COM_PORT << "Running RS-485 test " << i << " of 20.\n";
    digitalWrite(RS485_TX_ENABLE, HIGH);
    RS485_SERIAL.write('t');
    RS485_SERIAL.write(myModuleNumber);
    RS485_SERIAL.flush();
    digitalWrite(RS485_TX_ENABLE, LOW);
    clearSerialBuffer(RS485_SERIAL, 2); 
    
    // We should recieve modules numbers in a row
    long startTime = millis();
    int num = myModuleNumber + 1;
    boolean error = false;
    int numberOfModulesInTest = (lastModuleNumber - myModuleNumber);
    
    while(millis() - startTime < 150)
    {
      // Wait for data
      if (!RS485_SERIAL.available())
      {
        continue;
      }
      byte data = RS485_SERIAL.read();
      
      // If we've heard from every module we're done.
      if (num > lastModuleNumber)
      {
        break;              
      }
      
      // Look for each module to respond with it's module number (in sequence)
      if (data == num)
      {
        USB_COM_PORT << "Module #" << num << " is talking!\n";
      }
      else
      {
        USB_COM_PORT << "ERROR: Expected Module # '" << num << "'. Recieved '" << data << "'\n";
        error = true;
      }
      ++num;
    }
    // If we didn't recieve data from all modules.
    if (!error && num <= lastModuleNumber)
    {
      USB_COM_PORT << "ERROR: Did not recieve data from all " << numberOfModulesInTest << " modules.\n";
      error = true;     
    }
    USB_COM_PORT << "\n";
    
    if(error) ++numberOfErrors;
  }
  
  // Give the result of the tests
  if (numberOfErrors > 0)
  {
    static prog_uchar toPrint[] PROGMEM = "RESULT: ERROR! Some RS-485 communication tests failed.\n\n";
    printProgMemString(toPrint);
  }
  else
  {
    static prog_uchar toPrint[] PROGMEM = "RESULT: SUCCESS! All RS-485 communication tests passed.\n\n";
    printProgMemString(toPrint);
  }
  delay(500);
  clearSerialBuffer(RS485_SERIAL);
}


/**************************************************************************************
  manualTurnMotorOn(): Runs that motor at at one of three speed levels. manual mode must
                       be used to stop the motor.
 *************************************************************************************/

void manualTurnMotorOn()
{
  // Figure out what speed to run at
  while (USB_COM_PORT.available() == 0);
  
  char speedSelect = USB_COM_PORT.read();
  switch (speedSelect)
  {
    case '1':
      static prog_uchar toPrint1[] PROGMEM = "Motor running at speed 80 (SEND 's' TO TURN OFF).\n";
      printProgMemString(toPrint1);
      analogWrite(MOTOR_CONTROL, 80);      
      break;
    case '2':
      static prog_uchar toPrint2[] PROGMEM = "Motor running at speed 160 (SEND 's' TO TURN OFF).\n";
      printProgMemString(toPrint2);
      analogWrite(MOTOR_CONTROL, 160);  
      break;
    case '3':
      static prog_uchar toPrint3[] PROGMEM = "Motor running at speed 240 (SEND 's' TO TURN OFF).\n";
      printProgMemString(toPrint3);
      analogWrite(MOTOR_CONTROL, 240);      
      break;    
    default:
      USB_COM_PORT << "ERROR: " << speedSelect << " is not a valid speed selection.\n";
  }
}

/**************************************************************************************
  manualGoToSetpoints(): Runs the PID loop for 3 seconds to achieve the setpoints.
 *************************************************************************************/
void manualGoToSetpoint(char actuator)
{
  USB_COM_PORT << "Vertibrae " << (manualVertibraeSelect + 1) << " Moving ... ";
  
  // reset timeout timers
  if (actuator == 'h') horzTimerArray[manualVertibraeSelect] = millis();
  if (actuator == 'v') vertTimerArray[manualVertibraeSelect] = millis();
 
  // Do three seconds of movement
  unsigned long startTime = millis();
  while (millis() - startTime < actuatorTimeout)
  {
    executePID(manualMotorSpeed, true, true);
    delay(pidLoopTime);
  }
  stopMovement();
  
  USB_COM_PORT << "Done.\n";
}

/**************************************************************************************
  setHorzAngleSetpoint(): Sets a horizontal angle setpoint. The actuator will not move until the 
                          pid loops runs. Actuator numbers are 0 to 4. Angles are 0 ro 254.
                          Angle of 255 means the actuator is disabled and will not be moved.
 *************************************************************************************/
void setHorzAngleSetpoint(int actuator, byte angle)
{
  int i = actuator;
  int newHorzSetpoint = 0;
  if (angle == 255)
  {
    // 255 means this actuator is disabled.
    newHorzSetpoint = -1;
  }
  else
  {
    // Horizontal actuator is not disabled. Map to sensor space.
    newHorzSetpoint = map(angle, 0, 254, horzLowCalibration[i], horzHighCalibration[i]);
  }
   
  // If this is a new setpoint, reset the PID timout timer
  if (newHorzSetpoint != horzSensorSetpoints[i])
  {      
    PIDcontrollerHorizontal[i].zeroIntegral();
    horzTimerArray[i] = millis();
    horzSensorSetpoints[i] = newHorzSetpoint;
    horzAngleSetpoints[i] = angle;
  }
    
  // If we aren't limiting the slew rate make the incremental setpoint the same.
  if (horzControlSlewRate == false)
    horzSensorIncrementalSetpoints[i] = newHorzSetpoint;  
}

/**************************************************************************************
  setVertAngleSetpoint(): Sets a vertical angle setpoint. The actuator will not move until the 
                          pid loops runs. Actuator numbers are 0 to 4. Angles are 0 ro 254.
                          Angle of 255 means the actuator is disabled and will not be moved.
 *************************************************************************************/
void setVertAngleSetpoint(int actuator, byte angle)
{
    int i = actuator;
    int newVertSetpoint = 0;
    if (angle == 255)
    {
      // 255 means this actuaor is disabled.
      newVertSetpoint = -1;
    }
    else if (angle == 127)
    {
      // We have a special setpoint for vertical angle 127 (straight)
      newVertSetpoint = vertStraightArray[i];
    }
    else if (PIDcontrollerVertical[i].getEven())
    {
      // Even vertical acutuators are mapped normally.
      newVertSetpoint = map(angle, 0, 254, vertLowCalibration[i], vertHighCalibration[i]);
    }
    else
    {
      // Since vertical sensors alternate sides, we need to invert the mapping from angle space to sensor space.
      newVertSetpoint = map(angle, 0, 254, vertHighCalibration[i], vertLowCalibration[i]);
    }  
    
    // If this is a new setpoint, copy it in and reset the timeout timer.
    if (newVertSetpoint != vertSensorSetpoints[i])
    {
      PIDcontrollerVertical[i].zeroIntegral();
      vertTimerArray[i] = millis();
      vertSensorSetpoints[i] = newVertSetpoint;
      vertAngleSetpoints[i] = angle;      
    }
    
    // If we aren't limiting the slew rate make the incremental setpoint the same.
    if (vertControlSlewRate == false)
      vertSensorIncrementalSetpoints[i] = newVertSetpoint;    
}

/**************************************************************************************
  getNumberFromUsbComPort(): Allows us to type demical numbers in ASCII through the serial
                             com port. Can be reused anywhere. Stops interpretting characters
                             when there are no more left or when it find a space character.
                             VALID VALUES: 0 to 32767
 *************************************************************************************/
boolean getNumberFromUsbComPort(int &number)
{
  // Wait for a character to arrive + extra time to allow more characters to arrive.
  while (USB_COM_PORT.available() == 0);
  delay(50);
  
  number = 0;
  
  // Add up the digits
  while (USB_COM_PORT.available())
  {
    char character = USB_COM_PORT.read();
    byte digit = character - 48;
    if (character == ' ') return true;
    if (digit > 9) return false;    
    number = number * 10 + digit;
  }
  return true;
}

/**************************************************************************************
  manualSetHorizontalAngle(): For manual mode. Set the horizonal angle of the selected vertibrae.
 *************************************************************************************/
 void manualSetHorizontalAngleAndMove()
 {
   int angle = 0;
   if (getNumberFromUsbComPort(angle))
   {
     if (angle < 0 || angle > 255)
     {
        USB_COM_PORT << "Invalid entry. (" << angle << ")\n";
        return; 
     }
     setHorzAngleSetpoint(manualVertibraeSelect, (byte)angle);  
     USB_COM_PORT << "Vertibrae " << manualVertibraeSelect + 1 << " Horizontal Angle set to " << angle << "\n";   
   }
   else
   {
      USB_COM_PORT << "Invalid entry. (" << angle << ")\n";     
   }
   manualGoToSetpoint('h');
 }
 
 /**************************************************************************************
  manualSetVertAngleSetpoint(): For manual mode. Set the vertical angle of the selected vertibrae.
 *************************************************************************************/
void manualSetVerticalAngleAndMove()
{  
  int angle = 0;
  if (getNumberFromUsbComPort(angle))
  {
    if (angle < 0 || angle > 255)
    {
       USB_COM_PORT << "Invalid entry. (" << angle << ")\n";
       return; 
     }
     setVertAngleSetpoint(manualVertibraeSelect, (byte)angle);
     USB_COM_PORT << "Vertibrae " << manualVertibraeSelect + 1 << " Vertical Angle set to " << angle << "\n";
  }
  else
  {
    USB_COM_PORT << "Invalid entry. (" << angle << ")\n";     
  }
   manualGoToSetpoint('v');
}

/**************************************************************************************
  manualPrintFreeMemory(): 
 *************************************************************************************/
void manualPrintFreeMemory()
{
  USB_COM_PORT << "There are " << freeMemory() << " bytes of free memory. (MEGA has 8KB total of which only 7KB is usable) \n";
}

/**************************************************************************************
  displayMenu(): Shows the command menu for manual control over usb serial
 *************************************************************************************/
void displayMenu()
{  
  static prog_uchar menu[] PROGMEM =
    "\nManual Control Mode\n"                
    "commands: 1-5 to select vertebrae\n"                
    "          k/l - horizontal actuation - extend/contract\n"                
    "          o/i - vertical actuation - extend/contract\n"                
    "          d* - adjust actuation delay where *=s(small), m(medium), l(large)\n"                
    "          w* - move horizontal to angle where * is 0 to 255\n"                
    "          z* - move vertical to angle where * is 0 to 255\n\n"                
  
    "          c - calibrate horizontals\n"                
    "          v - calibrate verticals\n"                
    "          t - straighten horizontals\n"                  
    "          y - straighten verticals\n\n"                  
  
    "          r - save current vertical position as straight\n"                
    "          a - print battery voltage\n"                  
    "          f - print hydraulic pressure\n"                 
    "          p - print sensor diagnostics.\n"                
    "          P - print memory consumption.\n\n"                
  
    "          x* - turn motor on where *=1(slow), 2(medium), 3(fast)\n"                
    "          s - stop motor\n"                
    "          n/m - all leds on/off\n"                
    "          g - run serial chain communication test to downstram modules\n"                
    "          G - run RS-485 communication test to downstram modules\n"                
    "          u - manually set myModuleNumber\n\n"                
  
    "          e - menu\n"                
    "          q - quit\n\n";
                    
  printProgMemString(menu);
}

/**************************************************************************************
  manualControl(): Allows for manual control of this module
 *************************************************************************************/
void manualControl()
{
  runPidLoop = false;
  displayMenu();
  
  boolean manual = true;
  while(manual == true)
  {
    if(Serial.available() > 0)
    {
      char manualCommand = USB_COM_PORT.read();      
      switch(manualCommand)
      {        
        case '1':
          manualVertibraeSelect = 0;
          USB_COM_PORT.print("Vertibrae 1 Selected\n");
          break;
        case '2':
          manualVertibraeSelect = 1;
          USB_COM_PORT.print("Vertibrae 2 Selected\n");
          break;
        case '3':
          manualVertibraeSelect = 2;
          USB_COM_PORT.print("Vertibrae 3 Selected\n");
          break;
        case '4':
          manualVertibraeSelect = 3;
          USB_COM_PORT.print("Vertibrae 4 Selected\n");
          break;
        case '5':
          manualVertibraeSelect = 4;
          USB_COM_PORT.print("Vertibrae 5 Selected\n");
          break;  
    
        case 'e':
          displayMenu();
          break;
        case 'w':
          manualSetHorizontalAngleAndMove();
          break;
        case 'z':
          manualSetVerticalAngleAndMove();
          break;
          
          
        case 'c':
          calibrateHorizontal();
          break;
        case 'v':
          calibrateVertical();
          break;
        case 't':
          manualStraightenHorizontal();
          break;
        case 'y':
          manualStraightenVertical();
          break;
        case 'p':
          printSensorDiagnostics();
          break;
        case 'f':
          printHydraulicPressure();
          break;
        case 'r':
          saveVerticalPosition();
          break;
        case 'd':
          setManualActuationDelay();
          break;
        case 'l':
          manualHorizontalActuatorMove('c');
          break;
        case 'k':
          manualHorizontalActuatorMove('e');
          break;
        case 'i':
          manualVerticalActuatorMove('c');
          break;
        case 'o':
          manualVerticalActuatorMove('e');
          break;
        case 'n':
          manualSetAllLEDs(HIGH);
          break;
        case 'm':
          manualSetAllLEDs(LOW);
          break;
        case 'u':
          manualSetModuleNumber();
          break;
        case 'g':
          runSerialChainCommunicationTest();
          break;
        case 'G':
          runRS485CommunicationTest();
          break;
        case 'a':
          printBatteryVoltage();
          break;
        case 'P':
          manualPrintFreeMemory();
          break;
        case 'x':
          manualTurnMotorOn();
          break;
        case 's':
          USB_COM_PORT.print("Stopped all movement\n");
          stopMovement();
          break;          
        case 'q':
          manual = false;
          break;
      }//end switch
    }//end if serial
  }  
  
  stopMovement();
  USB_COM_PORT.println("\nManual Control mode exited");
  runPidLoop = true;
  
  // Clear serial buffers to try and get back in sync
  clearSerialBuffer(HEAD_SERIAL);
  clearSerialBuffer(TAIL_SERIAL);  
}


/**************************************************************************************
  StopMovement(): Stops movement of all actuators.
 *************************************************************************************/
void stopMovement()
{
  // Stop any current motion
  for(int i=0; i < 5; i++)
  {
    analogWrite(VERT_ACTUATOR[i], 0);
    analogWrite(HORZ_ACTUATOR[i], 0);
    vertValveOutputs[i] = 0;
    horzValveOutputs[i] = 0;
  }
  analogWrite(MOTOR_CONTROL, 0);
  return;
}







