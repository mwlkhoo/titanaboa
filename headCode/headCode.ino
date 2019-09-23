/*

  headCode.ino - Controls head of Titanoboa
  
  Created: July 9, 2011 
  Part of the titanaboa.ca project
  
  Decription: This code runs on an Arduino MEGA to control the entire
  Titanoboa snake. It is the brain of the operation and it schedules all
  communication. 

  Task list:
  - Requests status of switches and knobs from the joystick
  - Sets new settings and setpoints to every module
  - Requests diagnostic data and sets it out over wifi.
*/

#include "headPins.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <EEPROM.h>

#define INPUT_SERIAL Serial3            // Xbee
#define TAIL_SERIAL Serial2             // Serial to the downstream module
#define RS485_SERIAL Serial1            // Serial bus over the entire snake
#define USB_COM_PORT Serial             // Serial for debugging

// Enable serial stream writing
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

// Write to the USB com port only if we are in debug mode
boolean inDebugMode = false;
#define DEBUG_STREAM if(inDebugMode) USB_COM_PORT 

// Variables for controlling titanoboa. Currently this data comes
// from the joystick, but in the future it could come from an 
// Android tablet or a PC.
class ControllerData 
{  
  public:
  boolean right;
  boolean left;
  boolean up;
  boolean down;
  boolean killSwitchPressed;
  boolean openJaw;
  boolean closeJaw;
  boolean straightenVertical;
  boolean straightenVertOnTheFly;
  boolean calibrate;
} controller;

// Ethernet/Wi-Fi variables
EthernetUDP Udp;
byte myMacAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
byte myIPAddress[] = { 192, 168, 1, 2 };
byte broadcastIP[] = { 192, 168, 1, 255 };
unsigned int broadcastPort = 12345;
unsigned int listeningPort = 12346;

// Setpoints for all 30 vertibrae in the snake
// Angles are from 0 to 254, 255 means uninitialized.
byte vertSetpoints[30];
byte horzSetpoints[30];
boolean lights[30];

// Constants
const int horzLeftAngle = 10;
const int horzRightAngle = 245;
const int horzStraightAngle = 127;
const int vertUpAngle = 87;
const int vertDownAngle = 167;
const int vertStraightAngle = 127;
const int longDelayBetweenHeartBeats = 1100; 
const int shortDelayBetweenHeartBeats = 250;
const int ledDelayWhenRunning = 30;


// Global variables
byte numberOfModules = 0;                // Number of modules stored in EEPROM
boolean anglesAreInitialized = false;     // On initialization we wait for all modules to report their current angles
boolean joystickIsConnected = false;     // True if we are recieving data from the joystick
int lastLoopTime = 0;                    // Time it took to complete the main loop. (diagnostics, settings, etc)
int actualPropagationDelay = 0;          // We are polling when when to do propagation. This is the actual delay we get.
int manualActuatorSelect = 0;            // For selecting which actuator we are operating in manual mode
int manualActuationDelay = 200;          // How long to open the valve for when pulsing actuators in manual mode.
byte motorSpeed = 0; 
unsigned short propagationDelay = 0;

// PID and slew rate settings
byte horzKp = 255;
byte horzKi = 0;
byte horzKd = 0;
byte vertKp = 75;
byte vertKi = 50;
byte vertKd = 0;
byte horizontalSlewRate = 0;
byte verticalSlewRate = 0;


/*************************************************************************
 setup(): Initializes serial ports, led and actuator pins
**************************************************************************/
void setup()
{
  // Initialize serial ports
  USB_COM_PORT.begin(115200);
  TAIL_SERIAL.begin(115200);
  INPUT_SERIAL.begin(115200);
  RS485_SERIAL.begin(115200);
  
  // Setup the output that enables RS485 transmit  
  pinMode(RS485_TX_ENABLE, OUTPUT);
  digitalWrite(RS485_TX_ENABLE, LOW);  
  
  // Initialize Ethernet Board
  Ethernet.begin(myMacAddress, myIPAddress);
  Udp.begin(listeningPort);  
  
  // Load saved settings
  numberOfModules = EEPROM.read(0);
  USB_COM_PORT.println("Hi I'm the Titanoboa Head!");  
  USB_COM_PORT.print("Number of modules: ");
  USB_COM_PORT.println(numberOfModules);
  USB_COM_PORT.println();
 
  // Initialize setponts
  //   - Actuators are disabled (ie. angle 255)
  //   - Lights are off
  for (int i = 0; i < 30; ++i)
  {
    horzSetpoints[i] = 255;
    vertSetpoints[i] = 255;    
    lights[i] = false;    
  }
  
  // LED Outputs
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);  

  // We have the ability to do actuator dithering, but it's not 
  // really that useful the way our electronics are setup
  pinMode(DITHER, OUTPUT);
  digitalWrite(DITHER, LOW);
  
  // Initialize actuator PWM outputs
  for(int i=0;i<3;i++)
  {
    pinMode(HEAD_ACTUATORS[i],OUTPUT);
    analogWrite(HEAD_ACTUATORS[i],0);
  }
  
  // Initialize actuator control outputs
  // Setting control outputs to zero selects odd pairs (1,3,5 as opposed to 2,4,6)  
  for(int i=0;i<3;i++)
  {
    pinMode(HEAD_ACTUATOR_CTRLS[i], OUTPUT);
    digitalWrite(HEAD_ACTUATOR_CTRLS[i], LOW);
  }

  // Initialize position sensor inputs.
  for(int i=0;i<5;i++)
  {
    pinMode(HEAD_POS_SENSOR[i], INPUT);
  }
}

/*************************************************************************
 loop(): Titanoboa's main thought process. It tells the whole snake 
         what to do!
**************************************************************************/
void loop()
{
  long startTime = millis();
  
  // The first thing we do before we are allowed to run is get the current 
  // angles from every module. 
  if (!anglesAreInitialized)
  {
    initializeAngleArray();
  }
  else
  {
    // If the angle array had been initialized we are can the snake in
    // normal operation.
    doRuntimeLogic();
  }

  // Manual mode over USB Serial.
  if (USB_COM_PORT.available() > 0 && USB_COM_PORT.read() == 'M')
  {
    manualControl();
    synchronizeWithJoystick();
    return;
  }
  
  lastLoopTime = millis() - startTime;
}

/*************************************************************************
 runRuntimeLogic(): 
**************************************************************************/
void doRuntimeLogic()
{
  ////////////////////////////////////////////////////////////////////
  // This block of code runs the core functionality of Titanoboa 

  readAndRequestJoystickData();
  updateLights();
  updateSetpoints();  
  sendSetpointsAndSettings();
  getAndBroadcastDiagnostics(); 
  
  ////////////////////////////////////////////////////////////////////
  // The following functions don't run on a regular basis.
  // When they do run, they take a long time.
  
  // Should we calibrate?
  if (controller.calibrate && controller.killSwitchPressed)
  {
    runHorzSensorCalibration();
    synchronizeWithJoystick();
    return;
  }
  
  // Should we open the jaw?
  if (controller.openJaw && controller.killSwitchPressed)
  {
    openTheJaw();
    synchronizeWithJoystick();  
    return;
  }

  // Should we close the jaw?
  if (controller.closeJaw && controller.killSwitchPressed)
  {
    closeTheJaw();
    synchronizeWithJoystick(); 
    return;
  }  
}

/**************************************************************************************
  initializeAngleArray(): Initialize the setpoints based on the current position of the snake.
 *************************************************************************************/
void initializeAngleArray()
{
  USB_COM_PORT << "Trying to initialize angle array... ";
  
  byte horzPacket[125];
  
  // Get horizontal angles
  boolean result = getHorzAngleDiagnostics(horzPacket);  
  if (result)
  {
    for (int i = 0; i < 30; ++i)
    {
      byte angle = horzPacket[i * 4 + 1 + 1];
      
      // Force the actuator into left, straight or right
      if (angle < (horzStraightAngle + horzLeftAngle) / 2)
        horzSetpoints[i] = horzLeftAngle;
      else if (angle > (horzStraightAngle + horzRightAngle) / 2) 
        horzSetpoints[i] = horzRightAngle;
      else 
        horzSetpoints[i] = horzStraightAngle;
    }
    
    // Get vertical angles
    byte vertPacket[125];
    result = getVertAngleDiagnostics(vertPacket);      
    if (result)
    {
    for (int i = 0; i < 30; ++i)
    {
      byte angle = vertPacket[i * 4 + 1 + 1];
      
      // Force the actuator into up, straight or down
      if (angle < (vertStraightAngle + vertUpAngle) / 2)
        vertSetpoints[i] = vertUpAngle;
      else if (angle > (vertStraightAngle + vertDownAngle) / 2) 
        vertSetpoints[i] = vertDownAngle;
      else 
        vertSetpoints[i] = vertStraightAngle;
    }
      
      // Got all we need!!
      anglesAreInitialized = true;
      USB_COM_PORT << "Success!\n";
    }
  }
  
  if(!anglesAreInitialized)
  {
    USB_COM_PORT << "Failed (Modules not responding?)\n";
    delay(1000);
  }
}

/**************************************************************************************
  getAndBroadcastDiagnostics(): Gets information from the modules and sends it over WiFi
 *************************************************************************************/
void getAndBroadcastDiagnostics()
{
  // Clear the packet data array
  static byte packet[125];
  for (int i = 0; i < 125; ++i)
  {
    packet[i] = 0;
  }
  
  // What type of diagnostics packet should we send?
  // Cycle through each of the 5 packet each time this function is called.
  static byte packetType = 1;
  ++packetType;
  if (packetType > 5)
  {
    packetType = 1;
  }

  // Fill the packet with data
  boolean retVal = false;
  switch (packetType)
  {
    case 1:
      retVal = getHeadAndModuleDiagnostics(packet);
      break;
    case 2:
      retVal = getHorzAngleDiagnostics(packet);
      break;
    case 3:
      retVal = getVertAngleDiagnostics(packet);
      break;
    case 4:
      retVal = getHorzCalibrationDiagnostics(packet);
      break;
    case 5:
      retVal = getVertCalibrationDiagnostics(packet);
      break;
    default: 
      DEBUG_STREAM << "ERROR: Invalid packet type = " << packetType << "\n";
      return;
  }
  if (retVal == false)
  {
    // Packet was not filled for some reason.
    return;
  }
  
  // Add timestamp to end of packet
  long headCodeTimestamp = millis();  
  packet[121] = (byte)(headCodeTimestamp >> 24);
  packet[122] = (byte)(headCodeTimestamp >> 16);
  packet[123] = (byte)(headCodeTimestamp >> 8);
  packet[124] = (byte)(headCodeTimestamp >> 0);  
  
  // Send the diagnostics packet
  Udp.beginPacket(broadcastIP, broadcastPort);  
  for (int i = 0; i < 125; ++i)
  {
    Udp.write(packet[i]);
  }
  Udp.endPacket();
}

/**************************************************************************************
  getHeadAndModuleDiagnostics(): Fills a 125 byte buffer with head and general module info
 *************************************************************************************/
boolean getHeadAndModuleDiagnostics(byte* buffer)
{ 
  buffer[0] = 1;
  
  // Fill in head information
  int headBattery = map(analogRead(BAT_LEVEL_24V),0,1023,0,25000);
  buffer[1] = numberOfModules;
  buffer[2] = highByte(headBattery);
  buffer[3] = lowByte(headBattery);
    
  byte booleanByte = 0;
  booleanByte += ((byte)controller.right) << 0;
  booleanByte += ((byte)controller.left) << 1;
  booleanByte += ((byte)controller.up) << 2;
  booleanByte += ((byte)controller.down) << 3;
  booleanByte += ((byte)controller.killSwitchPressed) << 4;
  booleanByte += ((byte)controller.openJaw) << 5;
  booleanByte += ((byte)controller.closeJaw) << 6;
  booleanByte += ((byte)controller.calibrate) << 7;
  buffer[4] = booleanByte;  
  
  booleanByte = 0;
  booleanByte += ((byte)controller.straightenVertical) << 0;
  booleanByte += ((byte)controller.straightenVertOnTheFly) << 1;
  booleanByte += ((byte)joystickIsConnected) << 2;
  booleanByte += ((byte)anglesAreInitialized) << 3;
  booleanByte += ((byte)false) << 4;
  booleanByte += ((byte)false) << 5;
  booleanByte += ((byte)false) << 6;
  booleanByte += ((byte)false) << 7;
  buffer[5] = booleanByte;
  
  buffer[6] = motorSpeed;
  buffer[7] = highByte(propagationDelay);
  buffer[8] = lowByte(propagationDelay);
  buffer[9] = highByte(actualPropagationDelay);
  buffer[10] = lowByte(actualPropagationDelay);
  buffer[11] = highByte(lastLoopTime);
  buffer[12] = lowByte(lastLoopTime);
  buffer[13] = horzKp;
  buffer[14] = horzKi;
  buffer[15] = horzKd;
  buffer[16] = vertKp;
  buffer[17] = vertKi;
  buffer[18] = vertKd;
  buffer[19] = horizontalSlewRate;
  buffer[20] = verticalSlewRate;
  

  // Tell modules to send us this type of diagnostics
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(1);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[15];  
    if (TAIL_SERIAL.readBytes(data, 15) < 15)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all Module info from module " << i + 1 << "\n";
      return false;
    }
    buffer[i * 15 + 0 + 35] = data[0];
    buffer[i * 15 + 1 + 35] = data[1];
    buffer[i * 15 + 2 + 35] = data[2];
    buffer[i * 15 + 3 + 35] = data[3];
    buffer[i * 15 + 4 + 35] = data[4];
    buffer[i * 15 + 5 + 35] = data[5];
    //buffer[i * 15 + 14 + 35] = data[5]; Room for more bytes per module    
  }
}

/**************************************************************************************
  getHorzAngleDiagnostics(): Fills a 125 byte buffer with horzontal angle and setpoint information.
 *************************************************************************************/
boolean getHorzAngleDiagnostics(byte* buffer)
{
  buffer[0] = 2;
  
  // Tell modules to send us this type of diagnostics
  clearSerialBuffer(TAIL_SERIAL);  
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(2);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all HorzAngle info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];  
    }
  }
  return true;
}

/**************************************************************************************
  getHorzAngleDiagnostics(): Fills a 125 byte buffer with horzontal angle and setpoint information.
 *************************************************************************************/
boolean getVertAngleDiagnostics(byte* buffer)
{
  buffer[0] = 3;
  
  // Tell modules to send us this type of diagnostics
  clearSerialBuffer(TAIL_SERIAL);  
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(3);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all VertAngle info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];    
    }
  }
  return true;
}

/**************************************************************************************
  getHorzCalibrationDiagnostics(): Fills a 125 byte buffer with horizontal calibration info
 *************************************************************************************/
boolean getHorzCalibrationDiagnostics(byte* buffer)
{
  // Tell the wifi listeners what type of packet this is
  buffer[0] = 4;
  
  // Tell modules to send us this type of diagnostics
  clearSerialBuffer(TAIL_SERIAL);  
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(4);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all HorzCalibration info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];    
    }
  }
  return true;
}

/**************************************************************************************
  getVertCalibrationDiagnostics(): Fills a 125 byte buffer with vertical calibration info
 *************************************************************************************/
boolean getVertCalibrationDiagnostics(byte* buffer)
{
  // Tell the wifi listeners what type of packet this is
    buffer[0] = 5;
  
  // Tell modules to send us this type of diagnostics
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write('d');   
  TAIL_SERIAL.write(5);

  // Fill in the packet with module info as it comes in
  TAIL_SERIAL.setTimeout(30);      
  for (int i = 0; i < numberOfModules; ++i)
  {
    char data[20];  
    if (TAIL_SERIAL.readBytes(data, 20) < 20)
    {
      DEBUG_STREAM << "ERROR: Did not recieve all VertCalibration info from module " << i + 1 << "\n";
      return false;
    }
    for (int j = 0; j < 20; ++j)
    {
      buffer[i * 20 + j + 1] = data[j];    
    }
  }
  return true;
}

/**************************************************************************************
  waitForModuleAcknowledgments(): Waits for each module to report back it's module number.
 *************************************************************************************/
void waitForModuleAcknowledgments(char* commandName, short timeout)
{
  // Only wait so long for acknowledgments
  TAIL_SERIAL.setTimeout(timeout);
  
  // Wait for acknowledgments
  char acks[6];  
  byte acksRecieved = TAIL_SERIAL.readBytes(acks, numberOfModules);
  if (acksRecieved != numberOfModules)
  {
    DEBUG_STREAM << "ERROR: Only " << acksRecieved << " of " << numberOfModules << 
      " modules acknowledged " << commandName << " command within " << timeout << "ms\n";
    return;
  }
  
  // Each module should acknowledge by sending us it's module number
  // Make sure this is what happened. 
  for (int i = 0; i < numberOfModules; ++i)
  {
    byte data = acks[i];
    if (data != i + 1)
    {
      DEBUG_STREAM << "ERROR: Recieved bad acknowledgement from module # " << i + 1 << " (" 
        << data << ")\n";
    }
  }
  clearSerialBuffer(TAIL_SERIAL);
}

/**************************************************************************************
  runHorzSensorCalibration(): Calibrates the horizontal sensors on the actuators
 *************************************************************************************/
void runHorzSensorCalibration()
{
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.print("ch");
  //waitForModuleAcknowledgments("calibrate horz", 15000);
  byte lastModuleCalibrated = 0;
  while(lastModuleCalibrated < numberOfModules)
  {
    readAndRequestJoystickData();
    if (!controller.killSwitchPressed)
      {
        TAIL_SERIAL.write('s');
        return;
      }
    if (TAIL_SERIAL.available() > 0)
    {
      lastModuleCalibrated = TAIL_SERIAL.read();
    }        
  }
  // The snake will end up straight.
  for (int i = 0; i < 30; ++i)
  {
    horzSetpoints[i] = 127;  
  }
}

/**************************************************************************************
  runHorzSensorCalibration(): Calibrates the horizontal sensors on the actuators
 *************************************************************************************/
void runVertSensorCalibration()
{
  clearSerialBuffer(TAIL_SERIAL);
  TAIL_SERIAL.write("cv");
  waitForModuleAcknowledgments("calibrate vert", 15000); 
  
  // The snake will end up straight.
  for (int i = 0; i < 30; ++i)
  {
    vertSetpoints[i] = 127;  
  }
}

/**************************************************************************************
  sendSetpointsAndSettings(): Sends the settings down to every module.
 *************************************************************************************/
void sendSetpointsAndSettings()
{
  clearSerialBuffer(TAIL_SERIAL);
  byte settings[125];
  
  // Copy in setpoints
  for (int i = 0; i < 30; ++i)
  {
    settings[i] = horzSetpoints[i];
    settings[i + 30] = vertSetpoints[i];
  }
  
  // Copy in lights
  for (int i = 0; i < 5; ++i)
  {
    byte moduleLights = 0;
    moduleLights += ((byte)lights[i * 5 + 0]) << 0;
    moduleLights += ((byte)lights[i * 5 + 1]) << 1;
    moduleLights += ((byte)lights[i * 5 + 2]) << 2;
    moduleLights += ((byte)lights[i * 5 + 3]) << 3;
    moduleLights += ((byte)lights[i * 5 + 4]) << 4;
    settings[60 + i] = moduleLights;
  }
  
  // Misc 
  settings[70] = 0;
  settings[70] += controller.killSwitchPressed & B00000001;
  settings[72] = motorSpeed;
  settings[73] = horzKp;
  settings[74] = horzKi;
  settings[75] = horzKd;
  settings[76] = vertKp;
  settings[77] = vertKi;
  settings[78] = vertKd;
  settings[79] = horizontalSlewRate;
  settings[80] = verticalSlewRate;
  
  settings[124] = calculateChecksum(settings, 124); 
  
  // Send settings
  TAIL_SERIAL.write('s');
  TAIL_SERIAL.write(settings, 125);
  
  // Wait for ack
  waitForModuleAcknowledgments("settings", 40);
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
  updateSetpoints(): Propagates the setpoints when it it time to do so.
 *************************************************************************************/
void updateSetpoints()
{
  static unsigned long lastUpdateTime = 0;

  // Disable vertical actuation. If (1) the straighten vertical switch is
  // pressed or (2) vertical straighten on the fly is enabled, set all
  // verticals to straight.
  boolean allowVerticalControl = true;
  if (controller.killSwitchPressed &&
      (controller.straightenVertOnTheFly || controller.straightenVertical))
  {
    allowVerticalControl = false;
    for (int i = 0; i < 30; ++i)
    {
      vertSetpoints[i] = vertStraightAngle;
    }
  }

  // Propagate setpoints if it's time to do so.
  int timeSinceLastUpdate = millis() - lastUpdateTime;
  if (controller.killSwitchPressed &&
      timeSinceLastUpdate > propagationDelay)
  {
    actualPropagationDelay = timeSinceLastUpdate;
    
    // Propagation of all angles
    for (int i = 29; i > 0; --i)
    {
      horzSetpoints[i] = horzSetpoints[i - 1];
      vertSetpoints[i] = vertSetpoints[i - 1];     
    }
    
    // The first actuator gets the new setpoints
    if (controller.right)
    {
      horzSetpoints[0] = horzRightAngle;
    }
    else if (controller.left)
    {
      horzSetpoints[0] = horzLeftAngle;
    }
    else
    {
      horzSetpoints[0] = horzStraightAngle;
    }
      
    
    // Allow new vertical setpoints if we're not doing auto straightening
    if (controller.up && allowVerticalControl)
    {
      vertSetpoints[0] = vertUpAngle;
    }
    else if (controller.down && allowVerticalControl)
    {
      vertSetpoints[0] = vertDownAngle;
    }
    else if ( allowVerticalControl )
    {
      vertSetpoints[0] = vertStraightAngle;
    }
      
    
    lastUpdateTime = millis();  
  }
}

/**************************************************************************************
  updateLights(): Controls all the lights of the snake.
 *************************************************************************************/
 
void updateLights()
{
  static unsigned long lastUpdateTime = 0;
  
  // Propagate random pulses down the snake when we're slithering..
  // .. like nervous signals though a nervous system.
  if (controller.killSwitchPressed && 
      (millis() - lastUpdateTime > ledDelayWhenRunning))
  {
    for (int i = 29; i > 0; --i)
    {
      lights[i] = lights[i - 1];
    }
    lights[0] = (boolean)random(0, 2);
    lastUpdateTime = millis();
  }
  
  // Heart beats while we're stationary
  if (!controller.killSwitchPressed)
  {
    static boolean firstBeatSent = false;    

    // Propegate all lights
    for (int i = 29; i > 0; --i)
    {
      lights[i] = lights[i - 1];
    }
    
    // Default to propagating off light
    lights[0] = false;
    
    // Propagate heat beats when its time
    if (!firstBeatSent && (millis() - lastUpdateTime > longDelayBetweenHeartBeats))
    {
      lights[0] = true;
      firstBeatSent = true;
      lastUpdateTime = millis();
    }
    else if (firstBeatSent && (millis() - lastUpdateTime > shortDelayBetweenHeartBeats))
    {
      lights[0] = true;
      firstBeatSent = false;
      lastUpdateTime = millis();
    }
  }
}

/**************************************************************************************
  readAndRequestJoystickData(): Asks the joystick for data and reads its back. If the joystick hasn't
                     responded for 300ms it retries. After 1 retry we are disconnected.
                     It takes between 39 to 210ms for the joystick to responsed (avg: 76ms)
 *************************************************************************************/
void readAndRequestJoystickData()
{
  static unsigned long lastJoystickRequestTime = 0;
  static byte joystickRetryAttempts = 0;  
  
  // Check if there's a full joystick packet available
  if (INPUT_SERIAL.available() < 30)
  {
    // There's no joystick data. If it's been 300ms since 
    // the last request. Retry: Ask the joystick for data again.
    if (millis() - lastJoystickRequestTime > 300)
    { 
      // If we've retried already, now we know the joystick is disconnected!
      if (joystickRetryAttempts > 0)
      {
        DEBUG_STREAM << "ERROR: No joystick data. It is Disconnected.\n";
        joystickIsConnected = false;
        controller.killSwitchPressed = false;
      }
      ++joystickRetryAttempts;
      clearSerialBuffer(INPUT_SERIAL);
      INPUT_SERIAL.write('j');
      lastJoystickRequestTime = millis();       
    }
    return;
  }
 
  // Success! We have new data, so we are connected to the joystick.
  joystickIsConnected = true; 
  joystickRetryAttempts = 0;
  
  // Sort our new data.
  char packet[30];
  INPUT_SERIAL.readBytes(packet, 30);
  
  controller.killSwitchPressed = (boolean)packet[0];
  controller.left = (boolean)packet[1];
  controller.right = (boolean)packet[2];
  controller.up = (boolean)packet[3];
  controller.down = (boolean)packet[4];
  controller.openJaw = (boolean)packet[5];
  controller.closeJaw = (boolean)packet[6];
  controller.straightenVertical = (boolean)packet[7];
  controller.calibrate = (boolean)packet[8];
  controller.straightenVertOnTheFly = (boolean)packet[9];
  
  motorSpeed = packet[12];
  propagationDelay = word(packet[13], packet[14]);
  vertKi = packet[15];
  vertKp = packet[16];
  horizontalSlewRate = packet[17];
  verticalSlewRate = packet[18];
  
  // Request another joystick packet  
  lastJoystickRequestTime = millis();
  INPUT_SERIAL.write('j');  

  /*USB_COM_PORT.println(controller.killSwitchPressed);
  USB_COM_PORT.println(controller.left);
  USB_COM_PORT.println(controller.right);
  USB_COM_PORT.println(controller.up);
  USB_COM_PORT.println(controller.down);
  USB_COM_PORT.println(controller.openJaw);
  USB_COM_PORT.println(controller.closeJaw);
  USB_COM_PORT.println(controller.straightenVertical);
  USB_COM_PORT.println(controller.calibrate);
  USB_COM_PORT.println(controller.straightenVertOnTheFly);
  USB_COM_PORT.println(controller.motorSpeed);
  USB_COM_PORT.println(controller.propagationDelay);
  USB_COM_PORT.println(controller.ledDelay);
  USB_COM_PORT.println(controller.sendHeartBeatDelay);*/
  
  return;
}

/**************************************************************************************
  synchronizeWithJoystick(): Clears away any previous data from the joystick and makes sure
                             a fresh set is available. It's important to do this if we just
                             executed a command like calibrate where the joystick data wasn't
                             updating.
 *************************************************************************************/
void synchronizeWithJoystick()
{
  USB_COM_PORT.print("Getting back in sync with joystick. Waiting for data... ");
  do
  {
    clearSerialBuffer(INPUT_SERIAL);
    INPUT_SERIAL.write('j');
    delay(500);
  } 
  while (INPUT_SERIAL.available() != 30);
  USB_COM_PORT.println("Synchronized!");
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

/**************************************************************************************
  openTheJaw(): Opens the jaw.
 *************************************************************************************/
void openTheJaw()
{
  USB_COM_PORT << "Opening this jaw... ";
  
  // Tell the first module to turn on the motor for a bit
  TAIL_SERIAL.write("h");
  
  // Slowly open and close to jaw opening valve
  digitalWrite(JAW_CTRL, JAW_OPEN_CTRL_SELECT);  
  for(int i=100; i<=255; i++)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(10);
  }
  for(int i=255; i>=50; i--)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(6);
  }
  analogWrite(JAW_ACTUATOR, 0);
  
  USB_COM_PORT << "Done\n";   
}

/**************************************************************************************
  closeTheJaw(): Closes the jaw.
 *************************************************************************************/
void closeTheJaw()
{
  USB_COM_PORT << "Closing the jaw... ";
  
  // Tell the first module to turn on the motor for a bit
  TAIL_SERIAL.write("h");
  
  // Slowly open and close to jaw closing valve
  digitalWrite(JAW_CTRL, JAW_CLOSE_CTRL_SELECT);
  for(int i=0; i<=255; i++)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(4);
  }
  for(int i=255; i>=0; i--)
  {
    analogWrite(JAW_ACTUATOR, i);
    delay(4);
  }
  analogWrite(JAW_ACTUATOR, 0);
  
  USB_COM_PORT << "Done\n";  
}

/**************************************************************************************
  manualSetModuleCount(): Manually set the number of modules we are currently using.
 *************************************************************************************/
void manualSetModuleCount()
{
  USB_COM_PORT << "Manually program number of modules (1-6)...\n";
  while(USB_COM_PORT.available() < 1);
  byte numberOfModulesNew = USB_COM_PORT.read() - 48;
  if (numberOfModulesNew < 1 || numberOfModulesNew > 6)
  {
    USB_COM_PORT << "Invalid\n";
    return;
  }
  USB_COM_PORT << "Now set to " << numberOfModulesNew << "\n";
  EEPROM.write(0, numberOfModulesNew);
  numberOfModules = numberOfModulesNew;
}

/**************************************************************************************
  manualToggleDebugMessages(): To avoid slow downs, runtime debug messages are turned off
                               by default. This function allow them to be turned on.
 *************************************************************************************/
void manualToggleDebugMessages()
{
  if (inDebugMode)
  {
    USB_COM_PORT.println("Debug stream messages OFF");
    inDebugMode = false;
  }
  else
  {
    USB_COM_PORT.println("Debug stream messages ON");
    inDebugMode = true;
  }
}

/**************************************************************************************
  printSetpointAngleArrays(): Prints out the vertical and horizontal angle arrays for the whole snake.
 *************************************************************************************/
void printSetpointAngleArrays()
{
  USB_COM_PORT << "\nPrinting Angle Arrays \n";
  USB_COM_PORT << "V#\tHorz\tVert\n";
  
  for (int i = 0; i < 30; ++i)
  {
    USB_COM_PORT << i << "\t" << horzSetpoints[i] << "\t" << vertSetpoints[i] << "\n";
    if (i == (numberOfModules * 5) - 1)
    {
      USB_COM_PORT << "--- END OF SNAKE (" << numberOfModules << " modules)\n";
    }
  }
  USB_COM_PORT << "\n";
}

/**************************************************************************************
  manualExtendActuator(): Extends the actuator selected in manual mode
 *************************************************************************************/
void manualExtendActuator()
{
  StopMovement();
  
  // Instruct downsteam module to turn on motor, will run for 400ms
  TAIL_SERIAL.write('g');
  
  if (manualActuatorSelect == 0)
  {
    digitalWrite(JAW_CTRL, JAW_OPEN_CTRL_SELECT);
    analogWrite(JAW_ACTUATOR, 255);
    USB_COM_PORT << "Jaw Actuator Extended (" << manualActuationDelay << "ms)\n";  
  }
  else if (manualActuatorSelect == 1)
  {
    digitalWrite(HEAD_CTRL, HEAD_RAISE_CTRL_SELECT);
    analogWrite(HEAD_ACTUATOR, 255);
    USB_COM_PORT << "Head Actuator Extended (" << manualActuationDelay << "ms)\n";  
  }
  else if(manualActuatorSelect == 2)
  {
    digitalWrite(AUX_CTRL, AUX_EXTEND_CTRL_SELECT);
    analogWrite(AUX_ACTUATOR, 255);
    USB_COM_PORT << "Auxiliary Actuator Extended (" << manualActuationDelay << "ms)\n";  
  }
  delay(manualActuationDelay);
  StopMovement();
}

/**************************************************************************************
  manualContractActuator(): Contracts the actuator selected in manual mode
 *************************************************************************************/
void manualContractActuator()
{
  StopMovement();
  
  // Instruct downsteam module to turn on motor, will run for 400ms
  TAIL_SERIAL.write('g');
  
  if (manualActuatorSelect == 0)
  {
    digitalWrite(JAW_CTRL, JAW_CLOSE_CTRL_SELECT);
    analogWrite(JAW_ACTUATOR, 255);
    USB_COM_PORT << "Jaw Actuator Contracted (" << manualActuationDelay << "ms)\n";  
  }
  else if (manualActuatorSelect == 1)
  {
    digitalWrite(HEAD_CTRL, HEAD_LOWER_CTRL_SELECT);
    analogWrite(HEAD_ACTUATOR, 255);
    USB_COM_PORT << "Head Actuator Contracted (" << manualActuationDelay << "ms)\n";  
  }
  else if(manualActuatorSelect == 2)
  {
    digitalWrite(AUX_CTRL, AUX_RETRACT_CTRL_SELECT);
    analogWrite(AUX_ACTUATOR, 255);
    USB_COM_PORT << "Auxiliary Actuator Contracted (" << manualActuationDelay << "ms)\n";  
  }
  delay(manualActuationDelay);
  StopMovement();
}

/**************************************************************************************
  setManualActuationDelay(): sets the actuation delay for manual mode actuation
 *************************************************************************************/
void setManualActuationDelay()
{
  StopMovement();
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
  printHeadSensorValuesAndSetpoints(): Prints the values of the head's sensors.
                                       Will print setpoints when we get sensors mounted.
 *************************************************************************************/
void printHeadSensorValuesAndSetpoints()
{
  for(int k =0;k<6;k++)
  {
    USB_COM_PORT.print("\r\nSensor ");
    USB_COM_PORT.print(k+1,DEC);
    USB_COM_PORT.print(" reads ");
    USB_COM_PORT.print(analogRead(HEAD_POS_SENSOR[k]),DEC);
    USB_COM_PORT.print("\r\n");
  }
  USB_COM_PORT.print("\r\n");
}

/**************************************************************************************
  manualSetLEDState(): Waits for LED number over usb then sets it to On or Off
 *************************************************************************************/
void manualSetLEDState(boolean state)
{
  while(USB_COM_PORT.available() < 1)
  {
    delay(1);
  }
  byte led_choice = USB_COM_PORT.read()  - 48;
  if(led_choice == -1) return; //read() returns -1 if no data is available
  switch(led_choice)
  {
    case 1: digitalWrite(LED_1, state); break; 
    case 2: digitalWrite(LED_2, state); break;
    case 3: digitalWrite(LED_3, state); break;
    case 4: digitalWrite(LED_4, state); break;
    case 5: digitalWrite(LED_5, state); break;
    case 6: digitalWrite(LED_6, state); break;
    default: USB_COM_PORT << "Invalid LED Number.\n\r"; return;
  }
  USB_COM_PORT << "LED " << led_choice << " set to " << (state ? "ON" : "OFF") << "\r\n";
}

/**************************************************************************************
  runSerialChainCommunicationTest(): Makes sure all modules are responding and the last module
                                     is terminated. Returns true if it passed the test.
 *************************************************************************************/
boolean runSerialChainCommunicationTest()
{  
  clearSerialBuffer(TAIL_SERIAL);
  int numberOfErrors = 0;
  
  for (int i = 1; i <= 20; ++i)
  {
    USB_COM_PORT << "Running serial chain test " << i << " of 20.\n";
    TAIL_SERIAL.write('t');
    delay(100);
    
    // We should recieve modules number (1 2 3 4) followed by ASCII 't' aka ASCII 116
    long startTime = millis();
    int num = 1;
    boolean gotLastModuleT = false;
    boolean error = false;
    
    while(millis() - startTime < 150)
    {
      // Wait for data
      if (!TAIL_SERIAL.available())
      {
        continue;
      }
      byte data = TAIL_SERIAL.read();
      
      // If we've heard from every module look for ASCII charater 't'
      if (num > numberOfModules)
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
    if (!error && num <= numberOfModules)
    {
      USB_COM_PORT << "ERROR: Did not recieve data from all " << numberOfModules << " modules.\n";
      error = true;     
    }
    // If we didn't recieve 't' from the last module.
    if (!error && !gotLastModuleT)
    {
      USB_COM_PORT << "ERROR: Did not recieve 't' from the last module. Is the serial terminator plugged in?\n";
      error = true;
    }
    USB_COM_PORT << "\n";
    
    if(error) ++numberOfErrors;
  }
  
  // Give the result of the tests
  if (numberOfErrors > 0)
  {
    USB_COM_PORT << "RESULT: ERROR! Some serial chain communication tests failed.\n\n";
  }
  else
  {
    USB_COM_PORT << "RESULT: SUCCESS! All serial chain communication tests passed.\n\n";
  }
  delay(500);
  clearSerialBuffer(TAIL_SERIAL);
}

/**************************************************************************************
  runRS485CommunicationTest(): Makes sure all modules are responding on the RS485 bus
 *************************************************************************************/
boolean runRS485CommunicationTest()
{  
  clearSerialBuffer(RS485_SERIAL);
  int numberOfErrors = 0;
  
  for (int i = 1; i <= 20; ++i)
  {
    USB_COM_PORT << "Running RS-485 test " << i << " of 20.\n";
    digitalWrite(RS485_TX_ENABLE, HIGH);
    RS485_SERIAL.write('t');
    RS485_SERIAL.flush();
    digitalWrite(RS485_TX_ENABLE, LOW);
    clearSerialBuffer(RS485_SERIAL, 1); 
    
    
    // We should recieve modules number (1 2 3 4) followed by ASCII 't' aka ASCII 116
    long startTime = millis();
    int num = 1;
    boolean error = false;
    
    while(millis() - startTime < 150)
    {
      // Wait for data
      if (!RS485_SERIAL.available())
      {
        continue;
      }
      byte data = RS485_SERIAL.read();
      
      // If we've heard from every module we're done.
      if (num > numberOfModules)
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
    if (!error && num <= numberOfModules)
    {
      USB_COM_PORT << "ERROR: Did not recieve data from all " << numberOfModules << " modules.\n";
      error = true;     
    }
    USB_COM_PORT << "\n";
    
    if(error) ++numberOfErrors;
  }
  
  // Give the result of the tests
  if (numberOfErrors > 0)
  {
    USB_COM_PORT << "RESULT: ERROR! Some RS-485 communication tests failed.\n\n";
  }
  else
  {
    USB_COM_PORT << "RESULT: SUCCESS! All RS-485 communication tests passed.\n\n";
  }
  delay(500);
  clearSerialBuffer(RS485_SERIAL);
}

/**************************************************************************************
  displayMenu(): Shows the command menu for manual control over usb serial
 *************************************************************************************/
void displayMenu()
{
    USB_COM_PORT.print("\nManual Control Mode\n");
    USB_COM_PORT.print("commands: j or h to select jaw or head actuator.\n");
    USB_COM_PORT.print("          u to select Aux actuator\n");
    USB_COM_PORT.print("\n");
    USB_COM_PORT.print("          k/l - extend/contract actuator\n");
    USB_COM_PORT.print("          d* - adjust actuation delay where *=s(small), m(medium), l(large)\n");
    USB_COM_PORT.print("          a* - turn LED on where *=led number(1-5)\n");
    USB_COM_PORT.print("          b* - turn LED off where *=led number(1-5)\n");
    USB_COM_PORT.print("          c - print head sensor values \n");
    USB_COM_PORT.print("          p - print angle array\n");
    USB_COM_PORT.print("          n - manually set numberOfModules\n");
    USB_COM_PORT.print("          z - toggle printing of debug stream\n");   
    USB_COM_PORT.print("          t - test communication on serial chain\n");     
    USB_COM_PORT.print("          T - test communication on RS-485 bus\n");     
    USB_COM_PORT.print("          y - calibrate horizontal\n");     
    USB_COM_PORT.print("          i - calibrate vertical\n");
    USB_COM_PORT.print("          s - stop motor\n");     
    USB_COM_PORT.print("          e - menu\n");
    USB_COM_PORT.print("          q - quit\n\n");
}

/**************************************************************************************
  manualControl(): Allows for manual actuator control over the USB_COM_PORT
 *************************************************************************************/
void manualControl()
{
  displayMenu();

  boolean manual = true;  
  while(manual == true)
  {
    if(Serial.available() > 0)
    {
      byte manualCommand = USB_COM_PORT.read();
      
      switch(manualCommand)
      {       
        case 'j':
          manualActuatorSelect = 0;
          USB_COM_PORT.print("Jaw actuator selected\n");
          break;
        case 'h':
          manualActuatorSelect = 1;
          USB_COM_PORT.print("Head actuator selected\n");
          break;
        case 'u':
          manualActuatorSelect = 2;
          USB_COM_PORT.print("Auxiliary actuator selected\n");
          break;
        case 'y':
          USB_COM_PORT << "\nCalibrating horizontals... ";
          runHorzSensorCalibration();
          USB_COM_PORT << "Done\n";
          break;          
        case 'i':
          USB_COM_PORT << "\nCalibrating verticals... ";
          runVertSensorCalibration();
          USB_COM_PORT << "Done\n";
          break;
          
        case 'a':
          manualSetLEDState(HIGH);
          break;
        case 'b':
          manualSetLEDState(LOW);
          break;
        case 'c':
          printHeadSensorValuesAndSetpoints();
          break;
        case 'd':
          setManualActuationDelay();
          break;         
        case 'l':
          manualContractActuator();
          break;         
        case 'k':
          manualExtendActuator();
          break;       
        case 'n':
          manualSetModuleCount();
          break;     
        case 'z':
          manualToggleDebugMessages();
          break;
        case 'p':
          printSetpointAngleArrays();
          break;
        case 't':
          runSerialChainCommunicationTest();
          break;
        case 'T':
          runRS485CommunicationTest();
          break;
        case 'e':
          displayMenu();
          break;
        case 's':
          StopMovement();
          USB_COM_PORT.print("Stopped all movement\n");
          break;     
        case 'q':
          manual = false;
          break;
      }//end switch
    }//end if serial
  }
  USB_COM_PORT.print("\nManual Control mode exited\n");  
  
  // Clear serial buffers to try and get back in sync
  clearSerialBuffer(INPUT_SERIAL);
  clearSerialBuffer(TAIL_SERIAL);    
}

/**************************************************************************************
  StopMovement(): In manual mode, stops movement of all actuators.
 *************************************************************************************/
void StopMovement()
{
  analogWrite(JAW_ACTUATOR, 0);
  analogWrite(HEAD_ACTUATOR, 0);
  analogWrite(AUX_ACTUATOR, 0);
  return;
}

