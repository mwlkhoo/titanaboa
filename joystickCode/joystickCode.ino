/*

 joystickCode.ino - Sends wireless commands to Titanoboa
 
 Created: August 2011 
 Part of the titanaboa.ca project
 
 Description: Runs on the arduino MEGA in the joystick. When requested
 by the head, this code sends the status of all the switches and knobs
 to the head. 
 
 The joystick has:
 - A power switch
 - 4 way joystick with a momentary pushbutton on top.
 - 2 momentary switches.
 - 1 toggle switch.
 - 5 knobs.
 
 */

#define XBEE_SERIAL Serial3
#define USB_COM_PORT Serial

// Enable serial stream writing
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

// Joystick pins
#define JOYSTICK_LEFT_PIN   41
#define JOYSTICK_RIGHT_PIN  45
#define JOYSTICK_UP_PIN     43
#define JOYSTICK_DOWN_PIN   39
#define JOYSTICK_BUTTON     37

#define CALIBRATE_BUTTON 28
#define JAW_OPEN  47
#define JAW_CLOSE 49
#define VERTICAL_STRAIGHTEN 35

// Knobs
#define THROTTLE_PIN     1
#define MOTOR_SPEED_PIN  3
#define RSVD_0_PIN       0
#define RSVD_2_PIN       2
#define RSVD_4_PIN       4

/*************************************************************************
 * setup(): Initializes serial ports, pins
 **************************************************************************/
void setup()
{
  XBEE_SERIAL.begin(115200);
  USB_COM_PORT.begin(115200);

  // Internal pull up on the calibrate button
  digitalWrite(CALIBRATE_BUTTON, HIGH);

  USB_COM_PORT.println("Hi I'm the Joystick, ready to rule the world!\n");
  
}//end setup()


/*************************************************************************
 * loop(): Listens for commands from USB Serial or the Head board over XBee
 **************************************************************************/
void loop()
{
  // Check USB serial for command to enter manaul mode.
  if (USB_COM_PORT.available() > 0)
  {
    if (USB_COM_PORT.read() == 'M')
    {
      manualControl();
    }   
  }
  
  // Check XBEE serial for commands from the head.
  if (XBEE_SERIAL.available() > 0)
  {
    char command = XBEE_SERIAL.read();
    switch(command)
    {
      case 'j':
        processSwitchAndKnobRequest();
        break;
        
      default:
        USB_COM_PORT.print("invalid command recieved = ");
        USB_COM_PORT.println(command);
        clearSerialBuffer(XBEE_SERIAL);        
    }
  }    
}//end loop()

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

/*************************************************************************
 * processSwitchAndKnobRequest(): Sends the status of all switches and knobs
                                  to the head over XBEE.
 **************************************************************************/
void processSwitchAndKnobRequest()
{
  USB_COM_PORT.print("Sending joystick data... ");
  
  // Read switches
  boolean joystickButtonPressed = digitalRead(JOYSTICK_BUTTON) == HIGH;
  boolean joystickLeftPressed = digitalRead(JOYSTICK_LEFT_PIN) == HIGH;
  boolean joystickRightPressed = digitalRead(JOYSTICK_RIGHT_PIN) == HIGH;
  boolean joystickUpPressed =  digitalRead(JOYSTICK_UP_PIN) == HIGH;
  boolean joystickDownPressed =  digitalRead(JOYSTICK_DOWN_PIN) == HIGH;
  boolean jawOpenSwitchPressed = digitalRead(JAW_OPEN) == HIGH;
  boolean jawCloseSwitchPressed = digitalRead(JAW_CLOSE) == HIGH;
  boolean vertStraightenSwitchPressed = digitalRead(VERTICAL_STRAIGHTEN) == HIGH;
  boolean calibrateButtonPressed = digitalRead(CALIBRATE_BUTTON) == LOW;
  
  // Read knobs
  unsigned short motorSpeedKnob = analogRead(MOTOR_SPEED_PIN);
  byte motorSpeed = map(motorSpeedKnob, 0, 1023, 0, 255);
  
  unsigned short propagationDelayKnob = analogRead(THROTTLE_PIN);
  unsigned short propagationDelay = map(propagationDelayKnob, 0, 1023, 1000, 250);
  
  unsigned short spareKnob0 = analogRead(RSVD_0_PIN);
  boolean straightenVertOnTheFly = false;//(spareKnob0 > 512);
  byte horzSlewRate = 3;
  byte vertSlewRate = map(spareKnob0, 0, 1023, 0, 6);
  if (vertSlewRate == 6) vertSlewRate = 5;
  
  unsigned short unlabelledRightKnob = analogRead(RSVD_2_PIN);
  byte Ki = map(unlabelledRightKnob, 0, 1023, 0, 150);
  
  unsigned short unlabelledLeftKnob = analogRead(RSVD_4_PIN);
  byte Kp = map(unlabelledLeftKnob, 0, 1023, 0, 150); 
  
  // Transmit
  byte packet[30];  
  packet[0] = (byte)joystickButtonPressed;
  packet[1] = (byte)joystickLeftPressed;
  packet[2] = (byte)joystickRightPressed;
  packet[3] = (byte)joystickUpPressed;
  packet[4] = (byte)joystickDownPressed;
  packet[5] = (byte)jawOpenSwitchPressed;
  packet[6] = (byte)jawCloseSwitchPressed;
  packet[7] = (byte)vertStraightenSwitchPressed;
  packet[8] = (byte)calibrateButtonPressed;
  packet[9] = (byte)straightenVertOnTheFly;
  packet[10] = 0; // Spare space for booleans
  packet[11] = 0; // Spare space for booleans
  packet[12] = motorSpeed;
  packet[13] = highByte(propagationDelay);
  packet[14] = lowByte(propagationDelay);
  packet[15] = Ki;
  packet[16] = Kp;
  packet[17] = horzSlewRate;
  packet[18] = vertSlewRate;
  packet[19] = 0; // Lots more spares 19 to 29
  
  XBEE_SERIAL.write(packet, 30);
  USB_COM_PORT.println("Done!");
}

/**************************************************************************************
  manualControl(): Allows for manual control over the USB_SERIAL_PORT
 *************************************************************************************/
void manualControl()
{
    boolean manual = true;
    char byteIn = 'z';

    displayMenu();

    while(manual == true)
    {
        if(USB_COM_PORT.available() > 0)
        {
            byteIn = USB_COM_PORT.read();

            switch(byteIn)
            {
            case 'p':
                readAllButtons();
                break;

            case 'e':
                displayMenu();
                break;

            case 'q':
                manual = false;
                XBEE_SERIAL.flush();
                break;

            default:
                USB_COM_PORT.println("doesn't do anything\n");
                break;
            }//end switch
        }//end if USB_COM_PORT

        byteIn = 'z';
    }
    USB_COM_PORT.println("\nManual Control mode exited");
}// end manualcontrol


/**************************************************************************************
  displayMenu():
 *************************************************************************************/
void displayMenu()
{
    USB_COM_PORT.print("\nManual control mode menu\n");
    USB_COM_PORT.print("commands: p - print raw values of all knobs, buttons, etc\n");
    USB_COM_PORT.print("          e - menu\n");
    USB_COM_PORT.print("          q - quit\n");
}


/**************************************************************************************
  readAllButtons():
 *************************************************************************************/
void readAllButtons()
{
    int buttonVal = 0;

    // Get the current raw value of the throttle
    buttonVal = analogRead(THROTTLE_PIN);
    USB_COM_PORT.print("Prop delay: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of the motor speed
    buttonVal = analogRead(MOTOR_SPEED_PIN);
    USB_COM_PORT.print("Motor speed: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of reserved 1 knob
    buttonVal = analogRead(RSVD_0_PIN);
    USB_COM_PORT.print("AD 0: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of reserved 2 knob
    buttonVal = analogRead(RSVD_2_PIN);
    USB_COM_PORT.print("AD 2: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of reserved 3 knob
    buttonVal = analogRead(RSVD_4_PIN);
    USB_COM_PORT.print("Ad 4: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of jaw switch
    buttonVal = digitalRead(JAW_OPEN);
    USB_COM_PORT.print("Jaw open: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JAW_CLOSE);
    USB_COM_PORT.print("Jaw close: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of the vertical straighten switch
    buttonVal = digitalRead(VERTICAL_STRAIGHTEN);
    USB_COM_PORT.print("Vertical straighten: ");
    USB_COM_PORT.println(buttonVal);

    // Get the current raw value of the calibrate button
    buttonVal = digitalRead(CALIBRATE_BUTTON);
    USB_COM_PORT.print("Calibrate: ");
    USB_COM_PORT.println(buttonVal);
    USB_COM_PORT.println();

    // Get the current raw value of the joystick
    buttonVal = digitalRead(JOYSTICK_LEFT_PIN);
    USB_COM_PORT.print("Joystick left: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_RIGHT_PIN);
    USB_COM_PORT.print("Joystick right: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_UP_PIN);
    USB_COM_PORT.print("Joystick up: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_DOWN_PIN);
    USB_COM_PORT.print("Joystick down: ");
    USB_COM_PORT.println(buttonVal);
    buttonVal = digitalRead(JOYSTICK_BUTTON);
    USB_COM_PORT.print("Joystick button: ");
    USB_COM_PORT.println(buttonVal);
    USB_COM_PORT.println();
}


