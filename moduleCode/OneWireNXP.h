/*

  OneWireNXP.h - Communication with NXP One Wire devices 
  
  Created: April 30, 2013 
  Part of the titanaboa.ca project
  
  Decription: This code was built to communicate with an NXP
  KMA210 programmable angle sensor but it might work with
  other devices from NXP with a One-Wire Interface. Please note 
  that the One-Wire Interface spec from NXP is incompatible with 
  the popular 1-Wire spec from Maxim.

*/

#ifndef _ONE_WIRE_NXP_H_
#define _ONE_WIRE_NXP_H_

// Timing defines
#define COMMAND_TIME         20000                 // Tcmd(ent)
#define TURN_ON_TIME         5000                  // Ton
#define START_TIME           5                     // Tstart
#define STOP_TIME            5                     // Tstop
#define BIT_PERIOD           55                    // Tbit
#define BIT0_PULSE_WIDTH     0.25 * BIT_PERIOD     // Tw0
#define BIT1_PULSE_WIDTH     0.75 * BIT_PERIOD     // Tw1 
#define COM_RESET_TIME       220                   // Tto
#define SLAVE_TAKEOVER_TIME  3                     // Ttko(slv)
#define MASTER_TAKEOVER_TIME 0.25 * BIT_PERIOD;    // Ttko(mas)
#define PROGRAMMING_TIME     20000;                // Tprog
#define CHARGE_PUMP_TIME     1000;                 // Ttko

// Commands
#define ENTER_COMMAND_MODE_COMMAND 0x94
#define ENTER_COMMAND_MODE_BYTE1 0x16
#define ENTER_COMMAND_MODE_BYTE2 0xF4

// Registers for direct access to AVR inputs and ouputs
#if defined(__AVR__)
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM asm("r30")
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

#else
#error "Only AVR registers are supported."
#endif

/****************************************************************************
* OneWireNXP: Instatiating this class to get a One Wire Interface
**************************************************************************/
class OneWireNXP
{ 
  private: volatile IO_REG_TYPE *m_register;
  private: IO_REG_TYPE m_bitMask;
  private: int m_pin; 


  /****************************************************************************
  * OneWireNXP(): Constructor. Pass the pin connected to the NXP device
  **************************************************************************/
  public: OneWireNXP(byte pin)
  {
    m_pin = pin;
    m_register = PIN_TO_BASEREG(pin);
    m_bitMask = PIN_TO_BITMASK(pin);
  }

  /****************************************************************************
  * writeBytes(): writes an array of bytes to the NXP device
  **************************************************************************/
  private: void write_bytes(byte *buf, byte count)
  {
    noInterrupts();
    for (byte i = 0; i < count; ++i)
    {
      write_byte(buf[i]);      
    }    
    interrupts();    
  }
  
  /****************************************************************************
  * writeByte(): writes a single byte to the NXP device
  **************************************************************************/
  private: void write_byte(byte v)
  {
    for (byte bitMask = 0x01; bitMask != 0; bitMask <<= 1)
    {
      write_bit((bitMask & v) > 0);
    }  
  }

  /****************************************************************************
  * writeBit(): writes a single bit to the NXP device
  **************************************************************************/  
  private: void write_bit(boolean v)
  {
    if (v & 1)
    {
      // Write a 1 bit
      DIRECT_WRITE_HIGH(m_register, m_bitMask);
      delayMicroseconds(BIT1_PULSE_WIDTH);
      DIRECT_WRITE_LOW(m_register, m_bitMask);
      delayMicroseconds(BIT_PERIOD - BIT1_PULSE_WIDTH);      
    }
    else
    {
      // Write a 0 bit
      DIRECT_WRITE_HIGH(m_register, m_bitMask);
      delayMicroseconds(BIT0_PULSE_WIDTH);
      DIRECT_WRITE_LOW(m_register, m_bitMask);
      delayMicroseconds(BIT_PERIOD - BIT0_PULSE_WIDTH); 
    }
  }
  
  /****************************************************************************
  * start(): 
  **************************************************************************/  
  private: void signalStart()
  {
    DIRECT_MODE_OUTPUT(m_register, m_bitMask);
    DIRECT_WRITE_LOW(m_register, m_bitMask);
    delayMicroseconds(START_TIME);    
  }
  
  /****************************************************************************
  * stop(): 
  **************************************************************************/  
  private: void signalStop()
  {
    DIRECT_WRITE_LOW(m_register, m_bitMask);
    delayMicroseconds(STOP_TIME);    
  }
  
  /****************************************************************************
  * enterCommandMode(): Returns true if we sucessfully entered command mode
  **************************************************************************/
  public: boolean enterCommandMode()
  {
    signalStart();
    write_byte(ENTER_COMMAND_MODE_COMMAND);
    write_byte(ENTER_COMMAND_MODE_BYTE1);
    write_byte(ENTER_COMMAND_MODE_BYTE2);
    signalStop();
  }
};

#endif _ONE_WIRE_NXP_H_
