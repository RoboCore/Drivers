#ifndef RC_DRIVERS_H
#define RC_DRIVERS_H


/*
	RoboCore Drivers Library
		(v1.0 - 25/04/2013)

  Driver classes for Arduino
    (tested with Arduino 1.0.1)

  Released under the Beerware license
  Written by François
  
  Supports:
    - Pololu A4988 driver
  
  NOTES:
    # the Pololu A4988 driver can be
      automatically disabled/put to
      sleep when motor is not running
      and enabled/waken up when driving
      (setDriveMode() method)
  
*/


#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h> //for Arduino 1.0 or later
#else
#include <WProgram.h> //for Arduino 22
#endif

//-------------------------------------------------------------------------------------------------

#define A4988_DIR_CW 0
#define A4988_DIR_CCW 1

#define A4988_FULL_STEP 0      // 0b000
#define A4988_HALF_STEP 4      // 0b100
#define A4988_QUARTER_STEP 2   // 0b010
#define A4988_EIGHTH_STEP 6    // 0b110
#define A4988_SIXTEENTH_STEP 7 // 0b111

#define A4988_DRIVE_MODE_NONE 0
#define A4988_DRIVE_MODE_SLEEP 1
#define A4988_DRIVE_MODE_ENABLE 2


#define A4988_DEFAULT_SPEED_RPM 100
#define A4988_DEFAULT_STEPS 200
#define A4988_DEFAULT_DRIVE_MODE A4988_DRIVE_MODE_NONE


// The A4988 Stepper Motor Driver (Pololu)
class A4988{
  public:
    A4988(void);
    A4988(int pinDIR, int pinSTEP);
    A4988(int pinDIR, int pinSTEP, int pinMS1, int pinMS2, int pinMS3);
    
    void Disable(void); // ******************* ok12
    void Drive(long steps); //*********************************** USE DRIVE MODES !!!
    void Drive(long steps, unsigned int frequency); //*********************************** USE DRIVE MODES !!!
    void DriveRevolutions(int revolutions);
    void Enable(void); // ******************* ok12
    int getDir(void);
    int getDriveMode(void); // ******************* ok12
    int getMode(void);
    int getSpeed(void);
    unsigned int getSteps(void);
    boolean Initialize(int pinDIR, int pinSTEP);
    boolean Initialize(int pinDIR, int pinSTEP, int pinMS1, int pinMS2, int pinMS3);
    void InvertDirection(void);
    boolean isEnabled(void); // ******************* ok12
    boolean isSleeping(void); // ******************* ok12
    boolean ModePinsSet(void);
    void Reset(void);  // ******************* ok12
    void setDir(int dir);
    void setDriveMode(byte mode); // ******************* ok12
    void setENABLEpin(int pinENABLE); // ******************* ok12
    int setMode(byte mode);
    int setMSpins(int pinMS1, int pinMS2, int pinMS3);
    void setRSTpin(int pinRST); // ******************* ok12
    void setSLEEPpin(int pinSLEEP); // ******************* ok12
    void setSpeedRPM(int rpm);
    void setSteps(unsigned int steps);
    void Sleep(void); // ******************* ok12
    void WakeUp(void); // ******************* ok12
  
  private:
    boolean _initialized; // TRUE if initialized
    byte _pinDIR;
    byte _pinSTEP;
    byte _pinsMODE[3]; // mode pins = { MS1, MS2, MS3 }
    byte _pinENABLE; // *******************
    byte _pinRST; // *******************
    byte _pinSLEEP; // *******************
    byte _dir; // 0 or 1
    unsigned int _speed; // in [rpm]
    unsigned int _steps; // number of steps in one revolution
    byte _mode; // default is DRIVER_FULL_STEP
    boolean _mode_pins_set; // TRUE if set
    boolean _reset_pin_set; // TRUE if set *******************
    byte _use_ENABLE; // bit 0 = pin set || bit 1 = use ENABLE in Drive || bit 2 = isEnabled (1 when enabled) *******************
    byte _use_SLEEP; // bit 0 = pin set || bit 1 = use SLEEP in Drive (has 1 ms delay after waking up) || bit 2 = isSleeping (1 when sleeping) *******************
    
    byte getFactor(void);

};

#endif // RC_DRIVERS_H







