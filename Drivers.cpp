

/*
	RoboCore Drivers Library
		(v1.0 - 25/04/2013)

  Driver classes for Arduino
    (tested with Arduino 1.0.1)

  Copyright 2013 RoboCore (François) ( http://www.RoboCore.net )
  
  ------------------------------------------------------------------------------
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ------------------------------------------------------------------------------
  
  Supports:
    - Pololu A4988 driver
  
  NOTES:
    # the Pololu A4988 driver can be
      automatically disabled/put to
      sleep when motor is not running
      and enabled/waken up when driving
      (setDriveMode() method)
  
*/

#include "Drivers.h"



//-------------------------------------------------------------------------------------------------
//----------------------------------- A4988 -------------------------------------------------------
//-------------------------------------------------------------------------------------------------

// Default constructor
A4988::A4988(void){
  _initialized = false;
}

// Constructor
A4988::A4988(int pinDIR, int pinSTEP){
  Initialize(pinDIR, pinSTEP);
}

// Constructor
A4988::A4988(int pinDIR, int pinSTEP, int pinMS1, int pinMS2, int pinMS3){
  Initialize(pinDIR, pinSTEP, pinMS1, pinMS2, pinMS3);
}

//-------------------------------------------------------------------------------------------------

// Disable the driver
void A4988::Disable(void){
  //check if initialized
  if(!_initialized)
    return;
  
  if(_use_ENABLE & 0x01){ //pin set
    _use_ENABLE &= 0xFB; //reset bit 2
    digitalWrite(_pinENABLE, HIGH);
  }
}

//-------------------------------------------------------------------------------------------------

// Drive by step
//  NOTE: if set to QUARTER mode (for example), will drive real 25 steps (100 quarters of step)
//  NOTE: use negative steps to invert the direction
//  NOTE: the method does not return until it has finished driving
void A4988::Drive(long steps){
  //check if initialized
  if(!_initialized)
    return;
  
  //check whether to enable or wake up
  if(_use_ENABLE & 0x02)
    Enable();
  else if(_use_SLEEP & 0x02){
    WakeUp();
    delayMicroseconds(1000); //1 ms delay before returning to normal operation
  }
  
  //check number of steps
  if(steps == 0)
    return;
  else if(steps < 0)
    _dir != _dir;
  
  //use absolute values
  if(steps < 0)
    steps = -1 * steps;
  
  byte factor = getFactor();
  unsigned long period = 60 * 1000000 / ((unsigned long)factor * _steps * _speed);
  if(period < 2)
    period = 2;
  
  //drive
  for(int i=0 ; i < steps ; i++){
    digitalWrite(_pinSTEP, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(_pinSTEP, LOW);
    delayMicroseconds(period / 2);
  }
  
  //check whether to disable or sleep
  if(_use_ENABLE & 0x02)
    Disable();
  else if(_use_SLEEP & 0x02)
    Sleep();
}

//-------------------------------------------------------------------------------------------------

// Drive by step with a given frequency [Hz]
//  NOTE: if set to QUARTER mode (for example), will drive real 25 steps (100 quarters of step)
//  NOTE: use negative steps to invert the direction
//  NOTE: the method does not return until it has finished driving
void A4988::Drive(long steps, unsigned int frequency){
  //check if initialized
  if(!_initialized)
    return;
  
  //check whether to enable or wake up
  if(_use_ENABLE & 0x02)
    Enable();
  else if(_use_SLEEP & 0x02){
    WakeUp();
    delayMicroseconds(1000); //1 ms delay before returning to normal operation
  }
  
  //check number of steps
  if(steps == 0)
    return;
  else if(steps < 0)
    _dir != _dir;
  
  //use absolute values
  if(steps < 0)
    steps = -1 * steps;
  
  byte factor = getFactor();
  unsigned long period = 1000000 / frequency;
  
  //drive
  for(int i=0 ; i < steps ; i++){
    digitalWrite(_pinSTEP, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(_pinSTEP, LOW);
    delayMicroseconds(period / 2);
  }
  
  //check whether to disable or sleep
  if(_use_ENABLE & 0x02)
    Disable();
  else if(_use_SLEEP & 0x02)
    Sleep();
}

//-------------------------------------------------------------------------------------------------

// Drive by revolution
//  NOTE: use negative revolutions to invert the direction
//  NOTE: the method does not return until it has finished driving
void A4988::DriveRevolutions(int revolutions){
  //check if initialized
  if(!_initialized)
    return;
  
  byte factor = getFactor();
  Drive(factor * _steps * revolutions);
}

//-------------------------------------------------------------------------------------------------

// Enable the driver
void A4988::Enable(void){
  //check if initialized
  if(!_initialized)
    return;
  
  if(_use_ENABLE & 0x01){ //pin set
    _use_ENABLE |= 0x04; //set bit 2
    digitalWrite(_pinENABLE, LOW);
  }
}
  
//-------------------------------------------------------------------------------------------------

// Get the current direction
//  (returns -1 if not initialized)
int A4988::getDir(void){
  //check if initialized
  if(!_initialized)
    return -1;
  
  return _dir;
}

  
//-------------------------------------------------------------------------------------------------

// Get the current direction
//  (returns -1 if not initialized)
int A4988::getDriveMode(void){
  //check if initialized
  if(!_initialized)
    return -1;
  
  int mode = A4988_DRIVE_MODE_NONE;
  if(_use_ENABLE & 0x02){
    mode = A4988_DRIVE_MODE_ENABLE;
  } else if(_use_SLEEP & 0x02){
    mode = A4988_DRIVE_MODE_SLEEP;
  }
  
  return mode;
}

//-------------------------------------------------------------------------------------------------

// Get the factor for the mode
//  (returns 0 if not initialized)
byte A4988::getFactor(void){
  //check if initialized
  if(!_initialized)
    return 0;
  
  byte factor = 1; // A4988_FULL_STEP
  switch(_mode){
    case A4988_HALF_STEP:
      factor = 2;
      break;
    
    case A4988_QUARTER_STEP:
      factor = 4;
      break;
    
    case A4988_EIGHTH_STEP:
      factor = 8;
      break;
    
    case A4988_SIXTEENTH_STEP:
      factor = 16;
      break;
  }
  
  return factor;
}

//-------------------------------------------------------------------------------------------------

// Get the current mode
//  (returns -1 if not initialized)
int A4988::getMode(void){
  //check if initialized
  if(!_initialized)
    return -1;
  
  return _mode;
}

//-------------------------------------------------------------------------------------------------

// Get the current speed
//  (returns -1 if not initialized)
int A4988::getSpeed(void){
  //check if initialized
  if(!_initialized)
    return -1;
  
  return _speed;
}

//-------------------------------------------------------------------------------------------------

// Get the current mode
//  (returns 0 if not initialized)
unsigned int A4988::getSteps(void){
  //check if initialized
  if(!_initialized)
    return 0;
  
  return _steps;
}

//-------------------------------------------------------------------------------------------------

// Initialize the driver
//  (returns TRUE if initialized)
boolean A4988::Initialize(int pinDIR, int pinSTEP){
  //check if initialized
  if(_initialized)
    return true;
  
  if(pinDIR != pinSTEP){
    _initialized = true; //set
    
    _pinDIR = (byte)pinDIR;
    _pinSTEP = (byte)pinSTEP;
    
    pinMode(_pinDIR, OUTPUT);
    pinMode(_pinSTEP, OUTPUT);
    
    _dir = A4988_DIR_CW; //default
    _mode = A4988_FULL_STEP; //default
    _steps = A4988_DEFAULT_STEPS;
    setSpeedRPM(A4988_DEFAULT_SPEED_RPM);
    _mode_pins_set = false;
    _reset_pin_set = false;
    _use_ENABLE = 0;
    _use_SLEEP = 0;
    setDriveMode(A4988_DEFAULT_DRIVE_MODE);
    
    digitalWrite(_pinDIR, _dir);
    digitalWrite(_pinSTEP, LOW);
  } else {
    _initialized = false;
  }
  
  return _initialized;
}

//-------------------------------------------------------------------------------------------------

// Initialize the driver
//  (returns TRUE if initialized)
boolean A4988::Initialize(int pinDIR, int pinSTEP, int pinMS1, int pinMS2, int pinMS3){
  //check if initialized
  if(_initialized)
    return true;
  
  if(pinDIR != pinSTEP){
    _initialized = true; //set
    
    _pinDIR = (byte)pinDIR;
    _pinSTEP = (byte)pinSTEP;
    
    pinMode(_pinDIR, OUTPUT);
    pinMode(_pinSTEP, OUTPUT);
    
    _dir = A4988_DIR_CW; //default
    _mode = A4988_FULL_STEP; //default
    _steps = A4988_DEFAULT_STEPS;
    setSpeedRPM(A4988_DEFAULT_SPEED_RPM);
    
    if(setMSpins(pinMS2, pinMS2, pinMS3))
      _mode_pins_set = true;
    else
      _mode_pins_set = false;
    _reset_pin_set = false;
    _use_ENABLE = 0;
    _use_SLEEP = 0;
    setDriveMode(A4988_DEFAULT_DRIVE_MODE);
    
    digitalWrite(_pinDIR, _dir);
    digitalWrite(_pinSTEP, LOW);
  } else {
    _initialized = false;
  }
  
  return _initialized;
}

//-------------------------------------------------------------------------------------------------

// Invert the direction
void A4988::InvertDirection(void){
  //check if initialized
  if(!_initialized)
    return;
  
  if(_dir == A4988_DIR_CW)
    _dir = A4988_DIR_CCW;
  else
    _dir = A4988_DIR_CW;
  
  digitalWrite(_pinDIR, _dir);
}

//-------------------------------------------------------------------------------------------------

// Check if driver is enabled
//  (returns TRUE when enabled)
//  NOTE: also returns FALSE when not initialized
boolean A4988::isEnabled(void){
  //check if initialized
  if(!_initialized)
    return false;
  
  return (_use_ENABLE & 0x04);
}

//-------------------------------------------------------------------------------------------------

// Check if driver is sleeping
//  (returns TRUE when sleeping)
//  NOTE: also returns FALSE when not initialized
boolean A4988::isSleeping(void){
  //check if initialized
  if(!_initialized)
    return false;
  
  return (_use_SLEEP & 0x04);
}

//-------------------------------------------------------------------------------------------------

// Get if mode pins are set
boolean A4988::ModePinsSet(void){
  //check if initialized
  if(!_initialized)
    return false;
  
  return _mode_pins_set;
}

//-------------------------------------------------------------------------------------------------

// Send a 100 ms reset pulse
void A4988::Reset(void){
  //check if initialized
  if(!_initialized)
    return;
  
  if(_reset_pin_set){
    digitalWrite(_pinRST, LOW);
    delay(100);
    digitalWrite(_pinRST, HIGH);
  }
}

//-------------------------------------------------------------------------------------------------

// Set the direction
void A4988::setDir(int dir){
  //check if initialized
  if(!_initialized)
    return;
  
  if(dir > 0)
    _dir = A4988_DIR_CCW;
  else
    _dir = A4988_DIR_CW;
  
  digitalWrite(_pinDIR, _dir);
}

//-------------------------------------------------------------------------------------------------

// Set the drive mode
void A4988::setDriveMode(byte mode){
  //check if initialized
  if(!_initialized)
    return;
  
  if(mode == A4988_DRIVE_MODE_ENABLE){
    _use_ENABLE |= 0x02; //reset bit 1
    _use_SLEEP &= 0xFD; //set bit 1
  } else if(mode == A4988_DRIVE_MODE_SLEEP){
    _use_ENABLE &= 0xFD; //reset bit 1
    _use_SLEEP |= 0x02; //set bit 1
  } else { //NONE
    _use_ENABLE &= 0xFD; //reset bit 1
    _use_SLEEP &= 0xFD; //reset bit 1
  }
}

//-------------------------------------------------------------------------------------------------

// Set the enable pin
//  NOTE: enables the driver when called
void A4988::setENABLEpin(int pinENABLE){
  //check if initialized
  if(!_initialized)
    return;
  
  _pinENABLE = (byte)pinENABLE;
  _use_ENABLE |=  0x01; //bit 0
  Enable(); //default state
}

//-------------------------------------------------------------------------------------------------

// Set the mode (full, half, quarter, eighth or sixteenth)
// (returns -1 if not initialized, -2 if no pins set, 2 if set do A4988_FULL_STEP, 1 if other mode set)
//  NOTE: sets to FULL if no valid mode was given
int A4988::setMode(byte mode){
  //check if initialized
  if(!_initialized)
    return -1;
  
  //check if pins set
  if(!_mode_pins_set)
    return -2;
  
  int res;
      
  if((mode != A4988_HALF_STEP) && (mode != A4988_QUARTER_STEP) && (mode != A4988_EIGHTH_STEP) && (mode != A4988_SIXTEENTH_STEP)){ // use default
    digitalWrite(_pinsMODE[0], (A4988_FULL_STEP & 4));
    digitalWrite(_pinsMODE[1], (A4988_FULL_STEP & 2));
    digitalWrite(_pinsMODE[2], (A4988_FULL_STEP & 1));
    _mode = A4988_FULL_STEP;
    res = 2;
  } else { // use mode
    digitalWrite(_pinsMODE[0], (mode & 4));
    digitalWrite(_pinsMODE[1], (mode & 2));
    digitalWrite(_pinsMODE[2], (mode & 1));
    _mode = mode;
    res = 1;
  }
  
  return res;
}

//-------------------------------------------------------------------------------------------------

// Set the mode pins
//  (returns -1 if not initialized, -2 if MS pin is same as DIR pin, -3 if MS pin is same as STEP pin)
int A4988::setMSpins(int pinMS1, int pinMS2, int pinMS3){
  //check if initialized
  if(!_initialized)
    return -1;
  
  int res = 1;
  
  if((pinMS1 != _pinDIR) && (pinMS2 != _pinDIR) && (pinMS3 != _pinDIR)){
    if((pinMS1 != _pinSTEP) && (pinMS2 != _pinSTEP) && (pinMS3 != _pinSTEP)){
      //store pins
      _pinsMODE[0] = pinMS1;
      _pinsMODE[1] = pinMS2;
      _pinsMODE[2] = pinMS3;
      
      //configure pins
      pinMode(pinMS1, OUTPUT);
      pinMode(pinMS2, OUTPUT);
      pinMode(pinMS3, OUTPUT);

      setMode(_mode); //update
    } else {
      res = -3;
    }
  } else {
    res = -2;
  }
  
  return res;
}

//-------------------------------------------------------------------------------------------------

// Set the reset pin
void A4988::setRSTpin(int pinRST){
  //check if initialized
  if(!_initialized)
    return;
  
  _pinRST = (byte)pinRST;
  _reset_pin_set = true;
}

//-------------------------------------------------------------------------------------------------

// Set the sleep pin
//  NOTE: sets the driver on normal operation when called
void A4988::setSLEEPpin(int pinSLEEP){
  //check if initialized
  if(!_initialized)
    return;
  
  _pinSLEEP = (byte)pinSLEEP;
  _use_SLEEP |= 0x01; //bit 0
  WakeUp(); //default state
}

//-------------------------------------------------------------------------------------------------

// Set the speed in [rpm]
void A4988::setSpeedRPM(int rpm){
  //check if initialized
  if(!_initialized)
    return;
  
  if(rpm == 0)
    return;
  else if(rpm < 0)
    _speed = -1 * rpm;
  else
    _speed = rpm;
}

//-------------------------------------------------------------------------------------------------

// Set the number of steps in one revolution
//  NOTE: minimum of 4 steps
void A4988::setSteps(unsigned int steps){
  //check if initialized
  if(!_initialized)
    return;
  
  if(steps >= 4)
    _steps = steps;
}

//-------------------------------------------------------------------------------------------------

// Put the driver on sleep mode
void A4988::Sleep(void){
  //check if initialized
  if(!_initialized)
    return;
  
  if(_use_SLEEP & 0x01){ //pin set
    _use_SLEEP |= 0x04; //set bit 2
    digitalWrite(_pinSLEEP, LOW);
  }
}

//-------------------------------------------------------------------------------------------------

// Wake the driver up (normal operation)
void A4988::WakeUp(void){
  //check if initialized
  if(!_initialized)
    return;
  
  if(_use_SLEEP & 0x01){ //pin set
    _use_SLEEP &= 0xFB; //reset bit 2
    digitalWrite(_pinSLEEP, HIGH);
  }
}

//-------------------------------------------------------------------------------------------------



