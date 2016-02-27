/*
 *   PIC18F2550 driving a Pololu A4988 step motor driver with interrupt support
 *   Copyright (C) 2016  Jasper Büsch
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
/*******************************************************************************
 *  Author: Jasper Büsch
 *  Contact: jasper.buesch@gmail.com
*******************************************************************************/

#define N_EN       PORTB.f2
#define TRIS_N_EN  TRISB.f2
#define MS1        PORTB.f1
#define TRIS_MS1   TRISB.f1
#define MS2        PORTB.f0
#define TRIS_MS2   TRISB.f0
#define MS3        PORTA.f5
#define TRIS_MS3   TRISA.f5
#define N_RST      PORTA.f4
#define TRIS_N_RST TRISA.f4
#define N_SLP      PORTA.f3
#define TRIS_N_SLP TRISA.f3
#define STEP       PORTA.f2
#define TRIS_STEP  TRISA.f2
#define DIR        PORTA.f1
#define TRIS_DIR   TRISA.f1
#define SWITCH     PORTA.f0
#define TRIS_SWITCH TRISA.f0
#define OUTPUT_TRIS 0
#define INPUT_TRIS 1

#define STEP_DIVISOR 16
#define MAX_SPEED 400
#define MIN_SPEED 50
#define DELAY_STEP 300

typedef unsigned short uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef signed short int8_t;
typedef signed int int16_t;
typedef signed long int32_t;

uint8_t wait = 0;
uint8_t direction = 0;

uint16_t delayCounter = 0;
uint16_t delayValue = 0; // microsteps not included in this value
uint16_t delayCounterMax = 0;
uint16_t stepCounter = 0;
uint16_t stepStart = 0;
uint16_t stepCounterMax = 0;
uint8_t subStepCounter = 0;
uint16_t speed = 80; // in steps pro second
int8_t writeDelayValue = 0;
uint16_t clock = 0;
int8_t stepIncrement = 1;
uint8_t switchLatch = 0;

int8_t buttonClickCounter = 0;
uint16_t buttonClickDelay = 0;

/*******************************************************************************
 * The Pololu step motor controller A4988 supports micro steps
 * Set microstep pins (valid inputs are 1, 2, 4, 8 and 16)
 ******************************************************************************/
void setMS(uint8_t microSteps) {
  if ((microSteps == 2) || (microSteps == 8) || (microSteps == 16))
    MS1 = 1;
  else
    MS1 = 0;
  if ((microSteps == 4) || (microSteps == 8) || (microSteps == 16))
    MS2 = 1;
  else
    MS2 = 0;
  if (microSteps == 16)
    MS3 = 1;
  else
    MS3 = 0;
}

/*******************************************************************************
 * Timer frequency that drives the step interrupt is 1MHz
 ******************************************************************************/
uint16_t calculateDelay(uint16_t velocity) {
  uint32_t delay = (1000000L / (uint32_t) velocity);
  return (uint16_t) delay;
}

/*******************************************************************************
 * Function to perform one step and busy wait in multiples of 1ms
 ******************************************************************************/
void stepping(uint16_t delayTime) {
  uint16_t t = 0;
  STEP = 1;
  Delay_us(1);
  STEP = 0;
  for (t=0; t<delayTime; t++)
    Delay_ms(1);
}


void main() {
  // disable all analog inputs
  ADCON1 = 0x0f;

  // configure the pins
  TRIS_SWITCH = INPUT_TRIS;
  TRIS_N_EN   = OUTPUT_TRIS;
  TRIS_MS1    = OUTPUT_TRIS;
  TRIS_MS2    = OUTPUT_TRIS;
  TRIS_MS3    = OUTPUT_TRIS;
  TRIS_N_RST  = OUTPUT_TRIS;
  TRIS_N_SLP  = OUTPUT_TRIS;
  TRIS_STEP   = OUTPUT_TRIS;
  TRIS_DIR    = OUTPUT_TRIS;

  // setting default pin states
  N_EN = 0;
  setMS(1); // default disable micro stepping
  N_RST = 1;
  N_SLP = 1;
  STEP  = 0;
  DIR   = 1;

  // timer 0 drives the stepping
  T0CON.PSA = 0;
  // PreScaler of 000 -> 1/2 => 1us
  T0CON.T0PS2 = 0;
  T0CON.T0PS1 = 0;
  T0CON.T0PS0 = 0;
  T0CON.T0CS = 0;
  T0CON.T08BIT = 0;
  T0CON.TMR0ON = 0; // enabled later

  // timer 1 is used for wall clock
  T1CON.RD16 = 1;
  T1CON.TMR1ON = 1;

  RCON.IPEN = 1; // Int priorities enabled

  PIE1.TMR1IE = 1;
  IPR1.TMR1IP = 0; // TMR1 gets low priority
  INTCON.TMR0IE = 1;
  INTCON.GIE = 0;   // enabled after setup phase
  INTCON.GIEL = 1;

  // reloading values from EEPROM
  stepCounterMax = EEPROM_Read(0) + (EEPROM_Read(1) << 8);
  delayValue = EEPROM_Read(2) + (EEPROM_Read(3) << 8);
  stepStart = EEPROM_Read(4) + (EEPROM_Read(5) << 8);
  delayCounter = delayValue / STEP_DIVISOR;

  // Rotation angle setup if SWITCH is pressed during reset
  // First period until SWITCH pressed again defines the offset 
  // start step distance
  // Second press ends the work rotation distance
  if (!SWITCH) {
    stepCounter = 0;
    setMS(1);
    while (!SWITCH);
    while (SWITCH) {
      stepCounter++;
      stepping(50);
    }
    stepStart = stepCounter;
    EEPROM_Write(4, stepStart);
    EEPROM_Write(5, stepStart >> 8);
    stepCounter = 0;
    while (!SWITCH);
    while (SWITCH) {
      stepCounter++;
      stepping(50);
    }
    while (!SWITCH);
    stepCounterMax = stepCounter;
    stepCounter = 0;
    EEPROM_Write(0, stepCounterMax);
    EEPROM_Write(1, stepCounterMax >> 8);
  }
  // end rotation angle setup
  
  // going to zero position
  N_RST = 0; // by resetting the driver, the motor becomes limb
  Delay_ms(1000);
  N_RST = 1;
  while (stepCounter++ < stepStart)
    stepping(40);
  stepCounter = 0;
  // end going to zero position
  
  setMS(STEP_DIVISOR);
  INTCON.GIE = 1;
  T0CON.TMR0ON = 1;
 
 
  while (1) {
    while (!switchLatch && (buttonClickCounter==0)) {
      if (SWITCH) break;
      speed += stepIncrement;
      if (speed > MAX_SPEED)
        speed = MIN_SPEED;
      delay_ms(15);
      delayValue = calculateDelay(speed);
      delayCounter = delayValue / STEP_DIVISOR;
      writeDelayValue = 1;
    }
    if (writeDelayValue) {
      writeDelayValue = 0;
      EEPROM_Write(2, delayValue);
      EEPROM_Write(3, delayValue >> 8);
    }
  }
}

void interrupt_low() {
   // Interrupt routine for wall clock.
   TMR1H = 0xF8;
   TMR1L = 0x2f;
   clock++;
   PIR1.TMR1IF = 0;
}

void interrupt() {
   // Interrupt routine to drive the stepping.
   // Depending on the sub stepping setting, sub steps are
   // not counted in the global stepCounter variable
   uint16_t rms = 0xffff - delayCounter; // timer overflow at 0xffff
   TMR0H = rms >> 8;
   TMR0L = rms;
   // drive the step pins
   STEP = 1;
   Delay_us(1); // 1us is minimal pin toggle delay
   STEP = 0;
   subStepCounter++;
   if (subStepCounter >= STEP_DIVISOR) {
     subStepCounter = 0;
     stepCounter++;
   }
   if (stepCounter > stepCounterMax) {
     DIR = (DIR + 1) & 1;
     stepCounter = 0;
   }
   INTCON.TMR0IF = 0;
}