/*         Special Note                

Before Run this Code Please Update the Value in Special Spection Section At Line 107-126
to update the Value please use the Excel file(BEMF compensation) if you install the library then please 
found in your documents->Arduino->libraries->L6470 Folder. check your Holding current ,Running current, phase inductance (Lph)
and Phase resistance in the motor Datsheet,put these value into the excel file then you will get the KVAL value then update these KVAL values into the example Special section


please first connect the Motor Supply and then turn On the arduino.

if the Red Line is ON please immidiatelly switch off the Motor Power Supply otherwise you will demake your Board

*/




#include <SPI.h>    // include the SPI library:
#include "cL6470.h"

 

String cmd = "";

cL6470 L6470(11,12,10,13,5,6); //cL6470(MOSI, MISO, CS, CLK, BUSY , RESET ); 

void setup() 
{
 L6470.init();
//  // Standard serial port initialization for debugging.
  Serial.begin(9600);
  Serial.setTimeout(50);
  delay(500); 
  if (L6470._GetParam(x_CONFIG) == 0x2E88) 
  // The following function calls are for this demo application- 
  //  you will need to adjust them for your particular 
  //  application, and you may need to configure additional 
  //  registers.
  
  // First, let's set the step mode register:
  //   - x_SYNC_EN controls whether the BUSY/SYNC pin reflects
  //      the step frequency or the BUSY status of the chip. We 
  //      want it to be the BUSY status.
  //   - x_STEP_SEL_x is the microstepping rate- we'll go full 
  //      step.
  //   - x_SYNC_SEL_x is the ratio of (micro)steps to toggles
  //      on the BUSY/SYNC pin (when that pin is used for SYNC). 
  //      Make it 1:1, despite not using that pin.
  L6470._SetParam(x_STEP_MODE, 
                      !x_SYNC_EN | 
                      x_STEP_SEL_1_128 | 
                      x_SYNC_SEL_1);
                      
  // Configure the MAX_SPEED register- this is the maximum number
  //  of (micro)steps per second allowed. You'll want to mess 
  //  around with your desired application to see how far you can
  //  push it before the motor starts to slip. The ACTUAL 
  //  parameter passed to this function is in steps/tick; 
  //  MaxSpdCalc() will convert a number of steps/s into an 
  //  appropriate value for this function. Note that for any move 
  //  or goto type function where no speed is specified, this 
  //  value will be used.
  L6470._SetParam(x_MAX_SPEED, L6470._MaxSpdCalc(50));
  
  // Configure the FS_SPD register- this is the speed at which the
  //  driver ceases microstepping and goes to full stepping. 
  //  FSCalc() converts a value in steps/s to a value suitable for
  //  this register; to disable full-step switching, you can pass 
  //  0x3FF to this register.
  L6470._SetParam(x_FS_SPD, L6470._FSCalc(850));
  
  // Configure the acceleration rate, in steps/tick/tick. There is
  //  also a DEC register; both of them have a function (AccCalc()
  //  and DecCalc() respectively) that convert from steps/s/s into
  //  the appropriate value for the register. Writing ACC to 0xfff
  //  sets the acceleration and deceleration to 'infinite' (or as
  //  near as the driver can manage). If ACC is set to 0xfff, DEC 
  //  is ignored. To get infinite deceleration without infinite 
  //  acceleration, only hard stop will work.
  L6470._SetParam(x_ACC, 0xfff);
  
  // Configure the overcurrent detection threshold. The constants 
  //  for this are defined in the L6470.h file.
  L6470._SetParam(x_OCD_TH, x_OCD_TH_6000mA);
  
  // Set up the CONFIG register as follows:
  //  PWM frequency divisor = 1
  //  PWM frequency multiplier = 2 (62.5kHz PWM frequency)
  //  Slew rate is 290V/us
  //  Do NOT shut down bridges on overcurrent
  //  Disable motor voltage compensation
  //  Hard stop on switch low
  //  16MHz internal oscillator, nothing on output
  L6470._SetParam(x_CONFIG, 
                   x_CONFIG_PWM_DIV_1 | 
                   x_CONFIG_PWM_MUL_2 | 
                   x_CONFIG_SR_180V_us |
                   x_CONFIG_OC_SD_DISABLE | 
                   x_CONFIG_VS_COMP_DISABLE |
                   x_CONFIG_SW_HARD_STOP | 
                   x_CONFIG_INT_16MHZ);


//----------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>     Special Section <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                   
  // Configure the RUN KVAL. This defines the duty cycle of the 
  //  PWM of the bridges during running. 0xFF means that they are
  //  essentially NOT PWMed during run; this MAY result in more 
  //  power being dissipated than you actually need for the task.
  //  Setting this value too low may result in failure to turn.
  //  There are ACC, DEC, and HOLD KVAL registers as well; you may
  //  need to play with those values to get acceptable performance
  //  for a given application.
  L6470._SetParam(x_KVAL_HOLD, 0x05);
  L6470._SetParam(x_KVAL_RUN, 0x23);
  L6470._SetParam(x_KVAL_ACC, 0x23);
  L6470._SetParam(x_KVAL_DEC, 0x23);
  L6470._SetParam(x_ST_SLP, 0x18);
  L6470._SetParam(x_INT_SPD, 0x3550);
  L6470._SetParam(x_FN_SLP_ACC, 0x23);
  L6470._SetParam(x_FN_SLP_DEC, 0x23);

  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  // Calling GetStatus() clears the UVLO bit in the status 
  //  register, which is set by default on power-up. The driver 
  //  may not run without that bit cleared by this read operation.
  L6470._GetStatus();
  
  // Now we're going to set up a counter to track pulses from an
  //  encoder, to verify against the expected values.
  TCCR1A = 0;  // No waveform generation stuff.
  TCCR1B = B00000110; // Clock on falling edge, T1 pin.
  TCNT1 = 0;   // Clear the count.
}

void loop()
{
L6470._GetStatus(); // Clear the error
L6470._Move(FWD, 25600);
  while (digitalRead(L6470._Get_BUSY()) == LOW);  // Until the movement completes, the
        {}                                    //  BUSYN pin will be low.
L6470._SetParam(x_MAX_SPEED, L6470._MaxSpdCalc(300));  // Change the Speed
delay(500);
L6470._Move(REV, 25600);
  while (digitalRead(L6470._Get_BUSY()) == LOW);  // Until the movement completes, the
        {}                                    //  BUSYN pin will be low.
L6470._SetParam(x_MAX_SPEED, L6470._MaxSpdCalc(50));  // Change the Speed
delay(500);
}


