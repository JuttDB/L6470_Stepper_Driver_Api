/*
 * cL6470.h
 *
 * Created: 6/22/2016 8:40:34 PM
 *  Author: Atif
 */ 


#ifndef CL6470_H_
#define CL6470_H_

#include <Arduino.h>
#include <SPI.h>

// constant definitions for overcurrent thresholds. Write these values to
//  register x_OCD_TH to set the level at which an overcurrent even occurs.
#define x_OCD_TH_375mA  0x00
#define x_OCD_TH_750mA  0x01
#define x_OCD_TH_1125mA 0x02
#define x_OCD_TH_1500mA 0x03
#define x_OCD_TH_1875mA 0x04
#define x_OCD_TH_2250mA 0x05
#define x_OCD_TH_2625mA 0x06
#define x_OCD_TH_3000mA 0x07
#define x_OCD_TH_3375mA 0x08
#define x_OCD_TH_3750mA 0x09
#define x_OCD_TH_4125mA 0x0A
#define x_OCD_TH_4500mA 0x0B
#define x_OCD_TH_4875mA 0x0C
#define x_OCD_TH_5250mA 0x0D
#define x_OCD_TH_5625mA 0x0E
#define x_OCD_TH_6000mA 0x0F

// STEP_MODE option values.
// First comes the "microsteps per step" options...
#define x_STEP_MODE_STEP_SEL 0x07  // Mask for these bits only.
#define x_STEP_SEL_1     0x00
#define x_STEP_SEL_1_2   0x01
#define x_STEP_SEL_1_4   0x02
#define x_STEP_SEL_1_8   0x03
#define x_STEP_SEL_1_16  0x04
#define x_STEP_SEL_1_32  0x05
#define x_STEP_SEL_1_64  0x06
#define x_STEP_SEL_1_128 0x07

// ...next, define the SYNC_EN bit. When set, the BUSYN pin will instead
//  output a clock related to the full-step frequency as defined by the
//  SYNC_SEL bits below.
#define x_STEP_MODE_SYNC_EN	 0x80  // Mask for this bit
#define x_SYNC_EN 0x80

// ...last, define the SYNC_SEL modes. The clock output is defined by
//  the full-step frequency and the value in these bits- see the datasheet
//  for a matrix describing that relationship (page 46).
#define x_STEP_MODE_SYNC_SEL 0x70
#define x_SYNC_SEL_1_2 0x00
#define x_SYNC_SEL_1   0x10
#define x_SYNC_SEL_2   0x20
#define x_SYNC_SEL_4   0x30
#define x_SYNC_SEL_8   0x40
#define x_SYNC_SEL_16  0x50
#define x_SYNC_SEL_32  0x60
#define x_SYNC_SEL_64  0x70

// Bit names for the ALARM_EN register.
//  Each of these bits defines one potential alarm condition.
//  When one of these conditions occurs and the respective bit in ALARM_EN is set,
//  the FLAG pin will go low. The register must be queried to determine which event
//  caused the alarm.
#define x_ALARM_EN_OVERCURRENT       0x01
#define x_ALARM_EN_THERMAL_SHUTDOWN	 0x02
#define x_ALARM_EN_THERMAL_WARNING   0x04
#define x_ALARM_EN_UNDER_VOLTAGE     0x08
#define x_ALARM_EN_STALL_DET_A       0x10
#define x_ALARM_EN_STALL_DET_B       0x20
#define x_ALARM_EN_SW_TURN_ON        0x40
#define x_ALARM_EN_WRONG_NPERF_CMD   0x80

// CONFIG register renames.

// Oscillator options.
// The x needs to know what the clock frequency is because it uses that for some
//  calculations during operation.
#define x_CONFIG_OSC_SEL                 0x000F // Mask for this bit field.
#define x_CONFIG_INT_16MHZ               0x0000 // Internal 16MHz, no output
#define x_CONFIG_INT_16MHZ_OSCOUT_2MHZ   0x0008 // Default; internal 16MHz, 2MHz output
#define x_CONFIG_INT_16MHZ_OSCOUT_4MHZ   0x0009 // Internal 16MHz, 4MHz output
#define x_CONFIG_INT_16MHZ_OSCOUT_8MHZ   0x000A // Internal 16MHz, 8MHz output
#define x_CONFIG_INT_16MHZ_OSCOUT_16MHZ  0x000B // Internal 16MHz, 16MHz output
#define x_CONFIG_EXT_8MHZ_XTAL_DRIVE     0x0004 // External 8MHz crystal
#define x_CONFIG_EXT_16MHZ_XTAL_DRIVE    0x0005 // External 16MHz crystal
#define x_CONFIG_EXT_24MHZ_XTAL_DRIVE    0x0006 // External 24MHz crystal
#define x_CONFIG_EXT_32MHZ_XTAL_DRIVE    0x0007 // External 32MHz crystal
#define x_CONFIG_EXT_8MHZ_OSCOUT_INVERT  0x000C // External 8MHz crystal, output inverted
#define x_CONFIG_EXT_16MHZ_OSCOUT_INVERT 0x000D // External 16MHz crystal, output inverted
#define x_CONFIG_EXT_24MHZ_OSCOUT_INVERT 0x000E // External 24MHz crystal, output inverted
#define x_CONFIG_EXT_32MHZ_OSCOUT_INVERT 0x000F // External 32MHz crystal, output inverted

// Configure the functionality of the external switch input
#define x_CONFIG_SW_MODE                 0x0010 // Mask for this bit.
#define x_CONFIG_SW_HARD_STOP            0x0000 // Default; hard stop motor on switch.
#define x_CONFIG_SW_USER                 0x0010 // Tie to the GoUntil and ReleaseSW
//  commands to provide jog function.
//  See page 25 of datasheet.

// Configure the motor voltage compensation mode (see page 34 of datasheet)
#define x_CONFIG_EN_VSCOMP               0x0020  // Mask for this bit.
#define x_CONFIG_VS_COMP_DISABLE         0x0000  // Disable motor voltage compensation.
#define x_CONFIG_VS_COMP_ENABLE          0x0020  // Enable motor voltage compensation.

// Configure overcurrent detection event handling
#define x_CONFIG_OC_SD                   0x0080  // Mask for this bit.
#define x_CONFIG_OC_SD_DISABLE           0x0000  // Bridges do NOT shutdown on OC detect
#define x_CONFIG_OC_SD_ENABLE            0x0080  // Bridges shutdown on OC detect

// Configure the slew rate of the power bridge output
#define x_CONFIG_POW_SR                  0x0300  // Mask for this bit field.
#define x_CONFIG_SR_180V_us              0x0000  // 180V/us
#define x_CONFIG_SR_290V_us              0x0200  // 290V/us
#define x_CONFIG_SR_530V_us              0x0300  // 530V/us

// Integer divisors for PWM sinewave generation
//  See page 32 of the datasheet for more information on this.
#define x_CONFIG_F_PWM_DEC               0x1C00      // mask for this bit field
#define x_CONFIG_PWM_MUL_0_625           (0x00)<<10
#define x_CONFIG_PWM_MUL_0_75            (0x01)<<10
#define x_CONFIG_PWM_MUL_0_875           (0x02)<<10
#define x_CONFIG_PWM_MUL_1               (0x03)<<10
#define x_CONFIG_PWM_MUL_1_25            (0x04)<<10
#define x_CONFIG_PWM_MUL_1_5             (0x05)<<10
#define x_CONFIG_PWM_MUL_1_75            (0x06)<<10
#define x_CONFIG_PWM_MUL_2               (0x07)<<10

// Multiplier for the PWM sinewave frequency
#define x_CONFIG_F_PWM_INT               0xE000     // mask for this bit field.
#define x_CONFIG_PWM_DIV_1               (0x00)<<13
#define x_CONFIG_PWM_DIV_2               (0x01)<<13
#define x_CONFIG_PWM_DIV_3               (0x02)<<13
#define x_CONFIG_PWM_DIV_4               (0x03)<<13
#define x_CONFIG_PWM_DIV_5               (0x04)<<13
#define x_CONFIG_PWM_DIV_6               (0x05)<<13
#define x_CONFIG_PWM_DIV_7               (0x06)<<13

// Status register bit renames- read-only bits conferring information about the
//  device to the user.
#define x_STATUS_HIZ                     0x0001 // high when bridges are in HiZ mode
#define x_STATUS_BUSY                    0x0002 // mirrors BUSY pin
#define x_STATUS_SW_F                    0x0004 // low when switch open, high when closed
#define x_STATUS_SW_EVN                  0x0008 // active high, set on switch falling edge,
//  cleared by reading STATUS
#define x_STATUS_DIR                     0x0010 // Indicates current motor direction.
//  High is FWD, Low is REV.
#define x_STATUS_NOTPERF_CMD             0x0080 // Last command not performed.
#define x_STATUS_WRONG_CMD               0x0100 // Last command not valid.
#define x_STATUS_UVLO                    0x0200 // Undervoltage lockout is active
#define x_STATUS_TH_WRN                  0x0400 // Thermal warning
#define x_STATUS_TH_SD                   0x0800 // Thermal shutdown
#define x_STATUS_OCD                     0x1000 // Overcurrent detected
#define x_STATUS_STEP_LOSS_A             0x2000 // Stall detected on A bridge
#define x_STATUS_STEP_LOSS_B             0x4000 // Stall detected on B bridge
#define x_STATUS_SCK_MOD                 0x8000 // Step clock mode is active

// Status register motor status field
#define x_STATUS_MOT_STATUS                0x0060      // field mask
#define x_STATUS_MOT_STATUS_STOPPED       (0x0000)<<13 // Motor stopped
#define x_STATUS_MOT_STATUS_ACCELERATION  (0x0001)<<13 // Motor accelerating
#define x_STATUS_MOT_STATUS_DECELERATION  (0x0002)<<13 // Motor decelerating
#define x_STATUS_MOT_STATUS_CONST_SPD     (0x0003)<<13 // Motor at constant speed

// Register address redefines.
//  See the x_Param_Handler() function for more info about these.
#define x_ABS_POS              0x01
#define x_EL_POS               0x02
#define x_MARK                 0x03
#define x_SPEED                0x04
#define x_ACC                  0x05
#define x_DEC                  0x06
#define x_MAX_SPEED            0x07
#define x_MIN_SPEED            0x08
#define x_FS_SPD               0x15
#define x_KVAL_HOLD            0x09
#define x_KVAL_RUN             0x0A
#define x_KVAL_ACC             0x0B
#define x_KVAL_DEC             0x0C
#define x_INT_SPD              0x0D
#define x_ST_SLP               0x0E
#define x_FN_SLP_ACC           0x0F
#define x_FN_SLP_DEC           0x10
#define x_K_THERM              0x11
#define x_ADC_OUT              0x12
#define x_OCD_TH               0x13
#define x_STALL_TH             0x14
#define x_STEP_MODE            0x16
#define x_ALARM_EN             0x17
#define x_CONFIG               0x18
#define x_STATUS               0x19

//x commands
#define x_NOP                  0x00
#define x_SET_PARAM            0x00
#define x_GET_PARAM            0x20
#define x_RUN                  0x50
#define x_STEP_CLOCK           0x58
#define x_MOVE                 0x40
#define x_GOTO                 0x60
#define x_GOTO_DIR             0x68
#define x_GO_UNTIL             0x82
#define x_RELEASE_SW           0x92
#define x_GO_HOME              0x70
#define x_GO_MARK              0x78
#define x_RESET_POS            0xD8
#define x_RESET_DEVICE         0xC0
#define x_SOFT_STOP            0xB0
#define x_HARD_STOP            0xB8
#define x_SOFT_HIZ             0xA0
#define x_HARD_HIZ             0xA8
#define x_GET_STATUS           0xD0

/* x direction options */
#define FWD  0x01
#define REV  0x00

/* x action options */
#define ACTION_RESET  0x00
#define ACTION_COPY   0x01


class cL6470 {
		public:
	           
			   //Default constuctot
			   cL6470();
			   
			   //Overload Constructor
			   //@param int MOSI, int MISO, int CS, int CLK, int BUSY, int RESET
			   cL6470(int,int ,int ,int ,int ,int );
			   void init();
		       unsigned long _GetParam(byte);
			   byte _Data_To_Transfer(byte);
		       unsigned long _ParamHandler(byte , unsigned long );
			   void  _SetParam(byte , unsigned long );
			   unsigned long _Param(unsigned long, byte);
			   int _GetStatus();
			   unsigned long _SpdCalc(float);	
			   unsigned long _IntSpdCalc(float);	
			   unsigned long _FSCalc(float);	
			   unsigned long _MinSpdCalc(float);	
			   unsigned long _MaxSpdCalc(float);
			   unsigned long _DecCalc(float);
			   unsigned long _AccCalc(float);
				void _SetLSPDOpt(boolean);
				void _Run(byte, unsigned long);
				void _Step_Clock(byte);
				void _Move(byte, unsigned long);
				void _GoTo(unsigned long);
				void _GoTo_DIR(byte, unsigned long);
				void _GoUntil(byte, byte, unsigned long);
				void _ReleaseSW(byte, byte);
				void _GoHome();
				void _GoMark();
				void _ResetPos();
				void _ResetDev();
				void _SoftStop();
				void _HardStop();
				void _SoftHiZ();
				void _HardHiZ();
				
				
				//@param int MOSI 
			  void _Set_MOSI(int);
			  //@param int MISO 
			  void _Set_MISO(int);
			  //@param int CS 
			  void _Set_CS(int );
			  //@param int CLK	
			  void _Set_CLK(int);	
			  //@param int BUSY 
			  void _Set_BUSY(int );	
			  //@param int RESET
			  void _Set_RESET(int);	
			  
			  
			  
			  int _Get_MOSI();
			  int _Get_MISO();
			  int _Get_CS();
			  int _Get_CLK();
			  int _Get_BUSY();
			  int _Get_RESET();
			  
			  
		private:
				// Member Variable
				
				int _MOSI  = 0;
				int _MISO  = 0;
				int _CS	   = 0;
				int _CLK   = 0;
				int _BUSY  = 0;
				int _RESET = 0;
			};


#endif /* CL6470_H_ */
