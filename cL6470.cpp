/*
 * cL6470.cpp
 *
 * Created: 6/22/2016 8:41:00 PM
 *  Author: Atif
 */ 
#include "cL6470.h"



cL6470::cL6470(int MOSI,int MISO,int CS,int CLK,int BUSY,int RESET) 
			{
				_Set_MISO(MISO);
				_Set_MOSI(MOSI);
				_Set_CS(CS);
				_Set_CLK(CLK);
				_Set_BUSY(BUSY);
				_Set_RESET(RESET);
				
			}
			

void cL6470::init()
		{
		pinMode(_CS, OUTPUT);
		digitalWrite(_CS, HIGH);
		pinMode(_MOSI, OUTPUT);
		pinMode(_MISO, INPUT);
		pinMode(_CLK, OUTPUT);
		pinMode(_BUSY, INPUT);
		pinMode(_RESET, OUTPUT);
		
		digitalWrite(_RESET, HIGH);
		delay(1);
		digitalWrite(_RESET, LOW);
		delay(1);
		digitalWrite(_RESET, HIGH);
		delay(1);
		
		SPI.begin();
		SPI.setBitOrder(MSBFIRST);
		SPI.setClockDivider(SPI_CLOCK_DIV16); // or 2, 8, 16, 32, 64
		SPI.setDataMode(SPI_MODE3);
		}

void cL6470::_SetParam(byte param, unsigned long value)
{
	_Data_To_Transfer(x_SET_PARAM | param);
	_ParamHandler(param, value);
}


unsigned long cL6470::_GetParam(byte param)
	{
	_Data_To_Transfer(x_GET_PARAM | param);
	return _ParamHandler(param, 0);
	}

// Enable or disable the low-speed optimization option. If enabling,
//  the other 12 bits of
void cL6470::_SetLSPDOpt(boolean enable)
{
	_Data_To_Transfer(x_SET_PARAM | x_MIN_SPEED);
	if (enable) _Param(0x1000, 13);
	else _Param(0, 13);
}

// RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
// The SpdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void cL6470::_Run(byte dir, unsigned long spd)
{
	_Data_To_Transfer(x_RUN | dir);
	if (spd > 0xFFFFF) spd = 0xFFFFF;
	_Data_To_Transfer((byte)(spd >> 16));
	_Data_To_Transfer((byte)(spd >> 8));
	_Data_To_Transfer((byte)(spd));
}

// STEP_CLOCK puts the device in external step clocking mode. When active,
//  pin 25, STCK, becomes the step clock for the device, and steps it in
//  the direction (set by the FWD and REV constants) imposed by the call
//  of this function. Motion commands (RUN, MOVE, etc) will cause the device
//  to exit step clocking mode.
void cL6470::_Step_Clock(byte dir)
{
	_Data_To_Transfer(x_STEP_CLOCK | dir);
}

// MOVE will send the motor n_step steps (size based on step mode) in the
//  direction imposed by dir (FWD or REV constants may be used). The motor
//  will accelerate according the acceleration and deceleration curves, and
//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
void cL6470::_Move(byte dir, unsigned long n_step)
{
	_Data_To_Transfer(x_MOVE | dir);
	if (n_step > 0x3FFFFF) n_step = 0x3FFFFF;
	_Data_To_Transfer((byte)(n_step >> 16));
	_Data_To_Transfer((byte)(n_step >> 8));
	_Data_To_Transfer((byte)(n_step));
}

// GOTO operates much like MOVE, except it produces absolute motion instead
//  of relative motion. The motor will be moved to the indicated position
//  in the shortest possible fashion.
void cL6470::_GoTo(unsigned long pos)
{
	
	_Data_To_Transfer(x_GOTO);
	if (pos > 0x3FFFFF) pos = 0x3FFFFF;
	_Data_To_Transfer((byte)(pos >> 16));
	_Data_To_Transfer((byte)(pos >> 8));
	_Data_To_Transfer((byte)(pos));
}

// Same as GOTO, but with user constrained rotational direction.
void cL6470::_GoTo_DIR(byte dir, unsigned long pos)
{
	
	_Data_To_Transfer(x_GOTO_DIR);
	if (pos > 0x3FFFFF) pos = 0x3FFFFF;
	_Data_To_Transfer((byte)(pos >> 16));
	_Data_To_Transfer((byte)(pos >> 8));
	_Data_To_Transfer((byte)(pos));
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void cL6470::_GoUntil(byte act, byte dir, unsigned long spd)
{
	_Data_To_Transfer(x_GO_UNTIL | act | dir);
	if (spd > 0x3FFFFF) spd = 0x3FFFFF;
	_Data_To_Transfer((byte)(spd >> 16));
	_Data_To_Transfer((byte)(spd >> 8));
	_Data_To_Transfer((byte)(spd));
}

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.
void cL6470::_ReleaseSW(byte act, byte dir)
{
	_Data_To_Transfer(x_RELEASE_SW | act | dir);
}

// GoHome is equivalent to GoTo(0), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void cL6470::_GoHome()
{
	_Data_To_Transfer(x_GO_HOME);
}

// GoMark is equivalent to GoTo(MARK), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void cL6470::_GoMark()
{
	_Data_To_Transfer(x_GO_MARK);
}

// Sets the ABS_POS register to 0, effectively declaring the current
//  position to be "HOME".
void cL6470::_ResetPos()
{
	_Data_To_Transfer(x_RESET_POS);
}

// Reset device to power up conditions. Equivalent to toggling the STBY
//  pin or cycling power.
void cL6470::_ResetDev()
{
	_Data_To_Transfer(x_RESET_DEVICE);
}

// Bring the motor to a halt using the deceleration curve.
void cL6470::_SoftStop()
{
	_Data_To_Transfer(x_SOFT_STOP);
}

// Stop the motor with infinite deceleration.
void cL6470::_HardStop()
{
	_Data_To_Transfer(x_HARD_STOP);
}

// Decelerate the motor and put the bridges in Hi-Z state.
void cL6470::_SoftHiZ()
{
	_Data_To_Transfer(x_SOFT_HIZ);
}

// Put the bridges in Hi-Z state immediately with no deceleration.
void cL6470::_HardHiZ()
{
	_Data_To_Transfer(x_HARD_HIZ);
}

// Fetch and return the 16-bit value in the STATUS register. Resets
//  any warning flags and exits any error states. Using GetParam()
//  to read STATUS does not clear these values.
int cL6470::_GetStatus()
{
	int temp = 0;
	_Data_To_Transfer(x_GET_STATUS);
	temp = _Data_To_Transfer(0)<<8;
	temp |= _Data_To_Transfer(0);
	return temp;
}




unsigned long cL6470::_AccCalc(float stepsPerSecPerSec)
{
	float temp = stepsPerSecPerSec * 0.137438;
	if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
	else return (unsigned long) long(temp);
}

// The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long cL6470::_DecCalc(float stepsPerSecPerSec)
{
	float temp = stepsPerSecPerSec * 0.137438;
	if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
	else return (unsigned long) long(temp);
}

// The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is
//  250ns (datasheet value)- 0x041 on boot.
// Multiply desired steps/s by .065536 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
unsigned long cL6470::_MaxSpdCalc(float stepsPerSec)
{
	float temp = stepsPerSec * .065536;
	if( (unsigned long) long(temp) > 0x000003FF) return 0x000003FF;
	else return (unsigned long) long(temp);
}

// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is
//  250ns (datasheet value)- 0x000 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long cL6470::_MinSpdCalc(float stepsPerSec)
{
	float temp = stepsPerSec * 4.1943;
	if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
	else return (unsigned long) long(temp);
}

// The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is
//  250ns (datasheet value)- 0x027 on boot.
// Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
unsigned long cL6470::_FSCalc(float stepsPerSec)
{
	float temp = (stepsPerSec * .065536)-.5;
	if( (unsigned long) long(temp) > 0x000003FF) return 0x000003FF;
	else return (unsigned long) long(temp);
}

// The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is
//  250ns (datasheet value)- 0x408 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
unsigned long cL6470::_IntSpdCalc(float stepsPerSec)
{
	float temp = stepsPerSec * 4.1943;
	if( (unsigned long) long(temp) > 0x00003FFF) return 0x00003FFF;
	else return (unsigned long) long(temp);
}

// When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is
//  250ns (datasheet value).
// Multiply desired steps/s by 67.106 to get an appropriate value for this register
// This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.
unsigned long cL6470::_SpdCalc(float stepsPerSec)
{
	float temp = stepsPerSec * 67.106;
	if( (unsigned long) long(temp) > 0x000FFFFF) return 0x000FFFFF;
	else return (unsigned long)temp;
}




// This simple function shifts a byte out over SPI and receives a byte over
//  SPI. Unusually for SPI devices, the x requires a toggling of the
//  CS (slaveSelect) pin after each byte sent. That makes this function
//  a bit more reasonable, because we can include more functionality in it.
byte cL6470::_Data_To_Transfer(byte data)
{
	byte data_out;
	digitalWrite(_CS,LOW);
	data_out = SPI.transfer(data);
	digitalWrite(_CS,HIGH);
	return data_out;
}


// the register will be automatically zero.
//  When disabling, the value will have to be explicitly written by
//  the user with a SetParam() call. See the datasheet for further
//  information about low-speed optimization.// Much of the functionality between "get parameter" and "set parameter" is
//  very similar, so we deal with that by putting all of it in one function
//  here to save memory space and simplify the program.
unsigned long cL6470::_ParamHandler(byte param, unsigned long value)
{
	unsigned long ret_val = 0;   // This is a temp for the value to return.
	// This switch structure handles the appropriate action for each register.
	//  This is necessary since not all registers are of the same length, either
	//  bit-wise or byte-wise, so we want to make sure we mask out any spurious
	//  bits and do the right number of transfers. That is handled by the x_Param()
	//  function, in most cases, but for 1-byte or smaller transfers, we call
	//  Data_To_Transfer() directly.
	switch (param)
	{
		// ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
		//  in two's complement. At power up, this value is 0. It cannot be written when
		//  the motor is running, but at any other time, it can be updated to change the
		//  interpreted position of the motor.
		case x_ABS_POS:
		ret_val = _Param(value, 22);
		break;
		// EL_POS is the current electrical position in the step generation cycle. It can
		//  be set when the motor is not in motion. Value is 0 on power up.
		case x_EL_POS:
		ret_val = _Param(value, 9);
		break;
		// MARK is a second position other than 0 that the motor can be told to go to. As
		//  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
		case x_MARK:
		ret_val = _Param(value, 22);
		break;
		// SPEED contains information about the current speed. It is read-only. It does
		//  NOT provide direction information.
		case x_SPEED:
		ret_val = _Param(0, 20);
		break;
		// ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF
		//  to get infinite acceleration/decelaeration- there is no way to get infinite
		//  deceleration w/o infinite acceleration (except the HARD STOP command).
		//  Cannot be written while motor is running. Both default to 0x08A on power up.
		// AccCalc() and DecCalc() functions exist to convert steps/s/s values into
		//  12-bit values for these two registers.
		case x_ACC:
		ret_val = _Param(value, 12);
		break;
		case x_DEC:
		ret_val = _Param(value, 12);
		break;
		// MAX_SPEED is just what it says- any command which attempts to set the speed
		//  of the motor above this value will simply cause the motor to turn at this
		//  speed. Value is 0x041 on power up.
		// MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
		//  for this register.
		case x_MAX_SPEED:
		ret_val = _Param(value, 10);
		break;
		// MIN_SPEED controls two things- the activation of the low-speed optimization
		//  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
		//  is the 13th bit, and when it is set, the minimum allowed speed is automatically
		//  set to zero. This value is 0 on startup.
		// MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
		//  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
		case x_MIN_SPEED:
		ret_val = _Param(value, 12);
		break;
		// FS_SPD register contains a threshold value above which microstepping is disabled
		//  and the x operates in full-step mode. Defaults to 0x027 on power up.
		// FSCalc() function exists to convert steps/s value into 10-bit integer for this
		//  register.
		case x_FS_SPD:
		ret_val = _Param(value, 10);
		break;
		// KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
		//  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
		// The implications of different KVAL settings is too complex to dig into here, but
		//  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
		//  HOLD may result in excessive power dissipation when the motor is not running.
		case x_KVAL_HOLD:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		case x_KVAL_RUN:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		case x_KVAL_ACC:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		case x_KVAL_DEC:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		// INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
		//  compensation functionality. Please see the datasheet for details of this
		//  function- it is too complex to discuss here. Default values seem to work
		//  well enough.
		case x_INT_SPD:
		ret_val = _Param(value, 14);
		break;
		case x_ST_SLP:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		case x_FN_SLP_ACC:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		case x_FN_SLP_DEC:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		// K_THERM is motor winding thermal drift compensation. Please see the datasheet
		//  for full details on operation- the default value should be okay for most users.
		case x_K_THERM:
		ret_val = _Data_To_Transfer((byte)value & 0x0F);
		break;
		// ADC_OUT is a read-only register containing the result of the ADC measurements.
		//  This is less useful than it sounds; see the datasheet for more information.
		case x_ADC_OUT:
		ret_val = _Data_To_Transfer(0);
		break;
		// Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
		//  A set of defined constants is provided for the user's convenience. Default
		//  value is 3.375A- 0x08. This is a 4-bit value.
		case x_OCD_TH:
		ret_val = _Data_To_Transfer((byte)value & 0x0F);
		break;
		// Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
		//  4A in 31.25mA steps. This is a 7-bit value.
		case x_STALL_TH:
		ret_val = _Data_To_Transfer((byte)value & 0x7F);
		break;
		// STEP_MODE controls the microstepping settings, as well as the generation of an
		//  output signal from the x. Bits 2:0 control the number of microsteps per
		//  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
		//  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
		//  of the output signal relative to the full-step frequency; see datasheet for
		//  that relationship as it is too complex to reproduce here.
		// Most likely, only the microsteps per step value will be needed; there is a set
		//  of constants provided for ease of use of these values.
		case x_STEP_MODE:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		// ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
		//  is provided to make this easy to interpret. By default, ALL alarms will trigger the
		//  FLAG pin.
		case x_ALARM_EN:
		ret_val = _Data_To_Transfer((byte)value);
		break;
		// CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
		//  set of reasonably self-explanatory constants is provided, but users should refer
		//  to the datasheet before modifying the contents of this register to be certain they
		//  understand the implications of their modifications. Value on boot is 0x2E88; this
		//  can be a useful way to verify proper start up and operation of the x chip.
		case x_CONFIG:
		ret_val = _Param(value, 16);
		break;
		// STATUS contains read-only information about the current condition of the chip. A
		//  comprehensive set of constants for masking and testing this register is provided, but
		//  users should refer to the datasheet to ensure that they fully understand each one of
		//  the bits in the register.
		case x_STATUS:  // STATUS is a read-only register
		ret_val = _Param(0, 16);
		break;
		default:
		ret_val = _Data_To_Transfer((byte)(value));
		break;
	}
	return ret_val;
}

// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
unsigned long cL6470::_Param(unsigned long value, byte bit_len)
{
	unsigned long ret_val=0;        // We'll return this to generalize this function
	//  for both read and write of registers.
	byte byte_len = bit_len/8;      // How many BYTES do we have?
	if (bit_len%8 > 0) byte_len++;  // Make sure not to lose any partial byte values.
	// Let's make sure our value has no spurious bits set, and if the value was too
	//  high, max it out.
	unsigned long mask = 0xffffffff >> (32-bit_len);
	if (value > mask) value = mask;
	// The following three if statements handle the various possible byte length
	//  transfers- it'll be no less than 1 but no more than 3 bytes of data.
	// Data_To_Transfer() sends a byte out through SPI and returns a byte received
	//  over SPI- when calling it, we typecast a shifted version of the masked
	//  value, then we shift the received value back by the same amount and
	//  store it until return time.
	if (byte_len == 3) {
		ret_val |= _Data_To_Transfer((byte)(value>>16)) << 16;
		//Serial.println(ret_val, HEX);
	}
	if (byte_len >= 2) {
		ret_val |= _Data_To_Transfer((byte)(value>>8)) << 8;
		//Serial.println(ret_val, HEX);
	}
	if (byte_len >= 1) {
		Serial.print ("Value   =    ");
		Serial.println (value);
		Serial.print ("ret_val   =    ");
		Serial.println (ret_val);
		
		ret_val |= _Data_To_Transfer((byte)value);
		
		Serial.print ("ret_val   =    ");
		Serial.println (ret_val);
		
		//Serial.println(ret_val, HEX);
	}
	// Return the received values. Mask off any unnecessary bits, just for
	//  the sake of thoroughness- we don't EXPECT to see anything outside
	//  the bit length range but better to be safe than sorry.
	return (ret_val & mask);
}

// This simple function shifts a byte out over SPI and receives a byte over
//  SPI. Unusually for SPI devices, the x requires a toggling of the
//  CS (slaveSelect) pin after each byte sent. That makes this function
//  a bit more reasonable, because we can include more functionality in it.




















			
void cL6470::_Set_MOSI(int MOSI)

			{
				_MOSI = MOSI;
			}
			
void cL6470::_Set_MISO(int MISO)

			{
				_MISO = MISO;
			}
			
void cL6470::_Set_CS(int CS)

			{
				_CS = CS;
			}

void cL6470::_Set_CLK(int CLK)

			{
				_CLK = CLK;
			}

void cL6470::_Set_BUSY(int BUSY)

			{
				_BUSY = BUSY;
			}

void cL6470::_Set_RESET(int RESET)

			{
				_RESET = RESET;
			}

int cL6470::_Get_MOSI()
			{
			return _MOSI;
			}				
			
int cL6470::_Get_MISO()
			{
	return _MISO;
			}

int cL6470::_Get_CS()
			{
	return _CS;
			}
int cL6470::_Get_CLK()
			{
	return _CLK;
			}
			
int cL6470::_Get_BUSY()
			{
	return _BUSY;
			}			
int cL6470::_Get_RESET()
			{
	return _RESET;
			}
			
						
