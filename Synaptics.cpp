/*
 * ps2.cpp - an interface library for ps2 devices.  Common devices are
 * mice, keyboard, barcode scanners etc.  See the examples for mouse and
 * keyboard interfacing.
 * limitations:
 *      we do not handle parity errors.
 *      The timing constants are hard coded from the spec. Data rate is
 *         not impressive.
 *      probably lots of room for optimization.
 */

#include "Synaptics.h"
#include "Arduino.h"
/*
 * the clock and data pins can be wired directly to the clk and data pins
 * of the Ps2 connector.  No external parts are needed.
 */
Synaptics::Synaptics(int clkpin, int datapin)
{
	_ps2clk = clkpin;
	_ps2data = datapin;

	gohi(_ps2clk);
	gohi(_ps2data);
	disable();
	// read mode byte to have consistent data reading
	read_modes();
}

void
Synaptics::gohi(int pin)
{
	pinMode(pin, INPUT);
	digitalWrite(pin, HIGH);
}

void
Synaptics::golo(int pin)
{
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
}

/*
Perform a software reset and recalibration as described in section
3.3. Response is ACK ($FA), followed by $AA, $00 after a
calibration delay of 300–500ms
*/
bool
Synaptics::reset(void)
{
	uint8_t success;
	write(0xff); 		// send reset command
	read_ack();			// read ack
	delay(500);			// wait for calibration
	success = read();	// device response: 0xaa = success; 0xfc = failure
	read();				// read device id. Always 0x00
	return (success==0xaa);
}



/* Switch to Remote mode, as distinct from the default
Stream mode. In Remote mode, the device sends motion data packets only
in response to a Read Data ($EB) command */
void
Synaptics::set_remote_mode(void)
{
	write(0xf0); 	// send command "Set Remote mode"
	read_ack(); 	// read ack
}

void
Synaptics::disable(void) 
{
	write(0xf5); 	// send command "disable"
	read_ack(); 	// read ack
	_status = PAD_STATUS_DISABLED;
}

void
Synaptics::enable(void) 
{
	write(0xf4); 	// send command "enable"
	read_ack(); 	// read ack
	_status = PAD_STATUS_ENABLED;
}

/*
Response is an ACK ($FA), followed by a 3-byte status
packet consisting of the following data

		bit7	bit6	bit5	bit4	bit3	bit2	bit1 	bit0
____________________________________________________________________
byte1|	 0     Remote  Enable  Scaling    0     Left   Middle  Right
byte2|	 0        0       0       0       0       0      Resolution
byte3|                           Sample Rate

Remote: 1 = Remote (polled) mode, 0 = Stream mode.
Enable: 1 = Data reporting enabled, 0 = disabled. This bit only has effect in Stream mode.
Scaling: 1 = Scaling is 2:1, 0 = scaling is 1:1. See commands $E6 and $E7 below.
Left: 1 = Left button is currently pressed, 0 = released.
Middle: 1 = Middle button is currently pressed, 0 = released.
Right: 1 = Right button is currently pressed, 0 = released.
Resolution: The current resolution setting, from 0 to 3 (1, 2, 4, and 8 counts per mm).
Sample rate: The current sample rate setting, from 10 to 200.
*/
void 
Synaptics::status_request(void)
{
	write(0xe9);
	read_ack();
	data[0] = read();
	data[1] = read();
	data[2] = read();
}

/*
In the default Relative format, each motion packet consists of three bytes. The first byte
encodes various status bits, and the other two bytes encode the amount of motion in X
and Y that has occurred since the previous packet.

		bit7	bit6	bit5	bit4	bit3	bit2	bit1 	bit0
____________________________________________________________________
byte1| Y_ovfl  X_ovfl  Y_sign  X_sign    1     Middle   Right   Left
byte2|	 							X delta
byte3|                              Y delta

When Absolute mode is enabled, each motion report consists of six bytes. These bytes
encode the absolute X, Y location of the finger on the sensor pad, as well as the Z
(pressure) value and various other measurements and status bits. Section 2.3 discusses
the contents of the Absolute mode packet in great detail.
*/
void
Synaptics::read_data(void)
{
	write(0xeb);
	read_ack();
	data[0] = read();
	data[1] = read();
	data[2] = read();
	// chek for absolute mode (6 bytes)
	if (bitRead(_mode,7)) {
		data[3] = read();
		data[4] = read();
		data[5] = read();
	}
}


/*
* get the Z value from data packet. Returns 0 if relative mode
*/
uint8_t 
Synaptics::getZ(void) {
	if (isAbsolute())
		return(data[2]);
	else 
		return 0;
}

/*
* get the W value from data packet. Returns 0 if relative mode
*/
uint8_t 
Synaptics::getW(void) {
	// check if W=1
	if (bitRead(_mode,0)) {
		return (
			((data[0] & 0x30)>>2) |
			((data[0] & 0x04)>>1) |
			((data[4] & 0x04)>>2) 
		);
	} else 
		return 0;
}

/*
* get the X value from data packet
*/
int 
Synaptics::getX(void) {
	if (isAbsolute()) {
		return (
			data[4] 								|
			((data[1] & 0x0F)<<8) 	| 
			((data[3] & 0x10)<<8) 	 
		);
	} else {
		return (bitRead(data[0],4))?(0xff00 | data[1]):data[1];
	}
}

/*
* get the Y value from data packet
*/
int 
Synaptics::getY(void) {
	if (isAbsolute()) {
		return (
			data[5] 								|
			((data[1] & 0xF0)<<4) 	| 
			((data[3] & 0x20)<<7) 	 
		);
	} else {
		return (bitRead(data[0],5))?(0xff00 | data[2]):data[2];
	}
}

bool 
Synaptics::leftClicked(void) {
	return (bitRead(data[0],0));
}

bool 
Synaptics::rightClicked(void) {
	return (bitRead(data[0],1));
}


void
Synaptics::special_sequence(int sequence_type, uint8_t param)
{
	// send initial sequence
	write(0xe8); read_ack();
	write((param >> 6) & 0x03); read_ack();
	write(0xe8); read_ack();
	write((param >> 4) & 0x03); read_ack();
	write(0xe8); read_ack();
	write((param >> 2) & 0x03); read_ack();
	write(0xe8); read_ack();
	write(param & 0x03); read_ack();
	// send command
	if (sequence_type == PAD_SEQ_INFO) 
	{
		write(0xe9);
		read_ack();
		// returns data 
		data[0] = read();
		data[1] = read();	// usually 0x47 except for 0x03 param on recent models
		data[2] = read();
	} else if (sequence_type == PAD_SEQ_SET_MODE)
	{
		write(0xf3);
		read_ack();
		write(0x14);
		read_ack();
		//no result
	}
}


/*
		    bit7	  bit6	  bit5	  bit4	  bit3	  bit2	  bit1	 bit0
		____________________________________________________________________
byte1|	 							   InfoMinor
byte2|	 								 0x047
byte3|   			infoModelCode		    | 			infoMajor
*/
void 
Synaptics::identify(void)
{
	special_sequence(PAD_SEQ_INFO,0x00);
}

/**
* returns the mode byte. 
* bit(7): 1=>absolute mode; 0=>relative mode
* bit(6): 0=>40; 1=>80; packet per second
* bit(3): 1=>sleep; 0=>normal;
* bit(2): This bit is 0 to enable “tap” and “drag” gesture processing, or 1 to disable
		  detection of tap and drag gestures. When this bit is 1, the Relative mode
		  mouse packet reports the true physical button states, and the Absolute mode
		  packet’s Gesture bit always reports as zero. The DisGest bit is implemented
		  only for 40.x and later TouchPads (i.e., when infoMajor ≥ 4); for older pads,
		  the bit is reserved.
* bit(0): 0=>normal Absolute mode packets; 1=>enhanced Absolute packets that contain the “W”
*/
uint8_t 
Synaptics::read_modes(void)
{
	special_sequence(PAD_SEQ_INFO,0x01);
	_mode = data[2];
	return _mode;
}

/**
* returns 3 bytes device capabilities.
		    bit7	  bit6	  bit5	  bit4	  bit3	  bit2	  bit1	 		bit0
		________________________________________________________________________
byte1|	 cExended		-		-		-	   -		-		-	  	 	-
byte2|	 								 0x047
byte3|   	 -			-		-	 cSleep	cFourBtn 	-   cMultiFIng	 cPalmDet

capExtended (bit 15)
	This bit is set if the extended capability bits are supported. The host can
	examine this bit to see whether the other 15 extended capability bits are
	present;
capSleep (bit 4)
	For the PS/2 protocol, the capSleep bit is set if sleep mode is supported
capFourButtons (bit 3)
	For the PS/2 protocol, this bit is set if the pad is a “MultiSwitch” pad which
	supports four mouse buttons labeled Left, Right, Up, and Down. In the
	PS/2 protocol, the Up and Down buttons are reported only during Absolute
	Mode with the Wmode bit set
capMultiFinger (bit 1)
	This bit is set if multi-finger detection is supported. The pad is then able to
	count the number of simultaneous fingers on the pad and report the finger
	count via the W field of the Absolute packet. If this bit is 0, the pad does
	not support multi-finger detection; any finger contact will be assumed to be
	a single finger, with the position reported as the midpoint between all actual
	fingers, and, if capPalmDetect is set, with W reporting a (typically large)
	“width” for the assumed single finger.
capPalmDetect (bit 0)
	This bit is set if “palm detection” is supported. In “W mode,” the TouchPad
	measures the apparent size or width of the finger and reports the width in
	the W field of the Absolute mode packet. The host can use this information
	to help distinguish between intentional finger contact and accidental palm
	or hand contact.	
*/
void 
Synaptics::read_capabilities(void)
{
	special_sequence(PAD_SEQ_INFO,0x02);
}

void 
Synaptics::read_modelid(void)
{
	special_sequence(PAD_SEQ_INFO,0x03);
}

/**
* set mode for pad. See section 2.5 for details (or .h file)
*/
void
Synaptics::set_mode(uint8_t mode)
{
	special_sequence(PAD_SEQ_SET_MODE,mode);
	_mode = mode;
}

/* write a byte to the Synaptics device */
void
Synaptics::write(uint8_t databyte)
{
	uint8_t i;
	uint8_t parity = 1;
	
	gohi(_ps2data);
	gohi(_ps2clk);
	delayMicroseconds(300);
	golo(_ps2clk);
	delayMicroseconds(300);
	golo(_ps2data);
	delayMicroseconds(10);
	gohi(_ps2clk);	// start bit
	/* wait for device to take control of clock */
	while (digitalRead(_ps2clk) == HIGH) {;}
	// clear to send data
	for (i=0; i < 8; i++)
	{
		if (databyte & 0x01)
		{ 
			gohi(_ps2data);
		} else {
			golo(_ps2data);
		}
		// wait for clock
		while (digitalRead(_ps2clk) == LOW) {;}
		while (digitalRead(_ps2clk) == HIGH){;}
		parity = parity ^ (databyte & 0x01);
		databyte = databyte >> 1;
	}
	// parity bit
	if (parity)
	{
		gohi(_ps2data);
	} else {
		golo(_ps2data);
	}
	// clock cycle - like ack.
	while (digitalRead(_ps2clk) == LOW){;}
	while (digitalRead(_ps2clk) == HIGH){;}
	// stop bit
	gohi(_ps2data);
	delayMicroseconds(50);
	while (digitalRead(_ps2clk) == HIGH){;}
	// mode switch
	while ((digitalRead(_ps2clk) == LOW) || (digitalRead(_ps2data) == LOW)){;}
	// hold up incoming data
	golo(_ps2clk);
}

/*
* returns true if the pad is currently in absolute mode
*/
bool 
Synaptics::isAbsolute(void) {
	return (bitRead(_mode,7));
}

/*
* returns true if the pad is currently in relative mode
*/
bool 
Synaptics::isRelative(void) {
	return (!bitRead(_mode,7));
}


/*
 * read a byte of data from the ps2 device.  Ignores parity.
 */
uint8_t
Synaptics::read(void)
{
	uint8_t data = 0x00;
	uint8_t i;
	uint8_t bit = 0x01;

	// start clock
	gohi(_ps2clk);
	gohi(_ps2data);
	delayMicroseconds(50);
	while (digitalRead(_ps2clk) == HIGH)
		;
	delayMicroseconds(5);	// not sure why.
	while (digitalRead(_ps2clk) == LOW)
		;	// eat start bit
	for (i=0; i < 8; i++)
	{
		while (digitalRead(_ps2clk) == HIGH)
			;
		if (digitalRead(_ps2data) == HIGH)
		{
			data = data | bit;
		}
		while (digitalRead(_ps2clk) == LOW)
			;
		bit = bit << 1;
	}
	// eat parity bit, ignore it.
	while (digitalRead(_ps2clk) == HIGH)
		;
	while (digitalRead(_ps2clk) == LOW)
		;
	// eat stop bit
	while (digitalRead(_ps2clk) == HIGH)
		;
	while (digitalRead(_ps2clk) == LOW)
		;
	golo(_ps2clk);	// hold incoming data

	return data;
}

bool
Synaptics::read_ack(void) 
{
	return (read()==0xfa);
}
