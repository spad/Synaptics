/*
 * Synaptics.h - a library to interface with ps2 Synaptics
 * touchpad.
 * Written by Saverio Pieri.
 * Release into public domain.
 */
#include <inttypes.h>

#ifndef synaptics_h
#define synaptics_h

#define PAD_REMOTE_MODE 0x01 // only remote mode supported 
#define PAD_STREAM_MODE 0x02 // for reference only. Not used

#define PAD_STATUS_ENABLED  0x01 
#define PAD_STATUS_DISABLED 0x02 

#define PAD_SEQ_INFO  		0x01 // information queries command sequence 
#define PAD_SEQ_SET_MODE 	0x02 // set mode command sequence

#define PAD_MODE_ABSOLUTE	0x80 // bit7 1 to select Absolute mode
#define PAD_MODE_RATE_80	0x40 // bit6 1 to select high (80) packet rate
#define PAD_MODE_SLEEP		0x30 // bit3 1 to select sleep mode
#define PAD_MODE_DISGEST	0x20 // bit2 1 to disable detection of tap and drag gestures
#define PAD_MODE_PACKETSIZE 0x10 // not used in ps2 mode
#define PAD_MODE_WMODE		0x01 // 0 to select normal Absolute mode packets, or 1 to select
								 // enhanced Absolute packets that contain the “W” value
#define PAD_MODE_COMMON_00	0x01 // Always OK Relative mode
#define PAD_MODE_COMMON_04  0x04 // Version 4.x or later Relative mode with gestures disabled
#define PAD_MODE_COMMON_40 	0x40 // Always OK Relative mode with high packet rate
#define PAD_MODE_COMMON_80 	0x80 // capExtended = 0 Absolute mode
#define PAD_MODE_COMMON_81 	0x81 // capExtended = 1 Absolute mode with W
#define PAD_MODE_COMMON_C0  0xC0 // capExtended = 0 Absolute mode with high packet rate
#define PAD_MODE_COMMON_C1  0xC1 // capExtended = 1 Absolute mode with W, high packet rate
#define PAD_MODE_COMMON_0C  0x0C // capSleep = 1 Low-power sleep mode


class Synaptics
{
	public:
		uint8_t data[6];

		Synaptics(int clkpin, int datapin);
		void write(uint8_t databyte);
		uint8_t read(void);

		bool reset(void);
		void disable(void);
		void enable(void);
		void set_remote_mode(void);
		void status_request(void); // ...read buttons status
		void read_data(void);
		void special_sequence(int sequence_type, uint8_t param);
		// information queries
		void identify(void);
		uint8_t read_modes(void);
		void read_capabilities(void);
		void read_modelid(void);
		// set mode sequence
		void set_mode(uint8_t mode);
		bool isAbsolute(void);
		bool isRelative(void);
		// helper functions to parse data packet
		uint8_t getZ(void);
		uint8_t getW(void);	
		int getX(void);	
		int getY(void);	
		bool leftClicked(void);
		bool rightClicked(void);
	private:
		int _ps2clk;
		int _ps2data;
		uint8_t _mode;		// read_modes
		uint8_t _status; 	// PAD_STATUS_ENABLED | PAD_STATUS_DISABLED

		void golo(int pin);
		void gohi(int pin);
		bool read_ack(void);
};

#endif /* synaptics_h */

