/*
 * Penguino AVR USB firmware
 */

/*
  Copyright 2010 Icy Labs Pty. Ltd.

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include <string.h>

#include <util/delay.h>
//#include <avr/eeprom.h>

#include "USBtoSerial.h"
//#include "start_boot.h"

#define PENPROG_COMMAND_INVALID 0x00

#define PENPROG_COMMAND_GET_BOARD 0x01
#define PENPROG_COMMAND_RESET 0x02
#define PENPROG_COMMAND_JUMP_BOOTLOADER 0x03

#define PENPROG_COMMAND_JTAG_CLOCK_BITS 0x21

#define PENPROG_FEATURE_SYSTEM_RESET (1<<0)
#define PENPROG_FEATURE_TEST_RESET (1<<1)
#define PENPROG_FEATURE_JUMP_BOOTLOADER (1<<2)

#define PENPROG_JTAG_BIT_TDI 0x01
#define PENPROG_JTAG_BIT_TMS 0x02

#define FIRMWARE_VERSION 0x0001

/* Struct that lets us read and create commands of different types */
typedef struct {
	uint8_t type;
	
	union {
		// commands in
		struct {
			// none, static return
		} GetBoard;
		
		struct {
			uint8_t systemReset;
			uint8_t testReset;
		} Reset;
		
		struct {
			uint8_t numBits;
			uint8_t data[PENPROG_TXRX_EPSIZE-2];
		} JtagClockBits;
		
		
		// commands out
		struct {
			uint8_t boardFeatures;
			char boardName[16];
			uint16_t boardVersion;
			uint16_t firmwareVersion;
		} GetBoardResponse;
		
		struct {
			// none, always success
		} ResetResponse;
		
		struct {
			uint8_t numBits;
			uint8_t data[PENPROG_TXRX_EPSIZE-2];
		} JtagClockBitsResponse;
	} Command;
} PenProgCommand;

/* details for local ports and pins */
#define JTAG_PORT PORTB
#define JTAG_DDR DDRB
#define JTAG_PIN PINB

#define JTAG_BIT_TMS PB0
#define JTAG_BIT_TCK PB1
#define JTAG_BIT_TDI PB2
#define JTAG_BIT_TDO PB3
#define JTAG_BIT_NRST PB7

#define JTAG_PIN_TMS (1<<JTAG_BIT_TMS)
#define JTAG_PIN_TCK (1<<JTAG_BIT_TCK)
#define JTAG_PIN_TDI (1<<JTAG_BIT_TDI)
#define JTAG_PIN_TDO (1<<JTAG_BIT_TDO)
#define JTAG_PIN_NRST (1<<JTAG_BIT_NRST)


#define JTAG_SETTLE_DELAY_US 25
#define JTAG_RESPONSE_DELAY_US 50


#define SET_DDR_OUTPUT(ddr,pin) {ddr |= (pin);}
#define SET_DDR_INPUT(ddr,pin) {ddr &= ~(pin);}

#define DRIVE_PORT_HIGH(port,pin) {port |= (pin);}
#define DRIVE_PORT_LOW(port,pin) {port &= ~(pin);}

#define ENABLE_PORT_PULLUP(port,pin) {port |= (pin);}
#define DISABLE_PORT_PULLUP(port,pin) {port &= ~(pin);}

#define READ_PIN(port,pin) (port & (pin))

#define GET_CLOCK_BIT_FROM_COMMAND(cmd,bit) ((cmd)->Command.JtagClockBits.data[(bit)/4] >> (((bit)%4)*2))
#define SET_CLOCK_BIT_FROM_COMMAND(cmd,bit, val) ((cmd)->Command.JtagClockBitsResponse.data[(bit)/8] |= (val) << ((bit)%8))

static void jump_atmel_bootloader( void ) {
    // start by disabling USB
	USB_ShutDown( );
    
    // disable interrupts
    cli( );

	// Relocate the interrupt vector table to the bootloader section
    unsigned char temp = MCUCR;
	MCUCR = temp | (1 << IVCE);
	MCUCR = temp | (1 << IVSEL);

    // Jump to the bootloader section
    asm volatile ( "jmp 0x1000" );
}


static uint8_t clockJtagBit( uint8_t outputBits );

void runPenProgCommand( PenProgCommand *cmd ) {
//	uint8_t outputBits;
	PenProgCommand origCmd;
	
	switch ( cmd->type ) {
		case PENPROG_COMMAND_GET_BOARD:
			strcpy( cmd->Command.GetBoardResponse.boardName, "Penguino AVR" );
			cmd->Command.GetBoardResponse.boardVersion = 0x01;
			cmd->Command.GetBoardResponse.firmwareVersion = FIRMWARE_VERSION;
			cmd->Command.GetBoardResponse.boardFeatures = 
				PENPROG_FEATURE_SYSTEM_RESET |
				PENPROG_FEATURE_JUMP_BOOTLOADER;
			break;
		
	    case PENPROG_COMMAND_JUMP_BOOTLOADER:
			// on Penguino AVR, we support jumping to the Atmel bootloader
            jump_atmel_bootloader( );

			break;

		case PENPROG_COMMAND_RESET:
			// we only support system reset on Penguino AVR
			
			if ( cmd->Command.Reset.systemReset ) {
				// for a system reset, we set the pin to output and drive it low
				SET_DDR_OUTPUT( JTAG_DDR, JTAG_PIN_NRST );
				DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_NRST );
			} else {
				// before we leave reset, set our JTAG pins to inputs
				// for some extra safety
				DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_TDI );
				DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_TMS );
				DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_TCK );
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TDI );
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TMS );
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TCK );


				// in a normal state, set the port to input with no pull-up
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_NRST );
				DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_NRST );
			}
			
			break;
		
		case PENPROG_COMMAND_JTAG_CLOCK_BITS:
			origCmd = *cmd;
			
			for ( int i = 0; i < 30; i++ ) {
				cmd->Command.JtagClockBitsResponse.data[i] = 0;
			}
			
			for ( int i = 0; i < origCmd.Command.JtagClockBits.numBits; i++ ) {
				uint8_t bits = GET_CLOCK_BIT_FROM_COMMAND( &origCmd, i );
				
                uint8_t result = clockJtagBit( bits );
				
				SET_CLOCK_BIT_FROM_COMMAND( cmd, i, result );
			}
			
			break;

		default:
			cmd->type = PENPROG_COMMAND_INVALID;
			
			break;
	}
}

static uint8_t clockJtagBit( uint8_t outputBits ) {
    uint8_t response;
	
	// set all relevant pin directions
	DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TCK );
	DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TDI );
	DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TMS );
	SET_DDR_OUTPUT( JTAG_DDR, JTAG_PIN_TDI );
	SET_DDR_OUTPUT( JTAG_DDR, JTAG_PIN_TMS );
	SET_DDR_OUTPUT( JTAG_DDR, JTAG_PIN_TCK );
	SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TDO );
    
	if ( outputBits & PENPROG_JTAG_BIT_TDI ) {
		DRIVE_PORT_HIGH( JTAG_PORT, JTAG_PIN_TDI );
	}/* else {
		DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TDI );
	}*/
	
	if ( outputBits & PENPROG_JTAG_BIT_TMS ) {
		DRIVE_PORT_HIGH( JTAG_PORT, JTAG_PIN_TMS );
	}/* else {
		DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TMS );
	}*/
	
	// short wait here so the pins can settle
	_delay_us( JTAG_SETTLE_DELAY_US );
	
	// clock high
	DRIVE_PORT_HIGH( JTAG_PORT, JTAG_PIN_TCK );
	
	// long wait here so we get our response
	_delay_us( JTAG_RESPONSE_DELAY_US );
	
	response = ( READ_PIN( JTAG_PIN, JTAG_PIN_TDO ) ? 1 : 0 );
	
	// clock low
	DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TCK );
	
	// long wait here, do we need it?
	//_delay_us( JTAG_RESPONSE_DELAY_US );
	
	// set ports back to input
	SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TDI );
	SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TMS );
	
    return response;
}


/** Task to run PENPROG */
void PENPROG_Task(void) {
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	Endpoint_SelectEndpoint( PENPROG_DATA_EPNUM );
	
	if (!(Endpoint_IsOUTReceived()))
	    return;
	
	/* Remember how large the incoming packet is */
	uint16_t DataLength = Endpoint_BytesInEndpoint();
	
	/* Create a temp buffer big enough to hold the incoming endpoint packet */
	uint8_t  Buffer[PENPROG_TXRX_EPSIZE];
	
	/* Read in the incoming packet into the buffer */
	Endpoint_Read_Stream_LE(&Buffer, DataLength);
	
	/* Finalize the stream transfer to send the last packet */
	Endpoint_ClearOUT();
	
	/* switch endpoint direction (we're about to write back our response) */
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_IN);
	
	/* Process command */
	runPenProgCommand( (PenProgCommand *)Buffer );
	
	/* Write the received data to the endpoint */
	Endpoint_Write_Stream_LE(Buffer, PENPROG_TXRX_EPSIZE);
	
	/* Finalize the stream transfer to send the last packet */
	Endpoint_ClearIN();
	
	
	/* wait for packet to send, go back to reading */
	Endpoint_WaitUntilReady();
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_OUT);
}

