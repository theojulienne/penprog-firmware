#include <string.h>

#include <util/delay.h>
//#include <avr/eeprom.h>

#include "USBtoSerial.h"
//#include "start_boot.h"

#define PENPROG_COMMAND_INVALID 0x00

#define PENPROG_COMMAND_GET_BOARD 0x01
#define PENPROG_COMMAND_RESET 0x02
#define PENPROG_COMMAND_JUMP_BOOTLOADER 0x03
#define PENPROG_COMMAND_FIRMWARE_VERSION 0x04

#define PENPROG_COMMAND_JTAG_CLOCK_BIT 0x20
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
			
		} FirmwareVersion;
		
		struct {
			uint8_t bits;
		} JtagClockBit;
		
		struct {
			uint8_t numBits;
			uint8_t data[PENPROG_TXRX_EPSIZE-2];
		} JtagClockBits;
		
		
		// commands out
		struct {
			uint8_t boardFeatures;
			char boardName[16];
			uint8_t boardVersion;
		} GetBoardResponse;
		
		struct {
			// none, always success
		} ResetResponse;
		
		struct {
			uint32_t version;
		} FirmwareVersionResponse;
		
		struct {
			uint8_t bit;
		} JtagClockBitResponse;
		
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
#define JTAG_PIN_TMS (1<<PB0)
#define JTAG_PIN_TCK (1<<PB1)
#define JTAG_PIN_TDI (1<<PB2)
#define JTAG_PIN_TDO (1<<PB3)
#define JTAG_PIN_NRST (1<<PB7)


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

inline void jump_atmel_bootloader( void ) {
    // start by disabling USB
	USB_ShutDown( );

    cli( );

	/* Relocate the interrupt vector table to the bootloader section */
    unsigned char temp = MCUCR;
	MCUCR = temp | (1 << IVCE);
	MCUCR = temp | (1 << IVSEL);

    asm volatile ( "jmp 0x1000" );
}


void runPenProgCommand( PenProgCommand *cmd ) {
//	uint8_t outputBits;
//	PenProgCommand tmpCmd, origCmd;
	
	switch ( cmd->type ) {
		case PENPROG_COMMAND_GET_BOARD:
			strcpy( cmd->Command.GetBoardResponse.boardName, "Penguino AVR" );
			cmd->Command.GetBoardResponse.boardVersion = 0x01;
			cmd->Command.GetBoardResponse.boardFeatures = 
				PENPROG_FEATURE_SYSTEM_RESET |
				PENPROG_FEATURE_JUMP_BOOTLOADER;
			break;
		/*
		case PENPROG_COMMAND_FIRMWARE_VERSION:
			cmd->Command.FirmwareVersionResponse.version = FIRMWARE_VERSION;
			
			break;
		*/
		
	    case PENPROG_COMMAND_JUMP_BOOTLOADER:
			// on Penguino AVR, we support jumping to the Atmel bootloader
            jump_atmel_bootloader( );

			break;
#if 0
		case PENPROG_COMMAND_RESET:
			// we only support system reset on Penguino AVR
			
			if ( cmd->Command.Reset.systemReset ) {
				// for a system reset, we set the pin to output and drive it low
				SET_DDR_OUTPUT( JTAG_DDR, JTAG_PIN_NRST );
				DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_NRST );
			} else {
				// before we leave reset, set our JTAG pins to inputs
				// for some extra safety
				/*DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_TDI );
				DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_TMS );
				DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_TCK );
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TDI );
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TMS );
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TCK );*/


				// in a normal state, set the port to input with no pull-up
				SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_NRST );
				DISABLE_PORT_PULLUP( JTAG_PORT, JTAG_PIN_NRST );
			}
			
			break;
		
		case PENPROG_COMMAND_JTAG_CLOCK_BIT:
			outputBits = cmd->Command.JtagClockBit.bits;
			
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
			} else {
				DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TDI );
			}
			
			if ( outputBits & PENPROG_JTAG_BIT_TMS ) {
				DRIVE_PORT_HIGH( JTAG_PORT, JTAG_PIN_TMS );
			} else {
				DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TMS );
			}
			
			// short wait here so the pins can settle
			_delay_us( JTAG_SETTLE_DELAY_US );
			
			// clock high
			DRIVE_PORT_HIGH( JTAG_PORT, JTAG_PIN_TCK );
			
			// long wait here so we get our response
			_delay_us( JTAG_RESPONSE_DELAY_US );
			
			cmd->Command.JtagClockBitResponse.bit = ( READ_PIN( JTAG_PIN, JTAG_PIN_TDO ) ? 1 : 0 );
			
			// clock low
			DRIVE_PORT_LOW( JTAG_PORT, JTAG_PIN_TCK );
			
			// long wait here, do we need it?
			//_delay_us( JTAG_RESPONSE_DELAY_US );
			
			// set ports back to input
			SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TDI );
			SET_DDR_INPUT( JTAG_DDR, JTAG_PIN_TMS );
			
			break;
		
		case PENPROG_COMMAND_JTAG_CLOCK_BITS:
			origCmd = *cmd;
			
			for ( int i = 0; i < 30; i++ ) {
				cmd->Command.JtagClockBitsResponse.data[i] = 0;
			}
			
			for ( int i = 0; i < origCmd.Command.JtagClockBits.numBits; i++ ) {
				uint8_t bits = GET_CLOCK_BIT_FROM_COMMAND( &origCmd, i );
				
				tmpCmd.type = PENPROG_COMMAND_JTAG_CLOCK_BIT;
				tmpCmd.Command.JtagClockBit.bits = bits & 0x3;
				runPenProgCommand( &tmpCmd );
				
				SET_CLOCK_BIT_FROM_COMMAND( cmd, i, tmpCmd.Command.JtagClockBitResponse.bit & 0x1 );
			}
			
			break;
#endif
		default:
			cmd->type = PENPROG_COMMAND_INVALID;
			
			break;
	}
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
	Endpoint_Read_Stream_LE(&Buffer, DataLength, NO_STREAM_CALLBACK);
	
	/* Finalize the stream transfer to send the last packet */
	Endpoint_ClearOUT();
	
	
	/* Process command */
	runPenProgCommand( (PenProgCommand *)Buffer );
	
	
	/* switch endpoint direction (we're about to write back our response) */
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_IN);
	
	/* Write the received data to the endpoint */
	Endpoint_Write_Stream_LE(Buffer, PENPROG_TXRX_EPSIZE, NO_STREAM_CALLBACK);
	
	/* Finalize the stream transfer to send the last packet */
	Endpoint_ClearIN();
	
	
	/* wait for packet to send, go back to reading */
	Endpoint_WaitUntilReady();
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_OUT);
}

