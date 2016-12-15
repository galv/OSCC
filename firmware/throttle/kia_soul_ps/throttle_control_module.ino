/************************************************************************/
/* Copyright (c) 2016 PolySync Technologies, Inc.  All Rights Reserved. */
/*                                                                      */
/* This file is part of Open Source Car Control (OSCC).                 */
/*                                                                      */
/* OSCC is free software: you can redistribute it and/or modify         */
/* it under the terms of the GNU General Public License as published by */
/* the Free Software Foundation, either version 3 of the License, or    */
/* (at your option) any later version.                                  */
/*                                                                      */
/* OSCC is distributed in the hope that it will be useful,              */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of       */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        */
/* GNU General Public License for more details.                         */
/*                                                                      */
/* You should have received a copy of the GNU General Public License    */
/* along with OSCC.  If not, see <http://www.gnu.org/licenses/>.        */
/************************************************************************/

// Throttle control ECU firmware
// Firmware for control of 2014 Kia Soul throttle system
// Components:
//    Arduino Uno
//    Seed Studio CAN-BUS Shield, v1.2 (MCP2515)
//    Sainsmart 4 relay module
//    ETT ET-MINI SPI DAC (MCP4922)
// J Hartung, 2015; E Livingston, L Buckland, D Fernández, 2016

#include <SPI.h>
#include <PID_v1.h>
#include "mcp_can.h"
#include "can_frame.h"
#include "control_protocol_can.h"




// *****************************************************
// static global types/macros
// *****************************************************


// set CAN_CS to pin 9 for CAN
#define CAN_CS 9

//
#define CAN_BAUD (CAN_500KBPS)

//
#define SERIAL_DEBUG_BAUD (115200)

//
#define CAN_INIT_RETRY_DELAY (50)

//
#ifdef PSYNC_DEBUG_FLAG
    #define DEBUG_PRINT(x)  Serial.println(x)
#else
    #define DEBUG_PRINT(x)
#endif

// ms
#define PS_CTRL_RX_WARN_TIMEOUT (250)

//
#define GET_TIMESTAMP_MS() ((uint32_t) millis())

// Threshhold to detect when a person is pressing accelerator
#define PEDAL_THRESH 1000




// *****************************************************
// static global data
// *****************************************************


//
static uint32_t last_update_ms;

// construct the CAN shield object, Set CS pin for the CAN shield
MCP_CAN CAN( CAN_CS );

//
static can_frame_s rx_frame_ps_ctrl_throttle_command;

//
static can_frame_s tx_frame_ps_ctrl_throttle_report;




// *****************************************************
// static declarations
// *****************************************************


// corrects for overflow condition
static void get_update_time_delta_ms(
		const uint32_t time_in,
		const uint32_t last_update_time_ms,
		uint32_t * const delta_out )
{
    // check for overflow
    if( last_update_time_ms < time_in )
    {
		// time remainder, prior to the overflow
		( *delta_out ) = ( UINT32_MAX - time_in );

        // add time since zero
        ( *delta_out ) += last_update_time_ms;
    }
    else
    {
        // normal delta
        ( *delta_out ) = ( last_update_time_ms - time_in );
    }
}

// uses last_update_ms, corrects for overflow condition
static void get_update_time_ms(
                const uint32_t * const time_in,
                uint32_t * const delta_out )
{
    // check for overflow
    if( last_update_ms < ( *time_in ) )
    {
            // time remainder, prior to the overflow
            ( *delta_out ) = ( UINT32_MAX - ( *time_in ) );

            // add time since zero
            ( *delta_out ) += last_update_ms;
        }
    else
    {
            // normal delta
            ( *delta_out ) = ( last_update_ms - ( *time_in ) );
    }
}

//
static void init_serial( void )
{
    Serial.begin( 115200 );
}

//
static void init_can( void )
{
    // wait until we have initialized
    while( CAN.begin( CAN_BAUD ) != CAN_OK )
    {
        // wait a little
        delay( CAN_INIT_RETRY_DELAY );
    }

    // debug log
    DEBUG_PRINT( "init_obd_can: pass" );

}

// set up pins for interface with the ET-MINI DAV (MCP4922)
// Shutdown
#define SHDN                12

// Load data
#define LDAC                8

// Chip select pin
#define DAC_CS              10

// Power the DAC from the Arduino digital pins
#define DAC_PWR             A5

// Sensor wire from the accelerator sensor, low values
#define PSENS_LOW           A0

// Sensing input for the DAC output
#define PSENS_LOW_SPOOF     A2

// Sensor wire from the accelerator sensor, high values
#define PSENS_HIGH          A1

// Sensing input for the DAC output
#define PSENS_HIGH_SPOOF    A3

// Signal interrupt (relay) for low accelerator values (XXX wire)
#define PSENS_LOW_SIGINT    6

// Signal interrupt (relay) for high accelerator values (XXX wire)
#define PSENS_HIGH_SIGINT   7

// set up values for use in the steering control system
uint16_t PSensL_current,        // Current measured accel sensor values
         PSensH_current,
         PSpoofH,               // Current spoofing values
         PSpoofL;

// CAN message structs
can_frame_s can_frame;

bool control_enable_req,
     control_enabled,
     initial_ADC;

int local_override = 0;

double pedal_position_target,
       pedal_position;

uint8_t incoming_serial_byte;




/* ====================================== */
/* ============== CONTROL =============== */
/* ====================================== */


// a function to set the DAC output registers
void set_DAC( uint16_t data, char channel )
{
    uint8_t message[ 2 ];

    // Set DAC A enable
    if ( channel == 'A' )
    {
        data |= 0x3000;
    }
    // Set DAC B enable
    else if ( channel == 'B' )
    {
        data |= 0xB000;
    }

    // load data into 8 byte payloads
    message[ 0 ] = ( data >> 8 ) & 0xFF;
    message[ 1 ] = data & 0xFF;

    // select the DAC for SPI transfer
    digitalWrite( DAC_CS, LOW );

    // transfer the data payload over SPI
    SPI.transfer( message[ 0 ] );
    SPI.transfer( message[ 1 ] );

    // relinquish SPI control
    digitalWrite( DAC_CS, HIGH );
}

// a function to set the DAC output
void latch_DAC( )
{
    // pulse the LDAC line to send registers to DAC out.
    // must have set DAC registers with set_DAC() first for this to do anything.
    digitalWrite( LDAC, LOW );

    delayMicroseconds( 50 );

    digitalWrite( LDAC, HIGH );
}

// a function to enable TCM to take control
void enable_control( )
{
    // do a quick average to smooth out the noisy data
    static int AVG_max = 20;  // Total number of samples to average over
    long readingsL = 0;
    long readingsH = 0;

    for ( int i = 0; i < AVG_max; i++ )
    {
        readingsL += analogRead( PSENS_LOW ) << 2;
        readingsH += analogRead( PSENS_HIGH ) << 2;
    }

    PSensL_current = readingsL / AVG_max;
    PSensH_current = readingsH / AVG_max;

    // write measured torque values to DAC to avoid a signal discontinuity
    set_DAC( PSensH_current, 'B' );
    set_DAC( PSensL_current, 'A' );
    latch_DAC( );

    // TODO: check if the DAC value and the sensed values are the same.
    // If not, return an error and do NOT enable the sigint relays.

    // enable the signal interrupt relays
    digitalWrite( PSENS_LOW_SIGINT, LOW );
    digitalWrite( PSENS_HIGH_SIGINT, LOW );

    control_enabled = true;

}

// a function to disable TCM control
void disable_control( )
{
    // do a quick average to smooth out the noisy data
    static int AVG_max = 20;  // Total number of samples to average over
    uint16_t readingsL = 0;
    uint16_t readingsH = 0;

    for ( int i = 0; i < AVG_max; i++ ) {
        readingsL += analogRead( PSENS_LOW ) << 2;
        readingsH += analogRead( PSENS_HIGH ) << 2;
    }

    PSensL_current = readingsL / AVG_max;
    PSensH_current = readingsH / AVG_max;

    // write measured torque values to DAC to avoid a signal discontinuity
    set_DAC( PSensH_current, 'B' );
    set_DAC( PSensL_current, 'A' );
    latch_DAC( );


    // disable the signal interrupt relays
    digitalWrite( PSENS_LOW_SIGINT, HIGH );
    digitalWrite( PSENS_HIGH_SIGINT, HIGH );

    control_enabled = false;
}

//
void calculate_pedal_spoof( float pedal_position )
{
    // values calculated with min/max calibration curve and hand tuned for
    // neutral balance.
    // DAC requires 12-bit values, (4096steps/5V = 819.2 steps/V)
    PSpoofL = 819.2 * ( 0.0004 * pedal_position + 0.366 );
    PSpoofH = 819.2 * ( 0.0008 * pedal_position + 0.732 );

    PSpoofL = constrain( PSpoofL, 0, 1800 ); // range = 300 - ~1750
    PSpoofH = constrain( PSpoofH, 0, 3500 ); // range = 600 - ~3500

    //Serial.print("PSPOOF_LOW:");
    //Serial.print(PSpoofL);
    //Serial.print("PSPOOF_LOW");
    //Serial.println(PSpoofH);
}

//
void check_pedal_override( )
{
    if ( ( PSensL_current + PSensH_current) / 2 > PEDAL_THRESH )
    {
        disable_control( );
        local_override = 1;
    }
    else
    {
        local_override = 0;
    }
}

//
void check_spoof_voltage( bool firstADC )
{
    if ( firstADC == true )
    {
        return;
    }

    int psens_l_signal_read = 0;
    int psens_h_signal_read = 0;
    int spoof_l_signal_read = 0;
    int spoof_h_signal_read = 0;

    float psens_l_voltage_read = 0;
    float psens_h_voltage_read = 0;
    float spoof_l_voltage_read = 0;
    float spoof_h_voltage_read = 0;

    // energize the relay so we can read the values at the terminal
    digitalWrite( PSENS_LOW_SIGINT, HIGH );

    psens_l_signal_read = analogRead( PSENS_LOW );
    psens_h_signal_read = analogRead( PSENS_HIGH );
    spoof_l_signal_read = analogRead( PSENS_LOW_SPOOF );
    spoof_h_signal_read = analogRead( PSENS_HIGH_SPOOF );

    psens_l_voltage_read = psens_l_signal_read * 5.0 / 1024.0;
    psens_h_voltage_read = psens_h_signal_read * 5.0 / 1024.0;
    spoof_l_voltage_read = spoof_l_signal_read * 5.0 / 1024.0;
    spoof_h_voltage_read = spoof_h_signal_read * 5.0 / 1024.0;

    Serial.print( "Psens Low Value: " );
    Serial.print( psens_l_signal_read );
    Serial.print( "\tPsens Low Voltage: " );
    Serial.println( psens_l_voltage_read, 3 );

    Serial.print( "Spoof Low Value: " );
    Serial.print( spoof_l_signal_read );
    Serial.print( "\tSpoof Low Voltage: " );
    Serial.println( spoof_l_voltage_read, 3 );

    Serial.print( "Psens High Value: " );
    Serial.print( psens_l_signal_read );
    Serial.print( "\tPsens High Voltage: " );
    Serial.println( psens_l_voltage_read, 3 );

    Serial.print( "Spoof High Value: " );
    Serial.print( spoof_h_signal_read );
    Serial.print( "\tSpoof High Voltage: " );
    Serial.println( spoof_h_voltage_read, 3 );

    //debug signals then writeout fail criteria.
    //disable_control( );
    //local_override = 1;

}




/* ====================================== */
/* =========== COMMUNICATIONS =========== */
/* ====================================== */

// A function to parse incoming serial bytes
void process_serial_byte( )
{
    // accelerate
    if ( incoming_serial_byte == 'a' )
    {
        pedal_position_target += 1000;
    }

    // deaccelerate
    if ( incoming_serial_byte == 'd' )
    {
        pedal_position_target -= 1000;
    }

    // return to center
    if ( incoming_serial_byte == 's' )
    {
        pedal_position_target = 0;
    }

    // enable/disable control
    if ( incoming_serial_byte == 'p' )
    {
        control_enable_req = !control_enable_req;
    }
}

//
static void publish_ps_ctrl_throttle_report( void )
{
    // cast data
    ps_ctrl_throttle_report_msg * const data =
            (ps_ctrl_throttle_report_msg*) tx_frame_ps_ctrl_throttle_report.data;

    // set frame ID
    tx_frame_ps_ctrl_throttle_report.id =
            (uint32_t) (PS_CTRL_MSG_ID_THROTTLE_REPORT);

    // set DLC
    tx_frame_ps_ctrl_throttle_report.dlc = 8; //TODO

    // set override flag (bit 57)
    data->override = local_override;

    //// Set Pedal Command (PC)
    //data->pedal_command =

    //// Set Pedal Output (PO)
    //data->pedal_output = max()

    // publish to control CAN bus
    CAN.sendMsgBuf(
            tx_frame_ps_ctrl_throttle_report.id,
            0, // standard ID (not extended)
            tx_frame_ps_ctrl_throttle_report.dlc,
            tx_frame_ps_ctrl_throttle_report.data );

    // update last publish timestamp, ms
    tx_frame_ps_ctrl_throttle_report.timestamp = last_update_ms;
}

//
static void publish_timed_tx_frames( void )
{
    // local vars
    uint32_t delta = 0;

    // get time since last publish
    get_update_time_ms( &tx_frame_ps_ctrl_throttle_report.timestamp, &delta );

    // check publish interval
    if( delta >= PS_CTRL_THROTTLE_REPORT_PUBLISH_INTERVAL )
    {
        // publish frame, update timestamp
        publish_ps_ctrl_throttle_report( );
    }
}

//
static void process_ps_ctrl_throttle_command(
        const uint8_t * const rx_frame_buffer )
{
    // cast control frame data
    const ps_ctrl_throttle_command_msg * const control_data =
            (ps_ctrl_throttle_command_msg*) rx_frame_buffer;

    bool enabled = control_data->enabled == 1;

    // enable control from the PolySync interface
    if( enabled == 1 && !control_enabled )
    {
        control_enabled = true;
        enable_control( );
    }

    // disable control from the PolySync interface
    if( enabled == 0 && control_enabled )
    {
        control_enabled = false;
        disable_control( );
    }

    rx_frame_ps_ctrl_throttle_command.timestamp = GET_TIMESTAMP_MS( );

    pedal_position_target = control_data->pedal_command / 24;
    DEBUG_PRINT( pedal_position_target );

}

// A function to parse CAN data into useful variables
void handle_ready_rx_frames( void )
{
    // local vars
    can_frame_s rx_frame;

    if( CAN.checkReceive( ) == CAN_MSGAVAIL )
    {
        memset( &rx_frame, 0, sizeof( rx_frame ) );

        // update timestamp
        rx_frame.timestamp = last_update_ms;

        // read frame
        CAN.readMsgBufID(
                ( INT32U* ) &rx_frame.id,
                ( INT8U* ) &rx_frame.dlc,
                ( INT8U* ) rx_frame.data );

        // check for a supported frame ID
        if( rx_frame.id == PS_CTRL_THROTTLE_COMMAND_ID )
        {
            // process status1
            process_ps_ctrl_throttle_command( rx_frame.data );
        }
    }
}

//
static void check_rx_timeouts( void )
{
    // local vars
    uint32_t delta = 0;

    // get time since last receive
    get_update_time_delta_ms(
			rx_frame_ps_ctrl_throttle_command.timestamp,
			GET_TIMESTAMP_MS( ),
			&delta );

    // check rx timeout
    if( delta >= PS_CTRL_RX_WARN_TIMEOUT )
    {
        // disable control from the PolySync interface
        if( control_enabled )
        {
            Serial.println( "control disabled: timeout" );
            disable_control( );
        }
    }
}




/* ====================================== */
/* ================ SETUP =============== */
/* ====================================== */

//
void setup( )
{
    // zero
    last_update_ms = 0;

    memset(
            &rx_frame_ps_ctrl_throttle_command,
            0,
            sizeof( rx_frame_ps_ctrl_throttle_command ) );

    // set up pin modes
    pinMode( SHDN, OUTPUT );
    pinMode( LDAC, OUTPUT );
    pinMode( DAC_CS, OUTPUT );
    pinMode( DAC_PWR, OUTPUT );
    pinMode( PSENS_LOW, INPUT );
    pinMode( PSENS_LOW_SPOOF, INPUT );
    pinMode( PSENS_HIGH, INPUT );
    pinMode( PSENS_HIGH_SPOOF, INPUT );
    pinMode( PSENS_LOW_SIGINT, OUTPUT );
    pinMode( PSENS_HIGH_SIGINT, OUTPUT );

    // initialize the DAC board
    digitalWrite( DAC_PWR, HIGH );    // Supply power
    digitalWrite( DAC_CS, HIGH );     // Deselect DAC CS
    digitalWrite( SHDN, HIGH );       // Turn on the DAC
    digitalWrite( LDAC, HIGH );       // Reset data

    // initialize relay board
    digitalWrite( PSENS_LOW_SIGINT, HIGH );
    digitalWrite( PSENS_HIGH_SIGINT, HIGH );

    init_serial( );

    init_can( );

    publish_ps_ctrl_throttle_report( );

    // update last Rx timestamps so we don't set timeout warnings on start up
    rx_frame_ps_ctrl_throttle_command.timestamp = GET_TIMESTAMP_MS( );

    // update the global system update timestamp, ms
    last_update_ms = GET_TIMESTAMP_MS( );

    // debug log
    DEBUG_PRINT( "init: pass" );

    // skip first iteration of DAC/ADC diagnostic test
    initial_ADC = true;
}




/* ====================================== */
/* ================ LOOP ================ */
/* ====================================== */

//
void loop( )
{
    // update the global system update timestamp, ms
    last_update_ms = GET_TIMESTAMP_MS( );

    handle_ready_rx_frames( );

    publish_timed_tx_frames( );

    check_rx_timeouts( );

    // update state variables
    PSensL_current = analogRead( PSENS_LOW ) << 2;  //10 bit to 12 bit
    PSensH_current = analogRead( PSENS_HIGH ) << 2;

    // if someone is pressing the throttle pedal disable control
    if ( ( PSensL_current + PSensH_current) / 2 > PEDAL_THRESH )
    {
        disable_control( );
        local_override = 1;
    }
    else
    {
        local_override = 0;
    }

    /* Begin Untested */

    check_pedal_override( );

    check_spoof_voltage( initial_ADC );

    /* End Untested */

    // read and parse incoming serial commands
    if ( Serial.available( ) > 0 )
    {
        incoming_serial_byte = Serial.read( );
        process_serial_byte( );
    }

    // now that we've set control status, do throttle if we are in control
    if ( control_enabled )
    {
        calculate_pedal_spoof( pedal_position_target );

        // debug print statements
        //Serial.print("pedal_position_target = ");
        //Serial.print(pedal_position_target);
        //Serial.print(" Spoof error, H = ");
        //Serial.print(PSpoofH - (analogRead(PSENS_HIGH_SPOOF) << 2));
        //Serial.print(" Spoof error L = ");
        //Serial.println(PSpoofL - (analogRead(PSENS_LOW_SPOOF) << 2));

        set_DAC( PSpoofH, 'B' );
        set_DAC( PSpoofL, 'A' );
        latch_DAC( );

        //check_spoof_voltage( initial_ADC );
    }
    initial_ADC = false;
}
