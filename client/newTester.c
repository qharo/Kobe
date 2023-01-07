#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif
const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

static bool _check_pressed( uint8_t sn )
{
  int val;

  if ( sn == SENSOR__NONE_ ) {
    return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
  }
  return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}


int rotate(int degree){
	printf("%d", degree);
	return 0;
}

int position(uint8_t motor){
	printf("VALUE HERE: %d", motor);
	int pos;
	get_tacho_position(motor, &pos);
	printf("\nPosition is %d\n", pos);
	return 0;	
}

// MOVE FORWARD FUNCTION
int forward(int distance){	
	int dest_pos = distance*20;
	uint8_t WHEELS[2];
	int RIGHT_WHEEL_PORT = 65, LEFT_WHEEL_PORT = 66;
	if( ev3_search_tacho_plugged_in(RIGHT_WHEEL_PORT, 0, &WHEELS[0], 0) ){
		if( ev3_search_tacho_plugged_in(LEFT_WHEEL_PORT, 0, &WHEELS[1], 0) ){
			int max_speed;

			get_tacho_max_speed( WHEELS[0], &max_speed);
			printf("Max Speed is: %d", max_speed);			
			// STANDARD PROCEDURE 
			set_tacho_speed_sp( WHEELS[0], dest_pos/2 );
			//set_tacho_time_sp( WHEELS[0], time );
      			set_tacho_ramp_up_sp( WHEELS[0], 2 );
      			set_tacho_ramp_down_sp( WHEELS[0], 2 );
			set_tacho_speed_sp( WHEELS[1], dest_pos/2 );
			//set_tacho_time_sp( WHEELS[1], time );
      			set_tacho_ramp_up_sp( WHEELS[1], 2 );
      			set_tacho_ramp_down_sp( WHEELS[1], 2 );
			multi_set_tacho_position( WHEELS, 0);
			Sleep(1000);
			//printf("C%d", dest_pos);
			position(WHEELS[0]);
			multi_set_tacho_position_sp(WHEELS, dest_pos);
			Sleep(1000);
			multi_set_tacho_command_inx( WHEELS, TACHO_RUN_TO_ABS_POS);
			position(WHEELS[0]);
			printf("value now: %d", WHEELS[0]);
		//Sleep(time+20);
		}
	}
	return 0;	
}




int main( void )
{
  int i;
  uint8_t sn;
  uint8_t sn1;
  uint8_t sn2;
  FLAGS_T state;
  uint8_t sn_touch;
  uint8_t sn_color;
  uint8_t sn_compass;
  uint8_t sn_sonar;
  uint8_t sn_mag;
  char s[ 256 ];
  int val;
  float value;
  uint32_t n, ii;
#ifndef __ARM_ARCH_4T__
  /* Disable auto-detection of the brick (you have to set the correct address below) */
  ev3_brick_addr = "192.168.0.204";

#endif
  if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
  printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
  printf( "Waiting tacho is plugged...\n" );

#endif
  while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

  printf( "*** ( EV3 ) Hello! ***\n" );

  printf( "Found tacho motors:\n" );
  for ( i = 0; i < DESC_LIMIT; i++ ) {
    if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ ) {
      printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
      printf( "  port = %s\n", ev3_tacho_port_name( i, s ));
      printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
    }
  }
 
 //set_tacho_position(sn, 0);
 forward(40);
 //printf("WAITING...");
 //position();
 //Sleep(10000);
// position();

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}
