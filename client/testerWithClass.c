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

// 0/65 - LEFT WHEEL
// 1/66 - RIGHT WHEEL

typedef struct {
	uint8_t WHEELS_SN[2];
	uint8_t GRAB;
	uint8_t THROW;
	uint8_t GYRO;
	uint8_t COLOR;

	int PORTS[4];

} Robot;

int init_robot(Robot *robot){
	int i, val;
  	uint32_t n, ii;

	for(i=0;i<4;i++){
		robot->PORTS[i] = i+65;
	}

	if( ev3_search_tacho_plugged_in(robot->PORTS[0], 0, &(robot->WHEELS_SN[0]), 0) ){
		printf("\nLEFT WHEEL INITIALIZED!\n");
	}		
	if( ev3_search_tacho_plugged_in(robot->PORTS[1], 0, &(robot->WHEELS_SN[1]), 0) ){
		printf("RIGHT WHEEL INITIALIZED!\n");
	}

	if( ev3_search_tacho_plugged_in(robot->PORTS[2], 0, &(robot->GRAB), 0) ){
		printf("GRAB INITIALIZED!\n");
	}		
	if( ev3_search_tacho_plugged_in(robot->PORTS[3], 0, &(robot->THROW), 0) ){
		printf("THROW INITIALIZED!\n");
	}

	float value;
	ev3_sensor_init();
	char s[256];

  	printf( "Found sensors:\n" );
  	for ( i = 0; i < DESC_LIMIT; i++ ) {
    	if ( ev3_sensor[ i ].type_inx != SENSOR_TYPE__NONE_ ) {
      		printf( "  type = %s\n", ev3_sensor_type( ev3_sensor[ i ].type_inx ));
      		printf( "  port = %s\n", ev3_sensor_port_name( i, s ));
      		if ( get_sensor_mode( i, s, sizeof( s ))) {
        		printf( "  mode = %s\n", s );
      		}
      		if ( get_sensor_num_values( i, &n )) {
        		for ( ii = 0; ii < n; ii++ ) {
          			if ( get_sensor_value( ii, i, &val )) {
            				printf( "  value%d = %d\n", ii, val );
          			}	
        		}
      		}
    	}}

	return 0;
}

bool _check_pressed( uint8_t sn )
{
  int val;

  if ( sn == SENSOR__NONE_ ) {
    return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
  }
  return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}

// GYRO
int get_gyro(Robot *robot){

	float value;	
	
	if(ev3_search_sensor(NXT_ANALOG, &(robot->GYRO), 0)){
      		if ( !get_sensor_value0(robot->GYRO, &value )) {
        		value = 0;
      		}
	}
      	printf( "GYRO: (%f) \n", value);
	return 0;
}

int get_color(Robot *robot){

	int value;	
	
	if(ev3_search_sensor(LEGO_EV3_COLOR, &(robot->COLOR), 0)){
      		if ( !get_sensor_value(0, robot->COLOR, &value )) {
        		value = 0;
      		}
	}
      	printf( "COLOR: (%d) \n", value);

	return value;
}


// GRAB
int grab(Robot *robot){

	int max_speed;

        get_tacho_max_speed( robot->GRAB, &max_speed );

	printf("MAX SPEED OF GRAB: %d", max_speed);
	
	set_tacho_speed_sp( robot->GRAB, -max_speed/2 );
      	set_tacho_ramp_up_sp( robot->GRAB, 1000 );
      	set_tacho_ramp_down_sp( robot->GRAB, 1000 );
	set_tacho_position( robot->GRAB, 0);
	set_tacho_position_sp( robot->GRAB, -1080);
    	set_tacho_command_inx( robot->GRAB, TACHO_RUN_TO_REL_POS);
	Sleep(3000);
	set_tacho_command_inx( robot->GRAB, TACHO_STOP);	

	return 0;	
}

int throw(Robot *robot){

	int max_speed;

        get_tacho_max_speed( robot->THROW, &max_speed );
	
	printf("MAX SPEED OF THROW: %d", max_speed);
	
	set_tacho_speed_sp( robot->THROW, max_speed );
      	set_tacho_ramp_up_sp( robot->THROW, 90 );
      	set_tacho_ramp_down_sp( robot->THROW, 90 );
	set_tacho_position( robot->THROW, 0);
	set_tacho_position_sp( robot->THROW, 240);
    	set_tacho_command_inx( robot->THROW, TACHO_RUN_TO_REL_POS);
	Sleep(1000);
	
	set_tacho_speed_sp( robot->THROW, -max_speed );
      	set_tacho_ramp_up_sp( robot->THROW, 90 );
      	set_tacho_ramp_down_sp( robot->THROW, 90 );
	set_tacho_position( robot->THROW, 0);
	set_tacho_position_sp( robot->THROW, -240);
    	set_tacho_command_inx( robot->THROW, TACHO_RUN_TO_REL_POS);
	Sleep(1000);
	set_tacho_command_inx( robot->THROW, TACHO_STOP);	

	return 0;

}


// ROTATE
int rotate(Robot *robot, int side, int degree){
	printf("%d degrees! \n", degree);

	uint8_t rot_fd = robot->WHEELS_SN[side];
	uint8_t rot_bk = robot->WHEELS_SN[(side+1)%2];

	//printf("Dest_Pos is: %f", 4*31.4**(degree/360)); 
	
	int dest_pos = 4*31.4159*9.35*(degree/360.0);
	printf("DEST_POS IS: %d\n", dest_pos);	

	set_tacho_speed_sp( rot_fd, dest_pos/4 );      	
	set_tacho_speed_sp( rot_bk, -dest_pos/4 );
	multi_set_tacho_ramp_up_sp( robot->WHEELS_SN, 200 );
      	multi_set_tacho_ramp_down_sp( robot->WHEELS_SN, 200 );
	multi_set_tacho_position( robot->WHEELS_SN, 0);
	set_tacho_position_sp( rot_fd, dest_pos);
	set_tacho_position_sp( rot_bk, -dest_pos);
    	set_tacho_command_inx( robot->WHEELS_SN[0], TACHO_RUN_TO_REL_POS);	
    	set_tacho_command_inx( robot->WHEELS_SN[1], TACHO_RUN_TO_REL_POS);
   
 	//multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS);
	Sleep(4000);

	return 0;
}

int position(uint8_t motor){
	printf("VALUE HERE: %d", motor);
	int pos;
	get_tacho_position(motor, &pos);
	printf("\nPosition is %d\n", pos);
	return 0;	
}

// MOVE FORWARD 
int forward(Robot *robot, int distance){	
	int dest_pos = distance*20;
	int sign;

	if(distance < 0){
		sign = 1;
	}
	else {
		sign = 1;
	}
	
	set_tacho_speed_sp( robot->WHEELS_SN[0], sign*dest_pos/2 );
      	set_tacho_ramp_up_sp( robot->WHEELS_SN[0], 2 );
      	set_tacho_ramp_down_sp( robot->WHEELS_SN[0], 2 );
	set_tacho_speed_sp( robot->WHEELS_SN[1], sign*dest_pos/2 );
      	set_tacho_ramp_up_sp( robot->WHEELS_SN[1], 2 );
      	set_tacho_ramp_down_sp( robot->WHEELS_SN[1], 2 );
	multi_set_tacho_position( robot->WHEELS_SN, 0);
	multi_set_tacho_position_sp( robot->WHEELS_SN, sign*dest_pos);

    	set_tacho_command_inx( robot->WHEELS_SN[0], TACHO_RUN_TO_REL_POS);	
    	set_tacho_command_inx( robot->WHEELS_SN[1], TACHO_RUN_TO_REL_POS);

	//multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS );		
	//Sleep(4000); 	

	return 0;	
}




int main( void )
{
  int i;
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
 
  Robot robot;
  init_robot(&robot);


 // forward(&robot, -20);

  int colorVal;

  for( i=0;i<0;i++){
	colorVal = get_color(&robot);
	if(colorVal == 1){	
    		set_tacho_command_inx( robot.WHEELS_SN[0], TACHO_STOP);	
    		set_tacho_command_inx( robot.WHEELS_SN[1], TACHO_STOP);
		forward(&robot, 5);
		Sleep(2000);
		rotate(&robot, 0, 170);
		Sleep(2000);
		forward(&robot, 15);
		grab(&robot);
		throw(&robot);
	}
  }

  //forward(&robot, 100);
  rotate(&robot, 0, 180);
 // forward(&robot, 28);
  //grab(&robot);
  rotate(&robot, 0, 360);
  //throw(&robot);

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}
