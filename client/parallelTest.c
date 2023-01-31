#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "coroutine.h"
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

// STATES
#define INIT 0
#define GO_TO_BALL 1

#define RIGHT 0
#define LEFT 1

#define L_MOTOR_PORT      OUTPUT_A          /*the ports are defined in the ev3_ports.h file, line 340->*/
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define GRAB_PORT         OUTPUT_C
#define GRAB_EXT_PORT     EXT_PORT__NONE_
#define THROW_PORT        OUTPUT_D
#define THROW_EXT_PORT    EXT_PORT__NONE_

typedef struct {
	uint8_t WHEELS_SN[2];
	uint8_t GRAB;
	uint8_t THROW;
	uint8_t GYRO;
	uint8_t COLOR;
	uint8_t COMPASS;
	uint8_t SONAR;
	uint8_t TOUCH;

	int STATE;
	int SONAR_VAL;

	float compass_offset;
	float gyro_offset;

	//int PORTS[4];
	int MAMBA_MODE;

} Robot;

Robot robot;

int init_robot();
bool _check_pressed( uint8_t sn );
int get_color_method(Robot *robot);
float get_gyro_method(Robot *robot);
float get_sonar_method(Robot *robot);
int get_compass_method(Robot *robot);
int forward(Robot *robot, int distance, int mode);
int rotate(Robot *robot, int side, int degree);
int throw(Robot *robot);
int grab(Robot *robot);
int rotateWithGyro(Robot *robot, int degree);

CORO_CONTEXT( handle_touch );
CORO_CONTEXT( get_sonar );
CORO_CONTEXT( get_color );
CORO_CONTEXT( get_gyro );
CORO_CONTEXT( move );

int main( void )
{
  int i;
  char s[ 256 ];

  init_robot();

  robot.MAMBA_MODE = 1;
  //float value;
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

  //here the code start.
  robot.STATE = GO_TO_BALL;
  rotate(&robot, 1, 90);
  Sleep(3000);
  while(robot.MAMBA_MODE){
        CORO_CALL( handle_touch );
        CORO_CALL( move );
        CORO_CALL( get_sonar );
        Sleep( 10 );
  }
  //code end.

ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}

int init_robot(){
	int val;
	char s[256];
  	uint32_t n, i, ii;

	robot.STATE = INIT;

	robot.WHEELS_SN[0] = DESC_LIMIT;
	robot.WHEELS_SN[1] = DESC_LIMIT;

	/*for(i=0;i<4;i++){
		robot.PORTS[i] = i+65;
	}*/

	printf( "Found tacho motors:\n" );
        for ( i = 0; i < DESC_LIMIT; i++ ) {
	    printf("ABOVE THE IF \n");
       	    if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ ) {
      		printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
      		printf( "  port = %s\n", ev3_tacho_port_name( i, s ));
      		printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
    	    }
  	}
	
	/*if( ev3_search_tacho_plugged_in(robot.PORTS[0], 0, &(robot.WHEELS_SN[0]), 0) ){
		printf("\nLEFT WHEEL INITIALIZED!\n");
	}
	if( ev3_search_tacho_plugged_in(robot.PORTS[1], 0, &(robot.WHEELS_SN[1]), 0) ){
		printf("RIGHT WHEEL INITIALIZED!\n");
	}

	if( ev3_search_tacho_plugged_in(robot.PORTS[2], 0, &(robot.GRAB), 0) ){
		printf("GRAB INITIALIZED!\n");
	}


	if( ev3_search_tacho_plugged_in(robot.PORTS[3], 0, &(robot.THROW), 0) ){
		printf("THROW INITIALIZED!\n");
	}*/

	if( ev3_search_tacho_plugged_in(L_MOTOR_PORT, L_MOTOR_EXT_PORT, &(robot.WHEELS_SN[0]), 0) ){
		printf("\nLEFT WHEEL INITIALIZED!\n");
	}
	if( ev3_search_tacho_plugged_in(R_MOTOR_PORT, R_MOTOR_EXT_PORT, &(robot.WHEELS_SN[1]), 0) ){
		printf("RIGHT WHEEL INITIALIZED!\n");
	}

	if( ev3_search_tacho_plugged_in(GRAB_PORT, GRAB_EXT_PORT, &(robot.GRAB), 0) ){
		printf("GRAB INITIALIZED!\n");
	}

	if( ev3_search_tacho_plugged_in(THROW_PORT, THROW_EXT_PORT, &(robot.THROW), 0) ){
		printf("THROW INITIALIZED!\n");
	}

	float value;
	ev3_sensor_init();

	// COMPASS OFFSET
	//calibrate_compass_sensor(robot->COMPASS);
	float newVal;

    	if (ev3_search_sensor( HT_NXT_COMPASS, &(robot.COMPASS), 0) ){
    		if ( !get_sensor_value0(robot.COMPASS, &value )) {
        		value = 0;
      		}
      		printf( "COMPASS: (%f) \n", value);
    	}

	for( i=0; i<100; i++){

		printf("value: %d %f %f\n", i, newVal, value);

		get_sensor_value0( robot.COMPASS, &newVal);
		if( newVal - value > 1){
			i=0;
		}
	}

	robot.compass_offset = value;

	// GYRO OFFSET

    	if (ev3_search_sensor( LEGO_EV3_GYRO, &(robot.GYRO), 0) ){
    		if ( !get_sensor_value0(robot.GYRO, &value )) {
        		value = 0;
      		}
      		printf( "GYRO: (%f) \n", value);
    	}

	for( i=0; i<50; i++){

		printf("value: %d %f %f\n", i, newVal, value);

		get_sensor_value0( robot.GYRO, &newVal);
		if( newVal - value > 1){
			i=0;
		}

	}

	robot.gyro_offset = value;

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
	
	if (ev3_search_sensor(LEGO_EV3_US, &( robot.SONAR),0)){
                if ( !get_sensor_value0(robot.SONAR, &value )) {
                        value = 0;
                }
        }

	return 0;
}

CORO_DEFINE( handle_touch )
{
    CORO_LOCAL int val;
    CORO_BEGIN();
    if ( robot.TOUCH == DESC_LIMIT ) CORO_QUIT();
    for ( ; ; ) {
        /* Waiting the button is pressed */
        CORO_WAIT( get_sensor_value( 0, robot.TOUCH, &val ) && ( val ));
        /* Stop the vehicle */
        //command_drive = MOVE_NONE;
        /* Switch mode */
        //_set_mode(( mode == MODE_REMOTE ) ? MODE_AUTO : MODE_REMOTE );
        robot.MAMBA_MODE = 0;
        /* Waiting the button is released */
        CORO_WAIT( get_sensor_value( 0, robot.TOUCH, &val ) && ( !val ));
    }
    CORO_END();
}


CORO_DEFINE( get_sonar )
{
    CORO_LOCAL int pos;
    CORO_BEGIN();
    for ( ; ; ) {
        /* Waiting self-driving mode */
        //CORO_WAIT( mode == MODE_GYRO_TURN );
        get_sensor_value( 0, robot.SONAR, &pos );
        robot.SONAR_VAL = pos;

        CORO_YIELD();
    }
    CORO_END();
}


CORO_DEFINE( move )
{
    CORO_BEGIN();
    for ( ; ; ) {
        /* Waiting self-driving mode */
        //CORO_WAIT( mode == MODE_GYRO_TURN );
        switch(robot.STATE){
		case GO_TO_BALL:
			printf("Reading sonar val %d \n", robot.SONAR_VAL);
			if(robot.SONAR_VAL < 130){ 
				grab(&robot);
				Sleep(3000);
			}
			break;
	}

        CORO_YIELD();
    }
    CORO_END();
}

// GYRO
float get_gyro_method(Robot *robot){

	float value;

	if(ev3_search_sensor(LEGO_EV3_GYRO, &(robot->GYRO), 0)){
      		if ( !get_sensor_value0(robot->GYRO, &value )) {
        		value = 0;
      		}
	}
	value = value - robot->gyro_offset;
      	printf( "GYRO: (%f) \n", value);
	return value;
}

// COMPASS
int get_compass_method(Robot *robot){

    float value;

    if (ev3_search_sensor( HT_NXT_COMPASS, &(robot->COMPASS), 0) ){
    	if ( !get_sensor_value0(robot->COMPASS, &value )) {
        value = 0;
      }
      printf( "COMPASS: (%f) \n", value);
    }

    return 0;

}


// SONAR
float get_sonar_method(Robot *robot){

	float value;

	if (ev3_search_sensor(LEGO_EV3_US, &( robot->SONAR),0)){
      		if ( !get_sensor_value0(robot->SONAR, &value )) {
        		value = 0;
      		}
    	}

	return value;

}



// COLOR
int get_color_method(Robot *robot){

	int value;

	if(ev3_search_sensor(LEGO_EV3_COLOR, &(robot->COLOR), 0)){
      		if ( !get_sensor_value(0, robot->COLOR, &value )) {
        		value = 0;
      		}
	}
      	printf( "COLOR: (%d) \n", value);

	return value;
}


int rotateWithGyro(Robot *robot, int gyro_pos){
	float gyro_initial, gyro_final;
	gyro_initial = get_gyro_method(robot);
	int side;

	if(gyro_pos < gyro_initial){
		side = LEFT;
	}
	else{
		side = RIGHT;
	}

	float abs = gyro_pos - gyro_initial;
	if(abs < 0){
		abs = -1*abs;
	}

	printf("GYRO INITIAL: %f\n", gyro_initial);

	rotate(robot, side, abs);
	Sleep(4000);
	gyro_final = get_gyro_method(robot);

	printf("GYRO FINAL: %f\n", gyro_final);

	printf("GYRO DIFF: %f\n", abs);

	while(gyro_pos - gyro_final > 2){


		if(gyro_pos < gyro_final){
			side = LEFT;
		}
		else{
			side = RIGHT;
		}

		abs = gyro_pos - gyro_final;
		if(abs < 0){
			abs = -1*abs;
		}

		printf("GYRO DIFF: %f\n", abs);
		rotate(robot, side, abs);
		Sleep(3000);
		gyro_final = get_gyro_method(robot);
	}

	return 0;
}



// ---------------------- BASIC 4 OPERATIONS (FORWARD, ROTATE, THROW, GRAB) -------------------------------

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
	//set_tacho_command_inx( robot->GRAB, TACHO_STOP);

	return 0;
}

// THROW
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
	multi_set_tacho_position( robot->WHEELS_SN, 0);
	set_tacho_position_sp( rot_fd, dest_pos);
	set_tacho_position_sp( rot_bk, -dest_pos);

 	multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS);

	return 0;
}

// FORWARD
void forward_rel_pos(Robot *robot, int dest_pos, int max_speed){

	set_tacho_speed_sp( robot->WHEELS_SN[0], max_speed );
    set_tacho_speed_sp( robot->WHEELS_SN[1], max_speed );
    multi_set_tacho_position( robot->WHEELS_SN, 0);
	multi_set_tacho_position_sp( robot->WHEELS_SN, dest_pos);

	multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS );
}

//    ------------------------ END OF PROGRAM --------------------------

