#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "coroutine.h"


// STATES //////////////////////////////////
#define INIT 0
#define GO_TO_BALL 1
#define BALL_DETECT 2
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

// ROTATION DIRECTIONS /////////////////////
#define RIGHT 0
#define LEFT 1
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


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
	uint8_t SHOOT;
	uint8_t GYRO;
	uint8_t COLOR;
	uint8_t COMPASS;
	uint8_t SONAR;
	uint8_t TOUCH;

	int MAMBA_MODE;
	int STATE;
	int SONAR_VAL, GYRO_VAL, COLOR_VAL;

	float compass_offset;
	float gyro_offset;

	int PORTS[4];

} Robot;

Robot robot;

CORO_CONTEXT( handle_touch );
CORO_CONTEXT( move );
CORO_CONTEXT( fetch_gyro );
//CORO_CONTEXT( get_color );
CORO_CONTEXT( fetch_sonar );

int init_robot();
void stop();
void run_forever( int speed );
void rotate_until( int side ) ;
int forward(int distance);
int shoot();
int grab();
int rotateWithGyro(int degree);

int main( void )
{
  int i;
  char s[ 256 ];
  float value;
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


  init_robot();

// ------------- CODE BEGINS HERE ------------------

  robot.STATE = BALL_DETECT;
  while(robot.MAMBA_MODE){
	CORO_CALL( fetch_sonar );
	CORO_CALL( fetch_gyro );
	CORO_CALL( move );
	CORO_CALL( handle_touch );
	Sleep(10);
  }

// ------------- CODE ENDS HERE -------------------

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}

int init_robot(){
	int i, val;
  	uint32_t n, ii;

	robot.MAMBA_MODE = 1;

	for(i=0;i<4;i++){
		robot.PORTS[i] = i+65;
	}

	if( ev3_search_tacho_plugged_in(robot.PORTS[0], 0, &(robot.WHEELS_SN[0]), 0) ){
		printf("\nLEFT WHEEL INITIALIZED!\n");
	}
	if( ev3_search_tacho_plugged_in(robot.PORTS[1], 0, &(robot.WHEELS_SN[1]), 0) ){
		printf("RIGHT WHEEL INITIALIZED!\n");
	}

	if( ev3_search_tacho_plugged_in(robot.PORTS[2], 0, &(robot.GRAB), 0) ){
		printf("GRAB INITIALIZED!\n");
	}


	if( ev3_search_tacho_plugged_in(robot.PORTS[3], 0, &(robot.SHOOT), 0) ){
		printf("SHOOT INITIALIZED!\n");
	}



	// --------- SENSOR INIT -------------
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

    // --------------- GYRO OFFSET ---------------
	float value;
	float newVal;

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

	// SEARCHING FOR SONAR
	if (ev3_search_sensor(LEGO_EV3_US, &( robot.SONAR),0)){
      		printf("Sonar found! \n");
		robot.SONAR_VAL = 0;
    	}

	// SEARCHING FOR TOUCH
	if (ev3_search_sensor(LEGO_EV3_TOUCH, &( robot.TOUCH ), 0)){
      		printf("Touch found! \n");
    	}

	// SEARCHING FOR GYRO
	if (ev3_search_sensor(LEGO_EV3_GYRO, &( robot.GYRO),0)){
      		printf("Gyro found! \n");
		robot.GYRO_VAL = 0;
    	}

	return 0;
}


// HANDLE_TOUCH, for exiting the program
CORO_DEFINE( handle_touch ){
	CORO_LOCAL int val;
	CORO_BEGIN();

	if ( robot.TOUCH == DESC_LIMIT) CORO_QUIT();

	for(;;){
		CORO_WAIT( get_sensor_value( 0, robot.TOUCH, &val) && (val));

		robot.MAMBA_MODE = 0;
		stop();
		CORO_WAIT( get_sensor_value( 0, robot.TOUCH, &val) && (!val));

	}
	CORO_END();
}

// FETCH_SONAR, updating the sonar values in realtime
CORO_DEFINE( fetch_sonar ){

	CORO_LOCAL float value;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.SONAR, &value );
		robot.SONAR_VAL = value;
		CORO_YIELD();
	}
	CORO_END();
}


// FETCH_GYRO, updating the gyro values in realtime
CORO_DEFINE( fetch_gyro ){

	CORO_LOCAL float value;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.GYRO, &value );
		robot.GYRO_VAL = value-robot.gyro_offset;
		CORO_YIELD();
	}
	CORO_END();
}


// MOVE, here we define our strategy.
CORO_DEFINE( move ){

	CORO_LOCAL int max_speed, init_angle, curr_angle;

	CORO_BEGIN();

	init_angle = robot.GYRO_VAL;

	for(;;){
		// CHECK STATUS
		if(robot.STATE == GO_TO_BALL){
		 	/*get_tacho_max_speed(robot.WHEELS_SN[0], &max_speed);
			run_forever(-max_speed/4);
			while(init_angle - curr_angle > 2){
				rotate(&robot, RIGHT, init_angle-curr_angle);
				Sleep(3000);
				curr_angle = robot.GYRO_VAL;
			}
			while(init_angle - curr_angle < -2){
				rotate(&robot, LEFT, curr_angle-init_angle);
				Sleep(3000);
				curr_angle = robot.GYRO_VAL;
			}
			if(robot.SONAR_VAL >= 800){
				 stop();

			}*/

		}

		if(robot.STATE == BALL_DETECT){
			stop();
            		//rotateWithGyro(-90);
		}

		CORO_YIELD();
	}
	CORO_END();
}


// GRAB
int grab(){

	int max_speed;

    get_tacho_max_speed( robot.GRAB, &max_speed );

	printf("MAX SPEED OF GRAB: %d", max_speed);

	set_tacho_speed_sp( robot.GRAB, -max_speed/2 );
    set_tacho_ramp_up_sp( robot.GRAB, 1000 );
    set_tacho_ramp_down_sp( robot.GRAB, 1000 );
	set_tacho_position( robot.GRAB, 0);
	set_tacho_position_sp( robot.GRAB, -1080);
    set_tacho_command_inx( robot.GRAB, TACHO_RUN_TO_REL_POS);
	Sleep(3000);
	set_tacho_command_inx( robot.GRAB, TACHO_STOP);

	return 0;
}

// SHOOT
int shoot(){

	int max_speed;

    get_tacho_max_speed( robot.SHOOT, &max_speed );

	printf("MAX SPEED OF SHOOT: %d", max_speed);

    //shoot the ball
	set_tacho_speed_sp( robot.SHOOT, max_speed );
    set_tacho_ramp_up_sp( robot.SHOOT, 90 );
    set_tacho_ramp_down_sp( robot.SHOOT, 90 );
	set_tacho_position( robot.SHOOT, 0);
	set_tacho_position_sp( robot.SHOOT, 240);
    set_tacho_command_inx( robot.SHOOT, TACHO_RUN_TO_REL_POS);
	Sleep(1000);

    //arm go back to starting position.
	set_tacho_speed_sp( robot.SHOOT, -max_speed );
    set_tacho_ramp_up_sp( robot.SHOOT, 90 );
    set_tacho_ramp_down_sp( robot.SHOOT, 90 );
	set_tacho_position( robot.SHOOT, 0);
	set_tacho_position_sp( robot.SHOOT, -240);
    set_tacho_command_inx( robot.SHOOT, TACHO_RUN_TO_REL_POS);
	Sleep(1000);
	set_tacho_command_inx( robot.SHOOT, TACHO_STOP);

	return 0;

}

int rotateWithGyro(int gyro_pos){
    float gyro_initial;
    int side;
    int rotating = 1;
    gyro_initial = robot.GYRO_VAL;

    if(gyro_pos < gyro_initial){
		side = LEFT;
	}
	else{
		side = RIGHT;
	}
	rotate_until(side);
    while(rotating){
        printf("gyro rotate vale: %d\n", robot.GYRO_VAL);
        if (robot.GYRO_VAL>=gyro_pos && side == RIGHT){
            stop();
            rotating=0;
        }else if (robot.GYRO_VAL<=gyro_pos && side == LEFT){
            stop();
            rotating=0;
        }
    }

	return 0;
}

void rotate_until(int side){
    uint8_t rot_fd = robot.WHEELS_SN[side];
	uint8_t rot_bk = robot.WHEELS_SN[(side+1)%2];

	set_tacho_speed_sp( rot_fd, 50 );
	set_tacho_speed_sp( rot_bk, 50 );

	set_tacho_command_inx(robot.WHEELS_SN[0], TACHO_RUN_FOREVER);
	set_tacho_command_inx(robot.WHEELS_SN[1], TACHO_RUN_FOREVER);
}

// RUN FOREVER
void run_forever(int speed){
	set_tacho_speed_sp(robot.WHEELS_SN[0], speed);
	set_tacho_speed_sp(robot.WHEELS_SN[1], speed);

	set_tacho_command_inx(robot.WHEELS_SN[0], TACHO_RUN_FOREVER);
	set_tacho_command_inx(robot.WHEELS_SN[1], TACHO_RUN_FOREVER);

}
// STOP
void stop( void ){

	multi_set_tacho_command_inx(robot.WHEELS_SN, TACHO_STOP);

}


// MOVE FORWARD
int forward(int distance){
	int dest_pos = distance*20;
	int sign;

	if(distance < 0){
		sign = 1;
	}
	else {
		sign = 1;
	}

	set_tacho_speed_sp( robot.WHEELS_SN[0], sign*dest_pos/2 );
    set_tacho_ramp_up_sp( robot.WHEELS_SN[0], 2 );
    set_tacho_ramp_down_sp( robot.WHEELS_SN[0], 2 );
	set_tacho_speed_sp( robot.WHEELS_SN[1], sign*dest_pos/2 );
    set_tacho_ramp_up_sp( robot.WHEELS_SN[1], 2 );
    set_tacho_ramp_down_sp( robot.WHEELS_SN[1], 2 );
	multi_set_tacho_position( robot.WHEELS_SN, 0);
	multi_set_tacho_position_sp( robot.WHEELS_SN, sign*dest_pos);

    set_tacho_command_inx( robot.WHEELS_SN[0], TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx( robot.WHEELS_SN[1], TACHO_RUN_TO_REL_POS);

	//multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS );
	//Sleep(4000);

	return 0;
}

