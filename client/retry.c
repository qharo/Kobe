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


// BOOL ///////////////////////////////////
#define TRUE 1
#define FALSE 0
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
	uint8_t THROW;
	uint8_t GYRO;
	uint8_t COLOR;
	uint8_t COMPASS;
	uint8_t SONAR;
	uint8_t TOUCH;

	int SEEK_MODE;
	int GYRO_OBS[10000];
	float SONAR_OBS[10000];
	int NOBS;

	int MAMBA_MODE;
	int STATE;
	float SONAR_VAL, COLOR_VAL;
	float GYRO_VAL;

	float compass_offset;	
	float gyro_offset;

	int PORTS[4];

} Robot;

Robot robot;



//CORO_CONTEXT_INIT( handle_touch );
//CORO_CONTEXT_INIT( move );
//CORO_CONTEXT_INIT( fetch_gyro );
//CORO_CONTEXT_INIT( get_color );
//CORO_CONTEXT_INIT( fetch_sonar );


CORO_CONTEXT( handle_touch );
CORO_CONTEXT( move );
CORO_CONTEXT( fetch_gyro );
//CORO_CONTEXT( get_color );
CORO_CONTEXT( fetch_sonar );

int init_robot();
void stop();
void run_forever( int speed );
bool _check_pressed( uint8_t sn );
int get_color(Robot *robot);
float get_gyro(Robot *robot);
float get_sonar(Robot *robot);
int get_compass(Robot *robot);
int forward(Robot *robot, int distance);	
int rotate(Robot *robot, int side, int degree);
int throw(Robot *robot);
int grab(Robot *robot);
void rotate_until(int side);
//int rotateWithGyro(Robot *robot, int degree);

// MAIN
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

  robot.SEEK_MODE = FALSE;
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


// INIT_ROBOT
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


	if( ev3_search_tacho_plugged_in(robot.PORTS[3], 0, &(robot.THROW), 0) ){
		printf("THROW INITIALIZED!\n");
	}

	float value;

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


	// ----------- COMPASS OFFSET ----------
	//calibrate_compass_sensor(robot->COMPASS);
	float newVal;

	printf("Entering Sonar Initialization!\n");	
	
    	if (ev3_search_sensor( HT_NXT_COMPASS, &(robot.COMPASS), 0) ){
    		if ( !get_sensor_value0(robot.COMPASS, &value )) {
        		value = 0;
      		}	
    	}

	for( i=0; i<100; i++){
		get_sensor_value0( robot.COMPASS, &newVal);
		if( newVal - value > 1){
			i=0;
		}
	}

	robot.compass_offset = value;

	//	printf("Sonar Initialized! Sonar Off\n");	

	// --------------- GYRO OFFSET ---------------

	printf("Entering Gyro Initialization!\n");	
    	if (ev3_search_sensor( LEGO_EV3_GYRO, &(robot.GYRO), 0) ){
    		if ( !get_sensor_value0(robot.GYRO, &value )) {
        		value = 0;
      		}	
    	}


	for( i=0; i<50; i++){
		get_sensor_value0( robot.GYRO, &newVal);
		if( newVal - value > 1){
			i=0;
		}

	}

	robot.gyro_offset = value;

	printf("Gyro initialized successfully! Gyro Offset Value: %f\n", robot.gyro_offset);

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
	

	/*if (ev3_search_sensor(LEGO_EV3_US, &( robot.COMPASS),0)){
      		robot.SONAR_VAL = 0;
    	}*/
	

	return 0;
}


// HANDLE_TOUCH
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

// FETCHSONAR
CORO_DEFINE( fetch_sonar ){
	
	CORO_LOCAL float value;
	CORO_LOCAL int counter=0;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.SONAR, &value );
		printf("Value from FETCH_SONAR: %f\n", value);
		robot.SONAR_VAL = value;
		if(robot.SEEK_MODE){
			robot.SONAR_OBS[counter] = robot.SONAR_VAL;
			counter++;
			robot.NOBS = counter;
			
		}
		CORO_YIELD();
	}
		
	CORO_END();
}


// FETCH_GYRO
CORO_DEFINE( fetch_gyro ){
	
	CORO_LOCAL float value;
	CORO_LOCAL int counter = 0;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.GYRO, &value );
		robot.GYRO_VAL = value-robot.gyro_offset;
		if(robot.SEEK_MODE){
			robot.GYRO_OBS[counter] = robot.GYRO_VAL;
			counter++;
		}
		CORO_YIELD();
	}
	CORO_END();
}


// MOVE
CORO_DEFINE( move ){

	CORO_LOCAL int max_speed, init_angle, curr_angle, ball_angle, ball_distance, i;	
	CORO_LOCAL int PRE_SCAN, SCANNING, POST_SCAN;
	CORO_LOCAL float OBVS[1000];
	
	CORO_LOCAL int low=0, high;

	CORO_BEGIN();

	init_angle = robot.GYRO_VAL;

	PRE_SCAN = TRUE;
	SCANNING = FALSE;
	POST_SCAN = FALSE;
	
	for(;;){
		// CHECK STATUS			
		if(robot.STATE == GO_TO_BALL){
		 	get_tacho_max_speed(robot.WHEELS_SN[0], &max_speed);
			run_forever(-max_speed/4); 
			//while(init_angle - curr_angle > 2){
			//	rotate(&robot, RIGHT, init_angle-curr_angle);
			//	Sleep(3000);
			//	curr_angle = robot.GYRO_VAL;
			//}
			//while(init_angle - curr_angle < -2){
			//	rotate(&robot, LEFT, curr_angle-init_angle);
			//	Sleep(3000);
			//	curr_angle = robot.GYRO_VAL;
			//}
			printf("Value of US: %f\n", robot.SONAR_VAL);
			if(robot.SONAR_VAL >= 800){
				 stop();
			}
		}

		// BALLDETECTMODE
		if(robot.STATE == BALL_DETECT){
			// PRE-SCAN
			if(PRE_SCAN){
				printf("Pre-Scanning!\n");
				rotate_until(LEFT);
				if(robot.GYRO_VAL <= init_angle - 15){
					//tate_until(RIGHT);
					//(robot.GYRO_VAL >= init_angle);
					stop();
					PRE_SCAN = FALSE;
					SCANNING = TRUE;
					robot.SEEK_MODE = TRUE;
				}
			}
			
			// SCAN
			if(SCANNING){
				printf("Scanning!\n");
				rotate_until(RIGHT);
				if(robot.GYRO_VAL >= init_angle + 15){
					stop();
					SCANNING = FALSE;
					POST_SCAN = TRUE;
					robot.SEEK_MODE = FALSE;
				}
			}
			
			//POSTSCAN	
			if(POST_SCAN){
				printf("Post-Scanning!\n");
				rotate_until(LEFT);
				if(robot.GYRO_VAL <= init_angle){
					stop();
					POST_SCAN = FALSE;
					high = robot.NOBS;
				}

				for(i = 0; i < robot.NOBS-1; i++){
                                	OBVS[i] = robot.SONAR_OBS[i] - robot.SONAR_OBS[i+1];
					//printf("Observation %d is %f\n", i, OBVS[i]);
                        	}

				detection(OBVS, robot.NOBS);

				for(i=0; i < 0;i++){
                                	if(robot.SONAR_OBS[i] > -50 && robot.SONAR_OBS[i] < -40 ){
                                        	printf("%d is the start of the ball\n", i);
                                	}
                        	}
				
			}

		}

		CORO_YIELD();
	}
	CORO_END();
}

//DETECTION
int detection(float obvs[], int n){

	int i;

	for(i = 0; i < n; i++){

		printf("%f is the %d value, angle is %d\n", robot.SONAR_OBS[i], i, robot.GYRO_OBS[i]);
	
	}

	return 0;

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
	multi_set_tacho_ramp_up_sp( robot->WHEELS_SN, 200 );
      	multi_set_tacho_ramp_down_sp( robot->WHEELS_SN, 200 );
	multi_set_tacho_position( robot->WHEELS_SN, 0);
	set_tacho_position_sp( rot_fd, dest_pos);
	set_tacho_position_sp( rot_bk, -dest_pos);
    	set_tacho_command_inx( robot->WHEELS_SN[0], TACHO_RUN_TO_REL_POS);	
    	set_tacho_command_inx( robot->WHEELS_SN[1], TACHO_RUN_TO_REL_POS);
   
 	//multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS);
	//Sleep(4000);

	return 0;
}

// ROTATEUNTIL
void rotate_until(int side){
    uint8_t rot_fd = robot.WHEELS_SN[side];
	uint8_t rot_bk = robot.WHEELS_SN[(side+1)%2];

	set_tacho_speed_sp( rot_fd, 30);
	set_tacho_speed_sp( rot_bk, -30 );

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
	
