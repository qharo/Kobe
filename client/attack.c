#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "coroutine.h"


// STATES //////////////////////////////////
#define BALL0 0
#define BALL1 1
#define BALL2 2
#define BALL3 3
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
//const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
//#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

const char const *color_names[] = { "IDK", "BLACK", "WHITE", "GREENISH" };
#define IDK 0
#define BLACK 1
#define WHITE 2
#define GREENISH 3

#define R 0
#define G 1
#define B 2

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

	int TOUCH_VAL;

	int SEEK_MODE;
	int GYRO_OBS[10000];
	float SONAR_OBS[10000];
	int NOBS;

	int MAMBA_MODE, COLOR_VAL;
	int STATE;
	float SONAR_VAL;
	float GYRO_VAL;
	int MOTOR_POS;

	float compass_offset;	
	float gyro_offset;

	int PORTS[4];

} Robot;

Robot robot;

///// --------------- INIT COROUTINES -------------- /////

//CORO_CONTEXT_INIT( handle_touch );
//CORO_CONTEXT_INIT( move );
//CORO_CONTEXT_INIT( fetch_gyro );
CORO_CONTEXT( fetch_color );
//CORO_CONTEXT_INIT( fetch_sonar );
CORO_CONTEXT( handle_touch );
CORO_CONTEXT( move );
CORO_CONTEXT( fetch_gyro );
//CORO_CONTEXT( get_color );
CORO_CONTEXT( fetch_sonar );
CORO_CONTEXT( fetch_motorpos );

//----------------------------------------------------------


//// ---------------- INIT FUNCTIONS --------------- ////

int init_robot();
void stop();
void run_forever( int speed );
bool _check_pressed( uint8_t sn );
int get_color(Robot *robot);
float get_gyro(Robot *robot);
float get_sonar(Robot *robot);
int get_compass(Robot *robot);
int forward( int distance );	
int rotate( int side, int degree );
int throw( );
int grab( );
void rotate_until(int side);
void run_forever_correcting( int speed, int direction );


//int rotateWithGyro(Robot *robot, int degree);

//----------------------------------------------------------


// MAIN EXECUTES THE COROUTINES REPEATEDLY, WRITTEN BY ADI
// MAIN
int main( void )
{
  
  // CHECK TACHO, CONNECT 
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
  robot.STATE = BALL0;
  while(robot.MAMBA_MODE){
	CORO_CALL( fetch_sonar );
	CORO_CALL( fetch_motorpos );
	CORO_CALL( fetch_gyro );
	CORO_CALL( fetch_color );
	CORO_CALL( move );
	CORO_CALL( handle_touch );
  }

// ------------- CODE ENDS HERE -------------------

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}


// INIT_ROBOT, WRITTEN BY ANATOLE, ADITYA NAIR AND JOHANNE
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
	
	// SEARCHING FOR COLOR
 	if (ev3_search_sensor( LEGO_EV3_COLOR, &(robot.COLOR), 0) ){
		set_sensor_mode(robot.COLOR, "COL-COLOR");
		// if ( !get_sensor_value(0, robot.COLOR, &value )) {
		// 	value = 0;
		// }	
	}

	set_tacho_position( robot.WHEELS_SN[0], 0 );	
	set_tacho_position( robot.WHEELS_SN[1], 0 );

	/*if (ev3_search_sensor(LEGO_EV3_US, &( robot.COMPASS),0)){
      		robot.SONAR_VAL = 0;
    	}*/
	

	return 0;
}

////// ----------------------------- CORO ROUTINES ----------------------------- //////

// CONSTANTLY UPDATES WHETHER THE TOUCH WAS PRESSED, WRITTEN BY ADITYA AND JOHANNE
// HANDLE_TOUCH
CORO_DEFINE( handle_touch ){
	CORO_LOCAL int val;
	CORO_BEGIN();

	if ( robot.TOUCH == DESC_LIMIT) CORO_QUIT();

	for(;;){
		get_sensor_value(0, robot.TOUCH, &val);
		robot.TOUCH_VAL = val;
		CORO_YIELD();		
	} 
	CORO_END();
}

// CONSTANTLY UPDATES COLOR VALUES, WRITTEN ANATOLE
// FETCHCOLOR
CORO_DEFINE( fetch_color ){
	//printf("COLOR IS SENSED\n");
	CORO_LOCAL int color_val;
 	CORO_BEGIN();

 	for(;;){
		get_sensor_value(0, robot.COLOR, &color_val );
 		robot.COLOR_VAL = color_val;
 		//printf("Color value is: %d\n", color_val);

 		CORO_YIELD();
 	}
 	CORO_END();

}

// CONSTANTLY UPDATES TACHO MOTOR POSITION, WRITTEN BY ADITYA
// FETCHMOTORPOS
CORO_DEFINE( fetch_motorpos ){
	
	CORO_LOCAL int pos;
	CORO_BEGIN();

	for(;;){
		get_tacho_position(robot.WHEELS_SN[0], &pos );
		robot.MOTOR_POS = pos;
		CORO_YIELD();
	}
		
	CORO_END();
}


// CONSTANTLY UPDATES SENSOR VALUES, WRITTEN BY JOHANNE
// FETCHSONAR
CORO_DEFINE( fetch_sonar ){
	
	CORO_LOCAL float value;
	CORO_LOCAL int counter=0;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.SONAR, &value );
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

// CONSTANTLY UPDATES GYRO VALUES, WRITTEN BY ADITYA AND JOHANNE
// FETCHGYRO
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

// EXECUTES ACTION SEQUENCES, BASED ON UPDATED STATES AND SENSOR VALUES, WRITTEN BY ADITYA
// MOVE
CORO_DEFINE( move ){

	CORO_LOCAL int max_speed, curr_angle, i, high;	
	CORO_LOCAL int PRE_SCAN, SCANNING, POST_SCAN;
	CORO_LOCAL float OBVS[1000], init_angle;

	CORO_BEGIN();

	init_angle = robot.GYRO_VAL;


	// BALL0 SUBSTATES
	CORO_LOCAL int FORWARD0=TRUE, SHOOT0=FALSE, ROTATEREV0=FALSE, REVERSE0=FALSE;  

	// BALL1 SUBSTATES
	CORO_LOCAL int STEP11=TRUE, STEP12 = FALSE, STEP13 = FALSE, STEP14 = FALSE, STEP15 = FALSE,  STEP16 = FALSE;
	CORO_LOCAL int STEP17 = FALSE, STEP18 = FALSE, STEP19=FALSE, STEP110 = FALSE, STEP111= FALSE, STEP112= FALSE, STEP113 = FALSE;

	// BALL2 SUBSTATES
	CORO_LOCAL int STEP21=TRUE, STEP22 = FALSE, STEP23 = FALSE, STEP24 = FALSE, STEP25 = FALSE,  STEP26 = FALSE;
	CORO_LOCAL int STEP27 = FALSE, STEP28 = FALSE, STEP29=FALSE, STEP210 = FALSE, STEP211= FALSE, STEP212= FALSE, STEP213 = FALSE;


	get_tacho_max_speed(robot.WHEELS_SN[0], &max_speed);				

	for(;;){
		//printf("State: %d\n", robot.STATE);
		// BALLZERO			
		if(robot.STATE == BALL0){
			if(FORWARD0){ 
				if( robot.MOTOR_POS >= 2000 && robot.MOTOR_POS < 2400){
					run_forever( max_speed/4 );
				}
				else if( robot.MOTOR_POS <= 2000){
					run_forever( max_speed/2 );
				}
				else if( robot.MOTOR_POS >= 2400){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					FORWARD0 = FALSE;
					SHOOT0 = TRUE;
				}
				//printf("Robot Motor Pos: %d\n", robot.MOTOR_POS);
			}


			if(SHOOT0){
				throw();
				SHOOT0 = FALSE;
				REVERSE0 = TRUE;

			}

			if(ROTATEREV0){
				printf("Init angle %f, Current %f\n", init_angle, robot.GYRO_VAL); 
				if(robot.GYRO_VAL > init_angle){
					rotate_until(LEFT);
				}
				else{
					rotate_until(RIGHT);
				}
				//rotate_until(RIGHT);
				if(robot.GYRO_VAL == init_angle){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					REVERSE0 = TRUE;
					ROTATEREV0 = FALSE;
				}
			}

			if(REVERSE0){
				printf("Color detected is: %d Robot MotorPos: %d\n", robot.COLOR_VAL, robot.MOTOR_POS);	
				if( robot.COLOR_VAL == 1 || robot.COLOR_VAL == 2){
					printf("COLOR DETECTED!\n");
				}			
				
				run_forever( -max_speed/2 );
				if( robot.MOTOR_POS <= -1700){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					REVERSE0 = FALSE;
					robot.STATE = BALL1;
				}
		
			}
		}

		// BALLONE
		if(robot.STATE == BALL1){
	
			printf("CMES into ball1\n");	
			if(STEP11){
				rotate_until(RIGHT);
				printf("COLOR DETECTED %d GYRO VAL: %f\n", robot.COLOR_VAL, robot.GYRO_VAL);
				if(robot.GYRO_VAL >= init_angle+90.0){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP12 = TRUE;
					STEP11 = FALSE;
				}
			}
			
			if(STEP12){
				run_forever(-max_speed/2); 
				if(robot.TOUCH_VAL){
					init_angle = robot.GYRO_VAL - 90;
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP12 = FALSE;
					STEP13 = TRUE;
				}
			}

			if(STEP13){
				rotate_until(LEFT); 
				if(robot.GYRO_VAL <= init_angle-15){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP13 = FALSE;
					STEP14 = TRUE;
				}
			}

		
			if(STEP14){
				run_forever(max_speed/6); 
				if(robot.COLOR_VAL == 1 || robot.MOTOR_POS > 600){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP14 = FALSE;
					STEP15 = TRUE;
				}
			}


			if(STEP15){
				grab(); 
				set_tacho_position( robot.WHEELS_SN[0], 0 );	
				set_tacho_position( robot.WHEELS_SN[1], 0 );
				STEP15 = FALSE;
				STEP16 = TRUE;	
			}
	

			if(STEP16){		
				rotate_until(RIGHT); 
				if(robot.GYRO_VAL >= init_angle+90){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP16 = FALSE;
					STEP17 = TRUE;
				}
				
			}

			if(STEP17){
				run_forever(-max_speed/2); 
				if(robot.TOUCH_VAL){
					init_angle = robot.GYRO_VAL - 90;
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP17 = FALSE;
					
					//robot.MAMBA_MODE = FALSE;
					STEP18 = TRUE;
				}

			}

			if(STEP18){
				run_forever(max_speed/4); 
				if(robot.MOTOR_POS >= 300){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP18 = FALSE;
					STEP19 = TRUE;
				}
		
			}


			if(STEP19){
				rotate_until(LEFT);
				printf("ANGLE IS %f\n", robot.GYRO_VAL);
				if(robot.GYRO_VAL <= init_angle){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP19 = FALSE;
					STEP110 = TRUE;
				}
	

			}


			if(STEP110){
				run_forever(max_speed/4); 
				if(robot.MOTOR_POS >= 1100){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP110 = FALSE;
					STEP111 = TRUE;
				}

			}


			if(STEP111){
				throw();
				STEP111 = FALSE;
				//robot.MAMBA_MODE = FALSE;
				STEP112 = TRUE;

			}


			if(STEP112){	
				run_forever( -max_speed/2 );
				if( robot.MOTOR_POS <= -1800){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP112 = FALSE;
					robot.STATE = BALL2;
					//robot.MAMBA_MODE = FALSE;
				}

			}

		}

		
		// BALLTWO
		if(robot.STATE == BALL2){
	
			printf("CMES into ball1\n");	
			if(STEP21){
				rotate_until(LEFT);
				printf("COLOR DETECTED %d GYRO VAL: %f\n", robot.COLOR_VAL, robot.GYRO_VAL);
				if(robot.GYRO_VAL <= init_angle-90.0){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP22 = TRUE;
					STEP21 = FALSE;
				}
			}
			
			if(STEP22){
				run_forever(-max_speed/2); 
				if(robot.TOUCH_VAL){
					init_angle = robot.GYRO_VAL + 90;
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP22 = FALSE;
					STEP23 = TRUE;
				}
			}

			if(STEP23){
				rotate_until(RIGHT); 
				if(robot.GYRO_VAL >= init_angle+5){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP23 = FALSE;
					STEP24 = TRUE;
				}
			}

		
			if(STEP24){
				run_forever(max_speed/6); 
				if(robot.COLOR_VAL == 1 || robot.MOTOR_POS > 750){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP24 = FALSE;
					STEP25 = TRUE;
				}
			}


			if(STEP25){
				grab(); 
				set_tacho_position( robot.WHEELS_SN[0], 0 );	
				set_tacho_position( robot.WHEELS_SN[1], 0 );
				STEP25 = FALSE;
				STEP26 = TRUE;	
			}
	

			if(STEP26){		
				rotate_until(LEFT); 
				if(robot.GYRO_VAL <= init_angle-90){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP26 = FALSE;
					STEP27 = TRUE;
				}
				
			}

			if(STEP27){
				run_forever(-max_speed/2); 
				if(robot.TOUCH_VAL){
					init_angle = robot.GYRO_VAL + 90;
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP27 = FALSE;
					STEP28 = TRUE;
				}

			}

			if(STEP28){
				run_forever(max_speed/4); 
				if(robot.MOTOR_POS >= 300){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP28 = FALSE;
					STEP29 = TRUE;
				}
		
			}


			if(STEP29){
				rotate_until(RIGHT);
				printf("ANGLE IS %f\n", robot.GYRO_VAL);
				if(robot.GYRO_VAL >= init_angle){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP29 = FALSE;
					STEP210 = TRUE;
				}
	

			}


			if(STEP210){
				run_forever(max_speed/4); 
				if(robot.MOTOR_POS >= 1200){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP210 = FALSE;
					STEP211 = TRUE;
				}

			}


			if(STEP211){
				throw();
				STEP211 = FALSE;
				//robot.MAMBA_MODE = FALSE;
				STEP212 = TRUE;

			}


			if(STEP212){	
				run_forever( -max_speed/2 );
				if( robot.MOTOR_POS <= -1700){
					set_tacho_position( robot.WHEELS_SN[0], 0 );	
					set_tacho_position( robot.WHEELS_SN[1], 0 );
					stop();
					STEP212 = FALSE;
					robot.MAMBA_MODE = FALSE;
				}

			}

		}

		




		

		CORO_YIELD();
	}
	CORO_END();
}

//-----------------------------------------------------------------------------------------------------
//----------------------------------- BASIC FUNCTIONS -------------------------------------------------
//-----------------------------------------------------------------------------------------------------

// DETECTION WAS SUPPOSED TO USE IR VALUES TO DETECT BALL, WRITTEN BY ADITYA AND ANATOLE
//DETECTION
int detection(float obvs[], int n){

	int i;

	for(i = 0; i < n; i++){

		printf("%f is the %d value, angle is %d\n", robot.SONAR_OBS[i], i, robot.GYRO_OBS[i]);
	
	}

	return 0;

}

// GRABS, WRITTEN BY ADITYA
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

// THROWS BALL, WRITTEN BY ADITYA
// THROW
int throw(){

	int max_speed;

        get_tacho_max_speed( robot.THROW, &max_speed );
	
	printf("MAX SPEED OF THROW: %d", max_speed);
	
	set_tacho_speed_sp( robot.THROW, max_speed );
      	set_tacho_ramp_up_sp( robot.THROW, 90 );
      	set_tacho_ramp_down_sp( robot.THROW, 90 );
	set_tacho_position( robot.THROW, 0);
	set_tacho_position_sp( robot.THROW, 240);
    	set_tacho_command_inx( robot.THROW, TACHO_RUN_TO_REL_POS);
	Sleep(1000);
	
	set_tacho_speed_sp( robot.THROW, -max_speed );
      	set_tacho_ramp_up_sp( robot.THROW, 90 );
      	set_tacho_ramp_down_sp( robot.THROW, 90 );
	set_tacho_position( robot.THROW, 0);
	set_tacho_position_sp( robot.THROW, -240);
    	set_tacho_command_inx( robot.THROW, TACHO_RUN_TO_REL_POS);
	Sleep(1000);
	set_tacho_command_inx( robot.THROW, TACHO_STOP);	

	return 0;

}

// ROTATES INFINITELY, WRITTEN BY JOHANNE
// ROTATEUNTIL
void rotate_until( int side ){
    uint8_t rot_fd = robot.WHEELS_SN[side];
	uint8_t rot_bk = robot.WHEELS_SN[(side+1)%2];

	set_tacho_speed_sp( rot_fd, 60);
	set_tacho_speed_sp( rot_bk, -60 );

	set_tacho_command_inx(robot.WHEELS_SN[0], TACHO_RUN_FOREVER);
	set_tacho_command_inx(robot.WHEELS_SN[1], TACHO_RUN_FOREVER);
}


// ATTEMPTS TO CORRECT WHILE RUNNING, BASED ON GYRO VALS, WRITTEN BY ADITYA
// RUNCORRECT
void run_forever_correcting( int speed, int direction ){

	if(direction == 0){
		set_tacho_speed_sp(robot.WHEELS_SN[0], speed);
		set_tacho_speed_sp(robot.WHEELS_SN[1], speed);
	}

	if(direction > 0){
		set_tacho_speed_sp(robot.WHEELS_SN[0], speed+10);
		set_tacho_speed_sp(robot.WHEELS_SN[1], speed);
	}

	if(direction < 0){
		set_tacho_speed_sp(robot.WHEELS_SN[0], speed);
		set_tacho_speed_sp(robot.WHEELS_SN[1], speed+10);
	}

	set_tacho_command_inx(robot.WHEELS_SN[0], TACHO_RUN_FOREVER);
	set_tacho_command_inx(robot.WHEELS_SN[1], TACHO_RUN_FOREVER);
	
}


// RUNS INFINITELY, WRITTEN BY JOHANNE
// RUNFOREVER
void run_forever( int speed ){
	printf("COMES INTO RUNFOREVERTOO\n");
	set_tacho_speed_sp(robot.WHEELS_SN[0], speed);
	set_tacho_speed_sp(robot.WHEELS_SN[1], speed);

	set_tacho_command_inx(robot.WHEELS_SN[0], TACHO_RUN_FOREVER);
	set_tacho_command_inx(robot.WHEELS_SN[1], TACHO_RUN_FOREVER);
	
}


// STOPS THE INFINITE RUNS, WRITTEN BY JOHANNE
// STOP
void stop( void ){	
	
	multi_set_tacho_command_inx(robot.WHEELS_SN, TACHO_STOP);

}

// ------------------ NOT USED MUCH FUNCITONS ------------------------- //

// ROTATES A PRECISE AMOUNT BASED ON TACHO POSITIONS, WRITTEN BY ADITYA
// ROTATE
int rotate( int side, int degree ){
	printf("%d degrees! \n", degree);

	uint8_t rot_fd = robot.WHEELS_SN[side];
	uint8_t rot_bk = robot.WHEELS_SN[(side+1)%2];

	//printf("Dest_Pos is: %f", 4*31.4**(degree/360)); 
	
	int dest_pos = 4*31.4159*9.35*(degree/360.0);
	printf("DEST_POS IS: %d\n", dest_pos);	

	set_tacho_speed_sp( rot_fd, dest_pos/4 );      	
	set_tacho_speed_sp( rot_bk, -dest_pos/4 );
	multi_set_tacho_ramp_up_sp( robot.WHEELS_SN, 200 );
      	multi_set_tacho_ramp_down_sp( robot.WHEELS_SN, 200 );
	multi_set_tacho_position( robot.WHEELS_SN, 0);
	set_tacho_position_sp( rot_fd, dest_pos);
	set_tacho_position_sp( rot_bk, -dest_pos);
    	set_tacho_command_inx( robot.WHEELS_SN[0], TACHO_RUN_TO_REL_POS);	
    	set_tacho_command_inx( robot.WHEELS_SN[1], TACHO_RUN_TO_REL_POS);
   
 	//multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS);
	//Sleep(4000);

	return 0;
}

// MOVES FORWARD A PARTICULAR DISTANCE, BASED ON TACHO MOTOR POSITION, WRITTEN BY ADITYA
// FORWARDDISTANCE 
int forward( int distance ){	
	int dest_pos = distance*20;
	
	set_tacho_speed_sp( robot.WHEELS_SN[0], dest_pos/2 );
      	set_tacho_ramp_up_sp( robot.WHEELS_SN[0], 2 );
      	set_tacho_ramp_down_sp( robot.WHEELS_SN[0], 2 );
	set_tacho_speed_sp( robot.WHEELS_SN[1], dest_pos/2 );
      	set_tacho_ramp_up_sp( robot.WHEELS_SN[1], 2 );
      	set_tacho_ramp_down_sp( robot.WHEELS_SN[1], 2 );
	multi_set_tacho_position( robot.WHEELS_SN, 0);
	multi_set_tacho_position_sp( robot.WHEELS_SN, dest_pos);

    	set_tacho_command_inx( robot.WHEELS_SN[0], TACHO_RUN_TO_REL_POS);	
    	set_tacho_command_inx( robot.WHEELS_SN[1], TACHO_RUN_TO_REL_POS);

	//multi_set_tacho_command_inx( robot->WHEELS_SN, TACHO_RUN_TO_REL_POS );			

	return 0;	
}
	
