#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "coroutine.h"


/**************************************/
/*                                    */
/*                init                */
/*                                    */
/**************************************/


// STATES //////////////////////////////////
const char const *state_names[] = { "INIT", "RELEASE_BALL", "GO_TO_BALL", "BALL_DETECT", "NEXT_BALL", "DEFEND_GOAL" };
#define INIT 0
#define RELEASE_BALL 1
#define GO_TO_BALL 2
#define BALL_DETECT 3
#define NEXT_BALL 4
#define DEFEND_GOAL 5
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

	int SEEK_MODE;
	int GYRO_OBS[10000];
	float SONAR_OBS[10000];
	int NOBS;

	int MAMBA_MODE;
	int STATE;
	int COLOR_VAL;
	float SONAR_VAL;
	float GYRO_VAL;

	float compass_offset;
	float gyro_offset;

	int PORTS[4];

} Robot;

Robot robot;

CORO_CONTEXT( handle_touch );
CORO_CONTEXT( defence );
CORO_CONTEXT( fetch_gyro );
CORO_CONTEXT( fetch_color );
CORO_CONTEXT( fetch_sonar );

int init_robot();
void stop();
void run_forever( int speed );
bool _check_pressed( uint8_t sn );
int forward(int distance);
int rotate(int side, int degree);
int throw_ball();
int release();
int grab();
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
  int tour = 0;
  robot.SEEK_MODE = FALSE;
  robot.STATE = RELEASE_BALL;
  while(robot.MAMBA_MODE){
	CORO_CALL( fetch_sonar );
	CORO_CALL( fetch_gyro );
	CORO_CALL( fetch_color ) ;
	CORO_CALL( defence );
	CORO_CALL( handle_touch );
	tour ++;
	if (tour % 30 == 0){
		printf("\nsonar: %f \tangle: %f \tcol: %s \n",robot.SONAR_VAL,robot.GYRO_VAL,color_names[robot.COLOR_VAL]);
		tour = 0;
	}
  }

// ------------- CODE ENDS HERE -------------------

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}




/**************************************/
/*                                    */
/*             coroutines             */
/*                                    */
/**************************************/


// HANDLE_TOUCH
// used to check if we are touching grass... I mean walls
CORO_DEFINE( handle_touch ){
	//printf("#### HANDLE_TOUCH ####\n");
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


// FETCHCOLOR
CORO_DEFINE( fetch_color ){
	//printf("#### FETCH_COLOR ####\n");
	CORO_LOCAL int RGB_val[3];

	//BLACK
	CORO_LOCAL int RGB_black[3] = {17, 35, 43}; // RGB values that we found empiricaly
	CORO_LOCAL int leeway_black[3] = {12, 17, 18}; // defines the window where we have 99.99% of values
	
	//WHITE
	CORO_LOCAL int RGB_white[3] = {160, 309, 354};
	CORO_LOCAL int leeway_white[3] = {12, 27, 34};
	
	//YELLOW(for tests at home purposes, replace with greenish for the real thing)
	CORO_LOCAL int RGB_greenish[3] = {126, 141, 50};
	CORO_LOCAL int leeway_greenish[3] = {13, 12, 5};
	
	// //GREENISH
	// CORO_LOCAL int RGB_greenish[3] = {65, 138, 52};
	// CORO_LOCAL int leeway_greenish[3] = {10, 20, 29};
	// CORO_BEGIN();	
	

	CORO_BEGIN();	
	
	for(;;){
		//finding new rgb values
		get_sensor_value(R, robot.COLOR, &RGB_val[R]); //red
		get_sensor_value(G, robot.COLOR, &RGB_val[G]); //green
		get_sensor_value(B, robot.COLOR, &RGB_val[B]); //blu	
		
		// making a file to get the repartition of error of a specific colour(kind of training)
		// FILE *fp;
		// fp = fopen("RGB_data.csv", "a");
		// fprintf(fp, "%d,%d,%d\n", RGB_val[R], RGB_val[G], RGB_val[B]);
		// fclose(fp);
		
		//printf("Color value is: R:%d G:%d B:%d \n", RGB_val[R], RGB_val[G], RGB_val[B]);
		
		if (RGB_val[R]>RGB_white[R]-leeway_white[R] && RGB_val[R]<RGB_white[R]+leeway_white[R] /*if red amount is consistent with white*/
			&& RGB_val[G]>RGB_white[G]-leeway_white[G] && RGB_val[G]<RGB_white[G]+leeway_white[G] /*and green amount is consistent with white*/
			&& RGB_val[B]>RGB_white[B]-leeway_white[B] && RGB_val[B]<RGB_white[B]+leeway_white[B]) { /*and blue amount is also consistent with white*/
			robot.COLOR_VAL = WHITE;																	/*then its white*/
		} else if (RGB_val[R]>RGB_black[R]-leeway_black[R] && RGB_val[R]<RGB_black[R]+leeway_black[R] && RGB_val[G]>RGB_black[G]-leeway_black[G] && RGB_val[G]<RGB_black[G]+leeway_black[G] && RGB_val[B]>RGB_black[B]-leeway_black[B] && RGB_val[B]<RGB_black[B]+leeway_black[B]) {
			robot.COLOR_VAL = BLACK;
		} else if (RGB_val[R]>RGB_greenish[R]-leeway_greenish[R] && RGB_val[R]<RGB_greenish[R]+leeway_greenish[R] && RGB_val[G]>RGB_greenish[G]-leeway_greenish[G] && RGB_val[G]<RGB_greenish[G]+leeway_greenish[G] && RGB_val[B]>RGB_greenish[B]-leeway_greenish[B] && RGB_val[B]<RGB_greenish[B]+leeway_greenish[B]) {
			robot.COLOR_VAL = GREENISH;			
		} else {
			robot.COLOR_VAL = IDK;
		}

		//printf("Color name is %s \n", color_names[robot.COLOR_VAL]);
		CORO_YIELD();
	}
	CORO_END();
}


// // FETCHCOLOR
// CORO_DEFINE( fetch_color ){
// 	CORO_LOCAL int color_val;
// 	CORO_BEGIN();	
	
// 	for(;;){
// 		get_sensor_value(0, robot.COLOR, &color_val );
// 		robot.COLOR_VAL = color_val;
// 		printf("Color value is: %d\n", color[color_val]);

// 		CORO_YIELD();
// 	}		
// 	CORO_END();
// }


// FETCH_SONAR
CORO_DEFINE( fetch_sonar ){
	//printf("#### FETCH_SONAR ####\n");
	//time_t current_time = time(NULL);
	CORO_LOCAL float value;
	CORO_LOCAL int counter=0;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.SONAR, &value );
		robot.SONAR_VAL = value;
		//printf("time: %d\t IR Val %f\n", current_time ,robot.SONAR_VAL); // to check the effect of poll on frequency
		if(robot.SEEK_MODE){
			robot.SONAR_OBS[counter] = robot.SONAR_VAL;
			counter++;
			robot.NOBS = counter;
		}
		CORO_YIELD();
	}
	CORO_END();
}


// FETCHGYRO
CORO_DEFINE( fetch_gyro ){
	//printf("#### FETCH_GYRO ####\n");
	CORO_LOCAL float value;
	CORO_LOCAL int counter = 0;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.GYRO, &value );
		robot.GYRO_VAL = value-robot.gyro_offset;
		//printf("GYRO VAL: %f\n", robot.GYRO_VAL);	
		if(robot.SEEK_MODE){
			robot.GYRO_OBS[counter] = robot.GYRO_VAL;
			counter++;
		}
		CORO_YIELD();
	}
	CORO_END();
}


// DEFEND
CORO_DEFINE( defence ){
	//printf("#### DEFENCE ####\n");
	CORO_LOCAL int max_speed, init_angle, count, go_forward;
	CORO_LOCAL int PRE_SCAN, SCANNING, POST_SCAN;
	CORO_LOCAL int STEP1, STEP2, STEP3;
	CORO_LOCAL int STEP1GRAB, STEP2THROW;
	CORO_LOCAL int OBVS[300];

	CORO_LOCAL int lowest;
	CORO_LOCAL float diff;

	CORO_BEGIN();

	init_angle = robot.GYRO_VAL;

	count = 0;
	go_forward = 0;
	
	STEP1=1;
	STEP2=0;
	STEP3=0;

	STEP1GRAB=1;
	STEP2THROW=0;

	PRE_SCAN = TRUE;
	SCANNING = FALSE;
	POST_SCAN = FALSE;
	printf("---- ---- in %s ---- ---- \n",state_names[robot.STATE]);

	if(robot.STATE == RELEASE_BALL){printf("\tstep 1\n");}

	for(;;){
		// CHECK STATUS
		if(robot.STATE == RELEASE_BALL){
            
			get_tacho_max_speed(robot.WHEELS_SN[0], &max_speed);
			if (STEP1){
                // go forward 20 cm
				forward(20);
				Sleep(2500);
				STEP1=0;
				STEP2=1;
				printf("\tstep 2\n");
			}
			if (STEP2){
                // rotate right 30° - loop until we reach 30°
				rotate_until(RIGHT);
				if (robot.GYRO_VAL>=init_angle+25){
					stop();
                    // release the balls
					release();
					STEP2=0;
					STEP3=1;
					printf("\tstep 3\n");
				}
			}
			if (STEP3){
                // go back to the previous value
				rotate_until(LEFT);
				if (robot.GYRO_VAL<=init_angle){
					stop();
                    // go back of 10 cm
					forward(-10);
					Sleep(2500);
					robot.STATE = GO_TO_BALL;
					printf("---- ---- in %s ---- ---- \n",state_names[robot.STATE]);
					STEP3=0;
					STEP1=1;
					}
				}
			}

		if (robot.STATE == GO_TO_BALL){
            if (count == 0){
                // it is ball 1/2 (of the 2 we want to throw)
                // rotate until 90° mark
                rotate_until(RIGHT);
                if (robot.GYRO_VAL>=90){
                    stop();
					run_forever(max_speed/4);
                }
            } else {
                // it is ball 2/2
                // do the same, but without rotating
                run_forever(max_speed/4);
            }
			// if we are closer than 30cm wrt something - stop and go to ball detect
            if (robot.SONAR_VAL<=300){
				// now, we are either 30cm from ball or from wall
                stop();

                robot.STATE = BALL_DETECT;
				printf("---- ---- in BALL_DETECT ---- ---- \n\tPRE_SCAN\n");			
            }
        }

		// TODO - idk
		if (robot.STATE == NEXT_BALL){
            	rotate_until(LEFT);
			STEP1GRAB=1;
			STEP2THROW=0;
			if (robot.GYRO_VAL<=-90){
				stop();
				count++;
				robot.STATE = GO_TO_BALL;
				printf("---- ---- in %s ---- ---- \n",state_names[robot.STATE]);
			}
		}

		// BALLDETECTMODE
		/*
		if(robot.STATE == BALL_DETECT){
            printf("---- ---- in BALL_DETECT ---- ---- \n");
			if(STEP1GRAB){
				grab();
				STEP1GRAB=0;
				STEP2THROW=1;
			}
			if(STEP2) {
				throw_ball();
				if(count == 0){
					robot.STATE == NEXT_BALL;
					printf("---- ---- in %s ---- ---- \n",state_names[robot.STATE]);		
				} else {
					robot.STATE == DEFEND_GOAL;
					printf("---- ---- in %s ---- ---- \n",state_names[robot.STATE]);
				}
			}
		}
		*/
		if(robot.STATE == BALL_DETECT){
			// scan to understand where the f*****g ball is
			if(PRE_SCAN){
				rotate_until(LEFT);
				if(robot.GYRO_VAL <= init_angle - 30){
					//tate_until(RIGHT);
					//(robot.GYRO_VAL >= init_angle);
					stop();
					PRE_SCAN = FALSE;
					SCANNING = TRUE;
					robot.SEEK_MODE = TRUE;
					printf("\tSCAN!\n");
				}
			}
			// SCAN
			if(SCANNING){
				rotate_until(RIGHT);
				if(robot.GYRO_VAL >= init_angle + 30){
					stop();
					SCANNING = FALSE;
					POST_SCAN = TRUE;
					robot.SEEK_MODE = FALSE;
					printf("\tPOST_SCAN!\n");
				}
			}
			//POSTSCAN
			if(POST_SCAN){
				rotate_until(LEFT);
				if(robot.GYRO_VAL <= init_angle){
					stop();
					POST_SCAN = FALSE;
				}
			}
			if(STEP1GRAB){
				grab();
				STEP1GRAB=0;
				STEP2THROW=1;
			}
			if(STEP2) {
				throw_ball();
				if(count == 0){
					robot.STATE == NEXT_BALL;			
				} else {
					robot.STATE == DEFEND_GOAL;
				}
			}
		}	
			// PRE-SCAN
			/*

		}*/

	if (robot.STATE == DEFEND_GOAL){
        printf("---- ---- in DEFEND_GOAL ---- ---- \n");
        	if (!go_forward){
                	run_forever(-max_speed/2);
                	if (robot.GYRO_VAL>=900){
                		stop();
                		go_forward = 1;
                	}
     		}	
            	if (go_forward){
                	run_forever(max_speed/2);
                	if (robot.GYRO_VAL>=300){
                    		stop();
                    		go_forward = 0;
                	}
            	}
	}

	CORO_YIELD();
	}
	CORO_END();
}


// FIXME


/********************************************/
/*                                          */
/*        shit that's pissing me off        */ 
/*                                          */
/********************************************/


// --------------- init ----------------------
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
	float newVal;

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
		// set the sampling interval
		set_sensor_mode(robot.SONAR, "US-DIST-CM");
		set_sensor_poll_ms(robot.SONAR, 1);
    }

	// SEARCHING FOR TOUCH
	if (ev3_search_sensor(LEGO_EV3_TOUCH, &( robot.TOUCH ), 0)){
		printf("Touch found! %x\n", robot.TOUCH);
	}
		
	// SEARCHING FOR COLOR
 	if (ev3_search_sensor( LEGO_EV3_COLOR, &(robot.COLOR), 0) ){
		set_sensor_mode(robot.COLOR, "RGB-RAW");
	}

	// SEARCHING FOR GYRO
	if (ev3_search_sensor(LEGO_EV3_GYRO, &( robot.GYRO),0)){
		printf("Gyro found! \n");
		robot.GYRO_VAL = 0;
	}

	return 0;
}

// GRAB
int grab(){

	int max_speed;

    get_tacho_max_speed( robot.GRAB, &max_speed );
    get_tacho_max_speed( robot.GRAB, &max_speed );
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

// RELEASE
// first, we throw the ball in the grabbing arm	
// then, we rotate the throwing arm to BOOP other balls away
int release(){
	printf("running: release\n");
	int max_speed;

    get_tacho_max_speed( robot.THROW, &max_speed );

	printf("MAX SPEED OF THROW: %d\n", max_speed);

	// first - throwing arm
	printf("\tfirst spin\n");
	set_tacho_speed_sp( robot.THROW, -max_speed/4 );
	set_tacho_position( robot.THROW, 0);
	set_tacho_position_sp( robot.THROW, 120);
    set_tacho_command_inx( robot.THROW, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx( robot.THROW, TACHO_RUN_TO_REL_POS);
	Sleep(1000);
	set_tacho_command_inx( robot.THROW, TACHO_STOP);

	Sleep(1500);

	// second - throwing arm
	printf("\tsecond spin\n");
	set_tacho_speed_sp( robot.THROW, -max_speed/4 );
	set_tacho_position( robot.THROW, 0);
	set_tacho_position_sp( robot.THROW, 120);
    set_tacho_command_inx( robot.THROW, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx( robot.THROW, TACHO_RUN_TO_REL_POS);
	Sleep(1000);
	set_tacho_command_inx( robot.THROW, TACHO_STOP);

	Sleep(2000);

	// third - grabbing arm
	printf("\tthird spin\n");
	
	set_tacho_speed_sp( robot.GRAB, -max_speed/2 );
	set_tacho_position( robot.GRAB, 0);
	set_tacho_position_sp( robot.GRAB, 1080);
    set_tacho_command_inx( robot.GRAB, TACHO_RUN_TO_REL_POS);
	Sleep(3000);
	set_tacho_command_inx( robot.GRAB, TACHO_STOP);

	return 0;

}

// THROW
int throw_ball(){

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

// ROTATE
int rotate(int side, int degree){
	printf("%d degrees! \n", degree);

	uint8_t rot_fd = robot.WHEELS_SN[side];
	uint8_t rot_bk = robot.WHEELS_SN[(side+1)%2];

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

	return 0;
}

// ROTATE_UNTIL
void rotate_until(int side){
    uint8_t rot_fd = robot.WHEELS_SN[side];
	uint8_t rot_bk = robot.WHEELS_SN[(side+1)%2];

	set_tacho_speed_sp( rot_fd, 30 );
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

void stop(){

	multi_set_tacho_command_inx(robot.WHEELS_SN, TACHO_STOP);

}


// MOVE FORWARD
int forward(int distance){
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

	return 0;
}