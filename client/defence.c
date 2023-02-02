#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "coroutine.h"


// STATES //////////////////////////////////
#define INIT 0
#define RELEASE_BALL 1
#define GO_TO_BALL 2
#define GRAB_BALL 3
#define NEXT_BALL 4
#define BALL_TWO 5
#define DEFEND_GOAL 6
#define REORIENT 7
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
	int COLOR_VAL, TOUCH_VAL;
	float GYRO_VAL, SONAR_VAL;
	int MOTOR_POS;

	float compass_offset;
	float gyro_offset;

	int PORTS[4];

} Robot;

Robot robot;

CORO_CONTEXT( fetch_motorpos );
CORO_CONTEXT( handle_touch );
CORO_CONTEXT( defence );
CORO_CONTEXT( fetch_gyro );
CORO_CONTEXT( get_color );
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
void rotate_until(int side, int speed);
void run_forever_correcting( int speed, int direction );
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
  robot.STATE = RELEASE_BALL;
  while(robot.MAMBA_MODE){
    CORO_CALL( fetch_motorpos );
	CORO_CALL( fetch_sonar );
	CORO_CALL( fetch_gyro );
	CORO_CALL( get_color );
	CORO_CALL( defence );
	CORO_CALL( handle_touch );
	Sleep(10);
  }

// ------------- CODE ENDS HERE -------------------

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}


// INIT_ROBOT
//This function initialises the different motors and sensors
//Written by Johanne, Aditya and Anatole.
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
        //printf("Sonar found! \n");
		//robot.SONAR_VAL = 0;
		if (ev3_search_sensor(LEGO_EV3_US, &( robot.SONAR),0)){
            printf("Sonar found! \n");
            robot.SONAR_VAL = 0;
            // set the sampling interval
            set_sensor_mode(robot.SONAR, "US-DIST-CM");
            set_sensor_poll_ms(robot.SONAR, 1);
        }
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
	}

	return 0;
}


//MOTOR_POS
//this coro function will continuously read the tacho position of the wheels and update the robot.MOTOR_POS variable.
//This is used when we want to move forward a certain distance.
//Written by Aditya
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



// HANDLE_TOUCH
//This coro function will continuously update the value of the robot.TOUCH_VAL. This is used to detect a touch against the wall
//Written by Johanne and Aditya.
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

//anatole
//This coro function will continuously read the color values form the color sensor and update the robot.COLOR_VAL variable.
//This is used for detecting when we reach different lines.
CORO_DEFINE( get_color ){
    CORO_LOCAL int color_val;
 	CORO_BEGIN();

 	for(;;){
 		get_sensor_value(0, robot.COLOR, &color_val );
 		robot.COLOR_VAL = color_val;

 		CORO_YIELD();
 	}
 	CORO_END();
}

// FETCH_SONAR
//This coro function will continuously read the sonar values form the ultrasonic sensor and update the robot.SONAR_VAL variable.
//This is used to read the distance in mm from objects.
//Joahnne
CORO_DEFINE( fetch_sonar ){

	CORO_LOCAL float value;
	CORO_LOCAL int counter=0;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.SONAR, &value );
		robot.SONAR_VAL = value;
		printf("SONAR value is: %f\n", robot.SONAR_VAL);
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
//This coro function will continously read the GYRO values
//BOTH
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
//this is our code for the defence strategy. Executes defence strategy according to different sensor values.
//Written by Johanne
CORO_DEFINE( defence ){

	CORO_LOCAL int max_speed, init_angle, count, go_forward;
	CORO_LOCAL int PRE_SCAN, SCANNING, POST_SCAN;
	CORO_LOCAL int STEP1, STEP2, STEP3, STEP4;

	CORO_LOCAL int lowest;
	CORO_LOCAL float diff;

	CORO_BEGIN();

	init_angle = robot.GYRO_VAL;

	count = 0;
	go_forward = 0;

	PRE_SCAN = TRUE;
	SCANNING = FALSE;
	POST_SCAN = FALSE;

	STEP1=1;
	STEP2=0;
	STEP3=0;
	STEP4=0;

	for(;;){
		// CHECK STATUS
		//This first state will  throw our distraction ball
		if(robot.STATE == RELEASE_BALL){
		 	get_tacho_max_speed(robot.WHEELS_SN[0], &max_speed);
			if (STEP1){
                //first we rotate x degrees and then throw a distraction ball in hopes of hitting one of the balls in the
                //attacker area to make it be out of its place and making it harder for the attacker to find the ball.
                rotate_until(RIGHT, max_speed/8);
                if (robot.GYRO_VAL>=init_angle+25){
                    stop();
                    release();
                    Sleep(2000);
                    STEP1=0;
                    STEP2=1;
                }
            //The next steps here is to try to get into position to try to grab the ball to our right
			}else if(STEP2){
                rotate_until(LEFT, max_speed/8);
                if (robot.GYRO_VAL <= init_angle-85){
                    stop();
                    STEP2=0;
                    STEP3=1;
                }
			}else if(STEP3){
			    run_forever(-max_speed/4);
			    if (robot.TOUCH_VAL){
                    stop();

                    STEP3=0;
                    STEP4=1;
			    }

			}else if(STEP4){
                rotate_until(RIGHT, max_speed/8);
                if (robot.GYRO_VAL >= init_angle+5){
                    stop();
                    STEP4=0;
                    STEP1=1;
                    robot.STATE = GRAB_BALL;
                }
			}
        //here we actually grab the ball
		}else if(robot.STATE == GRAB_BALL){
            if (STEP1){
                run_forever(max_speed/4);
                if (robot.COLOR_VAL == 3 || robot.COLOR_VAL == 4 || robot.COLOR_VAL == 7){
                    stop();
                    grab();
                    Sleep(1000);
                    STEP1=0;
                    STEP2=1;
                }
            //when we have the ball, we want to try to turn towards where we think the other ball in the attacker
            //area is to hopefully get it out of its position.
            }else if(STEP2){
                rotate_until(LEFT, max_speed/8);
                if (robot.GYRO_VAL <= init_angle-60){
                    stop();
                    release();
                    Sleep(1000);
                    STEP2=0;
                    STEP3=1;
                }
            }else if(STEP3){
                rotate_until(RIGHT, max_speed/8);
                if (robot.GYRO_VAL >= (init_angle)){
                    //forward(5);
                    Sleep(2000);
                    STEP3=0;
                    STEP4=1;
                }
            //after we have thrown the ball we want to turn and go towards the other ball close to the attacker area.
            }else if(STEP4){
                rotate_until(LEFT, max_speed/8);
                if (robot.GYRO_VAL <= init_angle-83){
                    stop();
                    //robot.MAMBA_MODE = 0;
                    robot.STATE=BALL_TWO;
                    STEP4=0;
                    STEP1=1;
                }
            }
		}else if (robot.STATE==BALL_TWO){
            if (STEP1){
                //we hit the wall before going for the second ball to correct out angle.
                run_forever(-max_speed/4);
			    if (robot.TOUCH_VAL){
                    stop();
                    set_tacho_position(robot.WHEELS_SN[0], 0);
                    set_tacho_position(robot.WHEELS_SN[1], 0);
                    STEP1=0;
                    STEP2=1;
			    }
            //then we go forward, try to grab the ball and just throw it away.
            }else if (STEP2){
                run_forever(max_speed/4);
                if (robot.MOTOR_POS >= 1730){
                    stop();
                    grab();
                    Sleep(1000);
                    throw_ball(),
                    Sleep(1000);
                    STEP2=0;
                    STEP1=1;
                    //robot.MAMBA_MODE = 0;
                    robot.STATE=DEFEND_GOAL;
                }
            }
        //in this state we just go back and forward to try to defend our goal from any attacker. s
        }else if (robot.STATE==DEFEND_GOAL){
            if (STEP1){
                run_forever_correcting(-max_speed/2, init_angle-robot.GYRO_VAL);
                if (robot.TOUCH_VAL){
                    stop();
                    init_angle = robot.GYRO_VAL;
                    set_tacho_position(robot.WHEELS_SN[0], 0);
                    set_tacho_position(robot.WHEELS_SN[1], 0);
                    STEP1=0;
                    STEP2=1;
                    count++;
			    }
            }else if(STEP2){
                run_forever_correcting(max_speed/2, init_angle-robot.GYRO_VAL);
                if (robot.MOTOR_POS >= 1750){
                    stop();
                    STEP2=0;
                    STEP1=1;
                    count++;
                }
            }
            if (count==11){
                robot.MAMBA_MODE=0;
                }
            }
		CORO_YIELD();
	}
	CORO_END();
}

// GRAB
//This function will make the grabbing arm rotate 360 degrees..
//Written by Aditya
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

// RELEASE
//This function is part of the defence and will release/throw a ball. But it
//will throw the ball at a less speed then throw_ball, and also in the defence
//we do not care about the throwing arm getting back to the right angle.
//Written by Anatole
int release(){

	int max_speed;

    get_tacho_max_speed( robot.THROW, &max_speed );

	printf("MAX SPEED OF THROW: %d", max_speed);

	set_tacho_speed_sp( robot.THROW, -max_speed/4 );
	set_tacho_position( robot.THROW, 0);
	set_tacho_position_sp( robot.THROW, 120);
    set_tacho_command_inx( robot.THROW, TACHO_RUN_TO_REL_POS);
	Sleep(1000);

	set_tacho_command_inx( robot.THROW, TACHO_STOP);

	return 0;

}

// THROW
//This function will throw the ball, the arm will rotate in one direction and then rotate the same
//amount of distance to the opposite direction to make it stop at the same starting angle. This because
//we want to get as much speed as possible when throwing the ball to make it throw longer.
//Written by Aditya
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
//makes the robot rotate to the given side the certain amount of degrees passed in
//Written by Aditya
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
//this function will make the robot rotate to the given side with the given speed, and it will rotate until
//it is stopped by the stop() function
//written by Johanne
void rotate_until(int side, int speed){
    uint8_t rot_fd = robot.WHEELS_SN[side];
	uint8_t rot_bk = robot.WHEELS_SN[(side+1)%2];

	set_tacho_speed_sp( rot_fd, speed );
	set_tacho_speed_sp( rot_bk, -speed );

	set_tacho_command_inx(robot.WHEELS_SN[0], TACHO_RUN_FOREVER);
	set_tacho_command_inx(robot.WHEELS_SN[1], TACHO_RUN_FOREVER);
}


// RUN FOREVER
//this function will make the robot run forward or backwards until it is stopped by the stop function, it
//will run at the given speed passed into the function.
//written by Johanne
void run_forever(int speed){
	set_tacho_speed_sp(robot.WHEELS_SN[0], speed);
	set_tacho_speed_sp(robot.WHEELS_SN[1], speed);

	set_tacho_command_inx(robot.WHEELS_SN[0], TACHO_RUN_FOREVER);
	set_tacho_command_inx(robot.WHEELS_SN[1], TACHO_RUN_FOREVER);

}
// STOP
//this function makes the robot stop after a run_forever or run_until
//written by Johanne
void stop(){

	multi_set_tacho_command_inx(robot.WHEELS_SN, TACHO_STOP);

}


// MOVE FORWARD
//This function moves forward for the given amount of distance
//made by Aditya
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


//This function attempts to move forward and correcting the angle according to the gyro sensor. We use this to try
//to correct our diving angle if we suddenly get a small rotation.
//Written by Aditya
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



