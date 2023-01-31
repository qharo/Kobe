#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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

// ROTATION DIRECTIONS /////////////////////
#define RIGHT 0
#define LEFT 1
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

// BOOL ///////////////////////////////////
#define TRUE 1
#define FALSE 0
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

// PID PARAMETERS ////////////////////////
#define KP 0.5
#define KI 0.01
#define KD 0.1
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
    #include <windows.h>
// UNIX //////////////////////////////////////////
#else
    #include <unistd.h>
#endif


#define Sleep( msec ) usleep(( msec ) * 1000 )

int time_step = 1;  // sampling ratio of sensor, ms

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

CORO_CONTEXT( fetch_sonar );
CORO_CONTEXT( move_bitch );

int init_robot();

// MAIN
int main( void ) {
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
            printf( "  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
        }
    }

    init_robot();

    while(1) {
        CORO_CALL( fetch_sonar );
        CORO_CALL( move_bitch );
        Sleep(10);
    }
    
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );

    return ( 0 );
}



int init_robot(){
	int i, val;
  	uint32_t n, ii;

	for(i=0;i<4;i++){
		robot.PORTS[i] = i+65;
	}
    // first, we initialize the wheels
	if( ev3_search_tacho_plugged_in(robot.PORTS[0], 0, &(robot.WHEELS_SN[0]), 0) ){
		printf("\nLEFT WHEEL INITIALIZED!\n");
	}
	if( ev3_search_tacho_plugged_in(robot.PORTS[1], 0, &(robot.WHEELS_SN[1]), 0) ){
		printf("RIGHT WHEEL INITIALIZED!\n");
	}

    // then, we initialize the ultrasonic sensor
    // SEARCHING FOR SONAR
	if (ev3_search_sensor(LEGO_EV3_US, &( robot.SONAR),0)){
      	printf("Sonar found! \n");
		robot.SONAR_VAL = 0;
		// set the sampling interval
		set_sensor_mode(robot.SONAR, "US-DIST-CM");
		set_sensor_poll_ms(robot.SONAR, time_step);
    }

	return 0;
}

  while(robot.MAMBA_MODE){
	CORO_CALL( fetch_sonar );
	CORO_CALL( fetch_gyro );
	CORO_CALL( defence );
	CORO_CALL( handle_touch );
	Sleep(10);
  }


// FETCH_SONAR
CORO_DEFINE( fetch_sonar ){

	CORO_LOCAL float value;
	CORO_LOCAL int counter=0;
	CORO_BEGIN();

	for(;;){
		get_sensor_value0(robot.SONAR, &value );
		robot.SONAR_VAL = value;
		//printf("IR Val %f\n", robot.SONAR_VAL);
        /*
		if(robot.SEEK_MODE){
			robot.SONAR_OBS[counter] = robot.SONAR_VAL;
			counter++;
			robot.NOBS = counter;
		}
        */
		CORO_YIELD();
	}

	CORO_END();
}

CORO_DEFINE( move_bitch ) {
    CORO_BEGIN();

    // Motor speed
    CORO_LOCAL int speed = 50;

    // Initialize the variables for the PID controller
    CORO_LOCAL float error = 0;
    CORO_LOCAL float integral = 0;
    CORO_LOCAL float derivative = 0;
    CORO_LOCAL float previous_error = 0;

    CORO_LOCAL float target_distance = 20;
    CORO_LOCAL float output;

    // Get the current distance
    CORO_LOCAL float current_distance;
    current_distance = robot.SONAR_VAL;

    // Run the PID controller
    while(fabs(current_distance - target_distance) > 0.1) {
        // Get the current distance
        current_distance = robot.SONAR_VAL;
        // Calculate the error
        error = target_distance - current_distance;
        // Calculate the integral
        integral += error*time_step;
        // Calculate the derivative
        derivative = (error - previous_error)/time_step;
        // Calculate the output
        output = KP * error + KI * integral + KD * derivative;
        // Calculate the motor speeds
        speed = speed + output;
        // Set the motor speeds
        set_tacho_speed_sp(robot.WHEELS_SN[RIGHT], speed);
        set_tacho_speed_sp(robot.WHEELS_SN[LEFT], speed);
        set_tacho_command_inx(robot.WHEELS_SN[RIGHT], TACHO_RUN_FOREVER);
	    set_tacho_command_inx(robot.WHEELS_SN[LEFT], TACHO_RUN_FOREVER);
        // Update the previous error
        previous_error = error;
        // Wait for 100ms
        Sleep(100);
        printf("distance: %f, P: %f, I: %f, D: %f\n", current_distance, error, integral, derivative);

        CORO_YIELD();
    }
    set_tacho_command_inx(robot.WHEELS_SN[RIGHT], TACHO_STOP);
	set_tacho_command_inx(robot.WHEELS_SN[LEFT], TACHO_STOP);

    CORO_END();
}




// MOVE FORWARD
int forward(float target_distance){

    // Motor speed
    int speed = 50;

    // Initialize the variables for the PID controller
    float error = 0;
    float integral = 0;
    float derivative = 0;
    float previous_error = 0;

    // Get the current distance
    float current_distance;
    current_distance = robot.SONAR_VAL;

    // Run the PID controller
    while(fabs(current_distance - target_distance) > 0.1) {
        // Get the current distance
        current_distance = robot.SONAR_VAL;
        // Calculate the error
        error = target_distance - current_distance;
        // Calculate the integral
        integral += error*time_step;
        // Calculate the derivative
        derivative = (error - previous_error)/time_step;
        // Calculate the output
        float output = KP * error + KI * integral + KD * derivative;
        // Calculate the motor speeds
        speed = speed + output;
        // Set the motor speeds
        set_tacho_speed_sp(robot.WHEELS_SN[RIGHT], speed);
        set_tacho_speed_sp(robot.WHEELS_SN[LEFT], speed);
        set_tacho_command_inx(robot.WHEELS_SN[RIGHT], TACHO_RUN_FOREVER);
	    set_tacho_command_inx(robot.WHEELS_SN[LEFT], TACHO_RUN_FOREVER);
        // Update the previous error
        previous_error = error;
        // Wait for 100ms
        Sleep(100);
        printf("distance: %f, P: %d, I: %d, D: %d\n", current_distance, error, integral, derivative);
    }
    set_tacho_command_inx(robot.WHEELS_SN[RIGHT], TACHO_STOP);
	set_tacho_command_inx(robot.WHEELS_SN[LEFT], TACHO_STOP);

	return 0;
}