#include <stdio.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif

#define L_MOTOR_PORT      OUTPUT_A          /*the ports are defined in the ev3_ports.h file, line 340->*/
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_

#define IR_CHANNEL        0
#define SPEED_LINEAR      75  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR    50  /* ... for circular motion */

int max_speed;  /* Motor maximal speed */

#define DEGREE_TO_COUNT( d )  (( d ) * 260 / 90 )

int app_alive;

enum {
    MODE_AUTO,    /* Self-driving */
};
int mode;  /* Driving mode */

enum {
    MOVE_NONE,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_ANGLE,
    STEP_BACKWARD,
};
int moving;   /* Current moving */
int command;  /* Command for the 'drive' coroutine */
int angle;    /* Angle of rotation */
uint8_t ir;  /* Sequence numbers of sensor */
enum { L, R };
uint8_t motor[ 3 ] = { DESC_LIMIT, DESC_LIMIT, DESC_LIMIT };  /* Sequence numbers of motors */


static void _set_mode( int value ) //only have one mode, the IR_PROX mode
{
    if ( value == MODE_AUTO ) {
        /* IR measuring of distance */
        set_sensor_mode_inx( ir, LEGO_EV3_IR_IR_PROX );
        mode = MODE_AUTO;
    } 
}
static void _run_forever( int l_speed, int r_speed ) /*this means ev3 drive forever*/
{
    set_tacho_speed_sp( motor[ L ], l_speed );
    set_tacho_speed_sp( motor[ R ], r_speed );
    multi_set_tacho_command_inx( motor, TACHO_RUN_FOREVER );
}
static void _run_to_rel_pos( int l_speed, int l_pos, int r_speed, int r_pos ) /*degrees the robot need to turn*/
{
    set_tacho_speed_sp( motor[ L ], l_speed );
    set_tacho_speed_sp( motor[ R ], r_speed );
    set_tacho_position_sp( motor[ L ], l_pos );
    set_tacho_position_sp( motor[ R ], r_pos );
    multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
}

static void _run_for_time( int l_speed, int r_speed, int ms ) /*set time motor is running, guess this is the one we have used till now*/
{
    set_tacho_speed_sp( motor[ L ], l_speed );
    set_tacho_speed_sp( motor[ R ], r_speed );
    multi_set_tacho_time_sp( motor, ms );
    multi_set_tacho_command_inx( motor, TACHO_RUN_TIMED );
}
static int _check_if_is_running( void ) /*check if the ev3 is driving*/
{
    FLAGS_T state = TACHO_STATE__NONE_;
    get_tacho_state_flags( motor[ L ], &state );
    if ( state != TACHO_STATE__NONE_ ) return ( 1 );
    get_tacho_state_flags( motor[ R ], &state );
    if ( state != TACHO_STATE__NONE_ ) return ( 1 );
    return ( 0 );
}

static void _stop_ev3( void ) /*stop the ev3 from driving*/
{
    multi_set_tacho_command_inx( motor, TACHO_STOP );
}


int app_init( void )
{


	printf("ENTERING INIT\n");


	int i, val;
  	uint32_t n, ii;

    char s[ 16 ];
    if ( ev3_search_tacho_plugged_in( L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0 )) {
        get_tacho_max_speed( motor[ L ], &max_speed );
        /* Reset the motor */
        set_tacho_command_inx( motor[ L ], TACHO_RESET );
    } else {
        printf( "LEFT motor (%s) is NOT found.\n", ev3_port_name( L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s ));
        /* Inoperative without left motor */
        return ( 0 );
    }
    if ( ev3_search_tacho_plugged_in( R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0 )) {
        /* Reset the motor */
        set_tacho_command_inx( motor[ R ], TACHO_RESET );
    } else {
        printf( "RIGHT motor (%s) is NOT found.\n", ev3_port_name( R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s ));
        /* Inoperative without right motor */
        return ( 0 );
    }
    command = moving = MOVE_NONE;

/* code from twc */


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

//  end of code from twc

   // if ( ev3_search_sensor( LEGO_EV3_IR, &ir, 0 )) {
   //     _set_mode( MODE_AUTO );
   // } else {
   //     printf( "IR sensor is NOT found.\n" );
   //     /* Inoperative without IR sensor */
   //     return ( 0 );
   // }


	printf("EXITNG INIT\n");

    printf( "Press BACK on the EV3 brick for EXIT...\n" );
    return ( 1 );
}

CORO_CONTEXT( handle_ir_proximity );
CORO_CONTEXT( drive );


/* Coroutine of IR proximity handling (self-driving)*/
CORO_DEFINE( handle_ir_proximity )
{

	printf("ENTERING PROXIMITY\n");

    int i;
    CORO_LOCAL int front, prox;
    CORO_BEGIN();
    for ( i=0; i<1000000; i++ ) {
	printf("COROUTINE FOR HANDLE IR RUNNING, INSIDE THE FOR LOOP\n");
        /* Waiting self-driving mode */
        CORO_WAIT( mode == MODE_AUTO );
        prox = 0;
        get_sensor_value( 0, ir, &prox );

	    printf("Value of front is %d", prox);
 

        if ( prox == 0 ) {
            /* Oops! Stop the vehicle */
            command = MOVE_NONE;
        //if this close first look to the left and check dist, then look to the right and check dist.
        } else if ( prox < 20 ) {  /* Need for detour... */
            front = prox;
           /* Look to the left */
            angle = -30;
            command = TURN_ANGLE;
            CORO_WAIT( command == MOVE_NONE );
            prox = 0;
            get_sensor_value( 0, ir, &prox );
            if ( prox > front + 5) {
                //look to the right
                angle = 60;
                command = TURN_ANGLE;
                CORO_WAIT( command == MOVE_NONE );
                prox = 0;
                get_sensor_value( 0, ir, &prox );
                if (prox > front + 5){
                    angle = -30;
                    command = TURN_ANGLE;
                    CORO_WAIT( command == MOVE_NONE );
                    prox = 0;
                    get_sensor_value( 0, ir, &prox );
                    command = MOVE_FORWARD;
                    CORO_WAIT( command == MOVE_NONE );
                }
            }
        } else {
            /* Track is clear - Go! */
            command = MOVE_FORWARD;
        }

	printf("YIELDING PROXIMITY\n");

        CORO_YIELD();
    }

	printf("EXITING PROXIMITY\n");


    CORO_END();
}
/* Coroutine of control the motors */
CORO_DEFINE( drive )
{

	printf("ENTERING DRIVING\n");


    int i;
    CORO_LOCAL int speed_linear, speed_circular;
    CORO_LOCAL int _wait_stopped;
    CORO_BEGIN();
    speed_linear = max_speed * SPEED_LINEAR / 100;
    speed_circular = max_speed * SPEED_CIRCULAR / 100;
    for ( i=0; i<1000000; i++ ) {
        /* Waiting new command */
        CORO_WAIT( moving != command );
        _wait_stopped = 0;
        switch ( command ) {
        case MOVE_NONE:
            _stop_ev3();
            _wait_stopped = 1;
            break;
        case MOVE_FORWARD:
            _run_forever( speed_linear, speed_linear );
            break;
        case MOVE_BACKWARD:
            _run_forever( -speed_linear, -speed_linear );
            break;
        case TURN_LEFT:
            _run_forever( -speed_circular, speed_circular );
            break;
        case TURN_RIGHT:
            _run_forever( speed_circular, -speed_circular );
            break;
        case TURN_ANGLE:
            _run_to_rel_pos( speed_circular, DEGREE_TO_COUNT( -angle )
            , speed_circular, DEGREE_TO_COUNT( angle ));
            _wait_stopped = 1;
            break;
        case STEP_BACKWARD:
            _run_for_time( speed_linear, speed_linear, 1000 );
            _wait_stopped = 1;
            break;
        }
        moving = command;
        if ( _wait_stopped ) {
            /* Waiting the command is completed */
            CORO_WAIT( !_check_if_is_running());
            command = moving = MOVE_NONE;
        }
    }

	printf("EXITING DRIVE\n");


    CORO_END();
}
int main( void )
{
    printf( "Waiting the EV3 brick online...\n" );
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    ev3_sensor_init();
    ev3_tacho_init();
    app_alive = app_init();
    while ( app_alive ) {
        CORO_CALL( handle_ir_proximity );
        CORO_CALL( drive );
        Sleep( 10 );
    }
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}
