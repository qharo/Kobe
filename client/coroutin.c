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
#define SPEED_CIRCULAR    50

int max_speed;  /* Motor maximal speed */

#define DEGREE_TO_COUNT( d )  (( d ) * 260 / 90 )

int app_alive;

enum {
    MODE_MEASURE,
    MODE_MOVE,
};
int mode;

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
int command_drive;  /* Command for the 'drive' coroutine */
int angle_rotate;    /* Angle of rotation */

uint8_t sn_ir;
uint8_t sn_touch;

enum {L, R};
uint8_t motor[ 3 ] = { DESC_LIMIT, DESC_LIMIT, DESC_LIMIT };


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

static void _run_to_rel_pos( int l_speed, int l_pos, int r_speed, int r_pos )
{
    set_tacho_speed_sp( motor[ L ], l_speed );
    set_tacho_speed_sp( motor[ R ], r_speed );
    set_tacho_position_sp( motor[ L ], l_pos );
    set_tacho_position_sp( motor[ R ], r_pos );
    multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
}

static void _stop_ev3( void ) /*stop the ev3 from driving*/
{
    multi_set_tacho_command_inx( motor, TACHO_STOP );
}


int app_init( void )
{
    char s[ 256 ];
    int val;
    uint32_t n, i, ii;
    float value;

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
    command_drive = moving = MOVE_NONE;

    ev3_sensor_init();

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
        }
    }

    if (ev3_search_sensor(LEGO_EV3_US, &sn_ir,0)){
      printf("SONAR found, reading sonar...\n");
      if ( !get_sensor_value0(sn_ir, &value )) {
        value = 0;
      }
      printf( "\r(%f) \n", value);
      fflush( stdout );
    }

    if ( ev3_search_sensor( LEGO_EV3_TOUCH, &sn_touch, 0 )) {
        printf( "TOUCH sensor is found, press BUTTON for EXIT...\n" );
    } else {
        printf( "TOUCH sensor is NOT found, press UP on the EV3 brick for EXIT...\n" );
    }

    printf( "Press BACK on the EV3 brick for EXIT...\n" );
    return ( 1 );
}

CORO_CONTEXT( handle_touch );
CORO_CONTEXT( handle_ir_proximity );
CORO_CONTEXT( drive );


CORO_DEFINE( handle_touch )
{
    CORO_LOCAL int val;
    CORO_BEGIN();
    if ( sn_touch == DESC_LIMIT ) CORO_QUIT();
    for ( ; ; ) {
        /* Waiting the button is pressed */
        CORO_WAIT( get_sensor_value( 0, sn_touch, &val ) && ( val ));
        /* Stop the vehicle */
        command_drive = MOVE_NONE;
        /* Switch mode */
        //_set_mode(( mode == MODE_REMOTE ) ? MODE_AUTO : MODE_REMOTE );
        app_alive = 0;
        /* Waiting the button is released */
        CORO_WAIT( get_sensor_value( 0, sn_touch, &val ) && ( !val ));
    }
    CORO_END();
}

CORO_DEFINE( handle_ir_proximity )
{
    CORO_LOCAL int front, prox;
    CORO_BEGIN();
    for ( ; ; ) {
        /* Waiting self-driving mode */
        CORO_WAIT( mode == MODE_MEASURE );
        prox = 0;
        get_sensor_value( 0, sn_ir, &prox );
        printf("Value of IR Sensor is: %d", prox);
	if ( prox <= 300 ) {  /* Need for detour... */
            front = prox;
            /* Look to the left */
            angle_rotate = -30;
            command_drive = TURN_ANGLE;
            } //else {
            /* Track is clear - Go! */
            //command = MOVE_FORWARD;
        //}
        CORO_YIELD();
    }
    CORO_END();
}

CORO_DEFINE( drive )
{
    CORO_LOCAL int speed_linear, speed_circular;
    CORO_LOCAL int _wait_stopped;
    CORO_BEGIN();
    speed_linear = max_speed * SPEED_LINEAR / 100;
    speed_circular = max_speed * SPEED_CIRCULAR / 100;
    for ( ; ; ) {
        /* Waiting new command */
        CORO_WAIT( moving != command_drive );
        _wait_stopped = 0;
        switch ( command_drive ) {
        case MOVE_NONE:
            _stop_ev3();
            _wait_stopped = 1;
            break;
        case MOVE_FORWARD:
            //_run_forever( -speed_linear, -speed_linear );
            break;
        case MOVE_BACKWARD:
            //_run_forever( speed_linear, speed_linear );
            break;
        case TURN_LEFT:
            //_run_forever( speed_circular, -speed_circular );
            break;
        case TURN_RIGHT:
            //_run_forever( -speed_circular, speed_circular );
            break;
        case TURN_ANGLE:
            _run_to_rel_pos( speed_circular, DEGREE_TO_COUNT( -angle_rotate )
            , speed_circular, DEGREE_TO_COUNT( angle_rotate ));
            _wait_stopped = 1;
            break;
        case STEP_BACKWARD:
            _run_for_time( speed_linear, speed_linear, 1000 );
            _wait_stopped = 1;
            break;
        }
        moving = command_drive;
        if ( _wait_stopped ) {
            /* Waiting the command is completed */
            CORO_WAIT( !_check_if_is_running());
            command_drive = moving = MOVE_NONE;
        }
    }
    CORO_END();
}


int main( void )
{
    printf( "Waiting the EV3 brick online...\n" );
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    ev3_tacho_init();
    app_alive = app_init();
    mode = MODE_MEASURE;
    while ( app_alive ) {
        printf("COMES INTO THE LOOP\n");
	CORO_CALL( handle_ir_proximity );
        CORO_CALL( handle_touch );
        CORO_CALL( drive );
        Sleep( 10 );
    }
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}

