////------------------------------------------------------////
////------------------------------------------------------////
////                     Rabbit 3400                      ////
////                     Ateryx 2.5                       ////
////                     Ryan G. Beall                    ////
////------------------------------------------------------////
////------------------------------------------------------////

//-----------------------Definitions------------------------//
//#define USE_HIL 1
#define USE_MTEK_GPS 1
//#define USE_SONAR_ALTITUDE 1
//#define USE_SD_LOGGER 1
//#define USE_MAG 1

#define DEG2RAD 0.017453292519943
#define RAD2DEG 57.2957795130823
#define FT2MET  0.3048
#define MET2FT  3.28083989501
#define GRAVITY 9.80665 //32.174

//----Co-function timing capabilities----//
#define COF_READ_MAG_RAW_TIME_SLOT .016
#define COF_COMPUTE_MAG_TIME_SLOT .013
#define COF_TEMP_COMP_TIME_SLOT .017
#define COF_POWER_TIME_SLOT .016
#define COF_SD_LOGGER_TIME_SLOT .015
#define COF_SEND_MAVLINK_FAST .015
#define COF_SEND_MAVLINK_SLOW .017
#define COF_WP_NAV_TIME_SLOT .017 //2ms worst case
#define COF_LOST_COMM_DETECT_TIME_SLOT .016
#define COF_AIRBORNE_DETECT_TIME_SLOT 0.017

//-------------------------Includes-------------------------//
#use "AT_HEADER.LIB"
#use "AT_UTILITIES.LIB"
#use "AT_INIT.LIB"
#use "AT_SENSORS.LIB"
#use "AT_ESTIMATE_STATES.LIB"
#use "AT_MAG.LIB"
#use "AT_FLASH.LIB"
#use "AT_UBLOX.LIB"
#use "AT_MTEK.LIB"
#use "AT_CONTROL.LIB"
#use "AT_SERVOS.LIB"
#use "AT_NAVIGATION.LIB"

#use "mavlink_types.lib"
#use "checksum.lib"
#use "mavlink_utilities.lib"
#use "AT_SD_TEMPCOMP.LIB"

#use "mavlink_msg_heartbeat.lib"
#use "mavlink_msg_command_ack.lib"
#use "mavlink_msg_sys_status.lib"
#use "mavlink_msg_vfr_hud.lib"
#use "mavlink_msg_rc_scaled.lib"
#use "mavlink_msg_rc_raw.lib"
#use "mavlink_msg_attitude.lib"
#use "mavlink_msg_gps_global_origin.lib"
#use "mavlink_msg_gps_raw_int.lib"
#use "mavlink_msg_nav_output.lib"
#use "mavlink_msg_scaled_imu.lib"
#use "mavlink_msg_param_request_list.lib"
#use "mavlink_msg_param_value.lib"
#use "mavlink_msg_param_set.lib"

#use "mavlink_msg_mission.lib"
#use "mavlink_msg_mission_ack.lib"
#use "mavlink_msg_mission_count.lib"
#use "mavlink_msg_mission_current.lib"
#use "mavlink_msg_mission_request.lib"

#use "mavlink_comms.lib"

//---------------------Global Variables---------------------//
// Note: all individual globals are prefaced with g_
static char				m_bytes[NUMBER_OF_BYTES];
static xmem int16_t	m_ints[NUMBER_OF_INTS];
static xmem float		m_floats[NUMBER_OF_FLOATS];
static float			ram_UAV_limits[NUMBER_OF_UAV_LIMITS];

cmd g_cmds[256];								 // command structures holding UAV's flight plan
stack g_cmd_stack;							 // command stack structure used in the navigation library
static unsigned int g_number_of_commands;

PID_loop g_PIDS[9];		// PID feedback control structures
servo g_servos[NUMBER_OF_SERVO_STRUCTURES];	// Global servo control structures

static long g_old_time;	// Last Main loop time for dt calc
static long g_main_loop_start_time;

//Temp comp variables
static temp_comp g_tc;
static float g_ax_scale,g_ay_scale,g_az_scale;
static float g_press_to_alt[3];

static sensor sensors[NUMBER_OF_SENSORS];

//GPS variables
static char UBX_buffer[UBX_MAXPAYLOAD];
static char UBX_class,UBX_id;
static char MTK_buffer[MTK_MAXPAYLOAD];
static int gps_fix_flag;
static char chk_a,chk_b;

//Attitude variables
static float q0,q1,q2,q3;
static float phi_est,theta_est,psi_est;

//MAVlink Variables
mavlink_system_t mavlink_system;
static unsigned int g_mavlink_target_sysid;
static unsigned int g_mavlink_target_compid;
const int system_type = MAV_TYPE_FIXED_WING;
const int autopilot_type = MAV_AUTOPILOT_GENERIC;
char msg_status_txt[14]; //used for sending text msg or param names
char packet_sequence;
char g_crc_seeds[256]; //init in AT_INIT

//Flight Law Variables
char case_cmd_int;

//PWM Variables
unsigned int g_soft_pwm_state;
const int g_timerb_inc = PWM_MATCH_1MS&0xFF | ((PWM_MATCH_1MS<<6) & 0xC000);
unsigned int g_timerb_match;
//------------------------Prototypes------------------------//


//----Compiler Settings----//
#memmap xmem	//Defaults functions to extended memory
//#class auto		// All varaibles are now defaulted to normal C standard
nodebug
//----------------------------------------------------------//
//----------------------------------------------------------//
//                          MAIN                            //
//----------------------------------------------------------//
//----------------------------------------------------------//

xmem  void main(void)
{
//----Local Variables----//
    int i;

//----Initialization----//
    g_old_time = 0;
    g_main_loop_start_time = 0;

    //Init temp comp vars
    g_ax_scale =0;
    g_ay_scale =0;
    g_az_scale =0;
    g_press_to_alt[2] = 81.164;
    g_press_to_alt[1] = -4199.3;
    g_press_to_alt[0] = 44192.0; //standard day offset

    // Clear out Arrays
    memset(m_bytes,  0, sizeof(m_bytes));
    memset(m_ints,   0, sizeof(m_ints));
    memset(m_floats, 0, sizeof(m_floats));
    memset(ram_UAV_limits, 0, sizeof(ram_UAV_limits));
    memset(sensors,  0, sizeof(sensors));
    memset(&g_tc,  0, sizeof(g_tc));
    memset(g_PIDS,   0, sizeof(g_PIDS));
    for(i=0; i<15; i++)
        g_PIDS[i].loop_id = i;
    memset(g_servos, 0, sizeof(g_servos));

    g_number_of_commands = 1;
    memset(&g_cmds,      0, sizeof(g_cmds));
    memset(&g_cmd_stack, 0, sizeof(g_cmd_stack));
    m_ints[MI_UAV_MODE] = MAV_CMD_NAV_LOITER_UNLIM;

    m_ints[MI_FLC] &=FLC_ELEV_RESET;
    m_ints[MI_FLC] |= FLC_ELEV_PITCH;
    m_ints[MI_FLC] &=FLC_AILE_RESET;
    m_ints[MI_FLC] |= FLC_AILE_ROLL;
    m_ints[MI_FLC] &=FLC_ROLL_RESET;
    m_ints[MI_FLC] |= FLC_ROLL_HEADING;
    m_ints[MI_FLC] &=FLC_THRO_RESET;
    m_ints[MI_FLC] |= FLC_THRO_EH_PITCH_AIRSPD;

    memset(&g_crc_seeds, 0, sizeof(g_crc_seeds));
#ifdef USE_HIL
	 m_bytes[MB_HIL_STATUS] = 1;
#endif


    //Init GPS vars
    gps_fix_flag =0;
    chk_a = 0;
    chk_b = 0;

    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    phi_est = 0.0;
    theta_est = 0.0;
    psi_est = 0.0;

	//init MAVlink vars
   mavlink_system.sysid = 7;  ///< Used by the MAVLink message_xx_send() convenience function
   mavlink_system.compid = 1; ///< Used by the MAVLink message_xx_send() convenience function
   mavlink_system.type = 0;    ///< Unused, can be used by user to store the system's type
   mavlink_system.state = 0;   ///< Unused, can be used by user to store the system's state
   mavlink_system.mode = 0;    ///< Unused, can be used by user to store the system's mode

   g_mavlink_target_sysid = 255;
   g_mavlink_target_compid = 0;

	packet_sequence = 0;

   g_timerb_match = 0x00;

    Init_Ateryx();
    paledoff(Rx_led);
    paledoff(Tx_led);
    paledoff(GPS_led);

    readFlash(0); //restore all values from flash
    init_servos();

    MsDelay(100);
    for (i=0;i<100;i++)
    	{
    		gather_sensors();
      }
    zero_pressure(1);

    led_tester();

    init_soft_pwm();

    printf("\r\nAteryx 2.5 Init");
//----------------------------------------------------------//
//                        MAIN Loop                         //
//----------------------------------------------------------//
while(1)
{
   //Calculate dt (time between loops in seconds)
   m_floats[MF_DT] = (float)(MS_TIMER - g_old_time)*10e-4;
   g_old_time = MS_TIMER;
   g_main_loop_start_time = MS_TIMER;

#ifndef USE_HIL
	gather_sensors();
   estimate_states();
#endif

   low_level_control();
   update_servos();

//----------------------------------------------------------//
//                        Co-States                         //
//----------------------------------------------------------//

//.5 ms
costate{ wfd cof_temp_comp(); }		// Determine sensor temp comp

#ifdef  USE_SD_LOGGER
costate{ wfd cof_sd_logger(); }		//log data to sd card
#endif

//.5 ms
costate{ wfd cof_power(); }
#ifdef USE_MAG
costate{ wfd cof_read_mag_raw(); }
#endif

//costate{ wfd cof_debug_prints(); }

#ifndef USE_HIL
#ifdef USE_MTEK_GPS
costate{ wfd cof_read_mtek(); }
#endif
#ifndef USE_MTEK_GPS
costate{ wfd cof_read_ublox(); }
#endif
#endif

costate{ wfd cof_get_mavlink(); }
costate{ wfd cof_send_mavlink_slow(); }
costate{ wfd cof_send_mavlink_fast(); }

costate{ wfd cof_waypoint_navigation(); }
//1 ms
costate{ wfd cof_lost_com_detect(); }
costate{ wfd cof_check_if_airborne(); }

//costate{ wfd cof_blink_led(500,GPS_led,1); }
}//End Main while
}//End Main

