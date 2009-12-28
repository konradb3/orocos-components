////////////////////////////////////////////////////////////////////////////////
/*! \file     src/lib/com_buf.h
 *
 *  Data structures for IPC.
 *
 *  \author   tkornuta
 *  \date     2006-11-29
 *  \URL: https://segomo.elka.pw.edu.pl/svn/mrrocpp/base/trunk/include/lib/com_buf.h $
 *  $LastChangedRevision: 3273 $
 *  $LastChangedDate: 2009-12-18 22:10:13 +0100 (Fri, 18 Dec 2009) $
 *  $LastChangedBy: konradb3 $
 *
 *  \todo <ul>
 *          <li>Translate to English where necessary.</li>
 *          <li>Write detailed comments.</li>
 *          <li>Suplement comments for those consts, variables and structures
 *              that are not commented at all.</li>
 *          <li>Clean up the commented fragments of code.</li>
 *        </ul>
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef __COM_BUF_H
#define __COM_BUF_H

//#include "typedefs.h"
//#include "impconst.h"

#include <messip.h>

#include <boost/serialization/serialization.hpp>

#define MAX_SERVOS_NR 8
enum STATE
{
	GET_STATE,
	GET_SYNCHRO,
	SYNCHRO_TERMINATED,
	GET_INSTRUCTION,
	EXECUTE_INSTRUCTION,
	WAIT,
	WAIT_Q
};

namespace mrrocpp
{

namespace lib
{

typedef double frame_tab[3][4];
//------------------------------------------------------------------------------
/*!
 *  Type of command sent from MP to ECP.
 */
enum MP_COMMAND
{
	INVALID_COMMAND, START_TASK, NEXT_POSE, END_MOTION, NEXT_STATE, STOP
};

//------------------------------------------------------------------------------
/*!
 *  Type of reply from ECP to the MP command.
 */
enum ECP_REPLY
{
	INCORRECT_MP_COMMAND, ERROR_IN_ECP, ECP_ACKNOWLEDGE, TASK_TERMINATED
};

//------------------------------------------------------------------------------
/*!
 *  Type of arm position definition.
 */
enum POSE_SPECIFICATION
{
	INVALID_END_EFFECTOR, FRAME, JOINT, MOTOR, PF_VELOCITY
};

//------------------------------------------------------------------------------
/*!
 *  Type of arm position definition on the ECP level.
 */
enum ECP_POSE_SPECIFICATION
{
	ECP_INVALID_END_EFFECTOR,
	ECP_XYZ_ANGLE_AXIS,
	ECP_XYZ_EULER_ZYZ,
	ECP_JOINT,
	ECP_MOTOR
};

//------------------------------------------------------------------------------
/*!
 *  Reply types from UI to ECP and commands from UI (pressing a button).
 */
enum UI_TO_ECP_COMMAND
{
	INVALID_REPLY,
	NEXT,
	QUIT,
	ANSWER_YES,
	ANSWER_NO,
	FILE_LOADED,
	FILE_SAVED,

	/*! Commands from Force Control window. */
	FC_ADD_MACROSTEP,
	FC_CALIBRATE_SENSOR,
	FC_CHANGE_CONTROL,
	FC_MOVE_ROBOT,
	FC_SAVE_TRAJECTORY,
	FC_NEW_TRAJECTORY,
	FC_EXIT,
	FC_GET_DATA,

	/*! Commands from Trajectory Render window. */
	TR_LOAD_TRAJECTORY,
	TR_PAUSE_MOVE,
	TR_START_MOVE,
	TR_STOP_MOVE,
	TR_EXIT,
	TR_ZERO_POSITION,
	TR_SAVE_READINGS,
	TR_CALIBRATE_DIGITAL_SCALES_SENSOR,
	TR_CALIBRATE_FORCE_SENSOR,
	TR_TRY_MOVE_AGAIN,

	/*! Replies from the options window. */
	OPTION_ONE,
	OPTION_TWO,
	OPTION_THREE,
	OPTION_FOUR,

	/*!
	 *  Commands from the window
	 *  MAM_wnd_manual_moves_automatic_measures.
	 */
	MAM_START,
	MAM_STOP,
	MAM_CLEAR,
	MAM_SAVE,
	MAM_EXIT,
	MAM_CALIBRATE
};

//------------------------------------------------------------------------------
/*!
 *  Types of ECP to UI commands.
 */
enum ECP_TO_UI_COMMAND
{
	C_INVALID_END_EFFECTOR,
	C_FRAME,
	C_XYZ_ANGLE_AXIS,
	C_XYZ_EULER_ZYZ,
	C_JOINT,
	C_MOTOR,
	YES_NO,
	DOUBLE_NUMBER,
	INTEGER_NUMBER,
	SAVE_FILE,
	LOAD_FILE,
	MESSAGE,
	OPEN_FORCE_SENSOR_MOVE_WINDOW,
	OPEN_TRAJECTORY_REPRODUCE_WINDOW,
	TR_REFRESH_WINDOW,
	TR_DANGEROUS_FORCE_DETECTED,
	CHOOSE_OPTION,
	MAM_OPEN_WINDOW,
	MAM_REFRESH_WINDOW
};

//------------------------------------------------------------------------------
/*!
 *  Types of processes in MRROC++.
 */
typedef enum _PROCESS_TYPE
{
	UNKNOWN_PROCESS_TYPE, EDP, ECP, MP, VSP, UI
} process_type_t;

//------------------------------------------------------------------------------
/*!
 *  Definitions for available values of set_type i get_type.
 *  @author yoyek
 */
#define CONTROLLER_STATE_DV                     0x08
#define ARM_DV                                  0x04
#define RMODEL_DV                               0x02
#define OUTPUTS_DV                              0x01
#define NOTHING_DV                              0x00

//------------------------------------------------------------------------------
/*! Error numbers generated in EDP.	*/
#define OK                                      0x0000000000000000ULL

#define INVALID_INSTRUCTION_TYPE                0x0100000000000000ULL
#define INVALID_REPLY_TYPE                      0x0200000000000000ULL
#define INVALID_SET_RMODEL_TYPE                 0x0300000000000000ULL
#define INVALID_GET_RMODEL_TYPE                 0x0400000000000000ULL
#define ERROR_IN_RMODEL_REQUEST                 0x0500000000000000ULL
#define INVALID_HOMOGENEOUS_MATRIX              0x0600000000000000ULL

#define QUERY_EXPECTED                          0x1000000000000000ULL
#define QUERY_NOT_EXPECTED                      0x1100000000000000ULL
#define NO_VALID_END_EFFECTOR_POSE              0x1200000000000000ULL
#define INVALID_MOTION_TYPE                     0x1300000000000000ULL
#define INVALID_MOTION_PARAMETERS               0x1400000000000000ULL
#define INVALID_SET_END_EFFECTOR_TYPE           0x1500000000000000ULL
#define INVALID_GET_END_EFFECTOR_TYPE           0x1600000000000000ULL
#define STRANGE_GET_ARM_REQUEST                 0x1700000000000000ULL

//------------------------------------------------------------------------------
/*! Range exceeding error codes (nonfatal) - drive shaft. */
#define BEYOND_UPPER_LIMIT_AXIS_0               0x2000000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_1               0x2100000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_2               0x2200000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_3               0x2300000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_4               0x2400000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_5               0x2500000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_6               0x2600000000000000ULL
#define BEYOND_UPPER_LIMIT_AXIS_7               0x2700000000000000ULL

#define BEYOND_LOWER_LIMIT_AXIS_0               0x2800000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_1               0x2900000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_2               0x2A00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_3               0x2B00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_4               0x2C00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_5               0x2D00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_6               0x2E00000000000000ULL
#define BEYOND_LOWER_LIMIT_AXIS_7               0x2F00000000000000ULL

//------------------------------------------------------------------------------
/*! Range exceeding error codes (nonfatal) - internal coordinates. */
#define BEYOND_UPPER_D0_LIMIT                   0x3000000000000000ULL
#define BEYOND_UPPER_THETA1_LIMIT               0x3100000000000000ULL
#define BEYOND_UPPER_THETA2_LIMIT               0x3200000000000000ULL
#define BEYOND_UPPER_THETA3_LIMIT               0x3300000000000000ULL
#define BEYOND_UPPER_THETA4_LIMIT               0x3400000000000000ULL
#define BEYOND_UPPER_THETA5_LIMIT               0x3500000000000000ULL
#define BEYOND_UPPER_THETA6_LIMIT               0x3600000000000000ULL
#define BEYOND_UPPER_THETA7_LIMIT               0x3700000000000000ULL

#define BEYOND_LOWER_D0_LIMIT                   0x3800000000000000ULL
#define BEYOND_LOWER_THETA1_LIMIT               0x3900000000000000ULL
#define BEYOND_LOWER_THETA2_LIMIT               0x3A00000000000000ULL
#define BEYOND_LOWER_THETA3_LIMIT               0x3B00000000000000ULL
#define BEYOND_LOWER_THETA4_LIMIT               0x3C00000000000000ULL
#define BEYOND_LOWER_THETA5_LIMIT               0x3D00000000000000ULL
#define BEYOND_LOWER_THETA6_LIMIT               0x3E00000000000000ULL
#define BEYOND_LOWER_THETA7_LIMIT               0x3F00000000000000ULL

#define OUT_OF_WORKSPACE                        0x4100000000000000ULL
#define SINGULAR_POSE                           0x4200000000000000ULL

//------------------------------------------------------------------------------
/*! Standard mathematical functions - argument out of domain. */
#define ACOS_DOMAIN_ERROR                       0x4300000000000000ULL
#define ASIN_DOMAIN_ERROR                       0x4400000000000000ULL
#define ATAN2_DOMAIN_ERROR                      0x4500000000000000ULL
#define SQRT_DOMAIN_ERROR                       0x4600000000000000ULL
#define UNKNOWN_MATH_ERROR                      0x4700000000000000ULL

#define UNKNOWN_INSTRUCTION                     0x4800000000000000ULL
#define NOT_IMPLEMENTED_YET                     0x4900000000000000ULL

#define NOT_YET_SYNCHRONISED                    0x5200000000000000ULL
#define ALREADY_SYNCHRONISED                    0x5300000000000000ULL
#define UNKNOWN_SYNCHRO_ERROR                   0x5400000000000000ULL
#define INVALID_KINEMATIC_MODEL_NO              0x5500000000000000ULL
#define INVALID_KINEMATIC_CORRECTOR_NO          0x5600000000000000ULL
#define EDP_UNIDENTIFIED_ERROR                  0x5700000000000000ULL

#define NOT_A_NUMBER_JOINT_VALUE_D0             0x6000000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA1         0x6100000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA2         0x6200000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA3         0x6300000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA4         0x6400000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA5         0x6500000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA6         0x6600000000000000ULL
#define NOT_A_NUMBER_JOINT_VALUE_THETA7         0x6700000000000000ULL

//------------------------------------------------------------------------------
/*! Errors detected by SERVO_GROUP. */
#define SERVO_ERROR_IN_PASSIVE_LOOP             0x0004000000000000ULL
/*! Wrong command from EDP_MASTER. */
#define UNIDENTIFIED_SERVO_COMMAND              0x0008000000000000ULL
#define SERVO_ERROR_IN_PHASE_1                  0x000C000000000000ULL
#define SERVO_ERROR_IN_PHASE_2                  0x0010000000000000ULL

//------------------------------------------------------------------------------
/*! Errors detected in servo synchronization. */
#define SYNCHRO_SWITCH_EXPECTED                 0x0014000000000000ULL
#define SYNCHRO_ERROR                           0x0018000000000000ULL
#define SYNCHRO_DELAY_ERROR                     0x001C000000000000ULL

//------------------------------------------------------------------------------
/*! Error classes. */
typedef enum _ERROR_CLASS
{
	NEW_MESSAGE, SYSTEM_ERROR, FATAL_ERROR, NON_FATAL_ERROR
} error_class_t;

//------------------------------------------------------------------------------
/*! Detailed errors generated by ECP and MP. */
#define INVALID_MP_COMMAND                       0x1ULL
#define INVALID_POSE_SPECIFICATION               0x2ULL
#define INVALID_RMODEL_TYPE                      0x3ULL
#define INVALID_ECP_COMMAND                      0x4ULL
#define INVALID_EDP_REPLY                        0x5ULL
#define ECP_ERRORS                               0x6ULL
#define INVALID_COMMAND_TO_EDP                   0x7ULL
#define ECP_UNIDENTIFIED_ERROR                   0x8ULL
#define MP_UNIDENTIFIED_ERROR                    0x9ULL
#define EDP_ERROR                                0xAULL
#define NON_EXISTENT_DIRECTORY                   0xBULL
#define NON_EXISTENT_FILE                        0xCULL
#define READ_FILE_ERROR                          0xDULL
#define NON_TRAJECTORY_FILE                      0xEULL
#define NON_COMPATIBLE_LISTS                     0xFULL
#define ECP_STOP_ACCEPTED                       0x10ULL
#define MAX_ACCELERATION_EXCEEDED               0x11ULL
#define MAX_VELOCITY_EXCEEDED                   0x12ULL
#define NOT_ENOUGH_MEMORY                       0x13ULL
#define INVALID_TIME_SPECIFICATION              0x14ULL
#define INVALID_ECP_PULSE_IN_MP_START_ALL       0x16ULL
#define INVALID_ECP_PULSE_IN_MP_EXECUTE_ALL     0x17ULL
#define INVALID_ECP_PULSE_IN_MP_TERMINATE_ALL   0x18ULL

//------------------------------------------------------------------------------
/*! Detailed errors generated by ECP and MP to VSP - type SYSTEM_ERROR. */
#define CANNOT_SPAWN_VSP                        0x20ULL
#define CANNOT_LOCATE_DEVICE                    0x21ULL
#define CANNOT_READ_FROM_DEVICE                 0x22ULL
#define CANNOT_WRITE_TO_DEVICE                  0x23ULL
#define DEVICE_ALREADY_EXISTS                   0x24ULL

/*! Detailed errors generated by ECP and MP to VSP - type FATAL_ERROR. */
#define BAD_VSP_REPLY                           0x25ULL

/*! Detailed errors generated by ECP and MP to VSP - type NON_FATAL_ERROR. */
#define INVALID_VSP_REPLY                       0x26ULL

/*! Detailed errors generated by ECP and MP to VSP - other errors. */
#define VSP_UNIDENTIFIED_ERROR                  0x27ULL

/*! Detailed errors generated by ECP and MP to VSP - in a trajectory generator. */
#define DANGEROUS_FORCE_DETECTED                0x28ULL
#define SAVE_FILE_ERROR                         0x29ULL
#define NAME_ATTACH_ERROR                       0x2AULL

//------------------------------------------------------------------------------
/*! Detailed errors generated by VSP - type SYSTEM_ERROR. */
#define DISPATCH_ALLOCATION_ERROR                0x1ULL
#define DEVICE_EXISTS                            0x2ULL
#define DEVICE_CREATION_ERROR                    0x3ULL
#define DISPATCH_LOOP_ERROR                      0x4ULL

/*! Detailed errors generated by VSP - type FATAL_ERROR. */
#define SENSOR_NOT_CONFIGURED                    0x5ULL
#define READING_NOT_READY                        0x6ULL

/*! Detailed errors generated by VSP - type NON_FATAL_ERROR. */
#define INVALID_COMMAND_TO_VSP                   0x7ULL

/*! Detailed errors generated by VSP - type FATAL_ERROR in cyclic buffer. */
#define CYCLIC_BUFFER_PARSE_ERROR               0x08ULL
#define CYCLIC_BUFFER_UNDERRUN                  0x09ULL
#define CYCLIC_BUFFER_OVERFLOW                  0x10ULL

/*!
 *  Detailed errors generated by VSP - errors concerning CRS
 *  (finding a solutions for the Rubic's cube).
 */
#define RCS_INVALID_STATE                       0x10ULL
#define RCS_EXCEPTION                           0x11ULL

//------------------------------------------------------------------------------
enum GRIPPER_STATE_ENUM
{
	GRIPPER_START_STATE,
	GRIPPER_EXPAND_STATE,
	GRIPPER_NARROW_STATE,
	GRIPPER_BLOCKED_AFTER_EXPAND_STATE,
	GRIPPER_BLOCKED_AFTER_NARROW_STATE,
	GRIPPER_BLOCKED_STATE
};

//------------------------------------------------------------------------------
enum INSTRUCTION_TYPE
{
	INVALID, SET, GET, SET_GET, SYNCHRO, QUERY
};

//------------------------------------------------------------------------------
enum RMODEL_SPECIFICATION
{
	INVALID_RMODEL,
	TOOL_FRAME,
	ARM_KINEMATIC_MODEL,
	SERVO_ALGORITHM,
	FORCE_TOOL,
	FORCE_BIAS
};

//------------------------------------------------------------------------------
enum MOTION_TYPE
{
	ABSOLUTE, RELATIVE
};

//------------------------------------------------------------------------------
// MOTOR_INTERPOLATED_MOTION TASK_COORDINATES_INTERPOLATED_MOTION
enum INTERPOLATION_TYPE
{
	MIM, TCIM
};

//------------------------------------------------------------------------------
enum REPLY_TYPE
{
	ERROR,
	ACKNOWLEDGE,
	SYNCHRO_OK,
	ARM,
	RMODEL,
	INPUTS,
	ARM_RMODEL,
	ARM_INPUTS,
	RMODEL_INPUTS,
	ARM_RMODEL_INPUTS,
	CONTROLLER_STATE
/*
 * TODO: would not be it easier to handle with the following?
 ERROR = 0,
 ACKNOWLEDGE = 0x01,
 SYNCHRO_OK = 0x02,
 ARM = 0x04,
 RMODEL = 0x08,
 INPUTS = 0x10,
 ARM_RMODEL = 0x20,
 ARM_INPUTS = 0x40,
 RMODEL_INPUTS = 0x80,
 ARM_RMODEL_INPUTS = 0x100,
 CONTROLLER_STATE = 0x200
 */
};

//------------------------------------------------------------------------------
/*! @todo Rename from "behavior". */
enum BEHAVIOUR_SPECIFICATION
{
	UNGUARDED_MOTION, GUARDED_MOTION, CONTACT
};

//------------------------------------------------------------------------------
/*! Structure for error codes. */
struct edp_error
{
	uint64_t error0;
	uint64_t error1;
};

//------------------------------------------------------------------------------
enum SERVO_COMMAND
{
	MOVE, READ, SYNCHRONISE, SERVO_ALGORITHM_AND_PARAMETERS
};

//------------------------------------------------------------------------------
//                                  c_buffer
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
/*! rmodel */
typedef union c_buffer_rmodel
{
	//----------------------------------------------------------
	struct
	{
		/*! Tool trihedron ralative to the collar. */
		frame_tab tool_frame;
	} tool_frame_def;
	//----------------------------------------------------------
	struct
	{
		/*! Parameter set number for the kinematic model. */
		uint8_t kinematic_model_no;
	} kinematic_model;
	//----------------------------------------------------------
	struct
	{
		/*! Numbers for the servo-regulation algorithms. */
		uint8_t servo_algorithm_no[MAX_SERVOS_NR];
		/*! Parameter set numbers for the servo-regulation algorithms. */
		uint8_t servo_parameters_no[MAX_SERVOS_NR];
	} servo_algorithm;
	//----------------------------------------------------------
	struct
	{
		double position[3];
		double weight;
	} force_tool;

} c_buffer_rmodel_t;

//------------------------------------------------------------------------------
/*! arm */
typedef union c_buffer_arm
{
	struct
	{
		/*! A get_state command variant. */
		int command;
	} get_state_def;
	//----------------------------------------------------------
	struct
	{
		/*!  End's trihedron ralative to the base system. */
		frame_tab arm_frame;
		/*! XYZ + end's orientation relative to the base system. */
		double arm_coordinates[MAX_SERVOS_NR];
		/*! Given torque. */
		double desired_torque[MAX_SERVOS_NR];
		double inertia[6], reciprocal_damping[6];
		double force_xyz_torque_xyz[6];
		BEHAVIOUR_SPECIFICATION behaviour[6];
		/*! Dilation degree of the gripper. */
		double gripper_coordinate;
	} pf_def;

} c_buffer_arm_t;

//------------------------------------------------------------------------------
struct c_buffer
{
	/*! Type of the instruction. */
	INSTRUCTION_TYPE instruction_type;
	/*! Type of the SET instruction. */
	uint8_t set_type;
	/*! Type of the GET instruction. */
	uint8_t get_type;
	/*! Tool definition type - setting. */
	RMODEL_SPECIFICATION set_rmodel_type;
	/*! Tool definition type - reading. */
	RMODEL_SPECIFICATION get_rmodel_type;
	/*! Definition type of the end-effector's given position. */
	POSE_SPECIFICATION set_arm_type;
	/*! Definition type of the end-effector's read position. */
	POSE_SPECIFICATION get_arm_type;
	/*! Binary outputs values. */
	uint16_t output_values;

	/*! Type of interpolation. */
	INTERPOLATION_TYPE interpolation_type;

	/*! Type of motion - means of describing the shift. */
	MOTION_TYPE motion_type;
	/*! Number of steps for a given shift (macrostep). */
	uint16_t motion_steps;

	/*!
	 *  Number of steps for the 1st movemement phase.
	 *  Krok, w ktorym ma zostac przekazana informacja
	 *  o realizacji pierwszej fazy ruchu:
	 *  0 < value_in_step_no <= motion_steps + 1 .
	 *
	 *  Dla value_in_step_no = motion_steps
	 *  wiadomosc dotrze po zrealizowaniu makrokroku,
	 *  ale informacja o polozeniu bedzie dotyczyc
	 *  realizacji przedostatniego kroku makrokroku.
	 *
	 *  Dla value_in_step_no = motion_steps + 1
	 *  wiadomosc dotrze po zrealizowaniu jednego kroku
	 *  obiegu petli ruchu jalowego po zakonczeniu makrokroku,
	 *  ale informacja o polozeniu bedzie dotyczyc
	 *  realizacji calego makrokroku.
	 *
	 *  Dla value_in_step_no < motion_steps
	 *  wiadomosc dotrze przed zrealizowaniem makrokroku
	 *  i informacja o polozeniu bedzie dotyczyc
	 *  realizacji srodkowej fazy makrokroku.
	 */
	uint16_t value_in_step_no;
	c_buffer_rmodel_t rmodel;
	c_buffer_arm_t arm;
	/*
	 friend class boost::serialization::access;

	 template<class Archive>
	 void serialize(Archive & ar, const unsigned int version)
	 {
	 ar & instruction_type;
	 ar & set_type;
	 ar & get_type;
	 ar & set_rmodel_type;
	 ar & get_rmodel_type;
	 ar & set_arm_type;
	 ar & get_arm_type;
	 ar & output_values;
	 ar & interpolation_type;
	 ar & motion_type;
	 ar & motion_steps;
	 }
	 */
};

//------------------------------------------------------------------------------
//                                  r_buffer
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
/*! rmodel */
typedef union r_buffer_rmodel
{
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Macierz reprezentujaca narzedzie wzgledem konca lancucha kinematycznego.
		 *  @todo Translate to English.
		 */
		frame_tab tool_frame;
		//*! Byte for calculating the command's length. */
		// 	uint8_t address_byte;
	} tool_frame_def;
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Numer modelu kinematyki.
		 *  @todo Translate to English.
		 */
		uint8_t kinematic_model_no;

		//*! Byte for calculating the command's length. */
		// 	uint8_t address_byte;
	} kinematic_model;
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Numery algorytmow serworegulacji.
		 *  @todo Translate to English.
		 */
		uint8_t servo_algorithm_no[MAX_SERVOS_NR];
		/*!
		 *  Numery zestawu parametrow algorytmow serworegulacji.
		 *  @todo Translate to English.
		 */
		uint8_t servo_parameters_no[MAX_SERVOS_NR];
	} servo_algorithm;
	//----------------------------------------------------------
	struct
	{
		double position[3];
		double weight;
	} force_tool;

} r_buffer_rmodel_t;

//------------------------------------------------------------------------------
typedef struct _controller_state_t
{
	/*! Is robot synchronised? */
	bool is_synchronised;
	/*!
	 *  Czy wzmacniacze mocy sa zasilane?
	 *  @todo Translate to English.
	 */
	bool is_power_on;
	/*!
	 *  Czy szafa jest w���aczona?
	 *  @todo Translate to English.
	 *        Change the "wardrobe" thing for God's sake !!!
	 */
	bool is_wardrobe_on;
	/*!
	 *  Czy wyzerowano sterowanie na silnikach po awarii sprzetowej?
	 *  @todo Translate to English.
	 */
	bool is_robot_blocked;
} controller_state_t;

//------------------------------------------------------------------------------
/*! arm */
typedef union r_buffer_arm
{
	struct
	{
		/*! Given values for PWM fill (Phase Wave Modulation) - (usualy unnecessary). */
		int16_t PWM_value[MAX_SERVOS_NR];
		/*! Control current - (usualy unnecessary). */
		int16_t current[MAX_SERVOS_NR];
		/*!
		 *  Macierz reprezentujaca koncowke wzgledem bazy manipulatora.
		 *  @todo Translate to English.
		 */
		frame_tab arm_frame;
		/*!
		 *  XYZ + orientacja koncowki wzgledem ukladu bazowego.
		 *  @todo Translate to English.
		 */
		double arm_coordinates[MAX_SERVOS_NR];
		double force_xyz_torque_xyz[6];
		/*!
		 *  Stan w ktorym znajduje sie regulator chwytaka.
		 *  @todo Translate to English.
		 */
		short gripper_reg_state;
		/*!
		 *  Stopien rozwarcia chwytaka.
		 *  @todo Translate to English.
		 */
		double gripper_coordinate;
		//*! Byte for calculating the command's length. */
		// 	uint8_t address_byte;

	} pf_def;
	//----------------------------------------------------------
	struct
	{
		/*!
		 *  Czy mowi?
		 *  @todo Translate to English.
		 */
		int speaking;
		//*! Byte for calculating the command's length. */
		// 	uint8_t address_byte;
	} text_def;

} r_buffer_arm_t;

//------------------------------------------------------------------------------
struct r_buffer
{
	/*! Type of the reply. */
	REPLY_TYPE reply_type;
	/*! Number of the error (if it occured). */
	edp_error error_no;
	/*!
	 *  Sposob zdefiniowania narzedzia przy jego odczycie.
	 *  @todo Translate to English.
	 */
	RMODEL_SPECIFICATION rmodel_type;
	/*!
	 *  Sposob  zdefiniowania polozenia zadanego koncowki.
	 *  @todo Translate to English.
	 */
	POSE_SPECIFICATION arm_type;
	/*!
	 *  Wartosci wejsc binarnych.
	 *  @todo Translate to English.
	 */
	uint16_t input_values;
	/*! Analog input. */
	uint8_t analog_input[8];
	controller_state_t controller_state;
	/*! Number of the servo step. */
	unsigned long servo_step;
	/*! Byte for calculating the command's length. */
	uint8_t address_byte;
	/*! Given values for PWM fill (Phase Wave Modulation) - (usualy unnecessary). */
	int16_t PWM_value[MAX_SERVOS_NR];
	/*! Control current - (usualy unnecessary). */
	int16_t current[MAX_SERVOS_NR];
	r_buffer_rmodel_t rmodel;
	r_buffer_arm_t arm;

	//-----------------------------------------------------
	//                      METHODS
	//-----------------------------------------------------
	//r_buffer (void); // W odkomentowane
}__attribute__((__packed__));

struct ecp_command_buffer
{
	/*! This is a message buffer, so it needs a message header */
	//	msg_header_t hdr;
	c_buffer instruction;
};

} // namespace lib
} // namespace mrrocpp

#endif
