/** @file dynamixel.h @brief Interface to dxl*/

# pragma once

# include "stm32f3xx_hal.h"

// /////////// set/get packet methods //////////////////////////
# define MAXNUM_TXPARAM			(150)
# define MAXNUM_RXPARAM			(60)
# define BROADCAST_ID			(254)
enum DXL_INSTRUCTION
{
	INST_PING 			= 1,
	INST_READ			= 2,
	INST_WRITE			= 3,
	INST_REG_WRITE		= 4,
	INST_ACTION			= 5,
	INST_RESET			= 6,
	INST_SYNC_WRITE		= 131,
};

enum DXL_ERRBIT
{
	ERRBIT_VOLTAGE		= 1 << 0,
	ERRBIT_ANGLE		= 1 << 1,
	ERRBIT_OVERHEAT		= 1 << 2,
	ERRBIT_RANGE		= 1 << 3,
	ERRBIT_CHECKSUM		= 1 << 4,
	ERRBIT_OVERLOAD		= 1 << 5,
	ERRBIT_INSTRUCTION	= 1 << 6,
};

enum DXL_ERROR
{
	// DXL HAL errors
	DXL_TXERROR,
	DXL_RXERROR,

	DXL_TXBUSY,
	DXL_RXBUSY,

	DXL_TXTIMEOUT,
	DXL_RXTIMEOUT,

	// DYNAMIXEL errors
	DXL_TXFAIL,
	DXL_RXFAIL,

	DXL_TXCORRUPT,
	DXL_RXCORRUPT,

	DXL_SUCCESS,
	DXL_FAIL,

	/// current tx/rx transfer is not finished (using
	DXL_BUSY,
	DXL_ERROR,

	/// timeout reached when using dxl_process_status_packet
	DXL_TIMEOUT,
};

/// set id in instruction packet (0 <= id <= 253 or id == 255)
void dxl_set_id(const unsigned id);
/// set instruction in instruction packet (must be DXL_INSTRUCTION)
void dxl_set_instruction(const unsigned instruction);
/// set parameter #index in instruction packet
void dxl_set_parameter(const unsigned index, const unsigned value);
/// set number of parameter in instruction packet
void dxl_set_length(const unsigned length);
/// get length of parameters in instruction packet
int dxl_get_length(void);
/// get error of current instruction packet
int dxl_get_error(void);
// get name of error of type DXL_ERROR
const char* dxl_get_error_name(const int error);

// ////////// high communication methods ///////////////////////
/// Is address i a readable byte
static const int dxl_readable_byte[] =
	{ 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, \
	  1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, \
	  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0 };

/// Is address i a readable word
static const int dxl_readable_word[] =
	{ 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, \
	  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, \
	  1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1 };

/// Is address i a writable byte
static const int dxl_writable_byte[] =
	{ 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, \
	  1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, \
	  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };

/// Is address i a writable word
static const int dxl_writable_word[] =
	{ 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, \
	  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, \
	  1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

static const unsigned rw_permission_size = sizeof(dxl_readable_byte) / sizeof(int);

enum DXL_ADDRESS
{
	DXL_ADDRESS_MODEL_NUMBER 		= 0,
	DXL_ADDRESS_FIRMWARE_VERSION 	= 2,
	DXL_ADDRESS_ID					= 3,
	DXL_ADDRESS_BAUDRATE			= 4,
	DXL_ADDRESS_RETURN_DELAY		= 5,
	DXL_ADDRESS_CW_ANGLE_LIMIT		= 6,
	DXL_ADDRESS_CWW_ANGLE_LIMIT		= 8,
	DXL_ADDRESS_TEMPERATURE_LIMIT	= 11,
	DXL_ADDRESS_MIN_VOLT_LIMIT		= 12,
	DXL_ADDRESS_MAX_VOLT_LIMIT		= 13,
	DXL_ADDRESS_MAX_TORQUE			= 14,
	DXL_ADDRESS_STATUS_LEVEL_LIMIT	= 16,
	DXL_ADDRESS_ALARM_LED			= 17,
	DXL_ADDRESS_SHUTDOWN_ERR_INFO	= 18,
	DXL_ADDRESS_TORQUE_ENABLE		= 24,
	DXL_ADDRESS_LED_ENABLE			= 25,
	DXL_ADDRESS_CW_COMP_MARGIN		= 26,
	DXL_ADDRESS_CCW_COMP_MARGIN		= 27,
	DXL_ADDRESS_CW_COMP_SLOPE		= 28,
	DXL_ADDRESS_CCW_COMP_SLOPE		= 29,
	DXL_ADDRESS_GOAL_POSITION		= 30,
	DXL_ADDRESS_MOVING_SPEED		= 32,
	DXL_ADDRESS_TORQUE_LIMIT		= 34,
	DXL_ADDRESS_PRESENT_POSITION	= 36,
	DXL_ADDRESS_PRESENT_SPEED		= 38,
	DXL_ADDRESS_PRESENT_LOAD		= 40,
	DXL_ADDRESS_PRESENT_VOLTAGE		= 42,
	DXL_ADDRESS_PRESENT_TEMPERATURE	= 43,
	DXL_ADDRESS_REGISTERED_INST		= 44,
	DXL_ADDRESS_IS_MOVING			= 46,
	DXL_ADDRESS_EEPROM_LOCK			= 47,
	DXL_ADDRESS_PUNCH				= 48,
};

// initialize the connection
int dxl_initialize(UART_HandleTypeDef *huart, const int baudrate);
// terminate the connection
void dxl_terminate(void);
// change baudrate
int dxl_set_baudrate(const int baudrate);

/// abort the current tx/rx transfer
void dxl_abort(void);
/// tell whether a tx/rx transfer is still ongoing
int dxl_is_busy(void);
/// process the current rx operation (should be called often during it)
void dxl_process_status_packet(void);
/// return status of the tx/rx transfer
int dxl_get_txrx_status(void);

/// ping the AX12 \param id
int dxl_ping(const unsigned id);
/// read the 1 byte register \param address from AX12 \param id
int dxl_read_byte(const unsigned id, const int address, unsigned *value);
/// write the 1 byte register \param address from AX12 \param id
int dxl_write_byte(const unsigned id, const int address, const int value);
/// read the 2 bytes register \param address from AX12 \param id
int dxl_read_word(const unsigned id, const int address, unsigned *value);
/// write the 2 bytes register \param address from AX12 \param id
int dxl_write_word(const unsigned id, const int address, const int value);

int dxl_set_goal_position(const unsigned id, const unsigned angle);
int dxl_get_goal_position(const unsigned id, unsigned *angle);
int dxl_set_AX12_delay_time(const unsigned id, const unsigned time);
int dxl_get_temperature(const unsigned id, unsigned *temp);
int dxl_set_eeprom_lock(const unsigned id, const int state);
