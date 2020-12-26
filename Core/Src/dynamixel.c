# include "main.h"

# define ID                    (2)
# define LENGTH                (3)
# define INSTRUCTION        (4)
# define ERRBIT                (4)
# define PARAMETER            (5)

# define DXL_BYTE            (1)
# define DXL_WORD            (2)

unsigned char instruction_packet[MAXNUM_TXPARAM + 6] = { 0 };
unsigned char status_packet[MAXNUM_RXPARAM + 6] = { 0 };

/*========== INTERNAL ==========*/
/// pointer to write to when a read operation is finished
unsigned *read_value = NULL;
/// callback to call to when a read operation is finished
void (*read_callback)() = NULL;

/// is the current tx/rx transfer still running
int busy = 0;
/// return status of the last tx/rx transfer
int status = DXL_SUCCESS;

/// is the header part of the rx transfer received
int header_received = 0;
/// is the args part of the rx transfer received
int args_received = 0;

/// time after which timeout expires (in ms)
uint32_t timeout = 100;
/// time at which timeout was started (in ms)
uint32_t time = 0;

//# define USE_MUTEX
# ifdef USE_MUTEX
    # define LOCK_INIT         // TODO
    # define LOCK            // TODO
    # define UNLOCK            // TODO
    # define LOCK_DEINIT     // TODO
# else
    # define LOCK_INIT         0
    # define LOCK             0
    # define UNLOCK
    # define LOCK_DEINIT
# endif

static int HAL2DXL_error(const HAL_StatusTypeDef err, const int is_rx)
{
    ASSERT(is_rx == 0 || is_rx == 1);

    switch (err)
    {
        case HAL_OK:        return DXL_SUCCESS;
        case HAL_ERROR:     return DXL_TXERROR      + is_rx;
        case HAL_BUSY:      return DXL_TXBUSY       + is_rx;
        case HAL_TIMEOUT:   return DXL_TXTIMEOUT    + is_rx;
        default:            ASSERT(!"what the hell are you doing here");
    }

    return DXL_SUCCESS;
}

/// Set error and return, if condition is fulfilled
# define RETURN_IF(COND, ERR_VAR, ERR) \
    if (COND) \
    { \
        ERR_VAR = ERR; \
        return; \
    }

/// Abort transmission, set error and return, if condition is fulfilled
# define ABORT_IF(COND, ERR_VAR, ERR) \
    if (COND) \
    { \
        ERR_VAR = ERR; \
        dxl_abort(); \
        return; \
    }

/// Abort current transmission
void dxl_abort(void)
{
    dxl_hal_abort();

    if (busy)
    {
        busy = 0;
        UNLOCK;
    }
}

// TODO: replace manual timeout, by timer + callback
// timeout should be set to -1 on rx_*_callback

// timeout callback to call every ms
/*
/// time left until timeout
int timeout = -1;
void timeout_callback(void)
{
    if (timeout-- == 0)
    {
        status = DXL_RXTIMEOUT;
        dxl_abort();
    }
}
*/

void dxl_set_timeout(const unsigned ms)
{
    timeout = ms;
}

void dxl_timeout_start(void)
{
    time = HAL_GetTick();
}

int check_timeout(void)
{
    return (HAL_GetTick() - time) > timeout;
}

/// Callback executed after the rest of the status packet has been received
void rx_args_callback(void)
{
    args_received = 1;
}

/// Callback executed after header of the status packet has been received (first bytes)
void rx_header_callback(void)
{
    dxl_set_rx_callback(rx_args_callback);
    status = HAL2DXL_error(dxl_hal_rx(status_packet + INSTRUCTION, status_packet[LENGTH] - 1), 1);

    header_received = 1;
}

/// Callback executed after instruction packet has been sent
void tx_callback(void)
{
    // no response on broadcast
    RETURN_IF(instruction_packet[ID] == BROADCAST_ID, status, DXL_SUCCESS)

    dxl_set_timeout(250);
    dxl_set_rx_callback(rx_header_callback);
    dxl_timeout_start();

    status = HAL2DXL_error(dxl_hal_rx(status_packet, INSTRUCTION), 0);
}

/// Compute checksum
unsigned char checksum(const unsigned char *packet, const unsigned length)
{
    unsigned check = 0;
    for (unsigned i = 0; i < length; i++)
        check += packet[ID + i];

    return ~check & 0xFF;
}

/// Start a tx/rx transmission
int dxl_txrx_packet()
{
    // packet : 0xFF, 0xFF, ID, LENGTH, INSTRUCTION, PARAM 1, ..., PARAM N, CHECKSUM

    ASSERT(instruction_packet[LENGTH] <= (MAXNUM_TXPARAM + 2));
    ASSERT(instruction_packet[INSTRUCTION] == INST_PING
        || instruction_packet[INSTRUCTION] == INST_READ
        || instruction_packet[INSTRUCTION] == INST_WRITE
        || instruction_packet[INSTRUCTION] == INST_REG_WRITE
        || instruction_packet[INSTRUCTION] == INST_ACTION
        || instruction_packet[INSTRUCTION] == INST_RESET
        || instruction_packet[INSTRUCTION] == INST_SYNC_WRITE);

    instruction_packet[0] = 0xFF;
    instruction_packet[1] = 0xFF;

    unsigned length = instruction_packet[LENGTH];
    instruction_packet[LENGTH + length] = checksum(instruction_packet, length);

    length += 4;

    dxl_set_timeout(length);
    dxl_set_tx_callback(tx_callback);
    dxl_timeout_start();

    status = DXL_SUCCESS;
    return HAL2DXL_error(dxl_hal_tx(instruction_packet, length), 0);
}

int dxl_get_parameter(const int index)
{
    return status_packet[PARAMETER + index];
}

int dxl_makeword(const int lowbyte, const int highbyte)
{
    return (highbyte & 0xFF) << 8 | (lowbyte & 0xFF);
}

int dxl_get_lowbyte(const int word)
{
    return word & 0xFF;
}

int dxl_get_highbyte(const int word)
{
    return (word & 0xFF00) >> 8;
}

/*========== EXTERNAL ==========*/
/// Initialize dxl with UART \ref huart and baud rate \ref baudrate
int dxl_initialize(UART_HandleTypeDef *huart, const int baudrate)
{
    if (dxl_hal_open(huart, baudrate) != HAL_OK)
        return DXL_FAIL;

    if (LOCK_INIT)
        return DXL_FAIL;

    return DXL_SUCCESS;
}

/// Terminate the current dxl
void dxl_terminate(void)
{
    LOCK_DEINIT;
    dxl_hal_close();
}

int dxl_set_baudrate(const int baudrate)
{
    return dxl_hal_set_baudrate(baudrate) == HAL_OK;
}

void dxl_set_id(const unsigned id)
{
    ASSERT(id <= 253 || id == 255);
    instruction_packet[ID] = (unsigned char) id;
}

void dxl_set_instruction(const unsigned instruction)
{
    ASSERT(instruction < 0xFF);
    instruction_packet[INSTRUCTION] = (unsigned char) instruction;
}

void dxl_set_parameter(const unsigned index, const unsigned value)
{
    ASSERT(value < 0xFF);
    instruction_packet[PARAMETER + index] = (unsigned char) value;
}

void dxl_set_length(const unsigned length)
{
    ASSERT(length <= MAXNUM_TXPARAM + 2);
    instruction_packet[LENGTH] = (unsigned char) length;
}

int dxl_get_length(void)
{
    // parameter length = length - 2 (error and checksum)
    return status_packet[LENGTH] - 2;
}

/// Get error in status packet
int dxl_get_error(void)
{
    return status_packet[ERRBIT];
}

/// Get the name for a DXL_ERROR
const char* dxl_get_error_name(const int error)
{
    switch (error)
    {
        // DXL HAL errors
        case DXL_TXERROR:      return "DXL_TXERROR";
        case DXL_RXERROR:      return "DXL_RXERROR";

        case DXL_TXBUSY:       return "DXL_TXBUSY";
        case DXL_RXBUSY:       return "DXL_RXBUSY";

        case DXL_TXTIMEOUT:    return "DXL_TXTIMEOUT";
        case DXL_RXTIMEOUT:    return "DXL_RXTIMEOUT";

        // DYNAMIXEL errors
        case DXL_TXFAIL:       return "DXL_TXFAIL";
        case DXL_RXFAIL:       return "DXL_RXFAIL";

        case DXL_TXCORRUPT:    return "DXL_TXCORRUPT";
        case DXL_RXCORRUPT:    return "DXL_RXCORRUPT";

        case DXL_SUCCESS:      return "DXL_SUCCESS";
        case DXL_FAIL:         return "DXL_FAIL";

        case DXL_BUSY:         return "DXL_BUSY";
        case DXL_ERROR:        return "DXL_ERROR";
    }

    ASSERT(!"Invalid code for DXL_ERROR");

    return NULL;
}

/// Return true if a transmission is ongoing
int dxl_is_busy(void)
{
    return busy;
}

/// Get error status for current transmission (use it after dxl_process_status_packet)
int dxl_get_txrx_status()
{
    return status;
}

/// Check for errors in the status packet
void dxl_process_status_packet(void)
{
    // if error in a previous call
    if (status != DXL_SUCCESS)
        return;

    // packet : 0xFF, 0xFF, ID, LENGTH, ERROR, PARAM 1, ..., PARAM N, CHECKSUM
    if (header_received)
    {
        header_received = 0;

        // PART 1: check header
        ABORT_IF(status_packet[0] != 0xFF || status_packet[1] != 0xFF, status, DXL_RXCORRUPT)

        // PART 2: check id pairing and error
        ABORT_IF(instruction_packet[ID] != status_packet[ID], status, DXL_RXCORRUPT)
        ABORT_IF(status_packet[ERRBIT], status, DXL_ERROR)
    }

    if (args_received)
    {
        args_received = 0;

        // PART 3: retrieve parameter length
        const int length = dxl_get_length();

        // PART 4: check checksum (with error ID, LENGTH and ERROR)
        const unsigned char check = checksum(status_packet, length + 3);
        if (status_packet[PARAMETER + length] != check)
        	status = DXL_RXCORRUPT;
        else if (read_callback)
            (*read_callback)();

		read_callback = NULL;
		read_value = NULL;

        if (busy)
        {
            busy = 0;
            UNLOCK;
        }
    }

    // TODO: move timeout as a timer interrupt
    ABORT_IF(check_timeout(), status, DXL_RXTIMEOUT)
}

int dxl_ping(const unsigned id)
{
    if (LOCK)
        return DXL_BUSY;
    busy = 1;

    dxl_set_id(id);
    dxl_set_instruction(INST_PING);
    dxl_set_length(2);

    return dxl_txrx_packet();
}

static void read_byte_callback()
{
    *read_value = dxl_get_parameter(0);
    read_callback = NULL;
}

/// Read 1 byte at address \ref address from the AX12 \ref id
int dxl_read_byte(const unsigned id, const int address, unsigned *value)
{
    ASSERT(0 <= address);
    ASSERT(address <= rw_permission_size);
    ASSERT(dxl_readable_byte[address]);

    ASSERT(value != NULL);

    if (LOCK)
        return DXL_BUSY;
    busy = 1;

    dxl_set_id(id);
    dxl_set_instruction(INST_READ);
    dxl_set_parameter(0, address);
    dxl_set_parameter(1, DXL_BYTE);
    dxl_set_length(2);

    read_value = value;
    if (read_callback == NULL)
        read_callback = read_byte_callback;

    return dxl_txrx_packet();
}

/// Write 1 byte at address \ref address in the AX12 \ref id
int dxl_write_byte(const unsigned id, const int address, const int value)
{
    ASSERT(0 <= address);
    ASSERT(address <= rw_permission_size);
    ASSERT(dxl_writable_byte[address]);

    if (LOCK)
        return DXL_BUSY;
    busy = 1;

    dxl_set_id(id);
    dxl_set_instruction(INST_WRITE);
    dxl_set_parameter(0, address);
    dxl_set_parameter(1, value);
    dxl_set_length(2);

    return dxl_txrx_packet();
}

static void read_word_callback()
{
    *read_value = dxl_makeword(dxl_get_parameter(0), dxl_get_parameter(1));
}

/// Read 2 bytes at address \ref address from the AX12 \ref id
int dxl_read_word(const unsigned id, const int address, unsigned *value)
{
    ASSERT(0 <= address);
    ASSERT(address <= rw_permission_size);
    ASSERT(dxl_readable_word[address]);

    ASSERT(value != NULL);

    if (LOCK)
        return DXL_BUSY;
    busy = 1;

    dxl_set_id(id);
    dxl_set_instruction(INST_READ);
    dxl_set_parameter(0, address);
    dxl_set_parameter(1, DXL_WORD);
    dxl_set_length(2);

    read_value = value;
    if (read_callback == NULL)
        read_callback = read_word_callback;

    return dxl_txrx_packet();
}

/// Write 2 bytes at address \ref address in the AX12 \ref id
int dxl_write_word(const unsigned id, const int address, const int value)
{
    ASSERT(0 <= address);
    ASSERT(address <= rw_permission_size);
    ASSERT(dxl_writable_word[address]);

    if (LOCK)
        return DXL_BUSY;
    busy = 1;

    dxl_set_id(id);
    dxl_set_instruction(INST_WRITE);
    dxl_set_parameter(0, address);
    dxl_set_parameter(1, dxl_get_lowbyte(value));
    dxl_set_parameter(2, dxl_get_highbyte(value));
    dxl_set_length(3);

    return dxl_txrx_packet();
}

// SET_GOAL_POSITION
int dxl_set_goal_position(const unsigned id, const unsigned angle)
{
    ASSERT(angle <= 300);
    return dxl_write_byte(id, DXL_ADDRESS_GOAL_POSITION, angle * 1023 / 300);
}

// GET_GOAL_POSITION
static void read_byte_callback_goal_position(void)
{
    const unsigned word = dxl_makeword(dxl_get_parameter(0), dxl_get_parameter(1));
    *read_value = word / 1023 * 300;
}

int dxl_get_goal_position(const unsigned id, unsigned *angle)
{
    read_callback = read_byte_callback_goal_position;
    return dxl_read_word(id, DXL_ADDRESS_GOAL_POSITION, angle);
}

// SET_AX12_DELAY_TIME
int dxl_set_AX12_delay_time(const unsigned id, const unsigned time)
{
    return dxl_write_byte(id, DXL_ADDRESS_RETURN_DELAY, time / 2);
}

// GET_VOLTAGE
void read_byte_callback_voltage(void)
{
	// only value being float, so it will be an exception
    float *ptr = (float*) read_value;
    *ptr = (float) dxl_get_parameter(0) / 10;
}

int dxl_get_voltage(const unsigned id, float *volt)
{
    read_callback = read_byte_callback_voltage;
    return dxl_read_word(id, DXL_ADDRESS_GOAL_POSITION, (unsigned*) volt);
}

// GET_TEMPERATURE
int dxl_get_temperature(const unsigned id, unsigned *temp)
{
    return dxl_read_byte(id, DXL_ADDRESS_PRESENT_TEMPERATURE, temp);
}

// SET_EEPROM_LOCK
int dxl_set_eeprom_lock(const unsigned id, const int state)
{
    ASSERT(state == 0 || state == 1);
    return dxl_write_byte(id, DXL_ADDRESS_EEPROM_LOCK, state);
}
