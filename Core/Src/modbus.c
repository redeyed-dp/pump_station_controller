#include "stm32f1xx_hal.h"
#include "modbus.h"

uint8_t modbus_request[256];
uint8_t modbus_response[256];

uint16_t modbus_input_registers[MODBUS_IR_COUNT];
uint16_t modbus_holding_registers[MODBUS_HR_COUNT];
uint8_t modbus_coils[MODBUS_COILS_COUNT];
uint8_t modbus_discrete_inputs[MODBUS_DI_COUNT];


uint16_t calculate_crc16(uint8_t *msg, uint8_t DataLen) {
	uint8_t CRCHi = 0xFF;
	uint8_t CRCLo = 0xFF;
	uint8_t i ;
	while (DataLen) {
		i = CRCHi ^ *msg++ ;
		CRCHi = CRCLo ^ srCRCHi[i];
		CRCLo = srCRCLo[i];
		DataLen--;
	}
	return (CRCHi << 8 | CRCLo) ;
}
void modbus_exception(ModbusExceptions_t code) {
	modbus_response[FIELD_FUNCTION] = modbus_request[FIELD_FUNCTION] | 0x80;
	modbus_response[FIELD_ERROR] = code;
}
uint8_t read_1bit_registers(uint8_t total_reg_count, uint8_t *registers) {
	uint16_t start = (modbus_request[FIELD_START_ADDR_HI] << 8) | modbus_request[FIELD_START_ADDR_LO];
	uint16_t count = (modbus_request[FIELD_COUNT_HI] << 8) | modbus_request[FIELD_COUNT_LO];
	if (start + count > MODBUS_DI_COUNT) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
		return 2;
	}
	modbus_response[FIELD_FUNCTION] = modbus_request[FIELD_FUNCTION];
	modbus_response[FIELD_BYTE_COUNT] = 1;
	uint8_t p = FIELD_PAYLOAD_START;
	uint16_t mask = 0x0001;
	for(uint8_t i = start; i<start+count; i++) {
		if (mask == 0x0100) {
			mask = 0x0001;
			p += 1;
			modbus_response[p] = 0x00;
			modbus_response[FIELD_BYTE_COUNT] += 1;
		}
		if (registers[i]) {
			modbus_response[p] |= (uint8_t)mask;
		}
		mask = mask << 1;
	}
	return modbus_response[FIELD_BYTE_COUNT] + 2;
}
uint8_t read_16bit_registers(uint8_t total_reg_count, uint16_t *registers) {
	uint16_t start = (modbus_request[FIELD_START_ADDR_HI] << 8) | modbus_request[FIELD_START_ADDR_LO];
	uint16_t count = (modbus_request[FIELD_COUNT_HI] << 8) | modbus_request[FIELD_COUNT_LO];
	if (start + count > total_reg_count) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
		return 2;
	}
	modbus_response[FIELD_FUNCTION] = modbus_request[FIELD_FUNCTION];
	uint8_t p = FIELD_PAYLOAD_START;
	for(uint8_t i=start; i<start+count; i++) {
		modbus_response[p] = (uint8_t)(registers[i] >> 8);
		modbus_response[p+1] = (uint8_t)(registers[i] & 0xFF);
		modbus_response[FIELD_BYTE_COUNT] += 2;
		p += 2;
	}
	modbus_response[FIELD_BYTE_COUNT] = count << 1;
	return modbus_response[FIELD_BYTE_COUNT] + 2;
}

uint8_t modbus_transaction(uint8_t request_size) {
	// Check slave address
	if (modbus_request[FIELD_DEVICE_ADDRESS] != MODBUS_DEVICE_ADDRESS) {
		return 0;
	}

	// Check request CRC
	uint16_t msg_crc = (modbus_request[request_size - 2] << 8 | modbus_request[request_size - 1]);
	uint16_t calc_crc = calculate_crc16(modbus_request, request_size - 2);
	if (msg_crc != calc_crc) {
		return 0;
	}

	// Address of this device
	modbus_response[FIELD_DEVICE_ADDRESS] = MODBUS_DEVICE_ADDRESS;
	uint8_t response_size = 1;
	// Request processing
	switch(modbus_request[1]) {
	case MODBUS_READ_COILS:
		response_size += read_1bit_registers(MODBUS_COILS_COUNT, modbus_coils);
		break;
	case MODBUS_READ_DISCRETE_INPUTS:
		response_size += read_1bit_registers(MODBUS_DI_COUNT, modbus_discrete_inputs);
		break;
	case MODBUS_READ_HOLDING_REGISTERS:
		response_size += read_16bit_registers(MODBUS_HR_COUNT, modbus_holding_registers);
		break;
	case MODBUS_READ_INPUT_REGISTERS:
		response_size += read_16bit_registers(MODBUS_IR_COUNT, modbus_input_registers);
		break;
	case MODBUS_WRITE_SINGLE_COIL:
		modbus_exception(ILLEGAL_FUNCTION);
		response_size += 2;
		break;
	case MODBUS_WRITE_SINGLE_REGISTER:
		modbus_exception(ILLEGAL_FUNCTION);
		response_size += 2;
		break;
	default:
		modbus_exception(ILLEGAL_FUNCTION);
		response_size += 2;
		break;
	}
	// Add response CRC
	uint16_t crc = calculate_crc16(modbus_response, response_size);
	modbus_response[response_size] = crc >> 8;
	modbus_response[response_size+1] = crc & 0xFF;
	response_size += 2;
	return response_size;
}
