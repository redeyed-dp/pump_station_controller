#include "spi.h"
#include "stm32f1xx_hal.h"
#include "sx1262.h"

uint8_t spi_tx_buffer[8];
uint8_t spi_rx_buffer[8];

LoraModulationParams_t LoraModulation = {
	.SpreadingFactor = LORA_SF8,
	.Bandwidth = LORA_BW_125,
	.CodingRate = LORA_CR_4_5
};
LoraPacketParams_t LoraPacket = {
	.PreambleLength = 8,
	.HeaderType = LORA_PACKET_EXPLICIT,
	.PayloadLength = 255,
	.CrcMode = LORA_CRC_ON,
	.InvertIQ = LORA_IQ_NORMAL
};
uint32_t frequency = 868000000;

void Lora_Reset() {
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
}

void Lora_SPIWrite(uint8_t size) {
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spi_tx_buffer, size, 0x1000);
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}
void Lora_SPIRead(uint8_t size) {
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 0x1000);
	HAL_SPI_Receive(&hspi1, spi_rx_buffer, size, 0x1000);
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}
void Lora_WriteRegister(uint16_t address, uint8_t value) {
    spi_tx_buffer[0]= LORA_WRITE_REGISTER;
    spi_tx_buffer[1]= (address & 0xFF00) >> 8;
    spi_tx_buffer[2]= address & 0x00FF;
    Lora_SPIWrite(3);
}
void Lora_WriteBuffer(uint8_t *buffer, uint8_t offset, uint8_t size) {
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
	spi_tx_buffer[0] = LORA_WRITE_BUFFER;
	spi_tx_buffer[1] = offset;
	HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 0x1000);
	HAL_SPI_Transmit(&hspi1, buffer, size, 0x1000);
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}
void Lora_ReadBuffer(uint8_t *buffer, uint8_t offset, uint8_t size) {
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    spi_tx_buffer[0]= LORA_READ_BUFFER;
    spi_tx_buffer[1]= offset;
    spi_tx_buffer[2]= 0x00;
    HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 3, 0x1000);
    HAL_SPI_Receive(&hspi1, buffer, size, 0x1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void Lora_SetIrqParams(uint16_t irqMask) {
	spi_tx_buffer[0] = LORA_CFG_DIOIRQ;
	spi_tx_buffer[1] = (uint8_t) ((irqMask >> 8) & 0x00FF);
	spi_tx_buffer[2] = (uint8_t) (irqMask & 0x00FF);
	Lora_SPIWrite(3);
    HAL_Delay(1);
}
void Lora_ClearIrqStatus(uint16_t irqVal){
	spi_tx_buffer[0] = LORA_CLR_IRQSTATUS;
	spi_tx_buffer[1] = (uint8_t)((irqVal >> 8 ) & 0x00FF);
	spi_tx_buffer[2] = (uint8_t)(irqVal & 0x00FF);
    Lora_SPIWrite(3);
}
uint16_t Lora_GetIrqStatus() {
    spi_tx_buffer[0] = LORA_GET_IRQSTATUS;
    spi_tx_buffer[1] = 0;
    Lora_SPIRead(2);
    return ( spi_rx_buffer[0] << 8) | spi_rx_buffer[1];
}
void Lora_SetStandby(LoraStandbyModes_t standbyConfig) {
	spi_tx_buffer[0] = LORA_SET_STANDBY;
	spi_tx_buffer[1] = (uint8_t)standbyConfig;
    Lora_SPIWrite(2);
    HAL_Delay(10);
}
void Lora_SetRegulatorMode(LoraRegulatorMode_t mode) {
	spi_tx_buffer[0] = LORA_SET_REGULATORMODE;
	spi_tx_buffer[1] = mode;
	Lora_SPIWrite(2);
	HAL_Delay(10);
}
void Lora_SetRfSwitchMode(LoraRFSwitch_t mode){
	spi_tx_buffer[0] = LORA_SET_RFSWITCHMODE;
	spi_tx_buffer[1] = mode;
	Lora_SPIWrite(2);
	HAL_Delay(10);
}
void Lora_SetPacketType(LORAPacketTypes_t packetType) {
	spi_tx_buffer[0] = LORA_SET_PACKETTYPE;
	spi_tx_buffer[1] = (uint8_t)packetType;
	Lora_SPIWrite(2);
	HAL_Delay(5);
}
void Lora_SetSyncWord() {
  Lora_WriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
  HAL_Delay(5);
  Lora_WriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);
  HAL_Delay(5);
}
void Lora_Calibrate() {
	spi_tx_buffer[0] = LORA_CALIBRATE;
	spi_tx_buffer[1] = 0x7F;
	Lora_SPIWrite(2);
	HAL_Delay(70);
}
void Lora_CalibrateImage(uint32_t freq) {
	spi_tx_buffer[0] = LORA_CALIBRATEIMAGE;
    if (freq > 900000000) {
    	spi_tx_buffer[1] = 0xE1;
    	spi_tx_buffer[2] = 0xE9;
    } else if (freq > 850000000) {
    	spi_tx_buffer[1] = 0xD7;
    	spi_tx_buffer[2] = 0xD8;
    } else if (freq > 770000000) {
    	spi_tx_buffer[1] = 0xC1;
    	spi_tx_buffer[2] = 0xC5;
    } else if (freq > 460000000) {
    	spi_tx_buffer[1] = 0x75;
    	spi_tx_buffer[2] = 0x81;
    } else if (freq > 425000000) {
    	spi_tx_buffer[1] = 0x6B;
    	spi_tx_buffer[2] = 0x6F;
    }
    Lora_SPIWrite(3);
    HAL_Delay(70);
}
void Lora_SetRxBoosted(uint32_t timeout) {
    Lora_WriteRegister(REG_RX_GAIN, 0x96); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity
    spi_tx_buffer[0] = LORA_SET_RX;
    spi_tx_buffer[1] = (uint8_t) ((timeout >> 16) & 0xFF);
    spi_tx_buffer[2] = (uint8_t) ((timeout >> 8) & 0xFF);
    spi_tx_buffer[3] = (uint8_t) (timeout & 0xFF);
    Lora_SPIWrite(4);
}
void Lora_SetRx(uint32_t timeout) {
	Lora_WriteRegister(REG_RX_GAIN, 0x94); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity
	spi_tx_buffer[0] = LORA_SET_RX;
    spi_tx_buffer[1] = (uint8_t) ((timeout >> 16) & 0xFF);
    spi_tx_buffer[2] = (uint8_t) ((timeout >> 8) & 0xFF);
	spi_tx_buffer[3] = (uint8_t) (timeout & 0xFF);
	Lora_SPIWrite(4);
}
void Lora_SetTx(uint32_t timeout) {
	spi_tx_buffer[0] = LORA_SET_TX;
	spi_tx_buffer[1] = (uint8_t) ((timeout >> 16) & 0xFF);
	spi_tx_buffer[2] = (uint8_t) ((timeout >> 8) & 0xFF);
	spi_tx_buffer[3] = (uint8_t) (timeout & 0xFF);
	Lora_SPIWrite(4);
}
void Lora_SetDio3AsTcxoCtrl( LoraTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout ){
	spi_tx_buffer[0] = LORA_SET_TCXOMODE;
	spi_tx_buffer[1] = tcxoVoltage & 0x07;
	spi_tx_buffer[2] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
	spi_tx_buffer[3] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
	spi_tx_buffer[4] = ( uint8_t )( timeout & 0xFF );
	Lora_SPIWrite(5);
    HAL_Delay(10);
}
void Lora_SetTxParams(int8_t power, LORARampTimes_t rampTime) {
	spi_tx_buffer[0] = LORA_SET_PACONFIG;
	spi_tx_buffer[1] = 0x04; //paDutyCycle
	spi_tx_buffer[2] = 0x07; //hpMax:0x00~0x07; 7:22dbm
	spi_tx_buffer[3] = 0x00; //deviceSel: 0: SX1262; 1: SX1261
	spi_tx_buffer[4] = 0x01;
	Lora_SPIWrite(7);

    Lora_WriteRegister(REG_OCP, 0x38); // current max 160mA for the whole device

    spi_tx_buffer[0] = LORA_SET_TXPARAMS;
    spi_tx_buffer[1] = (uint8_t)power;
    spi_tx_buffer[2] = (uint8_t)rampTime;
    Lora_SPIWrite(3);
    HAL_Delay(20);
}
void Lora_SetModulationParams(LoraModulationParams_t *modulationParams) {
    uint8_t LowDatarateOptimize;
    switch (modulationParams->Bandwidth) {
    case LORA_BW_500:
      LowDatarateOptimize = 0x00;
      break;
    case LORA_BW_250:
      if (modulationParams->SpreadingFactor == 12)
        LowDatarateOptimize = 0x01;
      else
        LowDatarateOptimize = 0x00;
      break;
    case LORA_BW_125:
      if (modulationParams->SpreadingFactor >= 11)
        LowDatarateOptimize = 0x01;
      else
        LowDatarateOptimize = 0x00;
      break;
    case LORA_BW_062:
      if (modulationParams->SpreadingFactor >= 10)
        LowDatarateOptimize = 0x01;
      else
        LowDatarateOptimize = 0x00;
      break;
    case LORA_BW_041:
      if (modulationParams->SpreadingFactor >= 9)
        LowDatarateOptimize = 0x01;
      else
        LowDatarateOptimize = 0x00;
      break;
    case LORA_BW_031:
    case LORA_BW_020:
    case LORA_BW_015:
    case LORA_BW_010:
    case LORA_BW_007:
      LowDatarateOptimize = 0x01;
      break;
    default:
      break;
    }
    spi_tx_buffer[0] = LORA_SET_MODULATIONPARAMS;
    spi_tx_buffer[1] = modulationParams->SpreadingFactor;
    spi_tx_buffer[2] = modulationParams->Bandwidth;
    spi_tx_buffer[3] = modulationParams->CodingRate;
    spi_tx_buffer[4] = LowDatarateOptimize;
    Lora_SPIWrite(5);
    HAL_Delay(20);
}
void Lora_SetPacketParams(LoraPacketParams_t *packetParams) {
    spi_tx_buffer[0] = LORA_SET_PACKETPARAMS;
    spi_tx_buffer[1] = (packetParams->PreambleLength >> 8) & 0xFF;
    spi_tx_buffer[2] = packetParams->PreambleLength;
    spi_tx_buffer[3] = packetParams->HeaderType;
    spi_tx_buffer[4] = packetParams->PayloadLength;
    spi_tx_buffer[5] = packetParams->CrcMode;
    spi_tx_buffer[6] = packetParams->InvertIQ;
    Lora_SPIWrite(7);
    HAL_Delay(20);
}
void Lora_SetBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    spi_tx_buffer[0] = LORA_SET_BUFFERBASEADDRESS;
    spi_tx_buffer[1] = txBaseAddress;
    spi_tx_buffer[2] = rxBaseAddress;
    Lora_SPIWrite(3);
    HAL_Delay(10);
}
void Lora_GetRxBufferStatus() {
	spi_tx_buffer[0] = LORA_GET_RXBUFFERSTATUS;
	spi_tx_buffer[1] = 0;
	Lora_SPIRead(2);
}
LoraPacketStatus_t Lora_GetPacketStatus() {
	spi_tx_buffer[0] = LORA_GET_PACKETSTATUS;
	spi_tx_buffer[1] = 0;
	Lora_SPIRead(3);
	LoraPacketStatus_t status;
	status.Rssi = -(spi_rx_buffer[0] >> 1);
	status.Snr = spi_rx_buffer[1] >> 2;
	return status;
}
void Lora_SetFrequency(uint32_t frequency) {
    uint32_t freq = (uint32_t) ((double) frequency / (double) FREQ_STEP);
    spi_tx_buffer[0] = LORA_SET_RFFREQUENCY;
    spi_tx_buffer[1] = (uint8_t) ((freq >> 24) & 0xFF);
    spi_tx_buffer[2] = (uint8_t) ((freq >> 16) & 0xFF);
    spi_tx_buffer[3] = (uint8_t) ((freq >> 8) & 0xFF);
    spi_tx_buffer[4] = (uint8_t) (freq & 0xFF);
    Lora_SPIWrite(5);
}
void Lora_Init() {
	Lora_Reset();
    Lora_SetStandby(STDBY_RC);
    Lora_SetRegulatorMode(USE_LDO);
    Lora_SetRfSwitchMode(RF_SW_ON);
//    Lora_SetDio3AsTcxoCtrl(TCXO_CTRL_3_3V, 192 );
    Lora_Calibrate();
	Lora_CalibrateImage(frequency);
	Lora_SetPacketType(PACKET_TYPE_LORA);
	Lora_SetSyncWord();
	Lora_SetTxParams(20, LORA_RAMP_200_US);
	Lora_SetModulationParams(&LoraModulation);
	Lora_SetPacketParams(&LoraPacket);
	Lora_SetFrequency(frequency);
	Lora_SetBufferBaseAddresses(0x00, 0x00); //same FIFO for RX and TX
	Lora_SetStandby(STDBY_RC);
}
uint8_t Lora_Rx(uint8_t *data) {
	if (LoraPacket.PayloadLength != 255) {
		LoraPacket.PayloadLength = 255;
		Lora_SetPacketParams(&LoraPacket);
	}
	Lora_ClearIrqStatus(IRQ_ALL);
	Lora_SetIrqParams(IRQ_RX_DONE | IRQ_CRC_ERROR);
	Lora_SetRxBoosted(0);
	uint16_t timeout = 1000;
	while(timeout) {
		uint16_t irqRegs = Lora_GetIrqStatus();
		if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE) {
			if((irqRegs & IRQ_CRC_ERROR) == IRQ_CRC_ERROR) {
				return 0;
			}
			else {
				Lora_GetRxBufferStatus();
				uint8_t size = spi_rx_buffer[0];
				uint8_t offset = spi_rx_buffer[1];
				Lora_ReadBuffer(data, offset, size);
				return size;
			}
		}
		HAL_Delay(1);
		timeout--;
	}
	return 0;
}
void Lora_Tx(uint8_t *data, uint8_t size) {
	if (LoraPacket.PayloadLength != size) {
		LoraPacket.PayloadLength = size;
		Lora_SetPacketParams(&LoraPacket);
	}
	Lora_WriteBuffer(data, 0, size);
	Lora_SetIrqParams(IRQ_TX_DONE);
	Lora_ClearIrqStatus(IRQ_ALL);
	Lora_SetTx(0);
	uint16_t timeout = 1000;
	while(timeout) {
		uint16_t irqRegs = Lora_GetIrqStatus();
		if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE) {
			break;
		}
		HAL_Delay(1);
		timeout--;
	}
}

