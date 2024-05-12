#ifndef SX1262_H
#define	SX1262_H

#define LORA_FREQUENCY 868000000


#define XTAL_FREQ                       32000000
#define FREQ_DIV                        33554432
#define FREQ_STEP                       0.95367431640625 // ( ( double )( XTAL_FREQ / ( double )FREQ_DIV ) )
#define FREQ_ERR                        0.47683715820312

#define REG_LR_PACKETPARAMS             0x0704  //The address of the register holding the packet configuration
#define REG_LR_PAYLOADLENGTH            0x0702  //The address of the register holding the payload size
#define REG_LR_SYNCWORDBASEADDRESS      0x06C0  //The addresses of the registers holding SyncWords values
#define REG_LR_SYNCWORD                 0x0740  //The addresses of the register holding LoRa Modem SyncWord value
#define LORA_MAC_PRIVATE_SYNCWORD       0x1424  //Syncword for Private LoRa networks
#define LORA_MAC_PUBLIC_SYNCWORD        0x3444  //Syncword for Public LoRa networks
#define RANDOM_NUMBER_GENERATORBASEADDR 0x0819  //The address of the register giving a 4 bytes random number
#define REG_RX_GAIN                     0x08AC  //The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
#define REG_FREQUENCY_ERRORBASEADDR     0x076B  //The address of the register holding frequency error indication
#define REG_XTA_TRIM                    0x0911  //Change the value on the device internal trimming capacitor
#define REG_OCP                         0x08E7  //Set the current max value in the over current protection

typedef enum {
    MODE_SLEEP                          = 0x00, //! The LORA is in sleep mode
    MODE_STDBY_RC,                              //! The LORA is in standby mode with RC oscillator
    MODE_STDBY_XOSC,                            //! The LORA is in standby mode with XOSC oscillator
    MODE_FS,                                    //! The LORA is in frequency synthesis mode
    MODE_TX,                                    //! The LORA is in transmit mode
    MODE_RX,                                    //! The LORA is in receive mode
    MODE_RX_DC,                                 //! The LORA is in receive duty cycle mode
    MODE_CAD                                    //! The LORA is in channel activity detection mode
}LoraOperatingModes_t;

typedef enum {
    STDBY_RC                            = 0x00,
    STDBY_XOSC                          = 0x01,
}LoraStandbyModes_t;

typedef enum {
    USE_LDO                                 = 0x00,
    USE_DCDC                                = 0x01,
}LoraRegulatorMode_t;

typedef enum {
    RF_SW_ON                                 = 0x01,
    RF_SW_OFF                                = 0x00,
}LoraRFSwitch_t;


typedef enum {
    PACKET_TYPE_GFSK                        = 0x00,
    PACKET_TYPE_LORA                        = 0x01,
}LORAPacketTypes_t;

typedef enum {
    LORA_RAMP_10_US                        = 0x00,
    LORA_RAMP_20_US                        = 0x01,
    LORA_RAMP_40_US                        = 0x02,
    LORA_RAMP_80_US                        = 0x03,
    LORA_RAMP_200_US                       = 0x04,
    LORA_RAMP_800_US                       = 0x05,
    LORA_RAMP_1700_US                      = 0x06,
    LORA_RAMP_3400_US                      = 0x07,
}LORARampTimes_t;


typedef enum {
    LORA_SF5                                = 0x05,
    LORA_SF6                                = 0x06,
    LORA_SF7                                = 0x07,
    LORA_SF8                                = 0x08,
    LORA_SF9                                = 0x09,
    LORA_SF10                               = 0x0A,
    LORA_SF11                               = 0x0B,
    LORA_SF12                               = 0x0C,
}LoraSpreadingFactors_t;

typedef enum {
    LORA_BW_500                             = 6,
    LORA_BW_250                             = 5,
    LORA_BW_125                             = 4,
    LORA_BW_062                             = 3,
    LORA_BW_041                             = 10,
    LORA_BW_031                             = 2,
    LORA_BW_020                             = 9,
    LORA_BW_015                             = 1,
    LORA_BW_010                             = 8,
    LORA_BW_007                             = 0,
}LoraBandwidths_t;

typedef enum {
    LORA_CR_4_5                             = 0x01,
    LORA_CR_4_6                             = 0x02,
    LORA_CR_4_7                             = 0x03,
    LORA_CR_4_8                             = 0x04,
}LoraCodingRates_t;

typedef enum {
    LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
    LORA_PACKET_FIXED_LENGTH                = 0x01,         //!< The packet is known on both sides, no header included in the packet
    LORA_PACKET_EXPLICIT                    = LORA_PACKET_VARIABLE_LENGTH,
    LORA_PACKET_IMPLICIT                    = LORA_PACKET_FIXED_LENGTH,
}LoraPacketLengthsMode_t;

typedef enum {
    LORA_CRC_ON                             = 0x01,
    LORA_CRC_OFF                            = 0x00,
}LoraCrcModes_t;

typedef enum {
    LORA_IQ_NORMAL                          = 0x00,
    LORA_IQ_INVERTED                        = 0x01,
}LoraIQModes_t;

typedef enum {
    TCXO_CTRL_1_6V                          = 0x00,
    TCXO_CTRL_1_7V                          = 0x01,
    TCXO_CTRL_1_8V                          = 0x02,
    TCXO_CTRL_2_2V                          = 0x03,
    TCXO_CTRL_2_4V                          = 0x04,
    TCXO_CTRL_2_7V                          = 0x05,
    TCXO_CTRL_3_0V                          = 0x06,
    TCXO_CTRL_3_3V                          = 0x07,
}LoraTcxoCtrlVoltage_t;

typedef enum { //Represents the interruption masks available for the LORA (not all these interruptions are available for all packet types)
    IRQ_NONE                                = 0x0000,
    IRQ_TX_DONE                             = 0x0001,
    IRQ_RX_DONE                             = 0x0002,
    IRQ_PREAMBLE_DETECTED                   = 0x0004,
    IRQ_SYNCWORD_VALID                      = 0x0008,
    IRQ_HEADER_VALID                        = 0x0010,
    IRQ_HEADER_ERROR                        = 0x0020,
    IRQ_CRC_ERROR                           = 0x0040,
    IRQ_CAD_DONE                            = 0x0080,
    IRQ_CAD_ACTIVITY_DETECTED               = 0x0100,
    IRQ_RX_TX_TIMEOUT                       = 0x0200,
    IRQ_ALL                                 = 0xFFFF,
}LoraIrqMasks_t;

typedef enum LoraCommands_e { //Represents all possible opcode understood by the LORA
    LORA_GET_STATUS                        = 0xC0,
    LORA_WRITE_REGISTER                    = 0x0D,
    LORA_READ_REGISTER                     = 0x1D,
    LORA_WRITE_BUFFER                      = 0x0E,
    LORA_READ_BUFFER                       = 0x1E,
    LORA_SET_SLEEP                         = 0x84,
    LORA_SET_STANDBY                       = 0x80,
    LORA_SET_FS                            = 0xC1,
    LORA_SET_TX                            = 0x83,
    LORA_SET_RX                            = 0x82,
    LORA_SET_RXDUTYCYCLE                   = 0x94,
    LORA_SET_CAD                           = 0xC5,
    LORA_SET_TXCONTINUOUSWAVE              = 0xD1,
    LORA_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
    LORA_SET_PACKETTYPE                    = 0x8A,
    LORA_GET_PACKETTYPE                    = 0x11,
    LORA_SET_RFFREQUENCY                   = 0x86,
    LORA_SET_TXPARAMS                      = 0x8E,
    LORA_SET_PACONFIG                      = 0x95,
    LORA_SET_CADPARAMS                     = 0x88,
    LORA_SET_BUFFERBASEADDRESS             = 0x8F,
    LORA_SET_MODULATIONPARAMS              = 0x8B,
    LORA_SET_PACKETPARAMS                  = 0x8C,
    LORA_GET_RXBUFFERSTATUS                = 0x13,
    LORA_GET_PACKETSTATUS                  = 0x14,
    LORA_GET_RSSIINST                      = 0x15,
    LORA_GET_STATS                         = 0x10,
    LORA_RESET_STATS                       = 0x00,
    LORA_CFG_DIOIRQ                        = 0x08,
    LORA_GET_IRQSTATUS                     = 0x12,
    LORA_CLR_IRQSTATUS                     = 0x02,
    LORA_CALIBRATE                         = 0x89,
    LORA_CALIBRATEIMAGE                    = 0x98,
    LORA_SET_REGULATORMODE                 = 0x96,
    LORA_GET_ERROR                         = 0x17,
    LORA_SET_TCXOMODE                      = 0x97,
    LORA_SET_TXFALLBACKMODE                = 0x93,
    LORA_SET_RFSWITCHMODE                  = 0x9D,
    LORA_SET_STOPRXTIMERONPREAMBLE         = 0x9F,
    LORA_SET_LORASYMBTIMEOUT               = 0xA0,
}LoraCommands_t;

typedef struct {
    LoraSpreadingFactors_t  SpreadingFactor;   //!< Spreading Factor for the LoRa modulation
    LoraBandwidths_t        Bandwidth;         //!< Bandwidth for the LoRa modulation
    LoraCodingRates_t       CodingRate;        //!< Coding rate for the LoRa modulation
}LoraModulationParams_t;

typedef struct {
	uint16_t                PreambleLength;    //!< The preamble length is the number of LoRa symbols in the preamble
	LoraPacketLengthsMode_t HeaderType;        //!< If the header is explicit, it will be transmitted in the LoRa packet. If the header is implicit, it will not be transmitted
	uint8_t                 PayloadLength;     //!< Size of the payload in the LoRa packet
	LoraCrcModes_t          CrcMode;           //!< Size of CRC block in LoRa packet
	LoraIQModes_t           InvertIQ;          //!< Allows to swap IQ for LoRa packet
}LoraPacketParams_t;

typedef struct {
	int8_t Rssi;                                //!< The RSSI of the last packet
	int8_t Snr;                                 //!< The SNR of the last packet
}LoraPacketStatus_t;



void Lora_Init(void);
void Lora_Tx(uint8_t *data, uint8_t size);
uint8_t Lora_Rx(uint8_t *data);
LoraPacketStatus_t Lora_GetPacketStatus();

#endif
