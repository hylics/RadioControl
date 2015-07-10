/**
 rf22.c
 Copyright (C) 2014 Aethylic
 $Id: rf22.c,v 0.01 2014/08/03

*/

/* Includes ------------------------------------------------------------------*/
#include "rf22.h"
#include "spi.h"
#include "gpio.h"

/* Macro ---------------------------------------------------------------------*/
#define RF22_CONFIG_LEN   62
#define SS_LOW()          GPIOB->BSRRH = GPIO_PIN_1
#define SS_HIGH()         GPIOB->BSRRL = GPIO_PIN_1
#define TIMEOUT_SPI       2 //ms
#define TIMEOUT_SPI_BRST  5 //ms
#define OVERHEAD_RX       0 //byte
#define DBM_NMBRS         8

/* Private variables ---------------------------------------------------------*/
static const uint8_t rf22_dbm[DBM_NMBRS]= {
	RF22_TXPOW_1DBM,
	RF22_TXPOW_2DBM,
  RF22_TXPOW_5DBM,
	RF22_TXPOW_8DBM,
  RF22_TXPOW_11DBM,
	RF22_TXPOW_14DBM,
  RF22_TXPOW_17DBM,
  RF22_TXPOW_20DBM };


/**----------------------------------------------------------------------------
  * @brief RF22 Configuration massive
  ----------------------------------------------------------------------------*/
static const uint8_t rf22_config[RF22_CONFIG_LEN][2]={
	{RF22_REG_05_INTERRUPT_ENABLE1, 0x00},                       // disable interrupts 1
	{RF22_REG_06_INTERRUPT_ENABLE2, 0x00},                       // disable interrupts 2
	{RF22_REG_07_OPERATING_MODE1, RF22_XTON},                    // ready IDLE mode xton=1
	{RF22_REG_09_OSCILLATOR_LOAD_CAPACITANCE, 0x6d},             // don't touch!
	{RF22_REG_0A_UC_OUTPUT_CLOCK, RF22_UC_4MHZ},                 // MCU clock 4MHz
	{RF22_REG_0B_GPIO_CONFIGURATION0, RF22_GPIOX_DRV001PUP0|RF22_GPIOX_TX_STATE},     //gpio1 for TX State (output)
	{RF22_REG_0C_GPIO_CONFIGURATION1, RF22_GPIOX_DRV001PUP0|RF22_GPIOX_RX_STATE},     // gpio2 for RX State (output) 
	{RF22_REG_0D_GPIO_CONFIGURATION2, RF22_GPIO2_MCUCLOCK_OUT},  // GPIO2 as mcu clock out
	{RF22_REG_0E_IO_PORT_CONFIGURATION, 0x00},                   // IO antenna disable
	{RF22_REG_0F_ADC_CONFIGURATION, RF22_ADCSEL},                // No ADC
	{RF22_REG_10_ADC_SENSOR_AMP_OFFSET, 0x00},                   // No ADC
	{RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION, RF22_ENTSOFFS}, // no temperature sensor used
	{RF22_REG_13_TEMPERATURE_VALUE_OFFSET, 0x00},                // no temperature sensor used
	{RF22_REG_1C_IF_FILTER_BANDWIDTH, 0xac},                     // 24kbps, Dev 20 kHz, BW 51.2 kHz
	{RF22_REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE, RF22_ENAFC},       // AFC 0n?? or 0x44
	{RF22_REG_1E_AFC_TIMING_CONTROL, 0x0a},                      // AFC timing
	{RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE, 0x7d},        // clock recovery
	{RF22_REG_21_CLOCK_RECOVERY_OFFSET2, 0x01},                  // clock recovery
	{RF22_REG_22_CLOCK_RECOVERY_OFFSET1, 0x06},                  // clock recovery
	{RF22_REG_23_CLOCK_RECOVERY_OFFSET0, 0x25},                  // clock recovery
	{RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1, 0x00},        // clock recovery
	{RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0, 0xfe},        // clock recovery
	{RF22_REG_2A_AFC_LIMITER, 0x28},                             // AFC limiter
	{RF22_REG_2C_OOK_COUNTER_VALUE_1, 0x28},                     // ook counter value
	{RF22_REG_2D_OOK_COUNTER_VALUE_2, 0x34},                     // ook counter value
  {RF22_REG_2E_SLICER_PEAK_HOLD, 0x28},                        //
	{RF22_REG_30_DATA_ACCESS_CONTROL, 0xac},                     // data access control
	{RF22_REG_32_HEADER_CONTROL1, 0x8c},                         // broadcast = disable, header3 compare only
	{RF22_REG_33_HEADER_CONTROL2, 0xa},                          // 1byte header length, variable pkt length, 1byte sync word = syncword3
	{RF22_REG_34_PREAMBLE_LENGTH, 0x08},                         // preamble = 32 bits
	{RF22_REG_35_PREAMBLE_DETECTION_CONTROL1, 0x2a},             // preamble detect = 16bit, rssi_offset = 0
	{RF22_REG_36_SYNC_WORD3, 0x2d},                              // preamble 3
	{RF22_REG_37_SYNC_WORD2, 0xd4},                              // preamble 2
	{RF22_REG_38_SYNC_WORD1, 0x00},                              // preamble 1
	{RF22_REG_39_SYNC_WORD0, 0x00},                              // preamble 0
	{RF22_REG_3A_TRANSMIT_HEADER3, 0x00},                        // th header 3
	{RF22_REG_3B_TRANSMIT_HEADER2, 0x00},                        // tx header 2
	{RF22_REG_3C_TRANSMIT_HEADER1, 0x00},                        // tx header 1
	{RF22_REG_3D_TRANSMIT_HEADER0, 0x00},                        // tx header 0
	{RF22_REG_3E_PACKET_LENGTH, PKT_LEN},                        // pkt len = 25, not used
	{RF22_REG_3F_CHECK_HEADER3, 0x00},                           // rx header 3
	{RF22_REG_40_CHECK_HEADER2, 0x00},                           // rx header 2
	{RF22_REG_41_CHECK_HEADER1, 0x00},                           // rx header 1
	{RF22_REG_42_CHECK_HEADER0, 0x00},                           // rx header 0
	{RF22_REG_43_HEADER_ENABLE3, 0xff},                          // header 3 check mask
	{RF22_REG_44_HEADER_ENABLE2, 0xff},                          // header 2 check mask
	{RF22_REG_45_HEADER_ENABLE1, 0xff},                          // header 1 check mask
	{RF22_REG_46_HEADER_ENABLE0, 0xff},                          // header 0 check mask
	{RF22_REG_69_AGC_OVERRIDE1, 0x60},                           // automatic gain control enable fo preamble time
	{RF22_REG_6D_TX_POWER, RF22_TXPOW_2DBM},                     // transmit power, 0x0a ???
	{RF22_REG_6E_TX_DATA_RATE1, 0xc4},                           // tx data rate 1 0x4ea5*1000/2097152\ 9.6kbit
	{RF22_REG_6F_TX_DATA_RATE0, 0x9c},                           // tx data rate 0
	{RF22_REG_70_MODULATION_CONTROL1, 0x2c},                     // no manchester, no whitening, txdtrtscale =1 for < 30 kbps
	{RF22_REG_71_MODULATION_CONTROL2, RF22_DTMOD_FIFO|RF22_MODTYP_GFSK}, // 0x23
	{RF22_REG_72_FREQUENCY_DEVIATION, 0x50},                     // deviation 10kHz
	{RF22_REG_73_FREQUENCY_OFFSET1, 0x00},                       // frequency offset = 0
	{RF22_REG_74_FREQUENCY_OFFSET2, 0x00},                       // frequency offset = 0
	{RF22_REG_75_FREQUENCY_BAND_SELECT, 0x73},                   // band selection
	{RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1, 0x64},              // carrier freq (430+(0x6400/64000) ) (*2 if hbsel=1)
	{RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0, 0x00},              // carrier freq = 868
	{RF22_REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT, 0x00},        // freq channel = 0. bt default
	{RF22_REG_7A_FREQUENCY_HOPPING_STEP_SIZE, 0x02}              // freq hopping size = 20 kHz
};

/* Public variables ----------------------------------------------------------*/
__IO uint16_t it_status;


/* Private function prototypes -----------------------------------------------*/
static RF22_StatusTypeDef rf22_WriteReg(uint8_t addr, uint8_t wdata);
static uint8_t rf22_ReadReg(uint8_t addr);
static void rf22_ResetTxFIFO (void);
static void rf22_ResetRxFIFO (void);
static void rf22_ItEnable(void);

/**----------------------------------------------------------------------------
 * @brief  write config to rf22 modem
 * @param  config - massive of pairs "address reg, value reg"
 * @retval RF22 status
 -----------------------------------------------------------------------------*/
RF22_StatusTypeDef rf22_Init(void) {
	/*disable EXTi for ignoring chaotic interrupts*/
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	
	/*Software reset rf22 modem*/
	rf22_WriteReg(RF22_REG_07_OPERATING_MODE1, RF22_SWRES);
	
	//delay after reset
	HAL_Delay(25);
	
	if (rf22_ReadReg(RF22_REG_00_DEVICE_TYPE)!=0x08) {
		return RF22_ERROR;
	}
	if (rf22_ReadReg(RF22_REG_01_VERSION_CODE)!=0x06) {
		return RF22_ERROR;
	}
	if (!(rf22_ReadReg(RF22_REG_04_INTERRUPT_STATUS2)&((RF22_ICHIPRDY|RF22_IPOR)>>8))) {
		return RF22_ERROR;
	}
	
	/*write config in loop*/
	for (uint32_t n=0; n<=RF22_CONFIG_LEN; n++) {
		//16bit packet size
		uint16_t size=0x10;
		HAL_StatusTypeDef write_status;
		//construct massive for write in SPI
		uint8_t wdata[2]={(rf22_config[n][0]|RF22_SPI_WRITE_MASK), rf22_config[n][1]};
		/*spi transmit in blocking mode*/
		SS_LOW();
		write_status=HAL_SPI_Transmit(SPI_HANDLE, wdata, size, TIMEOUT_SPI);
		SS_HIGH();
		
		if (write_status!=HAL_OK) {
			//n--; //return one step
			return RF22_ERROR; 
		}
	}
	
	//read reg 02h device status
	if ((rf22_ReadReg(RF22_REG_02_DEVICE_STATUS)&RF22_CPS)!=RF22_CPS_IDLE) {
		return RF22_ERROR;
	}
	
	/*disable all interrupts*/
	rf22_WriteReg(RF22_REG_05_INTERRUPT_ENABLE1, 0x00);
	rf22_WriteReg(RF22_REG_06_INTERRUPT_ENABLE2, 0x00);
	
	/*clear all interrupt flag. Ignoring retval*/
	rf22_ReadReg(RF22_REG_03_INTERRUPT_STATUS1);
	rf22_ReadReg(RF22_REG_03_INTERRUPT_STATUS1);

	/*reset FIFO pointer*/
	rf22_ResetRxFIFO();
	rf22_ResetTxFIFO();
	
	//now enable interrupts
	rf22_ItEnable();
	
	/* ...and now enable EXTi*/
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	
	return RF22_OK;
}


/**----------------------------------------------------------------------------
 * @brief  write value to reg of rf22 modem. Private function
 * @param  addr - 
 * @param  wdata - data to write
 * @retval RF22 status
 -----------------------------------------------------------------------------*/
static RF22_StatusTypeDef rf22_WriteReg(uint8_t addr, uint8_t wdata) {
	//2 byte packet size
	uint16_t size=0x02;
	HAL_StatusTypeDef w_status;
	//construct massive for write in SPI
	uint8_t pwdata[2]={wdata, (addr|RF22_SPI_WRITE_MASK)};
	
	SS_LOW();
	w_status=HAL_SPI_Transmit(SPI_HANDLE, pwdata, size, TIMEOUT_SPI);
	SS_HIGH();
	switch (w_status) {
		case HAL_OK:
			return RF22_OK;
		case HAL_ERROR:
			return RF22_ERROR;
		case HAL_BUSY:
			return RF22_BUSY;
		case HAL_TIMEOUT:
			return RF22_BUSY;
		default:
			return RF22_ERROR;
	}
}

/**----------------------------------------------------------------------------
 * @brief  read value of reg of rf22 modem. Private function
 * @param  addr - 
 * @retval value of reg
 -----------------------------------------------------------------------------*/
static uint8_t rf22_ReadReg(uint8_t addr) {
	//2 byte packet size
	uint16_t size=0x02;
	uint8_t pwdata[2]={0x00, addr};
	uint8_t prdata[2];
	
	SS_LOW();
	HAL_SPI_TransmitReceive(SPI_HANDLE, pwdata, prdata, size, TIMEOUT_SPI);
	SS_HIGH();
	return prdata[0];
}


/**----------------------------------------------------------------------------
 * @brief  TX FIFO Reset/Clear. Setting ffclrrx =1 followed by ffclrtx = 0 will clear the 
   contents of the TX FIFO. Private function
 * @param  none
 * @retval none
 -----------------------------------------------------------------------------*/
static void rf22_ResetTxFIFO(void) {
	__IO uint8_t tmp_clr_tx;
	tmp_clr_tx=rf22_ReadReg(RF22_REG_08_OPERATING_MODE2);
	tmp_clr_tx|=RF22_FFCLRTX;
	rf22_WriteReg(RF22_REG_08_OPERATING_MODE2, tmp_clr_tx);
	tmp_clr_tx&=~RF22_FFCLRTX;
	rf22_WriteReg(RF22_REG_08_OPERATING_MODE2, tmp_clr_tx);
}

/**----------------------------------------------------------------------------
 * @brief  RX FIFO Reset/Clear. Setting ffclrrx =1 followed by ffclrrx = 0 will clear the 
 * contents of the RX FIFO. Private function
 * @param  none 
 * @retval none
 -----------------------------------------------------------------------------*/
static void rf22_ResetRxFIFO(void) {
	__IO uint8_t tmp_clr_rx;
	tmp_clr_rx=rf22_ReadReg(RF22_REG_08_OPERATING_MODE2);
	tmp_clr_rx|=RF22_FFCLRRX;
	rf22_WriteReg(RF22_REG_08_OPERATING_MODE2, tmp_clr_rx);
	tmp_clr_rx&=~RF22_FFCLRRX;
	rf22_WriteReg(RF22_REG_08_OPERATING_MODE2, tmp_clr_rx);
}

/**----------------------------------------------------------------------------
 * @brief  TX FIFO burst write.
 * @param  pointer to massive of data to write
 * @param  number of bytes to send
 * @retval status of transmit
 -----------------------------------------------------------------------------*/
RF22_StatusTypeDef rf22_BrstWriteFifo(uint8_t *pwbdata, uint8_t nbytes) {
	HAL_StatusTypeDef wbrst_status;
	//packet size in bits
	uint16_t size=(uint16_t)((nbytes+1));
	
	//construct massive for write in SPI
	uint8_t pwbdata_spi[nbytes+1];
  pwbdata_spi[nbytes+1]=RF22_REG_7F_FIFO_ACCESS|RF22_SPI_WRITE_MASK;
	
	/*Use memcpy? No, library is fat!*/
	for(uint32_t i=0; i<=nbytes; i++) {
		pwbdata_spi[nbytes-i]=*(pwbdata+i);
	}
	
	SS_LOW();
	wbrst_status=HAL_SPI_Transmit(SPI_HANDLE, pwbdata_spi, size, TIMEOUT_SPI_BRST);
	SS_HIGH();
	
	rf22_ResetTxFIFO();
	
	switch (wbrst_status) {
		case HAL_OK:
			return RF22_OK;
		case HAL_ERROR:
			return RF22_ERROR;
		case HAL_BUSY:
			return RF22_BUSY;
		case HAL_TIMEOUT:
			return RF22_BUSY;
		default:
			return RF22_ERROR;
	}
}

/**----------------------------------------------------------------------------
 * @brief  RX FIFO burst read.
 * @param  pointer to massive of data to read
 * @retval number of bytes to read
 -----------------------------------------------------------------------------*/
RF22_StatusTypeDef rf22_BrstReadFifo(uint8_t *prbdata, uint8_t nbytes) {
	HAL_StatusTypeDef rbrst_status; //Read Burst status
	/*maybe use massive? content in memory over 1 byte var is undefined...*/
	uint8_t addr_fifo=RF22_REG_7F_FIFO_ACCESS;
	/*size= packet+overhead+addr_size*/
	uint16_t size=(uint16_t)(nbytes+OVERHEAD_RX+1);
	uint8_t local_rbdata[size];
	
	SS_LOW();
	rbrst_status=HAL_SPI_TransmitReceive(SPI_HANDLE, &addr_fifo, local_rbdata, size, TIMEOUT_SPI_BRST);
	SS_HIGH();
	
	for (uint32_t i=0; i<=nbytes; i++) {
		*(prbdata+i)=local_rbdata[nbytes-i];
	}
	
	rf22_ResetRxFIFO();
	
	switch (rbrst_status) {
		case HAL_OK:
			return RF22_OK;
		case HAL_ERROR:
			return RF22_ERROR;
		case HAL_BUSY:
			return RF22_BUSY;
		case HAL_TIMEOUT:
			return RF22_BUSY;
		default:
			return RF22_ERROR;
	}	
}

/**----------------------------------------------------------------------------
 * @brief  enable interrupts
 * @param  none
 * @retval none
 -----------------------------------------------------------------------------*/
static void rf22_ItEnable(void) {
	uint8_t irq1=0x00;
	uint8_t irq2=0x00;
	
	irq1|=RF22_IFFERROR;
	irq1|=RF22_ITXFFAFULL;
	irq1|=RF22_ITXFFAEM;
	irq1|=RF22_IRXFFAFULL;
	irq1|=RF22_IEXT;
	irq1|=RF22_IPKSENT;
	irq1|=RF22_IPKVALID;
	irq1|=RF22_ICRCERROR;
	
	irq2|=RF22_ISWDET;
	irq2|=RF22_IPREAVAL;
	irq2|=RF22_IPREAINVAL;
	irq2|=RF22_IRSSI;
	irq2|=RF22_IWUT;
	//irq2|=RF22_ILBD;
	irq2|=RF22_ICHIPRDY;
	irq2|=RF22_IPOR;

	rf22_WriteReg(RF22_REG_05_INTERRUPT_ENABLE1, irq1);
	rf22_WriteReg(RF22_REG_06_INTERRUPT_ENABLE2, irq2);

	return;
}

/**----------------------------------------------------------------------------
 * @brief  return RSSI value
 * @param  none
 * @retval RSSI value uint8_t type, 0.5 dB per bit
 -----------------------------------------------------------------------------*/
uint8_t rf22_RssiRead(void) {
	return rf22_ReadReg(RF22_REG_26_RSSI);
}

/**----------------------------------------------------------------------------
 * @brief  set RSSI threshold for Clear Channel indicator
 * @param  threshold value
 * @retval status
 -----------------------------------------------------------------------------*/
RF22_StatusTypeDef rf22_RssiThreshholdSet(uint8_t rssi_threshold) {
	RF22_StatusTypeDef set_rssi_threshold_status;
	set_rssi_threshold_status = rf22_WriteReg(RF22_REG_27_RSSI_THRESHOLD, rssi_threshold);
	return set_rssi_threshold_status;
}

/**----------------------------------------------------------------------------
 * @brief  set txon bit
 * @param  threshold value
 * @retval status
 -----------------------------------------------------------------------------*/
RF22_StatusTypeDef rf22_TxOn(void) {
	RF22_StatusTypeDef set_txon_status;
	uint8_t tmp_reg;
	tmp_reg=rf22_ReadReg(RF22_REG_07_OPERATING_MODE1);
	tmp_reg|=RF22_TXON;
	set_txon_status=rf22_WriteReg(RF22_REG_07_OPERATING_MODE1, tmp_reg);
	return set_txon_status;
}

/**----------------------------------------------------------------------------
 * @brief  set rxon bit
 * @param  threshold value
 * @retval status
 -----------------------------------------------------------------------------*/
RF22_StatusTypeDef rf22_RxOn(void) {
	RF22_StatusTypeDef set_rxon_status;
	uint8_t tmp_reg;
	tmp_reg=rf22_ReadReg(RF22_REG_07_OPERATING_MODE1);
	tmp_reg|=RF22_RXON;
	set_rxon_status=rf22_WriteReg(RF22_REG_07_OPERATING_MODE1, tmp_reg);
	return set_rxon_status;
}

/**----------------------------------------------------------------------------
 * @brief  read interrupt status, call from interrupt or in code
 * @param  pointer to variable where being stored interrupt status
 * @retval status
 -----------------------------------------------------------------------------*/
RF22_StatusTypeDef rf22_ItRead(__IO uint16_t *ptr_status) {
	uint8_t irq1, irq2;
	
	irq1=rf22_ReadReg(RF22_REG_03_INTERRUPT_STATUS1);
	irq2=rf22_ReadReg(RF22_REG_04_INTERRUPT_STATUS2);
	
	*ptr_status = 0;
	*ptr_status |= (uint16_t)irq1;
	*ptr_status |= ((uint16_t)irq2<<8);
	return RF22_OK;
}

/**----------------------------------------------------------------------------
 * @brief  set tx power, increase or decrease to one step of power, or none, but return current power anyway
 * @param  command POW_DECR, POW_INCR, RF22_TXPOW_1DBM-RF22_TXPOW_20DBM
 * @retval current tx power
 -----------------------------------------------------------------------------*/
uint8_t rf22_PwrChange(uint8_t arg) {
	uint8_t current_pwr;
	uint8_t n=0;
	current_pwr=rf22_ReadReg(RF22_REG_6D_TX_POWER);
	
	for(uint32_t i=0; i<=DBM_NMBRS; i++) {
		if(rf22_dbm[i]&current_pwr) {
			n=i;
			break;
		}
	}
		
	switch (arg) {
		case POW_DECR:
			//
		  if(n>0) {
				n--;
			}
		  rf22_WriteReg(RF22_REG_6D_TX_POWER, rf22_dbm[n]);
		  break;
		case POW_INCR:
			//
		  if(n<DBM_NMBRS) {
				n++;
			}
		  rf22_WriteReg(RF22_REG_6D_TX_POWER, rf22_dbm[n]);
		  break;
		case RF22_TXPOW_1DBM:
		case RF22_TXPOW_2DBM:
		case RF22_TXPOW_5DBM:
	  case RF22_TXPOW_8DBM:
		case RF22_TXPOW_11DBM:
		case RF22_TXPOW_14DBM:
		case RF22_TXPOW_17DBM:
		case RF22_TXPOW_20DBM:
		  rf22_WriteReg(RF22_REG_6D_TX_POWER, arg);
		  break;
		default:
		  break;
	}
	
	return n;
}





