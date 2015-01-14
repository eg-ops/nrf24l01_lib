#ifndef __NRF24L01_H
#define __NRF24L01_H

//#include "stm8l15x.h"
//#include "nrf24l01_config.h"




/*****  Registers   *****/

#define REGISTER_MASK 0x1F
#define CHANNEL_MASK 0x7F

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/*****   Register Bit Mask Shift Values (use with _BV())   *****/

#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

#define AW          0

#define ARD         4
#define ARC         0

#define CONT_WAVE   7
#define RF_DR_LOW   5
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RF_PWR      1
#define LNA_HCURR   0

#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0

#define PLOS_CNT    4
#define ARC_CNT     0

#define TX_REUSE    6
#define TX_FIFO_FULL   5
#define TX_FIFO_EMPTY    4
#define RX_FIFO_FULL     1
#define RX_FIFO_EMPTY    0

#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0

/*****   Instructions    *****/
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NOACK  0xB0
#define NOP                 0xFF

/****************************/

typedef enum { 
RF_PWR_18DBM = 0,
RF_PWR_12DBM = 1,
RF_PWR_6DBM = 2,
RF_PWR_0DBM = 3
} RF_PWR_TypeDef;

#define RF_PWR_MASK  3


typedef enum { 
ARD_WAIT_250US = 0,
ARD_WAIT_500US,
ARD_WAIT_750US,
ARD_WAIT_1000US,
ARD_WAIT_1250US,
ARD_WAIT_1500US,
ARD_WAIT_1750US,
ARD_WAIT_2000US,
ARD_WAIT_2250US,
ARD_WAIT_2500US,
ARD_WAIT_2750US,
ARD_WAIT_3000US,
ARD_WAIT_3250US,
ARD_WAIT_3500US,
ARD_WAIT_3750US,
ARD_WAIT_4000US
} ARD_TypeDef;

#define ARD_MASK 0xF


typedef enum { 
ARC_DISABLE = 0,
ARC_1,
ARC_2,
ARC_3,
ARC_4,
ARC_5,
ARC_6,
ARC_7,
ARC_8,
ARC_9,
ARC_10,
ARC_11,
ARC_12,
ARC_13,
ARC_14,
ARC_15
} ARC_TypeDef;


#define ARC_MASK 0xF


typedef enum { 
AW_3_BYTES = 1,
AW_4_BYTES,
AW_5_BYTES
} AW_TypeDef;

#define AW_MASK 3

#define ARC_CNT_MASK 0xF
#define PLOS_CNT_MASK 0xF

#define uint8_t unsigned char

void nrf24l01p_init();
uint8_t nrf24l01p_spi_rw(uint8_t value);
uint8_t nrf24l01p_read(uint8_t reg, uint8_t * mem, int size);
uint8_t nrf24l01p_write(uint8_t reg, uint8_t * mem, int size);
uint8_t nrf24l01p_read_byte(uint8_t reg);
void nrf24l01p_write_byte(uint8_t reg, uint8_t value);

#define nrf24l01p_ce_high() GPIO_WriteBit(GPIOA, CE, SET)
#define nrf24l01p_ce_low() GPIO_WriteBit(GPIOA, CE, RESET)

#define nrf24l01p_csn_high() GPIO_WriteBit(GPIOC, CSN, SET)
#define nrf24l01p_csn_low() GPIO_WriteBit(GPIOC, CSN, RESET)

#define nrf24l01p_irq_enable() {}
//enableInterrupts()
#define nrf24l01p_irq_disable() {}
//disableInterrupts()

#define nrf24l01p_read_reg(reg,mem,size) nrf24l01p_read(R_REGISTER | (REGISTER_MASK & reg), mem, size)
#define nrf24l01p_write_reg(reg,mem,size) nrf24l01p_write(W_REGISTER | (REGISTER_MASK & reg), mem, size)

#define power_up() nrf24l01p_write_byte(W_REGISTER | CONFIG, nrf24l01p_read_byte(R_REGISTER | CONFIG) | (1 << PWR_UP))
#define power_down() nrf24l01p_write_byte(W_REGISTER | CONFIG, nrf24l01p_read_byte(R_REGISTER | CONFIG) & ~(1 << PWR_UP))

#define set_rx_mode() nrf24l01p_write_byte(W_REGISTER | CONFIG, nrf24l01p_read_byte(R_REGISTER | CONFIG) | (1 << PRIM_RX))
#define set_tx_mode() nrf24l01p_write_byte(W_REGISTER | CONFIG, nrf24l01p_read_byte(R_REGISTER | CONFIG) & ~(1 << PRIM_RX))


#define enable_irqs() nrf24l01p_write_byte(W_REGISTER | CONFIG, nrf24l01p_read_byte(R_REGISTER | CONFIG) & ~((1 << MASK_MAX_RT) | (1<<MASK_RX_DR) | (1 << MASK_TX_DS)))
#define disable_irqs() nrf24l01p_write_byte(W_REGISTER | CONFIG, nrf24l01p_read_byte(R_REGISTER | CONFIG) |  ((1 << MASK_MAX_RT) | (1<<MASK_RX_DR) | (1 << MASK_TX_DS)) )

#define crc_enable() nrf24l01p_write_byte(W_REGISTER | CONFIG, (nrf24l01p_read_byte(R_REGISTER | CONFIG) | (1 << EN_CRC)))
#define crc_disable() nrf24l01p_write_byte(W_REGISTER | CONFIG, (nrf24l01p_read_byte(R_REGISTER | CONFIG) & ~(1 << EN_CRC)))
#define is_crc_enabled() ((nrf24l01p_read_byte(R_REGISTER | CONFIG) & (1 << EN_CRC)) != 0)

#define set_1byte_crc() nrf24l01p_write_byte(W_REGISTER | CONFIG, (nrf24l01p_read_byte(R_REGISTER | CONFIG) & ~(1 << CRCO)))
#define set_2byte_crc() nrf24l01p_write_byte(W_REGISTER | CONFIG, (nrf24l01p_read_byte(R_REGISTER | CONFIG) | (1 << CRCO)))
#define get_crc_size() ((nrf24l01p_read_byte(R_REGISTER | CONFIG) >> CRCO) & 1)


#define set_channel(ch) nrf24l01p_write_byte(W_REGISTER | RF_CH, ch & CHANNEL_MASK)
#define get_channel() (nrf24l01p_read_byte(R_REGISTER | RF_CH) & CHANNEL_MASK)

#define set_rf_power(rf_power) nrf24l01p_write_byte(W_REGISTER | RF_SETUP, (nrf24l01p_read_byte(R_REGISTER | RF_SETUP) & ~(RF_PWR_MASK << RF_PWR)) | ((rf_power & RF_PWR_MASK) << RF_PWR) )
#define get_rf_power() ( (nrf24l01p_read_byte(R_REGISTER | RF_SETUP) >> RF_PWR) & RF_PWR_MASK)

#define set_250kbps_speed() nrf24l01p_write_byte(W_REGISTER | RF_SETUP, ((nrf24l01p_read_byte(R_REGISTER | RF_SETUP) & ~(1 << RF_DR_HIGH)) | (1 << RF_DR_LOW)) )
#define set_1mbps_speed() nrf24l01p_write_byte(W_REGISTER | RF_SETUP, (nrf24l01p_read_byte(R_REGISTER | RF_SETUP) & ~(1 << RF_DR_HIGH | 1 << RF_DR_LOW)))
#define set_2mbps_speed() nrf24l01p_write_byte(W_REGISTER | RF_SETUP, ((nrf24l01p_read_byte(R_REGISTER | RF_SETUP) & ~(1 << RF_DR_LOW)) | (1 << RF_DR_HIGH)))
#define get_speed() ((nrf24l01p_read_byte(R_REGISTER | RF_SETUP) >> RF_DR_HIGH) & 5)

#define lna_gain_enable() nrf24l01p_write_byte(W_REGISTER | RF_SETUP, (nrf24l01p_read_byte(R_REGISTER | RF_SETUP) | (1 << LNA_HCURR)))
#define lna_gain_disable() nrf24l01p_write_byte(W_REGISTER | RF_SETUP, (nrf24l01p_read_byte(R_REGISTER | RF_SETUP) & ~(1 << LNA_HCURR)))
#define is_lna_gain_enabled() ((nrf24l01p_read_byte(R_REGISTER | RF_SETUP) & (1 << LNA_HCURR)) != 0)

#define set_retry_delay(delay) nrf24l01p_write_byte(W_REGISTER | SETUP_RETR, (nrf24l01p_read_byte(R_REGISTER | SETUP_RETR) & (ARC_MASK << ARC)) | ((delay & ARD_MASK) << ARD))
#define set_retry_count(count) nrf24l01p_write_byte(W_REGISTER | SETUP_RETR, (nrf24l01p_read_byte(R_REGISTER | SETUP_RETR) & (ARD_MASK << ARD)) | ((count & ARC_MASK) << ARC))

#define get_retry_delay() ((nrf24l01p_read_byte(R_REGISTER | SETUP_RETR) >> ARD) & ARD_MASK)
#define get_retry_count() (nrf24l01p_read_byte(R_REGISTER | SETUP_RETR) & ARC_MASK)
#define is_retry_enabled() (get_retry_count() != 0)

#define set_address_width(aw) nrf24l01p_write_byte(W_REGISTER | SETUP_AW, aw & AW_MASK)
#define get_address_width() (nrf24l01p_read_byte(R_REGISTER | SETUP_AW) & AW_MASK)

#define reset_lost_count() set_channel(get_channel())
#define get_curr_retry_count() (nrf24l01p_read_byte(R_REGISTER | OBSERVE_TX) & ARC_CNT_MASK)
#define get_curr_lost_count() ((nrf24l01p_read_byte(R_REGISTER | OBSERVE_TX) >> PLOS_CNT) & PLOS_CNT_MASK)

#define is_carrier_detected() nrf24l01p_read_byte(R_REGISTER | CD)

#define is_data_sent() (nrf24l01p_read_byte(R_REGISTER | STATUS) & (1<<TX_DS))
#define is_max_tx() (nrf24l01p_read_byte(R_REGISTER | STATUS) & (1<<MAX_TX))

#define auto_ack_enable(pipeId)  nrf24l01p_write_byte(W_REGISTER | EN_AA, (nrf24l01p_read_byte(R_REGISTER | EN_AA) | (1 << pipeId)))
#define auto_ack_disable(pipeId)  nrf24l01p_write_byte(W_REGISTER | EN_AA, (nrf24l01p_read_byte(R_REGISTER | EN_AA) & ~(1 << pipeId)))
#define is_auto_ack_enabled(pipeId)  ((nrf24l01p_read_byte(R_REGISTER | EN_AA) & (1 << pipeId)) != 0)

#define pipe_enable(pipeId) nrf24l01p_write_byte(W_REGISTER | EN_RXADDR, (nrf24l01p_read_byte(R_REGISTER | EN_RXADDR) | (1 << pipeId)))
#define pipe_disable(pipeId)  nrf24l01p_write_byte(W_REGISTER | EN_RXADDR, (nrf24l01p_read_byte(R_REGISTER | EN_RXADDR) & ~(1 << pipeId)))
#define is_pipe_enabled(pipeId)  ((nrf24l01p_read_byte(R_REGISTER | EN_RXADDR) & (1 << pipeId)) != 0)

#define set_pipe_size(pipeId, size) nrf24l01p_write_byte(W_REGISTER | (RX_PW_P0+pipeId), size)
#define get_pipe_size(pipeId) nrf24l01p_read_byte(R_REGISTER | (RX_PW_P0+pipeId))

#define dyn_size_enable() nrf24l01p_write_byte(W_REGISTER | FEATURE, (nrf24l01p_read_byte(R_REGISTER | FEATURE) | (1 << EN_DPL)))
#define dyn_size_disable() nrf24l01p_write_byte(W_REGISTER | FEATURE, (nrf24l01p_read_byte(R_REGISTER | FEATURE) & ~(1 << EN_DPL)))
#define is_dyn_size_enabled() ((nrf24l01p_read_byte(R_REGISTER | FEATURE) & (1 << EN_DPL)) != 0)

#define pipe_dyn_size_enable(pipeId) nrf24l01p_write_byte(W_REGISTER | DYNPD, (nrf24l01p_read_byte(R_REGISTER | DYNPD) | (1 << pipeId)))
#define pipe_dyn_size_disable(pipeId) nrf24l01p_write_byte(W_REGISTER | DYNPD, (nrf24l01p_read_byte(R_REGISTER | DYNPD) & ~(1 << pipeId)))
#define is_pipe_dyn_size_enabled(pipeId)  ((nrf24l01p_read_byte(R_REGISTER | DYNPD) & (1 << pipeId)) != 0)

#define get_dyn_packet_size() nrf24l01p_read_byte(R_RX_PL_WID)

#define is_tx_fifo_empty(fifo_status) ( (fifo_status & (1 << TX_FIFO_EMPTY)) != 0 )
#define is_rx_fifo_empty(fifo_status) ( (fifo_status & (1 << RX_FIFO_EMPTY)) != 0 )


#define is_max_retrys(status) ( (status & (1 << MAX_RT)) != 0 )

#endif //__NRF24L01_H
