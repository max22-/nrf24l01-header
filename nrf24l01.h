#ifndef INCLUDE_NRF24l01_H
#define INCLUDE_NRF24l01_H

#include <stdint.h>

/* User provided functions ************************************************* */
extern void nrf24l01_select(); /* begin SPI transaction : drive CS pin low */
extern void nrf24l01_unselect(); /* end SPI transaction : drive CS pin high */
extern void nrf24l01_enable(); /* drive CE pin high */
extern void nrf24l01_disable(); /* drive CE pin low */
extern uint8_t nrf24l01_spi_transfer(uint8_t);
extern void nrf24l01_delay(unsigned int);
/* User provided constant */
extern const size_t nrf24l01_payload_length;

/* Exported library functions ********************************************** */
int nrf24l01_simple_tx_mode(uint8_t channel, const uint8_t *address);
int nrf24l01_transmit(const uint8_t *data);
int nrf24l01_simple_rx_mode(uint8_t channel, const uint8_t *address);
int nrf24l01_data_available();
void nrf24l01_receive(uint8_t *data);
void nrf24l01_dump();

#ifdef NRF24L01_IMPLEMENTATION

/* Registers *************************************************************** */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define RX_ADDR_P0  0x0a
#define RX_ADDR_P1  0x0b
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define FIFO_STATUS 0x17
#define DYNPD       0x1c
#define FEATURE     0x1d

/* Commands  *************************************************************** */
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xa0
#define FLUSH_TX      0xe1
#define FLUSH_RX      0xe2

static void nrf24l01_write_reg(uint8_t reg, uint8_t data) {
  nrf24l01_select();
  nrf24l01_spi_transfer(reg|(1<<5));
  nrf24l01_spi_transfer(data);
  nrf24l01_unselect();
}

static void nrf24l01_write_reg_multi(uint8_t reg, const uint8_t *data, size_t size) {
  nrf24l01_select();
  nrf24l01_spi_transfer(reg|(1<<5));
  for(int i = 0; i < size; i++)
    nrf24l01_spi_transfer(data[i]);
  nrf24l01_unselect();
}

static uint8_t nrf24l01_read_reg(uint8_t reg) {
  nrf24l01_select();
  nrf24l01_spi_transfer(reg);
  uint8_t ret = nrf24l01_spi_transfer(0);
  nrf24l01_unselect();
  return ret;
}

static void nrf24l01_read_reg_multi(uint8_t reg, uint8_t *data, size_t size) {
  nrf24l01_select();
  nrf24l01_spi_transfer(reg);
  for(int i = 0; i < size; i++)
    data[i] = SPI.transfer(0);
  nrf24l01_unselect();
}

static void nrf24l01_send_cmd(uint8_t cmd) { /* TODO: is it useful ? */
  nrf24l01_select();
  SPI.transfer(cmd);
  nrf24l01_unselect();
}

static int nrf24l01_check_comm() {
  nrf24l01_write_reg(0, 0);
  if(nrf24l01_read_reg(0) != 0)
    return 0;
  const uint8_t dummy_value = 0x7f;
  nrf24l01_write_reg(0, dummy_value);
  const uint8_t read_back = nrf24l01_read_reg(0);
  nrf24l01_write_reg(0, 0);
  return read_back == dummy_value;
}

int nrf24l01_simple_tx_mode(uint8_t channel, const uint8_t *address) {
  if(!nrf24l01_check_comm())
    return 0;
  nrf24l01_disable();
  nrf24l01_write_reg(CONFIG,      0x7e); /* CRC enabled and interrupt pin disabled */
  nrf24l01_write_reg(EN_AA,       0x00);
  nrf24l01_write_reg(EN_RXADDR,   0x00);
  nrf24l01_write_reg(SETUP_AW,    0x03); /* 5 bytes address width */
  nrf24l01_write_reg(SETUP_RETR,  0x00);
  nrf24l01_write_reg(RF_CH,       channel);
  nrf24l01_write_reg(RF_SETUP,    0x26); /* 250 kbps, 0dBm */
  nrf24l01_write_reg(STATUS,      0x70); /* reset RX_DR, TX_DS, MAX_RT flags */
  nrf24l01_write_reg_multi(TX_ADDR, address, 5);
  nrf24l01_write_reg(DYNPD,       0x00);
  nrf24l01_write_reg(FEATURE,     0x00);
  nrf24l01_send_cmd(FLUSH_TX);
  nrf24l01_enable();
  return 1;
}

int nrf24l01_transmit(const uint8_t *data) {
  nrf24l01_select();
  nrf24l01_spi_transfer(W_TX_PAYLOAD);
  for(int i = 0; i < nrf24l01_payload_length; i++)
    nrf24l01_spi_transfer(data[i]);
  nrf24l01_unselect();
  //nrf24l01_delay(1); /* TODO: avoid using a delay ************************** */
  uint8_t status = nrf24l01_read_reg(STATUS);
  auto timestamp = micros();
  while(!((status = nrf24l01_read_reg(STATUS)) & (1 << 5)) && micros() - timestamp < 10000);
  Serial.printf("STATUS: 0x%x\n", status);
  if(status & (1 << 5)) {
    nrf24l01_write_reg(STATUS, status); /* clear TX_DS flag */
    return 1;
  }
  else
    return 0;
}

int nrf24l01_simple_rx_mode(uint8_t channel, const uint8_t *address) {
  if(!nrf24l01_check_comm())
    return 0;
  nrf24l01_disable();
  nrf24l01_write_reg(CONFIG,      0x3f); /* CRC enabled and interrupt pin enable for RX_DR */
  nrf24l01_write_reg(EN_AA,       0x00);
  nrf24l01_write_reg(EN_RXADDR,   0x02); /* Enable pipe 1 */
  nrf24l01_write_reg(SETUP_AW,    0x03); /* 5 bytes address width */
  nrf24l01_write_reg(SETUP_RETR,  0x00);
  nrf24l01_write_reg(RF_CH,       channel);
  nrf24l01_write_reg(RF_SETUP,    0x26); /* 250 kbps, 0dBm */
  nrf24l01_write_reg(STATUS,      0x70); /* reset RX_DR, TX_DS, MAX_RT flags */
  nrf24l01_write_reg_multi(RX_ADDR_P1, address, 5);
  nrf24l01_write_reg(RX_PW_P1,    nrf24l01_payload_length);
  nrf24l01_write_reg(DYNPD,       0x00);
  nrf24l01_write_reg(FEATURE,     0x00);
  nrf24l01_send_cmd(FLUSH_RX);
  nrf24l01_enable();
  return 1;
}

int nrf24l01_data_available() {
  const uint8_t pipe = 1;
  uint8_t status = nrf24l01_read_reg(STATUS);
  if(status&(1<<6) && (status&(pipe<<1)))
    return 1;
  else 
    return 0;
}

void nrf24l01_receive(uint8_t *data) {
  nrf24l01_select();
  nrf24l01_spi_transfer(R_RX_PAYLOAD);
  for(int i = 0; i < nrf24l01_payload_length; i++)
    data[i] = nrf24l01_spi_transfer(0);
  nrf24l01_unselect();
  nrf24l01_send_cmd(FLUSH_RX);
  nrf24l01_write_reg(STATUS, 1<<6); /* Clear RX_DR flag */
}

void nrf24l01_dump() {
  Serial.println("\n* Dump **********");
  Serial.println("CONFIG");
  uint8_t config = nrf24l01_read_reg(CONFIG);
  Serial.printf("    CONFIG = 0x%x\n", config);
  Serial.printf("    EN_CRC: %d\n", !!(config&(1<<3)));
  Serial.printf("    CRCO: %d\n", !!(config&(1<<2)));
  Serial.printf("    PWR_UP: %d\n", !!(config&(1<<1)));
  Serial.printf("    PRIM_RX: %d\n", !!(config&(1<<0)));
  Serial.printf("EN_AA: 0x%x\n", nrf24l01_read_reg(EN_AA));
  Serial.println("SETUP_AW");
  if(nrf24l01_read_reg(SETUP_AW) == 0x3)
    Serial.println("    5 bytes addresses");
  else Serial.println("    unexpected address length");
  Serial.printf("SETUP_RETR: 0x%x\n", nrf24l01_read_reg(SETUP_RETR));
  Serial.printf("RF_CH: %d\n", nrf24l01_read_reg(RF_CH));
  Serial.printf("RF_SETUP: 0x%x\n", nrf24l01_read_reg(RF_SETUP));
  uint8_t status = nrf24l01_read_reg(STATUS);
  Serial.printf("STATUS: 0x%x\n", status);
  Serial.printf("    TX_DR: %d\n", !!(status&(1<<6)));
  Serial.printf("    TX_DS: %d\n", !!(status&(1<<5)));
  Serial.printf("    TX_FULL: %d\n", !!(status&(1<<0)));
  uint8_t rx_addr_p1[5];
  nrf24l01_read_reg_multi(RX_ADDR_P1, rx_addr_p1, 5);
  Serial.println("RX_ADDR_P1");
  Serial.printf("    ");
  for(int i = 0; i < 5; i++)
    Serial.printf("0x%x ", rx_addr_p1[i]);
  Serial.println();
  uint8_t tx_addr[5];
  nrf24l01_read_reg_multi(TX_ADDR, tx_addr, 5);
  Serial.println("TX_ADDR");
  for(int i = 0; i < 5; i++)
    Serial.printf("0x%x ", tx_addr[i]);
  Serial.println();
  uint8_t fifo_status = nrf24l01_read_reg(FIFO_STATUS);
  Serial.printf("FIFO_STATUS: 0x%x\n", fifo_status);
  Serial.printf("    TX_REUSE: %d\n", !!(fifo_status&(1<<6)));
  Serial.printf("    TX_FULL: %d\n", !!(fifo_status&(1<<5)));
  Serial.printf("    TX_EMPTY: %d\n", !!(fifo_status&(1<<4)));
  Serial.printf("    RX_FULL: %d\n", !!(fifo_status&(1<<1)));
  Serial.printf("    RX_EMPTY: %d\n", !!(fifo_status&(1<<0)));
}

#endif /* NRF24L01_IMPLEMENTATION */
#endif /* INCLUDE_NRF24l01_H */