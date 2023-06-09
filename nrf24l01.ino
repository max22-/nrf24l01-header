#include <SPI.h>
#define NRF24L01_IMPLEMENTATION
#include "nrf24l01.h"

static const uint8_t CE = 15, CSN = 17;
//static const uint8_t CE = D6, CSN = D7;
static SPISettings spi_settings = SPISettings(10000000, MSBFIRST, SPI_MODE0);
const uint8_t address[] = {0X20, 0XC3, 0XC2, 0XC1, 0XA0};

void setup() {
  Serial.begin(115200);
  while(!Serial);
  SPI.begin();
  Serial.println("nrf24l01 demo");
  pinMode(CE, OUTPUT);
  pinMode(CSN, OUTPUT);
  nrf24l01_unselect();

  if(!nrf24l01_simple_tx_mode(32, address)) {
    Serial.println("Failed to communicate with radio module");
    while(1);
  }

}

void loop() {
  if(Serial.available()) {
    int c = Serial.read();
    switch(c) {
      case 'd':
        nrf24l01_dump();
        break;
      case 't':
        if(nrf24l01_simple_tx_mode(32, address))
          Serial.println("transmit mode");
        else
          Serial.println("failed to communicate with radio module");
          break;
      case 'r':
        if(nrf24l01_simple_rx_mode(32, address))
          Serial.println("receive mode");
        else
          Serial.println("failed to communicate with radio module");
          break;
      case 's':
        transmit();
        break;
      case '\n':
      case '\r':
        break;
      default:
        Serial.println("invalid command");
    }
  }
  if(nrf24l01_data_available()) {
    uint8_t buffer[nrf24l01_payload_length];
    nrf24l01_receive(buffer);
    for(int i = 0; i < nrf24l01_payload_length; i++)
      Serial.printf("%d ", buffer[i]);
    Serial.println();
  }
  delay(10);
}

void transmit() {
  uint8_t buffer[nrf24l01_payload_length];
  for(int i = 0; i < nrf24l01_payload_length; i++)
    buffer[i] = i;
  Serial.print("transmitting buffer: ");
  if(nrf24l01_transmit(buffer))
    Serial.println("ok");
  else
    Serial.println("failed");
}

void nrf24l01_select() {
  SPI.beginTransaction(spi_settings);
  digitalWrite(CSN, LOW);
}
void nrf24l01_unselect() {
  digitalWrite(CSN, HIGH);
  SPI.endTransaction();
}
void nrf24l01_enable() {
  digitalWrite(CE, HIGH);
}
void nrf24l01_disable() {
  digitalWrite(CE, LOW);
}

uint8_t nrf24l01_spi_transfer(uint8_t data) {
  return SPI.transfer(data);
}

void nrf24l01_delay(unsigned int ms) {
  delay(ms);
}
/* User provided constant */
const size_t nrf24l01_payload_length = 32;