#include <stdint.h>
#include <SPI.h>
#include "sx126x.h"
#include "sx126x_regs.h"
#include "sx126x_hal.h"

#undef LED_BUILTIN
#define LED_BUILTIN 2
const int RADIO_SCLK_PIN = 18, RADIO_MISO_PIN = 19, RADIO_MOSI_PIN = 23, RADIO_NSS_PIN = 5,
          RADIO_BUSY_PIN = 4, RADIO_RST_PIN = 16, RADIO_DIO1_PIN = 22, RADIO_DIO2_PIN = 21;
SPISettings spiSettings = SPISettings(2E6L, MSBFIRST, SPI_MODE0);
const uint8_t flipByte[] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
  },
  whitening[]={
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1,
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98
  };

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
  int i;

  digitalWrite(RADIO_NSS_PIN, LOW);
  while (digitalRead(RADIO_BUSY_PIN) == HIGH)
    ;
  SPI.beginTransaction(spiSettings);
  for (i = 0; i < command_length; i++)
    SPI.transfer(command[i]);
  for (i = 0; i < data_length; i++)
    SPI.transfer(data[i]);
  SPI.endTransaction();
  digitalWrite(RADIO_NSS_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
  int i;

  digitalWrite(RADIO_NSS_PIN, LOW);
  while (digitalRead(RADIO_BUSY_PIN) == HIGH)
    ;
  SPI.beginTransaction(spiSettings);
  for (i = 0; i < command_length; i++)
    SPI.transfer(command[i]);
  for (i = 0; i < data_length; i++)
    data[i] = SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(RADIO_NSS_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
  digitalWrite(RADIO_RST_PIN, LOW);
  delayMicroseconds(120);
  digitalWrite(RADIO_RST_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
  Serial.println("WAKEUP");
  digitalWrite(RADIO_NSS_PIN, LOW);
  delay(1);
  digitalWrite(RADIO_NSS_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("----------------------------------------");

  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_NSS_PIN);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RADIO_NSS_PIN, OUTPUT);
  pinMode(RADIO_RST_PIN, OUTPUT);
  pinMode(RADIO_BUSY_PIN, INPUT);
  pinMode(RADIO_DIO1_PIN, INPUT);
  pinMode(RADIO_DIO2_PIN, INPUT);

  sx126x_mod_params_gfsk_t modParams = {
    .br_in_bps = 4800,
    .fdev_in_hz = 3600,                          //?
    .pulse_shape = SX126X_GFSK_PULSE_SHAPE_OFF,  //?
    .bw_dsb_param = SX126X_GFSK_BW_9700
  };
  sx126x_pkt_params_gfsk_t pktParams = {
    .preamble_len_in_bits = 320,
    .preamble_detector = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS,
    .sync_word_len_in_bits = 64,
    .address_filtering = SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
    .header_type = SX126X_GFSK_PKT_FIX_LEN,
    .pld_len_in_bytes = 88,  ////////////////////////
    .crc_type = SX126X_GFSK_CRC_OFF,
    .dc_free = SX126X_GFSK_DC_FREE_OFF
  };
  sx126x_status_t res = sx126x_reset(NULL);
  res = sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);
  Serial.printf("sx126x_set_standby %d\n", res);
  res = sx126x_set_dio3_as_tcxo_ctrl(NULL, SX126X_TCXO_CTRL_2_7V, 128);  //2ms
  delay(1);
  res = sx126x_set_pkt_type(NULL, SX126X_PKT_TYPE_GFSK);
  Serial.printf("sx126x_set_pkt_type %d\n", res);
  res = sx126x_set_rf_freq(NULL, 405950000UL);
  Serial.printf("sx126x_set_rf_freq %d\n", res);
  res = sx126x_set_gfsk_mod_params(NULL, &modParams);
  Serial.printf("sx126x_set_gfsk_mod_params %d\n", res);
  res = sx126x_set_gfsk_pkt_params(NULL, &pktParams);
  Serial.printf("sx126x_set_gfsk_pkt_params %d\n", res);
  res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_RX_DONE | SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_RX_DONE, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_NONE);
  Serial.printf("sx126x_set_dio_irq_params %d\n", res);
  uint8_t syncWord[] = { 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 };
  for (int i = 0; i < sizeof syncWord; i++)
    syncWord[i] = flipByte[syncWord[i]];
  res = sx126x_set_gfsk_sync_word(NULL, syncWord, sizeof syncWord);
  Serial.printf("sx126x_set_gfsk_sync_word %d\n", res);

  res = sx126x_cal_img(NULL, 0x6B, 0x6F);
  // uint8_t val = 0x96;
  // res = sx126x_write_register(NULL, SX126X_REG_RXGAIN, &val, 1);
  res = sx126x_set_rx_with_timeout_in_rtc_step(NULL, SX126X_RX_CONTINUOUS);
  Serial.printf("sx126x_set_rx %d\n", res);
  res = sx126x_clear_device_errors(NULL);
}

/*const char* chipModeDesc(uint8_t mode) {
  switch (mode) {
    case SX126X_CHIP_MODE_UNUSED: return "unused";
    case SX126X_CHIP_MODE_RFU: return "RFU";
    case SX126X_CHIP_MODE_STBY_RC: return "stby_rc";
    case SX126X_CHIP_MODE_STBY_XOSC: return "stby_xosc";
    case SX126X_CHIP_MODE_FS: return "FS";
    case SX126X_CHIP_MODE_RX: return "RX";
    case SX126X_CHIP_MODE_TX: return "TX";
    default: return "???";
  }
}

const char* statusDesc(uint8_t s) {
  switch (s) {
    case SX126X_CMD_STATUS_RESERVED: return "reserved";
    case SX126X_CMD_STATUS_RFU: return "RFU";
    case SX126X_CMD_STATUS_DATA_AVAILABLE: return "data available";
    case SX126X_CMD_STATUS_CMD_TIMEOUT: return "cmd timeout";
    case SX126X_CMD_STATUS_CMD_PROCESS_ERROR: return "cmd processor error";
    case SX126X_CMD_STATUS_CMD_EXEC_FAILURE: return "cmd exec failure";
    case SX126X_CMD_STATUS_CMD_TX_DONE: return "cmd tx done";
    default: return "???";
  }
}*/

void dump(uint8_t buf[], int size) {
  for (int i = 0; i < size; i++)
    Serial.printf("%02X%c", buf[i], i % 16 == 7 ? '-' : i % 16 == 15 ? '\n'
                                                                     : ' ');
  if (size % 16 != 0) Serial.println();
}

void loop() {
  sx126x_status_t res;
  sx126x_pkt_status_gfsk_t pktStatus;
  sx126x_rx_buffer_status_t bufStatus;

  digitalWrite(LED_BUILTIN, digitalRead(RADIO_DIO1_PIN));
  if (digitalRead(RADIO_DIO1_PIN) == HIGH) {
    res = sx126x_get_gfsk_pkt_status(NULL, &pktStatus);
    Serial.printf("PKT rssi_sync: %d, rssi_avg: %d\n", pktStatus.rssi_sync, pktStatus.rssi_avg);

    uint8_t buf[256] = {};
    res = sx126x_get_rx_buffer_status(NULL, &bufStatus);
    res = sx126x_read_buffer(NULL, bufStatus.buffer_start_pointer, buf, bufStatus.pld_len_in_bytes);
    for (int i = 0; i < bufStatus.pld_len_in_bytes; i++)
      buf[i] = whitening[i]^flipByte[buf[i]];
    dump(buf, bufStatus.pld_len_in_bytes);
    res = sx126x_clear_irq_status(NULL, SX126X_IRQ_RX_DONE);
  }
}
