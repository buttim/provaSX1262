#provaSX1262
An experiment in receiving packets from Vaisala RS41 meteo sondes with a [HT-RA62 SX1262 module](https://heltec.org/project/ht-ra62/) attached to an ESP32 Devkit. Interfacing with the radio chip is done though the [SX1262 driver](https://github.com/Lora-net/sx126x_driver). I omitted files relative to FHSS since they are not needed.

Since the sonde's packets are longer than 255 byte we need to use a hack, as recommended by SemTech application note [AN1200.53: Transmitting and receiving packets longer than 255 bytes on the SX1261/62](https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/3n000000qSr5/_k7DWETSW27pcUXl2q6mgqTRbGdaR02ue716zGGCLMQ).
A few modifications to the library's source were needed since it has not been updated to the latest version of the SX1262 driver.

The code is written for receiving an RS41-SG, so a fixed payload of 312 bytes is assumed, but could be easily modified to decide the payload length on the fly (e.g. for receiving an RS41-SG+XDATA, i.e. a sonde with ozone sensor attached)