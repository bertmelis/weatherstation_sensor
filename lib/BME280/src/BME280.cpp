/* BME280

Copyright 2019 Bert Melis

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include <BME280.h>

#define setPinAsOutput(x) DDRB |= (1 << x)
#define setPinAsInput(x) DDRB &= ~(1 << x)
#define setPinLow(x) PORTB &= ~(1 << x)
#define setPinHigh(x) PORTB |= (1 << x)

BME280::BME280(int8_t cePin) :
  _csPin(cePin),
  _gotData(false),
  _digT1(0),
  _digT2(0),
  _digT3(0),
  _digP1(0),
  _digP2(0),
  _digP3(0),
  _digP4(0),
  _digP5(0),
  _digP6(0),
  _digP7(0),
  _digP8(0),
  _digP9(0),
  _digH1(0),
  _digH2(0),
  _digH3(0),
  _digH4(0),
  _digH5(0),
  _digH6(0),
  _data{0},
  _tFine(0) {}

void BME280::setup() {
  setPinAsOutput(_csPin);
  setPinLow(_csPin);

  // soft reset the sensor
  // this will also clear settings and put the sensor to sleep mode
  setPinLow(_csPin);
  SPI.transfer(0xE0 & 0x7F);
  SPI.transfer(0xB6);
  setPinHigh(_csPin);

  // wait for sensor to start (eg. copy NVM data to registers)
  uint8_t status = 1;
  do {
    delay(2);
    setPinLow(_csPin);
    SPI.transfer(0xF3 | 0x80);
    status = SPI.transfer(0x00);
    setPinHigh(_csPin);
  }
  while ((status & (1 << 0)) != 0);

  // sensor is ready, get calibration data
  setPinLow(_csPin);
  SPI.transfer(0x88 | 0x80);  // 0x88 to 0xA1
  uint8_t calibData[25] = {0};
  for (uint8_t i = 0; i < 25; ++i) {
    calibData[i] = SPI.transfer(0x00);
  }
  setPinHigh(_csPin);
  _digT1 |= (uint16_t)calibData[1] << 8 | (uint16_t)calibData[0];
  _digT2 |= (uint16_t)calibData[3] << 8 | (uint16_t)calibData[2];
  _digT3 |= (uint16_t)calibData[5] << 8 | (uint16_t)calibData[4];
  _digP1 |= (uint16_t)calibData[7] << 8 | (uint16_t)calibData[6];
  _digP2 |= (uint16_t)calibData[9] << 8 | (uint16_t)calibData[8];
  _digP3 |= (uint16_t)calibData[11] << 8 | (uint16_t)calibData[10];
  _digP4 |= (uint16_t)calibData[13] << 8 | (uint16_t)calibData[12];
  _digP5 |= (uint16_t)calibData[15] << 8 | (uint16_t)calibData[14];
  _digP6 |= (uint16_t)calibData[17] << 8 | (uint16_t)calibData[16];
  _digP7 |= (uint16_t)calibData[19] << 8 | (uint16_t)calibData[18];
  _digP8 |= (uint16_t)calibData[21] << 8 | (uint16_t)calibData[20];
  _digP9 |= (uint16_t)calibData[23] << 8 | (uint16_t)calibData[22];
  _digH1 = calibData[24];
  setPinLow(_csPin);
  SPI.transfer(0xE1 | 0x80);  // 0xE1 to 0xE7
  for (uint8_t i = 0; i < 8; ++i) {
    calibData[i] = SPI.transfer(0x00);
  }
  setPinHigh(_csPin);
  _digH2 = (uint16_t)calibData[1] << 8 | (uint16_t)calibData[0];
  _digH3 = calibData[2];
  _digH4 = ((int8_t)calibData[3] << 4) | (calibData[4] & 0xF);
  _digH5 = ((int8_t)calibData[6] << 4) | (calibData[5] >> 4);
  _digH6 = calibData[7];

  // set humidity sensing in ctrl_hum, before writing to ctrl_meas
  setPinLow(_csPin);
  SPI.transfer(0xF2 & 0x7F);
  SPI.transfer(0b00000001);  // humid oversampling x1
  setPinHigh(_csPin);

  // set temp and press sensing, write to ctrl_meas
  uint8_t data = 0b00100000;   // temp oversampling x1
  data |= 0b00000100;          // press oversampling x1
  data |= 0b00000001;          // force mode
  setPinLow(_csPin);
  SPI.transfer(0xF4 & 0x7F);
  SPI.transfer(data);
  setPinHigh(_csPin);
}

void BME280::forceMeasurement() {
  setPinLow(_csPin);
  SPI.transfer(0xF4 | 0x80);
  uint8_t data = SPI.transfer(0x00);
  setPinHigh(_csPin);

  setPinLow(_csPin);
  data = (data & 0xFC) + 0x01;
  SPI.transfer(0xF4 & 0x7F);
  SPI.transfer(data);
  setPinHigh(_csPin);
  _gotData = false;
  // now you should wait for the sensor to acquire data, takes about 10ms
}

int32_t BME280::getTemperature() {
  if (!_gotData) {
    _getData();
  }
  // calculation done in _getData()
  int32_t temperature = (_tFine * 5 + 128) >> 8;
  return temperature;
}

uint32_t BME280::getPressure() {
  if (!_gotData) {
    _getData();
  }
  int32_t pRaw = ((uint32_t)_data[0] << 12) | ((uint32_t)_data[1] << 4) | ((_data[2] >> 4) & 0x0F);
  int64_t var1 = (int64_t)_tFine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)_digP6;
  var2 = var2 + ((var1 * (int64_t)_digP5) << 17);
  var2 = var2 + (((int64_t)_digP4) << 35);
  var1 = ((var1 * var1 * (int64_t)_digP3) >> 8) + ((var1 * (int64_t)_digP2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_digP1) >> 33;
  if (var1 == 0) return 0;  // avoid division by zero
  int64_t pressure = 1048576 - pRaw;
  pressure = (((pressure << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_digP9) * (pressure >> 13) * (pressure >> 13)) >> 25;
  var2 = (((int64_t)_digP8) * pressure) >> 19;
  pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)_digP7) << 4);
  return (uint32_t)pressure;
}

uint32_t BME280::getHumidity() {
  if (!_gotData) {
    _getData();
  }
  int32_t hRaw = ((uint32_t)_data[6] << 8) | ((uint32_t)_data[7]);
  int32_t var1 = _tFine - ((int32_t)76800);
  var1 = ((((hRaw << 14) - (((int32_t)_digH4) << 20) - (((int32_t)_digH5) *
         var1)) + ((int32_t)16384)) >> 15) * (((((((var1 * 
         ((int32_t)_digH6)) >> 10) * (((var1 * ((int32_t)_digH3)) >> 11) +
         ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)_digH2) +
         8192) >> 14);
  var1 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) *
         ((int32_t)_digH1)) >> 4);
  var1 = var1 < 0 ? 0 : var1;
  var1 = var1 > 419430400 ? 419430400 : var1;
  return (uint32_t)(var1 >> 12);
}

void BME280::_getData() {
  setPinLow(_csPin);
  SPI.transfer(0xF7 | 0x80);
  for (uint8_t i = 0; i < 8; ++i) {
    _data[i] = SPI.transfer(0x00);
  }
  setPinHigh(_csPin);
  _gotData = true;

  // do calculations for temperature now to update shared variable _tFine
  int32_t tRaw = ((uint32_t)_data[3] << 12) | ((uint32_t)_data[4] << 4) | (_data[5] >> 4 & 0x0F);
  int32_t var1 = ((((tRaw >> 3) - (_digT1 << 1))) * _digT2) >> 11;
  int32_t var2 = (((((tRaw >> 4) - _digT1) * ((tRaw >> 4) - _digT1)) >> 12) * _digT3) >> 14;
  _tFine = var1 + var2;
}
