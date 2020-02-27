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

#pragma once

#include <stdint.h>  // intX_t

#include <SPI.h>

class BME280 {
 public:
  explicit BME280(int8_t csPin);

  // Setup lib and put sensor in "force measurement" mode.
  // The sensor will take one measurement and go to sleep.
  void setup();

  // Force a measurement after which the sensor goes back to sleep.
  // New values are available after ~10ms.
  void forceMeasurement();

  // Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
  int32_t getTemperature();

  // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
  // Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  uint32_t getPressure();

  // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
  // Output value of "47445" represents 47445/1024 = 46. 333 %RH
  uint32_t getHumidity();

 private:
  const int8_t _csPin;
  bool _gotData;
  uint16_t _digT1;
  int16_t _digT2;
  int16_t _digT3;
  uint16_t _digP1;
  int16_t _digP2;
  int16_t _digP3;
  int16_t _digP4;
  int16_t _digP5;
  int16_t _digP6;
  int16_t _digP7;
  int16_t _digP8;
  int16_t _digP9;
  uint8_t _digH1;
  int16_t _digH2;
  uint8_t _digH3;
  int16_t _digH4;
  int16_t _digH5;
  int8_t _digH6;
  uint8_t _data[8];
  int32_t _tFine;

  // this method includes calculation for _tFine
  void _getData();
};
