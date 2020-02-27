#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Arduino.h>
#include <SPI.h>

#include <BME280.h>
#include <RF24.h>

#define setPinAsOutput(x) DDRB |= (1 << x)
#define setPinAsInput(x) DDRB &= ~(1 << x)
#define setPinLow(x) PORTB &= ~(1 << x)
#define setPinHigh(x) PORTB |= (1 << x)

constexpr uint8_t sleepCycles = 38;  // 38 *8s = 304s ~ every 5min
volatile uint8_t sleepCount = 38;  // times the watchdog woke the chip
                                   // initialize with 38 for measurement
                                   // on startup

BME280 bme280(PB4);
RF24 radio(-1, PB3);  // -1: CE pin is tied to Vcc
const uint8_t address[] = "weather";

// watchdog interrupt
ISR(WDT_vect) {
  (void) WDT_vect;
  ++sleepCount;
}

void prepareSleep() {
  radio.powerDown();
  SPI.end();
  setPinAsInput(PB4);  // CSN BME280, has pullup on BME280 board
  setPinAsInput(PB3);  // CSN nrf24
  setPinAsInput(PB2);  // SPI SCL
  setPinAsInput(PB1);  // SPI DO/MOSI
  setPinAsInput(PB0);  // SPI DI/MISO
}

void goToSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Enter sleep mode.
  // After waking from watchdog interrupt the code continues from this point.
  sleep_disable();  // Disable sleep mode after waking.
}

void restoreFromSleep() {
  sleepCount = 0;
  setPinAsOutput(PB0);
  setPinAsOutput(PB1);
  setPinAsOutput(PB2);
  setPinAsOutput(PB3);
  setPinAsOutput(PB4);
  SPI.begin();
  radio.powerUp();  // do early to give time
}

uint16_t getVcc() {
  ADCSRA |= 1<<ADEN;  // enable ADC
  ADMUX = (0<<REFS0) | (12<<MUX0);
  delay(2);
  ADCSRA |= (1<<ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  uint16_t vRaw = ADCW;
  uint16_t vBattery = 1081089L / vRaw;  // ref multimeter measurement
  ADCSRA &= ~(1<<ADEN);  // disable ADC
  return vBattery;  // returns Vcc in mV
}

void setup() {
  // setup watchdog
  MCUSR = 0;  // clear various "reset" flags
  WDTCR = (1 << WDCE) | (1 << WDE);  // allow changes, disable reset
  WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // set interrupt mode, interval and 8 seconds delay
  wdt_reset();  // pat the dog

  SPI.begin();

  bme280.setup();

  setPinAsOutput(PB3);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(address);
}

void loop() {
  if (sleepCount >= sleepCycles) {
    restoreFromSleep();

    bme280.forceMeasurement();
    delay(10);
    int32_t temp = bme280.getTemperature();
    uint32_t humid = bme280.getHumidity();
    uint32_t press = bme280.getPressure();
    uint16_t vBattery = getVcc();

    uint8_t payload[14] = {0};
    memcpy(&payload[0], &temp, sizeof(temp));
    memcpy(&payload[4], &humid, sizeof(humid));
    memcpy(&payload[8], &press, sizeof(press));
    memcpy(&payload[12], &vBattery, sizeof(vBattery));
    if (!radio.write(payload, 14)) {
      // failed
    }

    prepareSleep();
  }
  goToSleep();
}
