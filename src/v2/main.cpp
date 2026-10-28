//****************************************************************************************
// Touché-style swept-frequency touch sensor, firmware v2
// Based on the Illutron take on Disney's Touché, Dzl 2012 (see src/v1)
//
// Changes from v1:
//  - focused sweep: only steps START..START+COUNT-1, where the resonance lives
//  - each step sets the frequency, lets the detector settle, then averages OVERSAMPLE reads
//    (v1 read the previous step's frequency, so its curve was shifted by one step)
//  - faster ADC clock to pay for the oversampling
//  - no smoothing on the Arduino: the computer filters, so fast gestures are not blurred
//  - compact frame: 2 bytes per point instead of 8, no repeated x values
//****************************************************************************************

//                              10n
// PIN 9 --[10k]-+-----10mH---+--||-- OBJECT
//               |            |
//              3.3k          |
//               |            V 1N4148 diode
//              GND           |
//                            |
// Analog 0 ---+------+--------+
//            |      |
//          100pf   1MOmhm
//            |      |
//           GND    GND

/* Serial frame, 500000 baud:
     0xF0, START, COUNT, OVERSAMPLE,
     COUNT x (hi7, lo7)   ADC sum of OVERSAMPLE reads = hi7 << 7 | lo7
     checksum             (sum of the 2*COUNT data bytes) & 0x7F
   Only 0xF0 has its high bit set, so the receiver resyncs on it after any lost byte.
   Step d -> timer TOP = d -> frequency = 16 MHz / (d + 1) */

#include <Arduino.h>

const uint8_t START = 40;       // 390 kHz
const uint8_t COUNT = 120;      // up to step 159 = 100 kHz (v1 swept 0..159)
const uint8_t OVERSAMPLE = 4;   // ADC reads averaged per step
const uint16_t SETTLE_US = 100; // detector RC (100pF x 1M) = 100 us
const uint16_t FIRST_SETTLE_US = 1000; // big jump from the last step back to the first

static_assert(COUNT < 128 && START < 128 && OVERSAMPLE < 128, "header values must fit in 7 bits");
static_assert(1023UL * OVERSAMPLE < 16384, "ADC sum must fit in 14 bits");

void setFrequency(uint8_t d)
{
  TCCR1B &= ~_BV(CS10); // stop generator
  TCNT1 = 0;
  ICR1 = d;
  OCR1A = d / 2;
  TCCR1B |= _BV(CS10); // restart
}

void setup()
{
  TCCR1A = 0b10000010; // fast PWM, TOP = ICR1, output on pin 9
  TCCR1B = 0b00011001; // no prescaler
  setFrequency(START);

  pinMode(9, OUTPUT); // signal generator
  pinMode(8, OUTPUT); // sync (scope) pin

  // ADC prescaler 32: 500 kHz ADC clock, ~26 us per read (Arduino default 128: ~112 us)
  ADCSRA = (ADCSRA & ~0x07) | _BV(ADPS2) | _BV(ADPS0);

  Serial.begin(500000);
}

void loop()
{
  uint8_t checksum = 0;
  Serial.write(0xF0);
  Serial.write(START);
  Serial.write(COUNT);
  Serial.write(OVERSAMPLE);

  for (uint8_t i = 0; i < COUNT; i++)
  {
    setFrequency(START + i);
    delayMicroseconds(i == 0 ? FIRST_SETTLE_US : SETTLE_US);

    uint16_t v = 0;
    for (uint8_t k = 0; k < OVERSAMPLE; k++)
      v += analogRead(A0);

    uint8_t hi = (v >> 7) & 0x7F, lo = v & 0x7F;
    Serial.write(hi);
    Serial.write(lo);
    checksum += hi + lo;
  }

  Serial.write(checksum & 0x7F);
  PORTB ^= _BV(0); // toggle pin 8 after each sweep (good for scope)
}
