/**
 * PX4FLOW drop-in replacement using Bitcraze Flow Deck. Talks I2C to the PX4
 * flight stack, sending optical flow messages.
 *
 * The flow deck's optical flow sensor is called PMW3901, and its ToF range
 * sensor is VL53L0X. Talking to the PMW3901 is done over SPI, and the VL53L0X
 * over I2C.
 *
 * We have disabled the range sensor since its I2C communication seems to
 * interrupt the communication with the PX4 flight stack.
 *
 * Finally, note that since the flow deck has no IMU, the gyro data is zeroed
 * out. This means that gyro compensation will _not_ work unless reprogrammed
 * to use the PX4's main IMU (which I recommend anyhow.)
 *
 * @author Ludvig Ericson <ludv@kth.se>
 */

#include <Wire.h>
#include <inttypes.h>

// PX4FLOW's default I2C address
#define I2C_ADDRESS 0x42

// Fast I2C
#define I2C_FREQUENCY 400000

// Status LED. Undefined means no status LED.
// NOTE: DO NOT USE LED_BUILTIN aka D13! It's for SCK in SPI.
//#define STATUS_LED LED_BUILTIN

// Print messages to serial port
#define WITH_SERIAL

// Range sensor breaks the I2C communication.
//#define WITH_RANGE

// Measure optical flow (uh-doy)
#define WITH_OPTFLOW

// Clockwise board rotation (must be multiple of 90!)
// NB Only tested 90 degrees.
#define BOARD_ROTATION 90

// Below frame definition taken from the PX4 source code.
// We use this data structure as an accumulator.
#define ZERO_FRAME { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255 };
struct {
  uint16_t frame_count_since_last_readout; /* number of flow measurements since last I2C readout [#frames] */
  int16_t  pixel_flow_x_integral;          /* accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000] */
  int16_t  pixel_flow_y_integral;          /* accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000] */
  int16_t  gyro_x_rate_integral;           /* accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] */
  int16_t  gyro_y_rate_integral;           /* accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] */
  int16_t  gyro_z_rate_integral;           /* accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] */
  uint32_t integration_timespan;           /* accumulation timespan in microseconds since last I2C readout [microseconds] */
  uint32_t sonar_timestamp;                /* time since last sonar update [microseconds] */
  uint16_t ground_distance;                /* Ground distance in meters*1000 [meters*1000] */
  int16_t  gyro_temperature;               /* Temperature * 100 in centi-degrees Celsius [degcelsius*100] */
  uint8_t  qual;                           /* averaged quality of accumulated flow values [0:bad quality;255: max quality] */
} frame = ZERO_FRAME;

#ifdef WITH_OPTFLOW
#  include "Bitcraze_PMW3901.h"
Bitcraze_PMW3901 flow(10);
#endif

#ifdef WITH_RANGE
#  define LONG_RANGE
#  include <VL53L0X.h>
VL53L0X rangeSensor;
#endif

// Time of last accumulator flush (i.e. when integration began.)
unsigned long t0 = 0;

// Time of current accumulator integration point
unsigned long t1 = 0;

// Blink status LED ominously.
void blinkLoop() {
  while(1) {
#ifdef STATUS_LED
    digitalWrite(STATUS_LED, HIGH); delay(125);
    digitalWrite(STATUS_LED, LOW); delay(125);
    digitalWrite(STATUS_LED, HIGH); delay(125);
    digitalWrite(STATUS_LED, LOW);
    delay(1000);
#endif
  }
}

// Print an error message and blink status LED ominously.
void blinkLoop(const char *message) {
#ifdef WITH_SERIAL
  for (t0 = t1 = micros(); t1 - t0 < 2000000 && !Serial; t1 = micros());
  Serial.println(message);
#endif
  blinkLoop();
}

void setup() {
#ifdef WITH_SERIAL
  Serial.begin(9600);
#endif

#ifdef STATUS_LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
#endif

#ifdef WITH_OPTFLOW
  if (!flow.begin()) {
    blinkLoop("Initialization of the flow sensor failed");
  }
  t1 = micros();
#endif

  Wire.begin(I2C_ADDRESS);
  Wire.setClock(I2C_FREQUENCY);
  Wire.onReceive(recv);
  Wire.onRequest(req);

#ifdef WITH_RANGE
  rangeSensor.init();
  rangeSensor.setTimeout(50);
#endif
}

unsigned long ledHighUntil = 0;

void loop() {
  t1 = micros();

#ifdef STATUS_LED
  digitalWrite(STATUS_LED, t1 < ledHighUntil);
#endif

#ifdef WITH_OPTFLOW
  // Aperture is 30 pixels, or 4.2 degrees. Expressed as a ratio times 10 000.
  int16_t radPixelApertureRatio = 24.434f;
  int16_t deltaX = 0, deltaY = 0;
  flow.readMotionCount(&deltaX, &deltaY);
#  if BOARD_ROTATION == 0
  frame.pixel_flow_x_integral += deltaX*radPixelApertureRatio;
  frame.pixel_flow_y_integral += deltaY*radPixelApertureRatio;
#  elif BOARD_ROTATION == 90
  frame.pixel_flow_x_integral -= deltaY*radPixelApertureRatio;
  frame.pixel_flow_y_integral += deltaX*radPixelApertureRatio;
#  elif BOARD_ROTATION == 180
  frame.pixel_flow_x_integral -= deltaX*radPixelApertureRatio;
  frame.pixel_flow_y_integral -= deltaY*radPixelApertureRatio;
#  elif BOARD_ROTATION == 270
  frame.pixel_flow_x_integral += deltaY*radPixelApertureRatio;
  frame.pixel_flow_y_integral -= deltaX*radPixelApertureRatio;
#  else
#    error "BOARD_ROTATION is not a multiple of 90 degrees"
#  endif
#endif

#ifdef WITH_RANGE
  frame.ground_distance = rangeSensor.readRangeSingleMillimeters();
#endif

  frame.frame_count_since_last_readout++;

  if (t1 - t0 > 250000) {
    ledHighUntil = micros() + 25000;
#ifdef WITH_SERIAL
    Serial.print("Flushing accumulator. ");
    Serial.print('\t'); Serial.print(frame.pixel_flow_x_integral);
    Serial.print('\t'); Serial.print(frame.pixel_flow_y_integral);
    Serial.print('\t'); Serial.print(frame.ground_distance);
    Serial.println();
#endif
    frame = ZERO_FRAME;
    t0 = t1;
  }
}

byte reg = 0;

void recv(int n) {
  while (Wire.available() > 0)
    reg = Wire.read();
}

void req() {
  if (reg == 0x16) {
    ledHighUntil = micros() + 25000;
    frame.integration_timespan = t1 - t0;
    byte n = Wire.write((byte *)&frame, sizeof(frame));
    frame = ZERO_FRAME;
    t0 = t1;
  }
  reg = 0;
}
