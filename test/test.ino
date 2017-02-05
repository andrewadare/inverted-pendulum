#include "Pendulum.h"
#include "I2CCom.h"

#define MAGNET_OFFSET (4095 - 3596)

elapsedMillis printTimer = 0;
unsigned int printerval = 10; // ms
I2CCom i2c(0x36);

// Initialize with update interval and swing-up limit.
// Theta is positive clockwise in degrees, with theta = 0 when hanging at rest.
Pendulum pendulum(2 /* ms */, 100.0 /* deg */);


float readTheta()
{
  int reading = (i2c.readTwoBytes(0x0d, 0x0c) + MAGNET_OFFSET) % 4096;

  // Convert to degrees (359/4095)
  return reading*0.08766788766788766;
}

void setup()
{
  Serial.begin(115200);

  // Don't continue until a terminal is connected. Useful for development.
  while (!Serial) {;}
}

void loop()
{
  // Update pendulum state
  pendulum.update(0, readTheta(), millis());

  if (printTimer >= printerval)
  {
    printTimer -= printerval;
    // Serial.print(i2c.readTwoBytes(0x0d, 0x0c)); // Direct sensor reading
    // Serial.print(",");
    Serial.print(pendulum.theta);
    Serial.print(",");
    Serial.println(180 + 180*pendulum.omega); // Scale + offset for visibility with pendulum inverted
  }
}