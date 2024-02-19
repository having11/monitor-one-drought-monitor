// Uncomment for detailed BME680 debug info
// #define BME680_DEBUG

#include "Particle.h"
#include "edge.h"
#include "Adafruit_BME680.h"
#include "HC_SR04.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#if EDGE_PRODUCT_NEEDED
PRODUCT_ID(EDGE_PRODUCT_ID);
#endif // EDGE_PRODUCT_NEEDED
PRODUCT_VERSION(EDGE_PRODUCT_VERSION);

STARTUP(
    Edge::startup();
);

constexpr uint8_t TrigPin = D4;           // A4
constexpr uint8_t EchoPin = D5;           // A5
constexpr uint8_t WaterDetectPin = D6;    // A6

/*
Connect an HC-SR04 Range finder as follows:
Monitor One   HC-SR04
GND     GND
5V      VCC
D4      Trig
D5      Voltage divider output - see below

Echo --|
       >
       < 470 ohm resistor
       >
       ------ D5 on MO
       >
       < 470 ohm resistor
       >
GND ---|
*/

/** 
 * 8 > NC Red
 * 7 > GND Black
 * 6 > 3.3v Pink
 * 5 > NC Gray
 * 4 > A6 (WaterDetectPin) White
 * 3 > A5 (EchoPin) Blue
 * 2 > A4 (TrigPin) Yellow
 * 1 > 5v green
 */

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {
    { "app.gps.nmea", LOG_LEVEL_INFO },
    { "app.gps.ubx",  LOG_LEVEL_INFO },
    { "ncp.at", LOG_LEVEL_INFO },
    { "net.ppp.client", LOG_LEVEL_INFO },
});

constexpr float SeaLevelPressureHPa = 1013.25F;

typedef struct
{
    double temperatureC;
    double relativeHumidity;
    double pressureHPa;
    double gasResistanceKOhms;
    double altitudeM;
    double distanceCm;
    bool waterDetected;
    time_t lastWriteTime;
} SensorData_t;

bool readSensorData(SensorData_t *);
double getDistanceCm(void);
void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context);

Adafruit_BME680 bme;
SensorData_t data = { .temperatureC = -1.0f };

bool failure;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Edge::instance().init();

    TrackerLocation::instance().regLocGenCallback(myLocationGenerationCallback);

    pinMode(WaterDetectPin, INPUT);
    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);



    if (!bme.begin())
    {
        Log.error("Failed to start BME680");
        Particle.publish("Log", "Failed to start BME680");
        failure = true;
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    Particle.variable("fail", &failure, BOOLEAN);

    Log.info("SENSORS INITIALIZED");
}

void loop()
{
    Edge::instance().loop();
}

bool readSensorData(SensorData_t *senData)
{
    if (!bme.performReading())
    {
        return false;
    }

    senData->temperatureC = bme.temperature;
    senData->relativeHumidity = bme.humidity;
    senData->pressureHPa = bme.pressure / 100.0;
    senData->gasResistanceKOhms = bme.gas_resistance / 1000.0;
    senData->altitudeM = bme.readAltitude(SeaLevelPressureHPa);

    // Using random values in this example
    senData->distanceCm = random(20, 30);
    // senData->distanceCm = getDistanceCm();
    senData->waterDetected = digitalRead(WaterDetectPin);

    return true;
}

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
    readSensorData(&data);
    writer.name("weather").beginObject();
    writer.name("temperatureC").value(data.temperatureC);
    writer.name("humidity").value(data.relativeHumidity);
    writer.name("pressureHPa").value(data.pressureHPa);
    writer.name("gasResKOhm").value(data.gasResistanceKOhms);
    writer.name("altitudeM").value(data.altitudeM);
    writer.name("distanceCm").value(data.distanceCm);
    writer.name("waterDetected").value(data.waterDetected);
    writer.endObject();
}

double getDistanceCm()
{
    digitalWriteFast(TrigPin, LOW);
    delayMicroseconds(5);
    digitalWriteFast(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWriteFast(TrigPin, LOW);

    pinMode(EchoPin, INPUT);
    auto duration = pulseIn(EchoPin, HIGH);

    return (duration / 2.0) / 74.0;
}