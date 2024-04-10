#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define IR_Sensor 26     // Analog pin connected to the IR sensor
#define RelayPin 2       // Pin connected to a relay for control (optional)

static const u1_t PROGMEM APPEUI[8] = {0x5D, 0xE4, 0xDC, 0x6E, 0x87, 0x98, 0x43, 0x65};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = {0x86, 0x67, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = {0x2C, 0x6D, 0x9A, 0x33, 0x16, 0x78, 0x58, 0x76, 0x41, 0xB4, 0xBF, 0x6B, 0xC9, 0x97, 0x1A, 0x70};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

uint8_t Data;
static uint8_t mydata[1]; // Payload data (IR sensor reading)

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations)
const unsigned TX_INTERVAL = 30;

void irSensorRead()
{
    int sensorValue = digitalRead(IR_Sensor);
    //int thresholdValue = 500; // Adjust this based on sensor calibration

    //bool isDetected = (sensorValue > thresholdValue);
    //if (isDetected) {
    //    Serial.println("Obstacle detected");
    //} else {
    //    Serial.println("No obstacle");
    //}
     Serial.print("sensorValue: ");
    Serial.println(sensorValue);
    mydata[0] = sensorValue; // IR sensor state (DETECTED or NOT DETECTED)
}

const lmic_pinmap lmic_pins = {
    .nss = 15,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 17,
    .dio = {4, 33, 32},
};

void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        irSensorRead(); // Read IR sensor and update payload data
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0); // Send payload data
        Serial.println(F("Packet queued"));
    }

    // Schedule next transmission
    os_setTimedCallback(j, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("Received ack"));
            }
            if (LMIC.dataLen) {
                Serial.println(F("Received"));
            }
            break;
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void setup()
{
   pinMode(IR_Sensor, INPUT);
    pinMode(RelayPin, OUTPUT);
    Serial.begin(9600);
    Serial.println(F("Starting"));

    os_init();
    LMIC_reset();
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7, 14);

    do_send(&sendjob); // Start sending data
}

void loop()
{
    os_runloop_once(); // Process LMIC in the loop
}
