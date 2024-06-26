#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define DOOR_SENSOR_PIN  26 // ESP32 pin GPIO23 connected to the OUTPUT pin of door sensor
#define LED_PIN          5  // ESP32 pin GPIO17 connected to LED's pin
#define Relay 2
int doorState;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPEUI[8]={0x56, 0x76, 0x89, 0x67, 0x65, 0x87, 0x89, 0x67};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM DEVEUI[8]= {0x8F, 0x72, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// LoRaWAN end-device address (DevAddr)
static const u1_t PROGMEM APPKEY[16] = {0x70, 0x7F, 0xF1, 0xE8, 0x32, 0x63, 0xB8, 0x01, 0x93, 0x4F, 0x24, 0x21, 0xD3, 0x88, 0xDA, 0x8D};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

uint8_t Data;
static uint8_t mydata[1] = {0x00};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations

const unsigned TX_INTERVAL = 15;

void dht11()
{
  doorState = digitalRead(DOOR_SENSOR_PIN); // read state

  if (doorState == HIGH) {
    Serial.println("The door is open, turns on LED");
    digitalWrite(LED_PIN, HIGH); // turn on LED
  } else {
    Serial.println("The door is closed, turns off LED");
    digitalWrite(LED_PIN, LOW);  // turn off LED
  }
  mydata[0] =  doorState;
  
  delay(2000); //Delay of 1 second for ease of viewing
}

// Pin mapping
const lmic_pinmap lmic_pins = {
   .nss = 15,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 17,
    .dio = {4, 33},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
             Serial.println(F("EV_JOINED"));
             {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
           
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAIRelay"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAIRelay"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              for (int i = 0; i < LMIC.dataLen; i++) 
              {
               if (LMIC.frame[LMIC.dataBeg + i] < 0x10) 
               {
                 Data = (LMIC.frame[LMIC.dataBeg + i]);
               }
              }
              Serial.print("Downlink Data : ");
              Serial.println(Data);
              if (Data == 1)
              {
                digitalWrite(Relay, HIGH);
                Serial.println("Relay on");
              }
              else
              {
                digitalWrite(Relay, LOW);
                Serial.println("Relay off");
               }
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else 
    {
        dht11();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
     
    pinMode(Relay, OUTPUT);
    Serial.begin(9600);
   pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(LED_PIN, OUTPUT);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

        // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
