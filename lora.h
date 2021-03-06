#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define COW_ID  1

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x1C, 0x1D, 0x88, 0x76, 0x80, 0xED, 0x9E, 0x9A, 0x68, 0xF4, 0x73, 0xE4, 0x7B, 0x8C, 0x8F, 0xBC };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xEB, 0x41, 0xBB, 0x30, 0xA0, 0x86, 0x5A, 0xAA, 0x71, 0x23, 0xCE, 0xFD, 0x93, 0xAB, 0x42, 0xB4 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260B5B87 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;
int loraStatus = 0;
int checkLoraStatus = 0;

// Pin mapping
// Adapted for Feather M0 per p.10 of [feather]
const lmic_pinmap lmic_pins = {
    .nss = 18,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,                       // reset pin
    .dio = {26, 33, 32}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

struct loraMsg
{
  uint16_t CowID;
  uint8_t packetNo;
  float gpsLat;
  float gpsLong;
  float ACC_X;
  float ACC_Y;
  float ACC_Z;
}__attribute__((packed, aligned(1)));

struct loraMsg loraData;


uint8_t msgArray[255];

extern float TotalX;
extern float TotalY;
extern float TotalZ;

uint8_t packet_no = 0;
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
   /* if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {*/
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        
        loraData.CowID = COW_ID;
        loraData.packetNo = packet_no;
        packet_no++;
        loraData.ACC_X = TotalX;
        loraData.ACC_Y = TotalY;
        loraData.ACC_Z = TotalZ;
        TotalX = 0;
        TotalY = 0;
        TotalZ = 0;
        
        memcpy(&msgArray[0], (uint8_t *)&loraData, sizeof(struct loraMsg));
        msgArray[sizeof(struct loraMsg)] = 255;
        LMIC_setTxData2(1, msgArray, sizeof(struct loraMsg) + 1, 0);
    //}
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            //Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            //Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            //Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            //Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            //Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            //Serial.println(F("EV_JOINED"));
            do_send(&sendjob);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            //Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            //Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            loraStatus = 1;
            break;
        case EV_LOST_TSYNC:
            //Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            //Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            checkLoraStatus = 1;
            break;
        case EV_TXCANCELED:
            //Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            //Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            //Serial.print(F("Unknown event: "));
            //Serial.println((unsigned) ev);
            break;
    }
}


void loraInit()
{
     // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // If not running an AVR with PROGMEM, just use the arrays directly
     uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
/*     LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band */
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);
}

void loraSend()
{
    osjob_t testJob;
    os_setCallback(&testJob, do_send);
    //os_setTimedCallback(&sendjob, os_getTime(), do_send);
    os_runloop();
    //os_radio(RADIO_RST);
  
}


#if 0
setup
  loraInit();
loop
  loraSend();

#endif