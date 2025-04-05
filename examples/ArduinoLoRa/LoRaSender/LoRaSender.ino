// Only supports SX1276/SX1278
#include <LoRa.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include "LoRaBoards.h"

#include <MicroNMEA.h>


#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           923.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   20
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             250E3
#endif


#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples
#endif

int counter = 0;
SFE_UBLOX_GPS myGPS;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));



void setup()
{
    setupBoards();
    // When the power is turned on, a delay is required.
    delay(1500);

    if (myGPS.begin(SerialGPS) == false) {
        Serial.println(F("Ublox GPS not detected . Please check wiring. Freezing."));
        //while (1);
    }


#ifdef  RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

    Serial.println("LoRa Sender");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(CONFIG_RADIO_FREQ * 1000000)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);

    LoRa.setSignalBandwidth(CONFIG_RADIO_BW * 1000);

    LoRa.setSpreadingFactor(10);

    LoRa.setPreambleLength(16);

    LoRa.setSyncWord(0xAB);

    LoRa.disableCrc();

    LoRa.disableInvertIQ();

    LoRa.setCodingRate4(7);


}

void loop()
{
    Serial.print("Sending packet: ");
    Serial.println(counter);

    LoRa.setSendMode();
    delay(1000);

    // send packet
    LoRa.beginPacket();
    LoRa.print("hello, my name is A: ");
    LoRa.print(counter);
    LoRa.endPacket();

    myGPS.checkUblox();
    if (nmea.isValid() == true) {
        long latitude_mdeg = nmea.getLatitude();
        long longitude_mdeg = nmea.getLongitude();

        Serial.print("Latitude (deg): ");
        Serial.println(latitude_mdeg / 1000000., 6);
        Serial.print("Longitude (deg): ");
        Serial.println(longitude_mdeg / 1000000., 6);

        // send packet
        LoRa.beginPacket();
        LoRa.print("Latitude (deg): ");
        LoRa.println(latitude_mdeg / 1000000., 6);
        LoRa.print("Longitude (deg): ");
        LoRa.println(longitude_mdeg / 1000000., 6);
        LoRa.endPacket();


    } else {
        Serial.print("No Fix - ");
        Serial.print("Num. satellites: ");
        Serial.println(nmea.getNumSatellites());
    }

    counter++;
    // put the radio into receive mode
    LoRa.receive();

    delay(1000);
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        String recv = "";
        // read packet
        while (LoRa.available()) {
            recv += (char)LoRa.read();
        }

        Serial.println(recv);

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
    }

}
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
    //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
    //for sentence cracking
    nmea.process(incoming);
}
