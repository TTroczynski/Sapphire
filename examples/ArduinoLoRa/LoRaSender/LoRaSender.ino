// Only supports SX1276/SX1278
#include <LoRa.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include "LoRaBoards.h"

#include <MicroNMEA.h>
#include <cstdlib>

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           923.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   20
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             250E3
#endif
#define TOP_FIVE 5
#define TOP_TEN 10
#define DISCOVERY_TIME 5000000
#define DISCOVERY_PACKET_LENGTH 9
#define JOIN_REQ_PACKET_LENGTH 13
#define PACKET_TYPE_DISCOVERY 1
#define PACKET_TYPE_DISCONNECT 2
#define PACKET_TYPE_JOIN 2
#define ACK 1
#define NACK 0

#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples
#endif

int counter = 0;
SFE_UBLOX_GPS myGPS;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));



void setup()
{
    srand(time(0));
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

    LoRa.setPreambleLength(32);

                    //159875321
    LoRa.setSyncWord(0x98780F9);

    LoRa.disableCrc();

    LoRa.disableInvertIQ();

    LoRa.setCodingRate4(7);


}

//preamble - Need this


//assign node number - children (tell root, root re assigns numbers)
//ask to join:
    //B asks closest node
    //A receives request, check max child node count (yes or no), respond
    //B rejected
        //B tries next best node

    //B accepted - In this time the node being joined tells root, root sends a RENUMBER packet to all children - children reassign all node numbers
        //B gets ACK-JOIN - with a node number
        //B sends ACK
        //A adds node to list, B has new parent

    //nodes are allowed to send to parent + the child nodes in their list

    //sending packets a specific time - All nodes sync to GPS time, all nodes send 10ms*node# from the start of a second


class SyncGPSTime
{
    MicroNMEA* pGpsData;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t hundredths;
    unsigned long microSeconds;

    unsigned long updateTimeInHundredths()
    {
        microSeconds = (hundredths + second*100 + minute*60*100 + hour*60*60*100)/0.000001;
        return microSeconds;
    }

    public:


    SyncGPSTime(MicroNMEA* pGps)
    {
        pGpsData = pGps;
    }

    void updateTime()
    {
        hour = pGpsData->getHour();
        minute = pGpsData->getMinute();
        second = pGpsData->getSecond();
        hundredths = pGpsData->getHundredths();

        updateTimeInHundredths();
    }

    uint32_t getTimeMicro()
    {
        updateTime();
        return updateTimeInHundredths();
    }
};

class ChildrenNodes
{

};

class JoinNetwork
{

};

typedef int Node;

class NetworkConnection
{
LoRaClass* pLora;

Node candidateList[TOP_FIVE];
unsigned long nodeResponseTimes[TOP_FIVE];
int numberOfCandidates;
int numberOfChildren;
Node childList[TOP_FIVE];
SyncGPSTime* pTimeSync;
Node myNode;
Node myParent;

public:

    NetworkConnection(LoRaClass* plora, SyncGPSTime* pGpsSync): pLora(plora), pTimeSync(pGpsSync), myNode(rand() % 1000), candidateList{0}, myParent(0), childList{0}, numberOfChildren(0), nodeResponseTimes{0}, numberOfCandidates(0){}

    bool sortCandidateList()
    {
        unsigned long swapDelay;
        Node swapNodeID;

        for(int current = 0; current < TOP_FIVE; current++)
        {
            for(int i = 0; i < TOP_FIVE; i++)
            {
                if(nodeResponseTimes[current] > nodeResponseTimes[i])
                {
                    swapDelay = nodeResponseTimes[current];
                    swapNodeID = candidateList[current];
                    nodeResponseTimes[current] = nodeResponseTimes[i];
                    candidateList[current] = candidateList[i];
                    nodeResponseTimes[i] = swapDelay;
                    candidateList[i] = swapNodeID;
                }
            }
        }
        
    }
    //send a join request
    bool joinNetwork()
    {
        String recv;
        Node responder = 0;
        uint8_t response = 0;
        unsigned long time = pTimeSync->getTimeMicro();
        unsigned char joinRequestPacket[32] = { 0 };
        memcpy(joinRequestPacket, &myNode, sizeof(Node));
        memcpy(joinRequestPacket + sizeof(Node), &time, sizeof(unsigned long));
        joinRequestPacket[8] = PACKET_TYPE_JOIN;
        //sort candidate list from lowest to highest response times
        sortCandidateList();

        //iterate through list until ACK received:
        for(int i = 0; i < numberOfCandidates; i++)
        {
            //ask to join
            memcpy(joinRequestPacket + sizeof(Node) + sizeof(unsigned long), &candidateList[i], sizeof(Node));
            //send the response packet
            LoRa.beginPacket();
            LoRa.write(joinRequestPacket, JOIN_REQ_PACKET_LENGTH);
            LoRa.endPacket();

            LoRa.receive();
            
            //get response (ACK or NACK)
            while (LoRa.available()) {
                recv += (char)LoRa.read();
            }

            const char* recvPacket = recv.c_str();

            memcpy(&responder, &recv.c_str()[0], sizeof(Node));
            if(recvPacket[8] == PACKET_TYPE_JOIN && responder == candidateList[i])
            {
                response = (uint8_t)recvPacket[13];
                
                if(response == ACK)
                {
                    //set my new node number
                    memcpy(&myNode, &recvPacket[9], sizeof(Node));
                    //set parent node attribute
                    myParent = responder;
                    //send ACK
                    time = micros();
                    memset((void*)joinRequestPacket, 0, 32);
                    memcpy(joinRequestPacket, &myNode, sizeof(Node));
                    memcpy(joinRequestPacket + sizeof(Node), &time, sizeof(unsigned long));
                    joinRequestPacket[8] = PACKET_TYPE_JOIN;
                    memcpy(joinRequestPacket + sizeof(Node) + sizeof(unsigned long), &candidateList[i], sizeof(Node));
                    joinRequestPacket[12] = ACK;

                    LoRa.beginPacket();
                    LoRa.write(joinRequestPacket, JOIN_REQ_PACKET_LENGTH);
                    LoRa.endPacket();

                    LoRa.receive();

                    //if received ACK
                    //get new node number from packet - assigned by parent
                    //assign new child node numbers
                    //tell children their new node numbers
                    //calculate new transmission time - based on node number
                    
                }
                else
                {
                    //else, call getCandidateList to refresh the number of potential parents
                }
                
                
            }
        }
            

        
        


    }
    bool handleJoinRequest(Node child)
    {

    }

    bool handleNewNodeNumberAssignment()
    {

    }
    //updates the current time to prevent drift
    bool syncToGpsTime()
    {
        
    }

    bool handleDisconnectRequest(uint8_t packet[])
    {

    }

    bool handleIDRequest()
    {
        unsigned long timeOfResponse = 0;
        uint8_t responsePacket[128] = {0};

        //set the this node number so it can be identified
        memcpy(&responsePacket[0], (void*)myNode, sizeof(Node));

        timeOfResponse = micros();
        //set the time of the response
        memcpy(&responsePacket+sizeof(Node), (void*)&timeOfResponse, sizeof(unsigned long));
        //set the Discovery packet type
        responsePacket[sizeof(Node) + sizeof(unsigned long)] = PACKET_TYPE_DISCOVERY;

        //send the response packet
        LoRa.beginPacket();
        LoRa.write(responsePacket, 9);
        LoRa.endPacket();
    }

    //disconnect
    bool disconnectFromNetwork()
    {
        unsigned long timeOfResponse = 0;
        uint8_t responsePacket[128] = {0};

        //set the this node number so it can be identified
        memcpy(&responsePacket[0], (void*)myNode, sizeof(Node));

        timeOfResponse = micros();
        //set the time of the response
        memcpy(&responsePacket+sizeof(Node), (void*)&timeOfResponse, sizeof(unsigned long));
        responsePacket[8] = PACKET_TYPE_DISCONNECT;

        //send the response packet
        LoRa.beginPacket();
        LoRa.write(responsePacket, 9);
        LoRa.endPacket();
    }
    
    //list candidates to join
    bool getCandidateList()
    {
        // send Identify packet
        int responded = 0;
        unsigned long discoveryTimer = 0;
        unsigned long discoveryStart = micros();
        unsigned long timeOfRequestSend = 0;
        

        while(responded < TOP_FIVE && discoveryTimer < DISCOVERY_TIME)
        {
            String recv = "";
            unsigned long diffTime = 0;
            unsigned long timeOfResponseSend = 0;
            Node thatResponded = 0;
            Node inDiscovery = 0;

            //send out a request for ID
            LoRa.beginPacket();
            //Identity request packet - send to the node that was detected - specify a identity request
            LoRa.print("FORMAT THIS INTO AN ACTUAL PACKET");
            LoRa.endPacket();

            //put it into receive mode
            LoRa.receive();
            
            // read ID packet to know who to send to
            while (LoRa.available()) {
                recv += (char)LoRa.read();
            }

            if( recv.length() == DISCOVERY_PACKET_LENGTH && 
                recv.c_str()[8] == PACKET_TYPE_DISCOVERY )
            {
                //get the node number of a node that is talking near by
                memcpy(&thatResponded, (void*)recv.c_str()[0], sizeof(uint32_t)); //set the correct node field
                recv = "";

                //send to the node that was ID'ed
                LoRa.beginPacket();
                //Identity request packet - send to the node that was detected - specify a identity request
                LoRa.print("FORMAT THIS INTO AN ACTUAL PACKET");
                LoRa.endPacket();

                timeOfRequestSend = micros();

                //put it into receive mode
                LoRa.receive();

                //wait for response from specific node
                while (LoRa.available()) {
                    recv += (char)LoRa.read();
                    //drop the packet if its not an ID packet from the right node
                }

                memcpy(&inDiscovery, (void*)recv.c_str()[0], sizeof(Node));

                if(recv.length() == DISCOVERY_PACKET_LENGTH && 
                (inDiscovery == thatResponded) && 
                recv.c_str()[8] == PACKET_TYPE_DISCOVERY)
                {
                    memcpy(&timeOfResponseSend, (void*)recv.c_str()[4], sizeof(unsigned long));
                //subtract the nodes calculated transmission time (based on the node number) because this time is just for coordinating
                //between the different nodes

                    diffTime = timeOfRequestSend - timeOfResponseSend;
                    candidateList[responded] = thatResponded;
                    nodeResponseTimes[responded] = diffTime;
                    numberOfCandidates = responded;
                    responded++;
                }
                
                recv = "";
            }
            
            discoveryTimer = micros() - discoveryStart;
        }
            
    }
};




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
