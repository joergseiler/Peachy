#pragma PARTICLE_NO_PREPROCESSOR

#include "application.h"

SYSTEM_MODE(SEMI_AUTOMATIC);

SYSTEM_THREAD(ENABLED);
//SYSTEM_MODE(MANUAL);

#define VERSION_NUMBER "0.6.5"

#define FIRMWARE_VERSION 0xA1

//0.1.0 Basic structure for communicating with the ZRF module

#define debug_off
#define debug_CAN_out_on
#define debug_CAN_in_off
#define debug_MPS_counter_off

#define Heartbeat_off

#define CHECK_IN_TIME 3600000
#define ZRX_SERIAL_NUMBER 0x400702

#define ACCESSPOINT "Sewell"

CANChannel can(CAN_D1_D2, 100, 100);

bool messageInProgress = FALSE;
bool messageGood = FALSE;
uint8_t byteCount = 0;
unsigned long messageTimer = millis();

unsigned long checkInTime = CHECK_IN_TIME;
unsigned long checkInTimer = millis();
unsigned long testTimer = millis();
unsigned long messageLedTimer = millis(); // For CANBus traiffic indication LED
unsigned long messageCounterTimer = millis();
unsigned long moduleResetTimer = millis();
unsigned long keepAliveTimer = millis();
unsigned long keepAliveTimerInterval = 2000;
bool moduleReset = 0;
uint32_t zrxSerialNumber = 0xFFFFFF;
uint8_t zrxChannel = 0x11; //need to save in EEPROM, must be remembered even if power is lost
uint8_t children = 0;
//int absoluteTimer = millis();
int serverMessageCount = 0;
int messageCounter = 0;
int tcpMessageSentCounter =0;
int messageCounterOld = 0;
int messageCounterMax = 0;
uint64_t canbusMessage;
uint64_t candimId;
int state = 0;
char clientMessage[50];

uint8_t channelCommand[9] = {0x1F, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD};

bool sewellFound = FALSE;

uint8_t inbyte;
bool clientStopped = FALSE;
int characterCount = 0;
int connectionTries = 0;
int connectionRetries = 0;
unsigned long connectionTryTimer = millis();
unsigned long nextConnectionRetryTimeInterval = 5000;
char q[3] {0,0,0};

CANMessage message;

//typedef union {
    
    //CANMessage message;
    //uint8_t canMessage[15];
//}B;

//B message;


byte server[] = { 192, 168, 1, 212 };
TCPClient client;

struct zrxId {
    uint8_t objVersion; // used to determine initial setup and pending changes. Struct is filled with 0xFF when flashed
    uint8_t id0;
    uint8_t id1;
    uint8_t id2;
    uint8_t channel;
    uint8_t ciihb;  // check in interval low byte
    uint8_t ciilb; // check in interval high byte
    uint16_t checkInInterval; //(cii)
};

zrxId myId;

union A{
    uint8_t sData[30];
    struct {
    uint8_t header;
    uint8_t dtmt;
    uint8_t batteryLevel;
    uint8_t channel;
    uint8_t version;
    uint8_t transmitterId2;
    uint8_t transmitterId1;
    uint8_t transmitterId0;
    uint8_t broadcastFlag;
    uint8_t checkInTimer;
    uint8_t accelerometerReading2;
    uint8_t accelerometerReading1;
    uint8_t accelerometerReading0;
    uint8_t callCounting2;
    uint8_t callCounting1;
    uint8_t callCounting0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t repeaterRssi;
    uint8_t repeaterId2;
    uint8_t repeaterId1;
    uint8_t repeaterId0;
    uint8_t reserved3;
    uint8_t rfPackeageEnd;
    uint8_t rssi;
    uint8_t linkQuality;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
    uint8_t packageEnd;
    } data;
};

A zrx;


//************* Start of Functions ********************

int CAN_Message(uint32_t id, int len, bool ext, long long int can_Message, bool TCP) { // Calling this function without any parameters repeats the last message sent or received
    digitalWrite(D7, HIGH); // Turn on CANBus traffic LED
    messageLedTimer=millis(); // Reset CANBus traffic LED timer
    message.id = id;
    message.len = len;
    message.extended = ext;
    for (int i=len; i>0; i--) {
        message.data[len-i] = can_Message>>((i-1)*8);
    }
    can.transmit(message);
    

    if(can.errorStatus() == CAN_BUS_OFF) {
        Serial.println("Not properly connected to CAN bus");
    }
    if(can.errorStatus() == CAN_NO_ERROR || can.errorStatus() == CAN_ERROR_PASSIVE) {
        messageCounter++;
        
        #ifdef debug_CAN_out_on
        
	    Serial.printf("%8X",message.id);
	    for (int i=0; i<8; i++) {
	        if(i<message.len) {
	            Serial.printf("%3.2X",message.data[i]);
	        }
	        else {
	            Serial.printf("   ");
	        }
	    }
	    Serial.print(" ");
	    Serial.println(Time.timeStr());
	    
        #endif

    }
    
    if (client.connected() && !clientStopped && TCP) {
        tcpMessageSentCounter++;
	    client.printf("/+/%d, %X, %X, %X, %2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X/-/", tcpMessageSentCounter, message.id, message.len, message.extended, 
	    message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5], message.data[6], message.data[7]);
	    keepAliveTimer = millis();
    }
    

}


int Check_In (uint8_t checkInType) { //0 = power-up or reset, 1 = message acknowledgment
        
    //Serial.print("RedBear Check-in ");
    //Serial.println(Time.timeStr()); 
    //Serial.print(", ");
    //message.id = 0x2000000 | zrxSerialNumber;
    //message.len = 8;
    //message.extended = 1;
    
    if (!checkInType) {
        message.data[0] = 0x71;
        if (zrxChannel == 0x11) {
            //CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,0X710011A100000001);
        }
        else if (zrxChannel == 0x15) {
            //CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,0X710015A100000001);
        }
        else if (zrxChannel == 0x19) {
            //CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,0X710019A100000001);
        }
    }
    else {
        message.data[0] = 0x73;
        if (zrxChannel == 0x11) {
            //CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,0X730011A100000001);
        }
        else if (zrxChannel == 0x15) {
            //CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,0X730015A100000001);
        }
        else if (zrxChannel == 0x19) {
            //CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,0X730019A100000001);
        }

    }
    message.data[1] = 0x00;
    message.data[2] = myId.channel;
    message.data[3] = myId.objVersion;
    message.data[4] = children;
    message.data[5] = 0x00;
    message.data[6] = myId.ciihb;
    message.data[7] = myId.ciilb;
    uint64_t CANbusCheckInMessage = 0;

    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[0]) << 8;
    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[1]) << 8;
    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[2]) << 8;
    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[3]) << 8;
    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[4]) << 8;
    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[5]) << 8;
    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[6]) << 8;
    CANbusCheckInMessage = (CANbusCheckInMessage | message.data[7]);

    CAN_Message((0x2000000 | zrxSerialNumber), 8, 1, CANbusCheckInMessage, 1);
    //TCP_CAN_Message();
}   

void Wifi_Scan(WiFiAccessPoint* wap, void* data) {
    
    WiFiAccessPoint& ap = *wap;
    Serial.print("SSID: ");
    Serial.print(ap.ssid);
    Serial.print("     RSSI: ");
    Serial.println(ap.rssi);
    //AccessPoint=ap.ssid;
    if(String(ap.ssid)==ACCESSPOINT) {
        sewellFound=TRUE;
    }
    
    /*
    Serial.print("Security: ");
    Serial.println(ap.security);
    Serial.print("Channel: ");
    Serial.println(ap.channel);
    Serial.print("RSSI: ");
    Serial.println(ap.rssi);
    */
}

//************** End of Functions *********************

void setup() {
    
    Serial.begin(115200);	// Initialize serial communications with the PC
	Serial1.begin(38400);   // Initialize serial communications with the ZRF module
	//WiFi.on();

    int startUpTimer = millis();
    while(!Serial.available() && (millis() - startUpTimer <= 4500)) {
        //Particle.process();
        
    }

    //delay(4500);
    

    WiFi.on();
    delay(500);
    int result_count = WiFi.scan(Wifi_Scan);
    Serial.print(result_count);
    Serial.println(" APs scanned.");
    
    if (sewellFound) {
        
        Particle.connect();
        while (!Particle.connected) {
            delay(100);
        }
        
        Particle.syncTime();
        while (!Particle.syncTimeDone()) {
            delay(100);
        }
        
        Particle.disconnect();
    
    }
    else {
        WiFi.off();
    }
    
    Time.zone(-8);

    //delay(5000);
    
    EEPROM.get(1, myId);
    Serial.print("System Restart, Software version ");
    Serial.println(VERSION_NUMBER);
    Serial.printlnf("Firmware Version %2.2X", myId.objVersion);
    Serial.printlnf("Serial Number %2.2X%2.2X%2.2X", myId.id0, myId.id1, myId.id2);
    if(myId.objVersion == 0xFF) {
        // EEPROM was empty -> initialize myObj
        zrxId defaultObj = { 0, 0xFF, 0xFF, 0xFF, 0x11, 0x00, 0x01, 0x0001 };
        myId = defaultObj;
        EEPROM.put(1, myId);
        checkInTime = myId.checkInInterval*1000*60;
        Serial.println(checkInTime);
    }
    else {
        checkInTime = myId.checkInInterval*1000*60;
        zrxSerialNumber = 0;
	    zrxSerialNumber = (zrxSerialNumber | myId.id0) << 8;
	    zrxSerialNumber = (zrxSerialNumber | myId.id1) << 8;
	    zrxSerialNumber = (zrxSerialNumber | myId.id2);
    }
    

	if(!can.isEnabled()) {
	    can.begin(20000);
	}
	
	pinMode(D7, OUTPUT); // CANbus activity light
	digitalWrite(D7, LOW);
	pinMode(D0, OUTPUT); // Z module reset pin
	digitalWrite(D0, LOW); // Reset the module
	moduleResetTimer = millis();
	moduleReset = 1;
	messageLedTimer = millis();
	
	// Set a delay of 10 seconds for the initial check in after startup to allow the serail innterface to be started on the computer
	checkInTimer = millis() - (checkInTime - 10000);
	
}

void loop() {

    if(sewellFound && millis() - connectionTryTimer >= nextConnectionRetryTimeInterval) { // This needs to be changed to check for an available access point before trying to reconnect again
        
        if (WiFi.ready()) {

            if (!client.connected() && !Particle.connected() && !clientStopped ) {
                Serial.print("Sewell Found, trying to connect. ");
                if (client.connect(server, 3342)) {
                    Serial.println("Connected!");
                    connectionTries = 0;
                    connectionRetries = 0;
                    nextConnectionRetryTimeInterval = 5000;
                    keepAliveTimer = millis();
                    keepAliveTimerInterval = 2000;
                }
                else {
                    Serial.println("Connection Failed");
                    connectionTries++;
                    connectionTryTimer = millis();
                    nextConnectionRetryTimeInterval = 5000;
                    if (connectionTries >= 10) {
                        connectionTries = 0;
                        connectionRetries++;
                        connectionTryTimer = millis();
                        nextConnectionRetryTimeInterval = 60000;
                        Serial.println("Going to try again in one minute");
                        client.stop();
                        WiFi.off();
                        if (connectionRetries >=10) {
                            connectionTryTimer = millis();
                            nextConnectionRetryTimeInterval = 3600000;
                            connectionTries = 0;
                            connectionRetries = 0;
                            Serial.println("Going to try again in an hour");
                        }
                    }
                }
            }
        }
        else {
            WiFi.on();
            if (!WiFi.connecting()) {
                WiFi.connect();
            }
        }
    }
    
    
    // Turn CANBus traffic LED off if there have been no messages for 50ms
    
    if(millis() - messageLedTimer >= 50) {
        digitalWrite(D7, LOW);
    }
    
    // Serial communication with the module
    // Should this be blocking? Each transmission requires approx 0.8 to 1ms (0.7125ms). CANbus is 6ms between transmissions for 99% of packets
    
    while (Serial1.available()) {
        //static int i;
        int inByte = Serial1.read();
        if (inByte == 0xAB && !messageInProgress) { // If there is serial data equal to 0xAB and there is no message in progress then this must be the beginning of a new serial transfer
            messageInProgress = TRUE;
            messageGood = FALSE;
            byteCount = 0;
            zrx.sData[byteCount] = inByte; // Save first byte of transmission
            byteCount++;
            messageTimer = millis();
            //Serial.printf("%3.2X",zrx.sData[0]);
        }
        else if (inByte == 0xEF && byteCount == 29) { // if inByte equals 0xEF and it is byte 30(29) then this is the end of the message transfer
            zrx.sData[byteCount] = inByte;
            messageInProgress = FALSE;
            messageGood = TRUE;
            //Serial.printlnf("%3.2X",zrx.sData[byteCount]);
        }
        else if(byteCount > 0 && byteCount <29) { // if byteCount is between 0 and 29 then save the data in the zrfData message array
            zrx.sData[byteCount] = inByte;
            byteCount++;
            messageTimer = millis();
            //Serial.printf("%3.2X",zrx.sData[byteCount-1]);
        }
        else if(byteCount > 29 && messageInProgress) {
            messageInProgress = FALSE;
            messageGood = FALSE;
        }
        else if(!messageInProgress) {
            Serial.printlnf("Message corrupt");
            Serial.printlnf("%3.2X",inByte);
        }

    }
    
    //Check for CANBus traffic
   
	if (can.available() > 0) {
	    can.receive(message);
	    digitalWrite(D7, HIGH); // Turn on CANBus traffic LED
	    messageLedTimer = millis(); // Reset CANBus traffic LED timer
	    messageCounter++;
	    //client.printlnf("%8X",message.id);
	    //client.printlnf("%X",message);
	    
	    if (client.connected() && !clientStopped) {
	        tcpMessageSentCounter++;
    	    //client.printf("/+/%d, %X, %X, %X, %X%X%X%X%X%X%X%X/-/", tcpMessageSentCounter, message.id, message.len, message.extended, 
    	    client.printf("/+/%d, %X, %X, %X, %2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X/-/", tcpMessageSentCounter, message.id, message.len, message.extended, 
    	    message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5], message.data[6], message.data[7]);
    	    keepAliveTimer = millis();
	    }

	    #ifdef debug_CAN_in_on
	    
	    Serial.printf("%8X",message.id);
	    for (int i=0; i<8; i++) {
	        if(i<message.len) {
	            Serial.printf("%3.2X",message.data[i]);
	        }
	        else {
	            Serial.printf("   ");
	        }
	    }
	    Serial.print(" ");
	    Serial.println(Time.timeStr());
	    
	    #endif

	    // Turn on Wifi if you hear from this device
	    if(message.id == 0x1200358 || (message.id == 0x1100080 && (message.data[0] == 0x55 && message.data[6] == 0x88))) {
	        if (client.connected()) {
	            client.stop();
	            clientStopped = TRUE;
	            Serial.println("Disconnected");
	        }
	        Particle.connect();
	    }
	    
	    if(clientStopped && (message.id == 0x1100080 && (message.data[0] == 0x5F && message.data[6] == 0x88))) {
	        if (!client.connected()) {
	            clientStopped = FALSE;
	            Serial.println("Going to try reconnecting");
	        }
	        Particle.disconnect();
	    }

	    
	    if(message.id == 0x810000 && message.data[0] == 0xFF && message.data[1] == 0xFF && message.data[2] == 0xFF) {
	        myId.id0 = message.data[3];
	        myId.id1 = message.data[4];
	        myId.id2 = message.data[5];
	        //Serial.println (myId.id0);
	        //Serial.println (myId.id1);
	        //Serial.println (myId.id2);
	        zrxSerialNumber = 0;
	        zrxSerialNumber = (zrxSerialNumber | myId.id0) << 8;
	        zrxSerialNumber = (zrxSerialNumber | myId.id1) << 8;
	        zrxSerialNumber = (zrxSerialNumber | myId.id2);
	        Serial.printlnf("%8X", zrxSerialNumber);
	        myId.objVersion = FIRMWARE_VERSION;
	        EEPROM.put(1, myId);
	        Check_In(0);
	    }
	    
	    if(message.id == 0x810000 && message.data[0] == 0x40 && message.data[1] == 0x07 && message.data[2] == 0x02) { // Is this my address?

	        // Request Check in (Ping)
	        
	        if(message.len == 6) { 
	            if(message.data[3] == 0xFF && message.data[4] == 0xFF && message.data[5] == 0xFF) {
	                Check_In(0);
	            }
	            return;
	        }
	        
	        // Channel change, Reset, Check in time change
	        
	        if(message.len == 8) { // Channel change, Reset
	        
	            // Test for valid channel number,         
	        
	            if (message.data[3] == 11 || message.data[3] == 15 || message.data[3] == 19) {
	                
    	            channelCommand[1] = message.data[3];
    	            zrxChannel = message.data[3];
	            }

	            if(message.data[6] == 0xFF && message.data[7] == 0xFF ) { //If both byte 7 and byte 7 = 0xFF then reset. Otherwise these are check in time values
	                delay(5000);
	                System.reset();
	                //digitalWrite(D0, LOW);
	                //moduleResetTimer = millis();
	                //moduleReset = 1;
	            }
	            else if(message.data[6] == 0x00 && message.data[7] == 0x00) {
	                //Serial.write(channelCommand, 8);
	                Serial1.write(channelCommand, 8); // Send channel change to the module
	                Check_In(1); // Send message acknowledgement
	            }
	            else {
	                int t = 0;
	                bool dataInvalid = 0;
	                //Serial.println(message.data[6] & 0xF0);
	                
	                if (((message.data[6] & 0xF0) >= 0x00) && ((message.data[6] & 0xF0) <= 0x90)) {
	                    t = ((message.data[6] & 0xF0) >> 4) * 1000;
	                    if (((message.data[6] & 0x0F) >= 0x00) && ((message.data[6] & 0x0F) <= 0x09)) {
	                        t = t + (message.data[6] & 0x0F) * 100;
	                        
	                        if (((message.data[7] & 0xF0) >= 0x00) && ((message.data[7] & 0xF0) <= 0x90)) {
	                            t = t + ((message.data[7] & 0xF0) >> 4) * 10;
	                            if (((message.data[7] & 0x0F) >= 0x00) && ((message.data[7] & 0x0F) <= 0x09)) {
	                                t = t + (message.data[7] & 0x0F);
	                            }
	                            else {
	                                dataInvalid = 1;
	                            }
	                        }
	                        else {
	                            dataInvalid = 1;
	                        }
	                    }
	                    else {
	                        dataInvalid = 1;
	                    }
	                    if (!dataInvalid) {
                        Serial.println(t);
                        checkInTime = (t * 60) * 1000;
	                    }
	                }
	                
	                // would like to Set the timer byte 6 is hours, byte 7 is minutes in human readable hex
	                // In minutes for now
	                
	                
	                Check_In(1); // needs to return the check-in time in bytes 6 and 7
	                checkInTimer = millis();
	            }
	        }
	    }

    }
    
// Create the CANbus messages after receiving valid serial communication from the RF module

    if (!messageInProgress & messageGood) { // Process any good messages, race conditions exist between serial, processing, CANbus, needs fixing
        messageGood = FALSE;

        // Message 1
        
        canbusMessage = 0;
        
        canbusMessage = (canbusMessage | zrx.data.dtmt) << 8;
        canbusMessage = (canbusMessage | zrx.data.batteryLevel) << 8;
        if (zrx.data.channel == 0x0B) {
            canbusMessage = (canbusMessage | 0x11) << 8;
        }
        else if (zrx.data.channel == 0x0F) {
            canbusMessage = (canbusMessage | 0x15) << 8;
        }
        else {
            canbusMessage = (canbusMessage | zrx.data.channel) << 8;
        }
        canbusMessage = (canbusMessage | zrx.data.version) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId2) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId1) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId0) << 8;
        zrx.data.rssi = ~zrx.data.rssi + 1;
        canbusMessage = (canbusMessage | zrx.data.rssi);
        
        CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,canbusMessage, 1); // Send fake call from device 4000FC
        //TCP_CAN_Message();
        
        #ifdef debug_on
        
        Serial.printf("2400702");
        Serial.printf("%3.2X",zrx.data.dtmt);
        Serial.printf("%3.2X",zrx.data.batteryLevel);
        if (zrx.data.channel == 0x0B) {
            Serial.printf("%3.2X",0x11);
        }
        else if (zrx.data.channel == 0x0F) {
            Serial.printf("%3.2X",0x15);
        }
        else {
            Serial.printf("%3.2X",zrx.data.channel);
        }
        Serial.printf("%3.2X",zrx.data.version);
        Serial.printf("%3.2X",zrx.data.transmitterId2);
        Serial.printf("%3.2X",zrx.data.transmitterId1);
        Serial.printf("%3.2X",zrx.data.transmitterId0);
        Serial.printlnf("%3.2X",zrx.data.rssi);
        
        #endif
        
        // Message 2
        
        canbusMessage = 0;
        
        canbusMessage = (canbusMessage | ((zrx.data.dtmt & 0xF0) | 0x02)) << 8;
        canbusMessage = (canbusMessage | zrx.data.callCounting2) << 8;
        canbusMessage = (canbusMessage | zrx.data.callCounting1) << 8;
        canbusMessage = (canbusMessage | zrx.data.callCounting0) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId2) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId1) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId0) << 8;
        canbusMessage = (canbusMessage | zrx.data.broadcastFlag);
        
        CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,canbusMessage, 1);
        //TCP_CAN_Message();
        
        #ifdef debug_on
        
        Serial.printf("4006FB");
        Serial.printf("%3.2X",0x12);
        Serial.printf("%3.2X",zrx.data.callCounting2);
        Serial.printf("%3.2X",zrx.data.callCounting1);
        Serial.printf("%3.2X",zrx.data.callCounting0);
        Serial.printf("%3.2X",zrx.data.transmitterId2);
        Serial.printf("%3.2X",zrx.data.transmitterId1);
        Serial.printf("%3.2X",zrx.data.transmitterId0);
        Serial.printlnf("%3.2X",zrx.data.broadcastFlag);
        
        #endif
        
        // Message3
        
        canbusMessage = 0;
        
        canbusMessage = (canbusMessage | ((zrx.data.dtmt & 0xF0) | 0x03)) << 8;
        canbusMessage = (canbusMessage | zrx.data.accelerometerReading2) << 8;
        canbusMessage = (canbusMessage | zrx.data.accelerometerReading1) << 8;
        canbusMessage = (canbusMessage | zrx.data.accelerometerReading0) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId2) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId1) << 8;
        canbusMessage = (canbusMessage | zrx.data.transmitterId0) << 8;
        canbusMessage = (canbusMessage | zrx.data.checkInTimer);
        
        CAN_Message((0x2000000 | zrxSerialNumber) , 8,1,canbusMessage, 1);
        //TCP_CAN_Message();
        
        #ifdef debug_on
        
        Serial.printf("4006FB");
        Serial.printf("%3.2X",0x13);
        Serial.printf("%3.2X",zrx.data.accelerometerReading2);
        Serial.printf("%3.2X",zrx.data.accelerometerReading1);
        Serial.printf("%3.2X",zrx.data.accelerometerReading0);
        Serial.printf("%3.2X",zrx.data.transmitterId2);
        Serial.printf("%3.2X",zrx.data.transmitterId1);
        Serial.printf("%3.2X",zrx.data.transmitterId0);
        Serial.printlnf("%3.2X",zrx.data.checkInTimer);
        
        #endif
        
    }

// RedBear Check-in //

    if (millis() - checkInTimer >= checkInTime){
        Check_In(0);
        checkInTimer = millis();
        
    }
    
    // Hold the module reset pin low for 100ms, release and allow module to finish network 
    // negotiation before putting a check-in on the CANbus
    
    if (millis() - moduleResetTimer >= 100 && moduleReset) { 
        digitalWrite(D0, HIGH);
        checkInTimer = millis() - (CHECK_IN_TIME - 5000); // Wait 5 seconds before checking in
        moduleReset = 0;
        
    }
    
// if serial communication between the module and the photon breakdown reset serial communications

    if (millis() - messageTimer > 2000 & messageInProgress) { // 1 sec is an arbitray number chosen as a reasonable timeout
        messageInProgress = FALSE;
        messageGood = FALSE;
        Serial.println("Message timed out");
        // Maybe a reset is also in order here, discuss
    }
    
    //Process messages received over TCP
    
    while (client.available()) {
        digitalWrite(D7, HIGH);
        messageLedTimer = millis();
        inbyte = client.read();

        // check for a valid start to a CANbus message "/+/"
        
        if (inbyte == 0x2F || state > 0) {
            switch (state) {
                case 0:
                    state++;
                    break;
                    
                case 1:
                    if (inbyte == 0x2F) {
                        break;
                    }
                    //Serial.println("Testing second character");
                    if (inbyte == 0x2B) {
                        //Serial.println("Passed second character");
                        state++;
                        break;
                    }
                    else {
                        state = 0;
                        break;
                    }
                    
                case 2:
                    //Serial.println("Testing third character");
                    if (inbyte == 0x2F) {
                        state++;
                        //Serial.println("Message Start");
                        byteCount = 0;
                        for (int i=0; i<=49; i++) {
                            clientMessage[i] = 0;
                        }
                        break;
                    }
                    else {
                        state = 0;
                        break;
                    }
                    
                case 3:
                    if (inbyte != 0x2F && byteCount <= 50) {
                        clientMessage[byteCount] = inbyte;
                        byteCount++;
                        if (byteCount >=51) {
                            //Serial.print(inbyte);
                            state = 0;
                            break;
                        }
                    }
                    else {
                        state++;
                        //Serial.println("Start of end of message");
                        break;
                    }
                    
                case 4:
                    if (inbyte == 0x2D) {
                        state++;
                        //Serial.print("Second character of message end passed");
                        break;
                    }
                
                case 5:
                    if (inbyte == 0x2F) {
                        int i, l, e, ml, mh;
                        uint64_t m = 0;
                        state = 0;
                        //Serial.println("Message good");
                        Serial.println(clientMessage);
                        sscanf (clientMessage,"%x, %x, %x, %x, %8x%8x", &serverMessageCount, &i, &l, &e, &ml, &mh);
                        switch (l) {
                            case 8:
                                m = ((m | ml) << 32) | mh;
                                break;
                            case 6:
                                //if padding for equal length the following line must be added
                                //mh = mh >> 16;
                                m = ((m | ml) << 16) | mh;
                                break;
                            case 4:
                                m = m | ml;
                                break;
                        }
                        CAN_Message(i,l,e,m,0);
                        break;
                    }

            }
            
        }
         
        //Serial.printf("%c",inbyte);
        keepAliveTimer = millis();
        keepAliveTimerInterval = 2000;
        
    }
    

    if (millis() - keepAliveTimer >= keepAliveTimerInterval) { // Rotating line thingy
        keepAliveTimerInterval = keepAliveTimerInterval + 2000;
        //Serial.print("No TCP data, keepAliveTimerInterval ");
        //Serial.println(keepAliveTimerInterval);
        if (client.connected() && !clientStopped) {
        
            q[0] = 0x0D;
            if (characterCount == 0) {
                q[1]=0x2D;
                //client.write(0x0D2D);
                //client.write(0x2D);
                client.write(q);
                
                //client.print("-");
                characterCount++;
                //keepAliveTimer = millis();
            }
            else if (characterCount == 1) {
                q[1]=0x5C;
                //client.write(0x0D5C);
                //client.write(0x5C);
                client.write(q);
                //client.print("\\");
                characterCount++;
                //keepAliveTimer = millis();
            }
            else if (characterCount == 2) {
                q[1]=0x7C;
                //client.write(0x0D);
                //client.write(0x7C);
                client.write(q);
                //client.print("|");
                characterCount++;
                //keepAliveTimer = millis();
            }
            else if (characterCount >= 3) {
                q[1]=0x2F;
                //client.write(0x0D);
                //client.write(0x2F);
                client.write(q);
                //client.print("/");
                characterCount = 0;
                //keepAliveTimer = millis();
            }
        }
        
        if (client.connected() && millis() - keepAliveTimer >= 10000) {
            Serial.printlnf("I seem to be disconnected %X or the client has stopped, %X", client.connected(), clientStopped);
            client.stop();
            keepAliveTimerInterval = 2000;
            keepAliveTimer = millis();
        }
        
    }
    
    
    
// Messages per second with Max feature

    #ifdef debug_MPS_counter_on

    if(millis() - messageCounterTimer >= 1000) {
        int i = messageCounter - messageCounterOld;
        messageCounterTimer=millis();
        if(i > messageCounterMax) {
            messageCounterMax = (i);
        }
        //Serial.print("Msgs / Sec ,");
        Serial.print("MPS ,");
        Serial.print(i);
        Serial.print(", Max ,");
        Serial.println(messageCounterMax);
        messageCounterOld = messageCounter;
        
    }
    
    #endif
    
}