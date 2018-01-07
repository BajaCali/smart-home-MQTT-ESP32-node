/*
 *MQTT node for smart home system
 *
 *Created: 6. 1. 2018
 *Author: BajaCali
 *
 */

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string>
#include <sstream>
template <typename T>
std::string to_string(T value) { //converts number to string
    //create an output string stream
    std::ostringstream os ;

    //throw the value into the string stream
    os << value ;

    //convert the string stream into a string and return
    return os.str() ;
}

// wifi name and pass in "credentials.h"
#include "credentials.h"

// ip of MQTT broker
const char* mqtt_server = "192.168.2.119";

const String THIS_ESP_NAME = "BedESP";

/* create an instance of PubSubClient client */
WiFiClient espClient;
PubSubClient client(espClient);

//  lights GPIO pins
//               01, 02, 03, 04
#define NUM_OF_LIGHTS 4
int lights[NUM_OF_LIGHTS] = {25, 26, 27, 33};
const char light01 = lights[0];
const char light02 = lights[1];
const char light03 = lights[2];
const char light04 = lights[3];

// switches GPIO pins
//                01, 02, 03, 04
#define NUM_OF_SWITCHES 4
int pins_of_switches[NUM_OF_SWITCHES] = {12, 14, 13, 15};
const char switch01 = pins_of_switches[0];
const char switch02 = pins_of_switches[1];
const char switch03 = pins_of_switches[2];
const char switch04 = pins_of_switches[3];

// vars for correctly working switches
volatile unsigned long time_highs[NUM_OF_SWITCHES] = { 0 };
volatile unsigned long minDifTime = 100;

// table using while no connection, [number_of_switch][light(s), which wants to be changed]
int no_connection_switch_table[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

/* topics */
#define LIGHT01_TOPIC     "home/floor1/myroom/bed_esp/light01" /* 0 = off, 1 = on, 2 = change */
#define LIGHT02_TOPIC     "home/floor1/myroom/bed_esp/light02" /* 0 = off, 1 = on, 2 = change */
#define LIGHT03_TOPIC     "home/floor1/myroom/bed_esp/light03" /* 0 = off, 1 = on, 2 = change */
#define LIGHT04_TOPIC     "home/floor1/myroom/bed_esp/light04" /* 0 = off, 1 = on, 2 = change */

#define SWITCH01_TOPIC     "home/floor1/myroom/bed_esp/switch01" /* 0 = off, 1 = on, 2 = change */
#define SWITCH02_TOPIC     "home/floor1/myroom/bed_esp/switch02" /* 0 = off, 1 = on, 2 = change */
#define SWITCH03_TOPIC     "home/floor1/myroom/bed_esp/switch03" /* 0 = off, 1 = on, 2 = change */
#define SWITCH04_TOPIC     "home/floor1/myroom/bed_esp/switch04" /* 0 = off, 1 = on, 2 = change */

void WiFi_setup(); //conects to WiFi from credentials.h
void mqtt_connect(); //connects to MQTT broker 
void lights_switches_setup(); // setups via pinMode() for each light and switch, also sets interrupt for switches
void change_light(int pin, int operation); // on/off/change state of a pin
void callback(char* topic, byte* payload, unsigned int length); // is launched, when subscribed topic msg arrives, starts othr functions based on topic
void switches(int number_of_switch);


void setup() {
    Serial.begin(115200);
    WiFi_setup();
    lights_switches_setup();
    // configure the MQTT server with IPaddress and port
    client.setServer(mqtt_server, 1883);
    // this receivedCallback function will be invoked when client received subscribed topic
    client.setCallback(callback);
}

void loop() {
    /* if client was disconnected then try to reconnect again */
    if (!client.connected()) {
        mqtt_connect();
    }
    /* this function will listen for incomming 
    subscribed topic-process-invoke callback() */
    client.loop();
}

void WiFi_setup(){
	//pinMode(CONNECTED_LED, OUTPUT);
	//digitalWrite(CONNECTED_LED, 0);
	WiFi.begin(ssid, password);
	//oldhandler = esp_event_loop_set_cb(hndl, nullptr);
	Serial.print("Connecting");
	int stat = WiFi.status();
	printf("\nStatus pred whilem: %d",stat);
	if(stat == 255){
		Serial.println("Restarting!!!");
		fflush(stdout);
		//reset();
	}
	while (stat != WL_CONNECTED) {
		printf("\nStatus: %d",stat);
		if (stat != WL_DISCONNECTED && stat != WL_CONNECTED){
			WiFi.begin(ssid, password);
			Serial.println("Reconnecting");
		}
		Serial.print(".");
		delay(500);
		stat = WiFi.status();
	}
	//digitalWrite(CONNECTED_LED, 1);
	printf("\nStatus po whilu: %d",stat);
	Serial.print("\nConnected!\nIP address: ");
	Serial.print(WiFi.localIP());
}

void mqtt_connect() {
    /* Loop until reconnected */
    while (!client.connected()) {
        Serial.print("MQTT connecting ...");
        /* client ID */
        String clientId = THIS_ESP_NAME;
        /* connect now */
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish("nodes", "Hi!");
            /* subscribe topic with default QoS 0*/
            client.subscribe(LIGHT01_TOPIC);
            client.subscribe(LIGHT02_TOPIC);
            client.subscribe(LIGHT03_TOPIC);
            client.subscribe(LIGHT04_TOPIC);
        } 
        else {
            Serial.print("failed, status code =");
            Serial.print(client.state());
            Serial.println("try again in 2 seconds");
            /* Wait 2 seconds before retrying */
            delay(2000);
        }
    }
}

void lights_switches_setup() {
    /* set led as output to control led on-off */
    for( int i = 0; i < NUM_OF_LIGHTS; i++)
        pinMode(lights[i], OUTPUT);;
    // 
    for( int i = 0; i < NUM_OF_SWITCHES; i++) {
        pinMode(pins_of_switches[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(pins_of_switches[0]), [](){switches(0);}, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins_of_switches[1]), [](){switches(1);}, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins_of_switches[2]), [](){switches(2);}, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins_of_switches[3]), [](){switches(3);}, CHANGE);
}

void change_light(int pin, int operation){ // operation: 0=off, 1=on, 2=change
    switch(operation) {
        case 0:
                digitalWrite(pin, LOW);
                Serial.println("off");
                break;
        case 1:
            digitalWrite(pin, HIGH);
            Serial.println("on");
            break;
        case 2:
            if(digitalRead(pin) == LOW){
                digitalWrite(pin, HIGH);
                Serial.println("on");
            }
            else {
                digitalWrite(pin, LOW);
                Serial.println("off");
            }
            break;
        default:
            Serial.print("change_light: wrong operation, was:");
            Serial.println(operation);
            break;
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    // anoucment about intopic to serial
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    
    // create std:string this_board_topic, eg. light01
    std::string topic_string(topic);
    std::size_t index = topic_string.find("bed_esp");
    std::string this_board_topic = topic_string.substr(index + 8);
    Serial.print("this board topic: ");
    Serial.println(this_board_topic.c_str());

    // starts change_light(), when topic is light, with correct pin from topic
    std::size_t index_of_topic = this_board_topic.find("light");
    if(index_of_topic != std::string::npos) {
        std::string s_light_num = this_board_topic.substr(5);
        int i_light_num = atoi(s_light_num.c_str());
        change_light(lights[i_light_num], payload[0]);
    }

    /*  
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
        digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
        // but actually the LED is on; this is because
        // it is acive low on the ESP-01)
    } else {
        digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
    }
    */

}
  
void switches(int number_of_switch) {
	if((millis() - time_highs[number_of_switch]) > minDifTime){
        // creates topic with the right string of topic based on number_of_switch
        std::string topic = std::string(SWITCH01_TOPIC).substr(0,std::string(SWITCH01_TOPIC).length()-1-1);
        topic.append(to_string(number_of_switch+1));

		client.publish(topic.c_str(), "2");  
	}
	time_highs[number_of_switch] = millis();
}