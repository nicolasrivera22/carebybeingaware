#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <time.h>
//#include <Adafruit_NeoPixel.h>
// #include <DHT.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <math.h>
#include <Servo.h>

// --------------------------------------------------------------------------------------------
//        UPDATE CONFIGURATION TO MATCH YOUR ENVIRONMENT
// --------------------------------------------------------------------------------------------

// Watson IoT connection details
#define MQTT_HOST "ytiy72.messaging.internetofthings.ibmcloud.com"
#define MQTT_PORT 8883
#define MQTT_DEVICEID "d:ytiy72:ESP8266:dev01"
#define MQTT_USER "use-token-auth"
#define MQTT_TOKEN "niklasiotproject2020"
#define MQTT_TOPIC "iot-2/evt/status/fmt/json"
#define MQTT_TOPIC_DISPLAY "iot-2/cmd/display/fmt/json"
#define MQTT_TOPIC_INITIAL_PERSONS "iot-2/cmd/initial_persons/fmt/json"
#define CA_CERT_FILE "/rootCA_certificate.pem"
#define KEY_FILE "/SecuredDev01_key_nopass.pem"
#define CERT_FILE "/SecuredDev01_crt.pem"
#define MQTT_TOPIC_LED "iot-2/cmd/led_order/fmt/json"
#define MQTT_TOPIC_DELAY_GAS "iot-2/cmd/delay_gas/fmt/json"
// #define LED D0 //Define the port for LED 

//Timezone info
#define TZ_OFFSET -5  //Hours timezone offset to GMT (without daylight saving time)
#define TZ_DST    60  //Minutes timezone offset for Daylight saving


// Add WiFi connection information
char ssid[] = "George IPhone";  // your network SSID (name)
char pass[] = "zalupakonskaya";  // your network password

// Declare servo variable
Servo servoMotor;

const int EchoPin = 0;
const int TriggerPin = 4;
const int LED_Light = 16;
const int ServoPin = 5;

int actual_dist;
int previous_dist;
int actual_person = 0; // we start with two persons inside the house
int person_initial_default = actual_person;
int previous_person = actual_person;
int32_t delay_gas = 5000;
int aux_delay_gas = 0;
bool led_order = 0;
int delay_loop = 1000;

// MQTT objects
void callback(char* topic, byte* payload, unsigned int length);
BearSSL::WiFiClientSecure wifiClient;
PubSubClient mqtt(MQTT_HOST, MQTT_PORT, callback, wifiClient);

BearSSL::X509List *rootCert;
BearSSL::X509List *clientCert;
BearSSL::PrivateKey *clientKey;

// variables to hold data
StaticJsonDocument<100> jsonDoc;
JsonObject payload = jsonDoc.to<JsonObject>();
JsonObject status = payload.createNestedObject("d");
StaticJsonDocument<100> jsonReceiveDoc;
static char msg[50];

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");

  payload[length] = 0; // ensure valid content is zero terminated so can treat as c-string
  Serial.println((char *)payload);
  DeserializationError err = deserializeJson(jsonReceiveDoc, (char *)payload);
  if (err) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(err.c_str());
  } else {
    JsonObject cmdData = jsonReceiveDoc.as<JsonObject>();
    if (0 == strcmp(topic, MQTT_TOPIC_DISPLAY)) {
        //valid message received
        // r = cmdData["r"].as<bool>(); // this form allows you specify the type of the data you want from the JSON object
        Serial.println("New message for topic display");
    } else if (0 == strcmp(topic, MQTT_TOPIC_INITIAL_PERSONS)) {
      //valid message received
    //   ReportingInterval = cmdData["Interval"].as<int32_t>(); // this form allows you specify the type of the data you want from the JSON object
        Serial.print("Setting new initial value for persons to ");
        Serial.print(cmdData["persons_initial"].as<int>());
        actual_person = cmdData["persons_initial"].as<int>();
        // Serial.println(ReportingInterval);
        jsonReceiveDoc.clear();
    } else if (0 == strcmp(topic, MQTT_TOPIC_DELAY_GAS)) {
        if (cmdData["delay_gas"].as<int32_t>() < 2*delay_loop) {
            Serial.print("Error, the requested delay (");
            Serial.print(cmdData["delay_gas"].as<int32_t>());
            Serial.print(" milliseconds) must be greater than ");
            Serial.print(delay_loop*2);
            Serial.print(" milliseconds. The delay will still be ");
            Serial.print(delay_gas);
            Serial.println(" milliseconds.");
        } else {
            Serial.print("Setting new delay for gas valve to: ");
            Serial.println(cmdData["delay_gas"].as<int32_t>());
            // int cmdData["delay_gas"];
            delay_gas = cmdData["delay_gas"].as<int32_t>();
        }
        jsonReceiveDoc.clear();
    } else if (0 == strcmp(topic, MQTT_TOPIC_LED)) {
        Serial.print("turning on/off the LED: ");
        Serial.print(cmdData["led_order"].as<bool>());
        led_order = cmdData["led_order"].as<bool>();
        jsonReceiveDoc.clear();
    } else {
        Serial.println("Unknown command received");
    }
  } 
}


void setup() {
    char *ca_cert = nullptr;
    char *client_cert = nullptr;
    char *client_key = nullptr;

    Serial.begin(9600);
    pinMode(TriggerPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    pinMode(LED_Light,OUTPUT);
    servoMotor.attach(ServoPin);
    
    servoMotor.write(0); // SET THE SERVO TO 0°

    Serial.println("Care By Being Aware Application");

    // Start WiFi connection
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi Connected");


    // Get cert(s) from file system
    LittleFS.begin();
    File ca = LittleFS.open(CA_CERT_FILE, "r");
    if(!ca) {
        Serial.println("Couldn't load CA cert");
    } else {
        size_t certSize = ca.size();
        ca_cert = (char *)malloc(certSize);
        if (certSize != ca.readBytes(ca_cert, certSize)) {
            Serial.println("Loading CA cert failed");
        } else {
            Serial.println("Loaded CA cert");
            rootCert = new BearSSL::X509List(ca_cert);
            wifiClient.setTrustAnchors(rootCert);
        }
        free(ca_cert);
        ca.close();
    }

    File key = LittleFS.open(KEY_FILE, "r");
    if(!key) {
        Serial.println("Couldn't load key");
    } else {
        size_t keySize = key.size();
        client_key = (char *)malloc(keySize);
        if (keySize != key.readBytes(client_key, keySize)) {
            Serial.println("Loading key failed");
        } else {
            Serial.println("Loaded key");
            clientKey = new BearSSL::PrivateKey(client_key);
        }
        free(client_key);
        key.close();
    }

    File cert = LittleFS.open(CERT_FILE, "r");
    if(!cert) {
        Serial.println("Couldn't load cert");
    } else {
        size_t certSize = cert.size();
        client_cert = (char *)malloc(certSize);
    if (certSize != cert.readBytes(client_cert, certSize)) {
        Serial.println("Loading client cert failed");
    } else {
        Serial.println("Loaded client cert");
        clientCert = new BearSSL::X509List(client_cert);
    }
    free(client_cert);
    cert.close();
    }

    wifiClient.setClientRSACert(clientCert, clientKey);

    // Set time from NTP servers
    configTime(TZ_OFFSET * 3600, TZ_DST * 60, "1.pool.ntp.org", "0.pool.ntp.org");
    Serial.println("\nWaiting for time");
    unsigned timeout = 60000;
    unsigned start = millis();
    while (millis() - start < timeout) {
        time_t now = time(nullptr);
        if (now > (2020 - 1970) * 365 * 24 * 3600) {
            break;
        }
        delay(100);
    }
    delay(1000); // Wait for time to fully sync
    Serial.println("Time sync'd");
    time_t now = time(nullptr);
    Serial.println(ctime(&now));

    // Connect to MQTT - IBM Watson IoT Platform
    while(! mqtt.connected()){
        if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) { // Token Authentication
        //    if (mqtt.connect(MQTT_DEVICEID)) { // No Token Authentication
            Serial.println("set up-MQTT Connected");
            mqtt.subscribe(MQTT_TOPIC_DISPLAY);
            mqtt.subscribe(MQTT_TOPIC_INITIAL_PERSONS);
            mqtt.subscribe(MQTT_TOPIC_DELAY_GAS);
            mqtt.subscribe(MQTT_TOPIC_LED);
        } else {
            Serial.print("set up-last SSL Error = ");
            Serial.print(wifiClient.getLastSSLError(msg, 50));
            Serial.print(" : ");
            Serial.println(msg);
            Serial.println("set up-MQTT Failed to connect! ... retrying");
            delay(500);
        }
    }
}
 
void sense_persons() {
    mqtt.loop();
    while (!mqtt.connected()) {
        Serial.print("loop-Attempting MQTT connection...");
        // Attempt to connect
        if (mqtt.connect(MQTT_DEVICEID, MQTT_USER, MQTT_TOKEN)) { // Token Authentication
        //    if (mqtt.connect(MQTT_DEVICEID)) { // No Token Authentication
        Serial.println("loop-MQTT Connected");
        mqtt.subscribe(MQTT_TOPIC_DISPLAY);
        mqtt.subscribe(MQTT_TOPIC_INITIAL_PERSONS);
        mqtt.subscribe(MQTT_TOPIC_DELAY_GAS);
        mqtt.subscribe(MQTT_TOPIC_LED);
        mqtt.loop();
        } else {
        Serial.print("loop-last SSL Error = ");
        Serial.print(wifiClient.getLastSSLError(msg, 50));
        Serial.print(" : ");
        Serial.println(msg);
        Serial.println("loop-MQTT Failed to connect!");
        delay(5000);
        }
    }


    int actual_dist = ping(TriggerPin, EchoPin); // call the function that converts time in distance

    // SOMEONE IS CLOSING TO THE DOOR
    //     5 < previous measure - actual measure < 60
    //     I chose 5 to give a range of error
    previous_person = actual_person;
    if  (((previous_dist - actual_dist) < 60) and ((previous_dist - actual_dist) > 5)){
        actual_person = actual_person-1;   
    }

    // SOMEONE IS GETTING INTO THE HOUSE
    //     5 <  actual measure - previous measure < 60
    //     I chose 5 to give a range of error

    if  (((actual_dist - previous_dist) < 60) and ((actual_dist - previous_dist) > 5)){
        actual_person = actual_person+1;   
    }
    //--------ALARM---------
    if (actual_person < 0){
        Serial.println("ERROR. NEGATIVE NUMBERS OF PERSONS. SETTING NUMBER TO DEFAULT VALUE. Setting to: ");
        Serial.print(person_initial_default);
        Serial.print(" persons.");
        actual_person = person_initial_default;
        previous_person = actual_person;
    }
   
    // print distance value in serial port
    Serial.print("Distance(cm): ");
    Serial.println(actual_dist);
    Serial.print("persons in the house: ");
    Serial.println(actual_person);
    delay(delay_loop);
    previous_dist = actual_dist;

    status["people"] = actual_person;

    serializeJson(jsonDoc, msg, 50);
    if (!mqtt.publish(MQTT_TOPIC, msg)) {
      Serial.println("MQTT Publish failed");
    }

    //if (actual_person > 0) {
    //  digitalWrite(LED_light, HIGH);
    //} else {
    //    digitalWrite(LED_light, LOW);
    //}
    
    digitalWrite(LED_Light, led_order);
}

void loop() {
    sense_persons();
    if (actual_person == 0 && actual_person < previous_person) {
        // NOBODY IS IN THE HOUSE. CHECK THAT NOBODY IS COMING. CLOSE THE GAS VALVE AFTER THE DELAY
        aux_delay_gas = 0;
        while (aux_delay_gas <= delay_gas) { // THE IDEA IS THAT 
            sense_persons();
            aux_delay_gas = aux_delay_gas + delay_loop;
        }
        if (actual_person == 0) {
            // CLOSE THE VALVE
            servoMotor.write(0);
        }
    } else if (actual_person > 0 and previous_person == 0) {
        // SOMEBODY CAME IN. OPEN THE GAS VALVE
        // OPEN THE VALVE
        servoMotor.write(180);
    }
}


// function to obtain the distance in cm captured by the sensor 
//      input: trigger and echo pin
//      output: distance in cm

int ping(int TriggerPin, int EchoPin) {
   long duration, distanceCm;
   
   // operations: https://www.luisllamas.es/medir-distancia-con-arduino-y-sensor-de-ultrasonidos-hc-sr04/
   
   digitalWrite(TriggerPin, LOW);  //to generate a clean pulse, first we have to turn it LOW 4us
   delayMicroseconds(4);
   digitalWrite(TriggerPin, HIGH);  //generates trigger of 10 us
   delayMicroseconds(10);
   digitalWrite(TriggerPin, LOW);
   
   duration = pulseIn(EchoPin, HIGH);  //measures the time between pulses in us
   
   distanceCm = duration * 10 / 292/ 2;   //converts to distance in cm
   return distanceCm;
}
