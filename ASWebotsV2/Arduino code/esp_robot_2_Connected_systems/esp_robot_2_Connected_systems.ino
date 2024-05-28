
// Voor de esp met de kleine lampjes is dit de pin out

#define lamp_forward 18
#define lamp_left 19
#define lamp_right 17
#define lamp_backward 16
#define emergency_button 5


// Voor de esp met de grote lampjes is dit de pin out
// Button doet het nog niet

// #define lamp_forward 18
// #define lamp_left 5
// #define lamp_backward 17
// #define lamp_right 19
// #define emergency_button 16



#include <WiFiClientSecure.h>
#include <WiFi.h>
#include "time.h"
#include <PubSubClient.h>

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

WiFiClient espClient;
// EthernetClient espClient;
PubSubClient client(espClient);

// const char *MQTT_HOST = "192.168.178.193"; // Webiste waarmee je wilt verbinden
const char *MQTT_HOST = "145.137.36.189"; // Webiste waarmee je wilt verbinden

// const char *MQTT_HOST = "broker.emqx.io"; 
const int MQTT_PORT = 1883; // Voer hier de MQTT-broker poort in op de aangegeven plek
const char *MQTT_CLIENT_ID = "esp_robot_2"; 
// const char *MQTT_USER = "emqx"; // Vul hier de gebruikersnaam in voor de broker
// const char *MQTT_PASS = "public"; 



const char ssid[] = "Tesla IoT"; //  SSID van je WiFi-netwerk
const char pass[] = "fsL6HgjN"; 



int value_emergency_button;
bool emergency_state = false;

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(lamp_forward, OUTPUT);
  pinMode(lamp_backward, OUTPUT);
  pinMode(lamp_left, OUTPUT);
  pinMode(lamp_right, OUTPUT);
  pinMode(emergency_button, INPUT_PULLDOWN);

  setup_wifi();

  //Zet secret.h als certificaat voor Wificlientsecure
  // espClient.setCACert(local_root_ca);

  //Configuur de tijd en server om de tijd te krijgen
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //Zet de serverhost en port.
  client.setServer(MQTT_HOST, MQTT_PORT);

  //Callback voor wanneer er berichten worden gestuurd
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
 

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String robot_message;
  
  //Het binnengekomen bericht aan elkaar plakken
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    robot_message += (char)message[i];
  }
  Serial.println();

  //Temperatuur terug sturen
  if(robot_message == "web_robot_2:dire:forward"){
    digitalWrite(lamp_forward, HIGH);
    digitalWrite(lamp_backward, LOW);
    digitalWrite(lamp_left, LOW);
    digitalWrite(lamp_right, LOW);
  }
  else if(robot_message == "web_robot_2:dire:backwards"){
    digitalWrite(lamp_forward, LOW);
    digitalWrite(lamp_backward, HIGH);
    digitalWrite(lamp_left, LOW);
    digitalWrite(lamp_right, LOW);
  }
   else if(robot_message == "web_robot_2:dire:left"){
    digitalWrite(lamp_forward, LOW);
    digitalWrite(lamp_backward, LOW);
    digitalWrite(lamp_left, HIGH);
    digitalWrite(lamp_right, LOW);
  }
   else if(robot_message == "web_robot_2:dire:right"){
    digitalWrite(lamp_forward, LOW);
    digitalWrite(lamp_backward, LOW);
    digitalWrite(lamp_left, LOW);
    digitalWrite(lamp_right, HIGH);
  }
   else if(robot_message == "web_robot_2:dire:off"){
    digitalWrite(lamp_forward, LOW);
    digitalWrite(lamp_backward, LOW);
    digitalWrite(lamp_left, LOW);
    digitalWrite(lamp_right, LOW);
  }

  if(robot_message == "dsh_board_0:emgc:off"){
    Serial.println("Emergency button has been reset");
    emergency_state = false;
  }





}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // if (client.connect(MQTT_CLIENT_ID,MQTT_USER,MQTT_PASS)) {
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("publish/server");
      client.subscribe("publish/robots");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void emergency(){

  client.publish("publish/robots",  "esp_robot_2:emgc:on");
  emergency_state = true;
  Serial.println("Emergency button has been pushed");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  value_emergency_button = digitalRead(emergency_button);
  if(value_emergency_button == 1 && emergency_state == false){
    emergency();
  }

}
