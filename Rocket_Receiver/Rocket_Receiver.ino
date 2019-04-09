#include <RH_E32.h>
#include <HardwareSerial.h>

#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define BUFFSIZE 58          // buffer one command at a time, 12 bytes is longer than the max length

RH_E32  driver(&Serial, 14, 12, 36);


/**************************************************************************/
unsigned int counter = 0;

boolean wifiConnected = true;
boolean MQTTconnected = false;

#define OS_PACK      __attribute__ ((packed)) //ensure the compiler leaves no spacing in bitstream to COSMOS

typedef struct {
  uint32_t length;
  uint32_t ID;
  uint32_t packet;
  float latitude;
  float longitude;
  float roll;
  float pitch;
  float yaw;
  float accelx;
  float accely;
  float accelz;
  float magx;
  float magy;
  float magz;
  float gyrox;
  float gyroy;
  float gyroz;
  float temperature;
  float pressure;
  float altitude;
  uint32_t endID;
} OS_PACK telemetry_packet;


telemetry_packet data;
/**************************************************************************/


//WiFi network variables
/*
  const char* ssid = "MOTO5074";
  const char* password = "zdchrwzzrj";
  const char* mqtt_server = "192.168.0.11";
*/

const char* ssid = "iPhone";
const char* password = "981kjqsj99680";
const char* mqtt_server = "172.20.10.2";
const char* id = "receiver_pub";



WiFiClient esp32Client;
PubSubClient client(esp32Client);
long lastMsg = 0;
int value = 0;

/**************************************************************************/
void setup_wifi() {

  delay(1000);
  // We start by connecting to a WiFi network
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long wifi_time = millis();
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(16, HIGH);

    if (millis() - wifi_time > 10000) {//try connecting to WiFi for a 10 secs
      //Serial.println();
      //Serial.println("Could not connect to Wifi");
      digitalWrite(16, LOW);
      wifiConnected = false;
      break;
    }

    //Serial.print(".");
  }

  randomSeed(micros());

  if (wifiConnected) {
    //Serial.println("");
    //Serial.println("WiFi connected");
    //Serial.println("IP address: ");
    //Serial.println(WiFi.localIP());
  }

}

/**************************************************************************/

void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
  }
  //Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(16, HIGH);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
  } else {
    digitalWrite(16, LOW);  // Turn the LED off by making the voltage HIGH
  }

}

/**************************************************************************/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    digitalWrite(16, HIGH);
    //Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      //Serial.println("connected");
      // Once connected, publish an announcement..
      //publish Rocket telemetry.
      client.publish("Rocket", "hello world");
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
    }
  }
}
/**************************************************************************/
//HardwareSerial Serial1(1);
void setup() {
  Serial.begin(9600, SERIAL_8N1);
  while (!Serial);

  Serial1.begin(9600, SERIAL_8N1, 39, 23);
  while (!Serial1);

  //e32
  if (!driver.init(0)) {
    Serial.println("init failed");
  }

  //WiFi setup
  setup_wifi();

  if (wifiConnected) {

    client.setServer(mqtt_server, 1883);

    while (!client.connected()) {
      //Serial.println("Connecting to MQTT...");

      if (client.connect(id)) {

        //Serial.println("connected");
        client.publish("esp32", "ESP32 receiver is connected");
      } else {

        // Serial.print("failed with state ");
        //Serial.print(client.state());
      }
    }
  }
}

/**************************************************************************/
void loop() {
  readSerial();
}
/**************************************************************************/
void readSerial() {

  int count = 1;
  float c;    // holds one parsed float from the serial port
  static char outstr[20];
  String buf = ",";



  driver.waitAuxHigh();
  while (Serial1.available()) {
    driver.waitAuxLow();
    c = Serial1.parseFloat();         // read one character

    //only begin reading at start ID
    while ((c != 0xDEAD)  && count == 1) {
      driver.waitAuxLow();
      c = Serial1.parseFloat();
    }

    switch (count) {
      case 1:
        data.ID = (uint32_t) c;
        
        dtostrf(c, 7, 0, outstr);
        memset(outstr,0,sizeof(outstr));
        
        count++;
        break;
      case 2:
        data.packet = (uint32_t) c;

        if (wifiConnected) {
          dtostrf(((double)data.packet), 7, 0, outstr);
          publishMQTT(outstr, count);
        }
          buf+=",";
          buf += outstr;
          buf+=",";
          memset(outstr,0,sizeof(outstr));
        count++;
        break;
      case 3:
        data.latitude = c;

        if (wifiConnected) {
          dtostrf(((double)data.latitude), 7, 4, outstr);

          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
          memset(outstr,0,sizeof(outstr));
        count++;
        break;
      case 4:
        data.longitude = c;

        if (wifiConnected) {
          dtostrf(((double)data.longitude), 7, 4, outstr);

          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
          memset(outstr,0,sizeof(outstr));
        count++;
        break;
      case 5:
        data.roll = c;

        if (wifiConnected) {
          dtostrf(((double)data.roll), 6, 2, outstr);
          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
          memset(outstr,0,sizeof(outstr));
        count++;
        break;
      case 6:
        data.pitch = c;

        if (wifiConnected) {
          dtostrf(((double)data.pitch), 6, 2, outstr);

          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
          memset(outstr,0,sizeof(outstr));
        count++;
        break;
      case 7:
        data.yaw = c;

        if (wifiConnected) {
          dtostrf(((double)data.yaw), 6, 2, outstr);

          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
          memset(outstr,0,sizeof(outstr));
        count++;
        break;
      case 8:
        data.accelx = c;

        if (wifiConnected) {
          dtostrf(((double)data.accelx), 6, 2, outstr);

          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
        count++;
        break;
      case 9:
        data.accely = c;

        if (wifiConnected) {
          dtostrf(((double)data.accely), 6, 2, outstr);
          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
        count++;
        break;
      case 10:
        data.accelz = c;

        if (wifiConnected) {
          dtostrf(((double)data.accelz), 6, 2, outstr);
          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
        count++;
        break;
      case 11:
        data.magx = c;

        if (wifiConnected) {
          dtostrf(((double)data.magx), 6, 2, outstr);

          publishMQTT(outstr, count);
        }

          buf += outstr;
          buf+=",";
        count++;
        break;
      case 12:
        data.magy = c;

        if (wifiConnected) {
          dtostrf(((double)data.magy), 6, 2, outstr);
          publishMQTT(outstr, count);
        }

         buf += outstr;
          buf+=",";
        count++;
        break;
      case 13:
        data.magz = c;

        if (wifiConnected) {
          dtostrf(((double)data.magz), 6, 2, outstr);
          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
        count++;
        break;
      case 14:
        data.gyrox = c;

        if (wifiConnected) {
          dtostrf(((double)data.gyrox), 6, 2, outstr);

          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
        count++;
        break;
      case 15:
        data.gyroy = c;

        if (wifiConnected) {
          dtostrf(((double)data.gyroy), 6, 2, outstr);

          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
        count++;
        break;
      case 16:
        data.gyroz = c;

        if (wifiConnected) {
          dtostrf(((double)data.gyroz), 6, 2, outstr);
          publishMQTT(outstr, count);
        }
          buf += outstr;
          buf+=",";
        count++;
        break;
      case 17:
        data.temperature = c;

        if (wifiConnected) {
          dtostrf(((double)data.temperature), 6, 2, outstr);
          publishMQTT(outstr, count);
        }
        buf += outstr;
          buf+=",";
        count++;
        break;
      case 18:
        data.pressure = c;

        if (wifiConnected) {
          dtostrf(((double)data.pressure), 7, 3, outstr);
          publishMQTT(outstr, count);
        }

          buf += outstr;
          buf+=",";
        count++;
        break;
      case 19:
        data.altitude = c;

        if (wifiConnected) {
          dtostrf(((double)data.altitude), 7, 3, outstr); 
          publishMQTT(outstr, count);
        }

        buf += outstr;
          buf +=",";

        count++;
        break;
      case 20:
        data.endID = (uint32_t) c;
        //Serial.print(data.endID);
        //Serial.println("");
        
        if (wifiConnected) {
          publishMQTT(buf.c_str(), count);
        }

        count = 1;
        buf = "";
        sendTlm();
        break;
      default:
        break;
    }
  }

}


/**************************************************************************/
void publishMQTT(String msg, int topic) {

  if (!client.connected()) {
    reconnect();
    //wifiConnected = false;
  }

  client.loop();
  //loop through parse packet and publish the information to MQTT broker
  int n = msg.length();
  char char_array[n + 1];


  // copying the contents of the
  // string to char array
  strcpy(char_array, msg.c_str());

  switch (topic) {
    case 1:
      //client.publish("esp32", char_array);
      break;
    case 2:
      client.publish("Rocket/packet", char_array);
      break;
    case 3:
      client.publish("Rocket/lat", char_array);
      break;
    case 4:
      client.publish("Rocket/lng", char_array);
      break;
    case 5:
      client.publish("Rocket/roll", char_array);
      break;
    case 6:
      client.publish("Rocket/pitch", char_array);
      break;
    case 7:
      client.publish("Rocket/yaw", char_array);
      break;
    case 8:
      client.publish("Rocket/accelx", char_array);
      break;
    case 9:
      client.publish("Rocket/accely", char_array);
      break;
    case 10:
      client.publish("Rocket/accelz", char_array);
      break;
    case 11:
      client.publish("Rocket/magx", char_array);
      break;
    case 12:
      client.publish("Rocket/magy", char_array);
      break;
    case 13:
      client.publish("Rocket/magz", char_array);
      break;
    case 14:
      client.publish("Rocket/gyrox", char_array);
      break;
    case 15:
      client.publish("Rocket/gyroy", char_array);
      break;
    case 16:
      client.publish("Rocket/gyroz", char_array);
      break;
    case 17:
      client.publish("Rocket/temp", char_array);
      break;
    case 18:
      client.publish("Rocket/pressure", char_array);
      break;
    case 19:
      client.publish("Rocket/alt", char_array);
      break;
    case 20:
      client.publish("wspacket", char_array);
      break;
    default:
      break;
  }

  return;
}
/**************************************************************************/
void sendTlm()
{
  data.length = sizeof(data);
  writeTlm((const char *)&data, sizeof(data));

}
/**************************************************************************/
void writeTlm(const char* pkt, byte size)
{

  for (int i = 0; i < size; i++)
  {
    Serial.write(pkt[i]);
  }
}
