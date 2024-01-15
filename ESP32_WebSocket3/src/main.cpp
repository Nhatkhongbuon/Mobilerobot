#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Arduino_JSON.h>
#include <HardwareSerial.h>

// const char* ssid = "Nguyen Phu Cau";
// const char* password = "0904868337";

const char* ssid = "Mobile Robot ESP32";
const char* password = "88888888";
AsyncWebServer server(80);

const char* Param_Input_1 = "input1";
const char* Param_Input_2 = "input2";
const char* Param_Input_3 = "input3";
const char* Param_Input_4 = "input4";
const char* Param_Input_5 = "input5";

String buffer[5];

String input1;
String input2;
String input3;
String input4;
String input5;

const char* input1Path = "/input1.txt";
const char* input2Path = "/input2.txt";
const char* input3Path = "/input3.txt";
const char* input4Path = "/input4.txt";
const char* input5Path = "/input5.txt";

JSONVar values;

void initSPIFFS() { 
  if(!SPIFFS.begin(true)) {
    Serial.println("Can't init SPIFFS!!!");
  } else {
    Serial.println("Init SPIFFS successful");
  }
}

String readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()) {
    Serial.println("Failed to open file for reading");
    return String();
  }

  String fileContent;
  while(file.available()) {
    fileContent = file.readStringUntil('\n');
    break;
  }
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\r\n", path);
  
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
}

// void initWiFi() {
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password);
//   Serial.println("Conneting to WiFi " + String(ssid));
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     Serial.print(".");
//     delay(500);
//   }
//   Serial.println("");
//   Serial.print("Connected successed with IP: ");
//   Serial.println(WiFi.localIP());
// }

void initWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("Setting Access Point");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

String getCurrentInputValue() {
  values["textValue"] = input1;
  values["numberValue"] = input2;
  values["thetaValue"] = input3;
  values["xrValue"] = input4;
  values["yrValue"] = input5;
  String jsonString = JSON.stringify(values);
  return jsonString;
}

void setup() {
  HardwareSerial SerialPort(2);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();


  input1 = readFile(SPIFFS, input1Path);
  input2 = readFile(SPIFFS, input2Path);
  input3 = readFile(SPIFFS, input3Path);
  input4 = readFile(SPIFFS, input4Path);
  input5 = readFile(SPIFFS, input5Path);

  buffer[0] = input1;
  buffer[1] = input2;
  buffer[2] = input3;
  buffer[3] = input4;
  buffer[4] = input5;

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/values", HTTP_GET, [](AsyncWebServerRequest * request){
    String json = getCurrentInputValue();
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/", HTTP_POST, [](AsyncWebServerRequest * request){
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()) {
        if(p->name() == Param_Input_1) {
          input1 = p->value().c_str();
          buffer[0] = input1;
          Serial.print("X set to: ");
          Serial.println(input1);
          Serial.print("Buffer 0: ");
          Serial.println(buffer[0]);
          for(int i=0; i<5; i++) {
            Serial2.print(buffer[i]);
          }
          writeFile(SPIFFS, input1Path, input1.c_str());
        }

        if(p->name() == Param_Input_2) {
          input2 = p->value().c_str();
          buffer[1] = input2;
          Serial.print("Y set to: ");
          Serial.println(input2);
          Serial.print("Buffer 1: ");
          Serial.println(buffer[1]);
          for(int i=0; i<5; i++) {
            Serial2.print(buffer[i]);
          }
          writeFile(SPIFFS, input2Path, input2.c_str());
        }

        if(p->name() == Param_Input_3) {
          input3 = p->value().c_str();
          buffer[2] = input3;
          Serial.print("Theta set to: ");
          Serial.println(input3);
          Serial.print("Buffer 2: ");
          Serial.println(buffer[2]);
          for(int i=0; i<5; i++) {
            Serial2.print(buffer[i]);
          }
          writeFile(SPIFFS, input3Path, input3.c_str());
        }

        if(p->name() == Param_Input_4) {
          input4 = p->value().c_str();
          buffer[3] = input4;
          Serial.print("X_r set to: ");
          Serial.println(input4);
          Serial.print("Buffer 3: ");
          Serial.println(buffer[3]);
          for(int i=0; i<5; i++) {
            Serial2.print(buffer[i]);
          }
          writeFile(SPIFFS, input4Path, input4.c_str());
        }

        if(p->name() == Param_Input_5) {
          input5 = p->value().c_str();
          buffer[4] = input5;
          Serial.print("Y_r set to: ");
          Serial.println(input5);
          Serial.print("Buffer 4: ");
          Serial.println(buffer[4]);
          for(int i=0; i<5; i++) {
            Serial2.print(buffer[i]);
          }
          writeFile(SPIFFS, input5Path, input5.c_str());
        }

        Serial.print("Buffer: ");
        for(int i=0; i<5; i++) {
          Serial.print(buffer[i]);
        }
      }
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.begin();
}

void loop() {

}