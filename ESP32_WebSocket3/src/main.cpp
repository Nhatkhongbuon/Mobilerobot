#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Arduino_JSON.h>

const char* ssid = "Nguyen Phu Cau";
const char* password = "0904868337";

AsyncWebServer server(80);

const char* Param_Input_1 = "input1";
const char* Param_Input_2 = "input2";

String input1;
String input2;

const char* input1Path = "/input1.txt";
const char* input2Path = "/input2.txt";

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

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Conneting to WiFi " + String(ssid));
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.print("Connected successed with IP: ");
  Serial.println(WiFi.localIP());
}

String getCurrentInputValue() {
  values["textValue"] = input1;
  values["numberValue"] = input2;
  String jsonString = JSON.stringify(values);
  return jsonString;
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();

  input1 = readFile(SPIFFS, input1Path);
  input2 = readFile(SPIFFS, input2Path);

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
          Serial.print("Input 1 set to: ");
          Serial.println(input1);

          writeFile(SPIFFS, input1Path, input1.c_str());
        }

        if(p->name() == Param_Input_2) {
          input2 = p->value().c_str();
          Serial.print("Input 2 set to: ");
          Serial.println(input2);

          writeFile(SPIFFS, input2Path, input2.c_str());
        }
      }
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.begin();
}

void loop() {

}