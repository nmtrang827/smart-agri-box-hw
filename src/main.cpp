#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
// #include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <SocketIoClient.h>
#include <ArduinoJson.h>
//==============================================================================
#define dw digitalWrite
#define dr digitalRead
#define COI 15
#define RS_TX 17
#define RS_RX 16
#define RELAY1 2
#define RELAY2 0
#define NUT1 32
#define NUT2 33
#define NUT3 25
#define LED1 26
#define LED2 27
// #define ESP_RX 12
// #define ESP_TX 13
//---------------------------------------
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
//---------------------------------------
#define SERVER "smartagribox.com"
#define PORT 80
#define ssid "Wokwi-GUEST"
#define pass ""

#define TOPIC_MEASURE "/esp/measure"
#define TOPIC_CONTROL "/esp/control"
#define TOPIC_OTHER "/esp/other"
//==============================================================================
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// SoftwareSerial RS485Serial(RS_RX, RS_TX); // RX, TX
// SoftwareSerial ESPSerial(ESP_RX, ESP_TX); // RX, TX
ModbusMaster node;
Adafruit_AHTX0 aht;
SocketIOclient socketIO;
//==============================================================================
sensors_event_t humidity, temp;
float ph = -1.0, soilMoisture = -1.0, soilTemp = -1.0, EC = -1.0, n = -1.0, p = -1.0, k = -1.0;
unsigned long lastCoiHigh, lastSend, lastCheckSensor, lastFillRectWhite, lastPress1, lastPress2, lastPress3;
int buttonState1 = HIGH, buttonState2 = HIGH, buttonState3 = HIGH;
int lastButtonState1 = HIGH, lastButtonState2 = HIGH, lastButtonState3 = HIGH;
bool isWifiConnected = false, isSocketIOConnected = false;
int modeScreen = 0;    // modeScreen hien thi man hinh oled
int mode = 0;          // thu cong hoac tu dong
int selectedRelay = 0; // relay dang duoc chon
//==============================================================================

#define FRAME_DELAY (42)
#define FRAME_WIDTH (32)
#define FRAME_HEIGHT (32)
#define FRAME_COUNT (sizeof(frames) / sizeof(frames[0]))
const byte PROGMEM frames[][128] = {
    {0, 0, 0, 0, 0, 3, 192, 0, 0, 2, 64, 0, 0, 6, 96, 0, 1, 4, 32, 128, 3, 132, 33, 192, 4, 124, 62, 32, 12, 32, 12, 48, 4, 0, 0, 32, 2, 0, 0, 64, 3, 0, 0, 192, 3, 3, 192, 192, 2, 12, 48, 64, 30, 8, 16, 124, 96, 16, 8, 14, 64, 16, 8, 6, 64, 16, 8, 6, 112, 16, 8, 14, 30, 8, 16, 120, 2, 4, 48, 64, 3, 3, 192, 192, 3, 0, 0, 192, 2, 0, 0, 64, 4, 0, 0, 32, 12, 48, 4, 48, 4, 124, 62, 32, 3, 132, 33, 192, 1, 4, 32, 128, 0, 6, 96, 0, 0, 2, 64, 0, 0, 3, 192, 0, 0, 0, 0, 0},
};
//==============================================================================
void displayStartScreen()
{
  display.clearDisplay();
  display.drawBitmap(40, 16, frames[0], FRAME_WIDTH, FRAME_HEIGHT, 1);
  display.display();
  delay(FRAME_DELAY);
}
//==============================================================================
void displayDisplay(String s, int delayTime, bool isNeedNewLine)
{
  if (isNeedNewLine)
  {
    display.println(s);
  }
  else
  {
    display.print(s);
  }
  display.display();
  delay(delayTime);
}
//==============================================================================
// read aht sensor and save to humidity and temp
void readAHT()
{
  aht.getEvent(&humidity, &temp);
  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" *C");
}

//==============================================================================
// toggle led for x times y ms
void toggleLED(int led, int delayTime, int times)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(led, HIGH);
    delay(delayTime);
    digitalWrite(led, LOW);
    delay(delayTime);
  }
}
//==============================================================================
// read rs485
void readRS485()
{
  uint8_t result;
  result = node.readHoldingRegisters(0x0006, 2);
  if (result == node.ku8MBSuccess)
  {
    ph = (float)node.receive() / 100.0;
    Serial.print(ph);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading ph");
    Serial.println(result);
  }
  result = node.readHoldingRegisters(0x0015, 2);
  if (result == node.ku8MBSuccess)
  {
    EC = (float)node.receive();
    Serial.print(EC);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading EC");
    Serial.println((int)result);
  }
  result = node.readHoldingRegisters(0x001e, 2);
  if (result == node.ku8MBSuccess)
  {
    n = (float)node.receive();
    Serial.print(n);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading n");
  }
  result = node.readHoldingRegisters(0x001f, 2);
  if (result == node.ku8MBSuccess)
  {
    p = (float)node.receive();
    Serial.print(p);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading p");
  }
  result = node.readHoldingRegisters(0x0020, 2);
  if (result == node.ku8MBSuccess)
  {
    k = (float)node.receive();
    Serial.print(k);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading k");
  }
  result = node.readHoldingRegisters(0x0012, 2);
  if (result == node.ku8MBSuccess)
  {
    soilMoisture = (float)node.receive() / 10.0;
    Serial.print(soilMoisture);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading soilMoisture");
  }
  result = node.readHoldingRegisters(0x0013, 2);
  if (result == node.ku8MBSuccess)
  {
    k = (float)node.receive() / 10.0;
    Serial.print(k);
    Serial.print('\t');
  }
  else
  {
    Serial.print("Error reading soilTemp");
    Serial.println(result);
  }
}
//==============================================================================
// send data to server
void sendDataToServer(int type, int buttonType, String message)
{
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  if (type == 1)
  {
    array.add(TOPIC_MEASURE);
    JsonObject data = array.createNestedObject();
    data["deviceID"] = "esp32";
    data["temp"] = temp.temperature;
    data["humi"] = humidity.relative_humidity;
    data["ph"] = ph;
    data["soilMoisture"] = soilMoisture;
    data["soilTemp"] = soilTemp;
    data["n"] = n;
    data["p"] = p;
    data["k"] = k;
    data["EC"] = EC;
  }
  else if (type == 2)
  {
    array.add(TOPIC_CONTROL);
    JsonObject data = array.createNestedObject();
    data["deviceID"] = "esp32";
    data["button"] = buttonType;
  }
  else
  {
    array.add(TOPIC_OTHER);
    JsonObject data = array.createNestedObject();
    data["deviceID"] = "esp32";
    data["message"] = message;
  }
  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
}
//==============================================================================
// update data on oled display
void updateDataOnDisplay(int piece)
{
  if (piece == 1)
  {
    display.fillRect(0, 0, 128, 55, SH110X_BLACK);
    display.setCursor(0, 0);
    display.print("Temp: ");
    display.println(temp.temperature);
    display.drawFastHLine(0, 8, display.width(), SH110X_WHITE);
    display.setCursor(0, 10);
    display.print("Humi: ");
    display.println(humidity.relative_humidity);
    display.drawFastHLine(0, 18, display.width(), SH110X_WHITE);
    display.setCursor(0, 20);
    display.print("ph: ");
    display.println(ph);
    display.drawFastHLine(0, 28, display.width(), SH110X_WHITE);
    display.setCursor(0, 30);
    display.print("sHum: ");
    display.println(soilMoisture);
    display.drawFastHLine(0, 38, display.width(), SH110X_WHITE);
    display.setCursor(0, 40);
    display.print("sTemp: ");
    display.println(soilTemp);
    display.drawFastHLine(0, 48, display.width(), SH110X_WHITE);
    display.setCursor(0, 50);
  }
  else if (piece == 2)
  {
    display.drawFastVLine(display.width() / 2, 0, display.height(), SH110X_WHITE);
    display.setCursor(70, 0);
    display.print("n: ");
    display.println(n);
    display.setCursor(70, 10);
    display.print("p: ");
    display.println(p);
    display.setCursor(70, 20);
    display.print("k: ");
    display.println(k);
    display.setCursor(70, 30);
    display.print("EC: ");
    display.println(EC);
    display.setCursor(70, 40);
  }
  else
  {
    display.setCursor(0, 0);
    display.println("test");
  }
}
//==============================================================================
// handle button press
void handleBtnPress(int state)
{
  dw(COI, HIGH);
  lastCoiHigh = millis();
  Serial.print("Button pressed  - ");
  Serial.println(state);
  sendDataToServer(2, state, "");
  // ESPSerial.println("state: " + String(state));
  switch (state)
  {
  case 1:
    if (modeScreen == 0)
      modeScreen = 1;
    else
      modeScreen = 0;
    break;
  case 2:
    if (mode == 0)
      mode = 1;
    else
      mode = 0;
    break;
  case 3:
    if (selectedRelay == 0)
      selectedRelay = 1;
    else
      selectedRelay = 0;
    break;
  case 4:
    if (selectedRelay == 0)
    {
      dw(RELAY1, !dr(RELAY1));
      dw(LED1, !dr(LED1));
    }
    else
    {
      dw(RELAY2, !dr(RELAY2));
      dw(LED2, !dr(LED2));
    }
    break;
  default:
    break;
  }
}
//==============================================================================
#define USE_SERIAL Serial
void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case sIOtype_DISCONNECT:
  {
    isSocketIOConnected = false;
    USE_SERIAL.printf("[IOc] Disconnected!\n");
    break;
  }
  case sIOtype_CONNECT:
  {
    USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
    isSocketIOConnected = true;
    // join default namespace (no auto join in Socket.IO V3)
    socketIO.send(sIOtype_CONNECT, "/");
    break;
  }
  case sIOtype_EVENT:
  {
    display.setCursor(1, 57);
    display.fillRect(0, 56, 128, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.print("R");
    display.setTextColor(SH110X_BLACK); // Draw white text
    display.display();
    String temp = String((char *)payload);
    if (temp.indexOf("/esp/control") != -1)
    {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, temp);
      JsonObject data = doc["data"];
      int button = data["button"];
      handleBtnPress(button);
    }
    else if (temp.indexOf("/esp/other") != -1)
    {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, temp);
      JsonObject data = doc["data"];
      String message = data["message"];
      Serial.println(message);
    }
  }
  break;
  case sIOtype_ACK:
    USE_SERIAL.printf("[IOc] get ack: %u\n", length);
    break;
  case sIOtype_ERROR:
    USE_SERIAL.printf("[IOc] get error: %u\n", length);
    break;
  case sIOtype_BINARY_EVENT:
    USE_SERIAL.printf("[IOc] get binary: %u\n", length);
    break;
  case sIOtype_BINARY_ACK:
    USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
    break;
  }
}
//==============================================================================
void setup()
{
  Serial.begin(115200);
  // ESPSerial.begin(115200);
  Serial.println("Starting...");
  //---------------------------------------
  if (!display.begin(SCREEN_ADDRESS, true))
  {
    Serial.println(F("Oled allocation failed"));
    // toggle led1 for 2 times 1000ms each
    toggleLED(LED1, 300, 5);
  }
  displayStartScreen();
  delay(2000);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  displayDisplay("Smart Agri Box", 100, true);
  displayDisplay("Starting...", 100, true);
  //---------------------------------------
  displayDisplay("Pin...", 100, false);
  pinMode(COI, OUTPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(NUT1, INPUT_PULLUP);
  pinMode(NUT2, INPUT_PULLUP);
  pinMode(NUT3, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  dw(COI, LOW);
  dw(RELAY1, HIGH);
  dw(RELAY2, HIGH);
  dw(LED1, LOW);
  dw(LED2, LOW);
  displayDisplay("Done", 0, true);
  //---------------------------------------
  displayDisplay("AHT20...", 100, false);
  if (aht.begin())
  {
    Serial.println("Found AHT20");
  }
  else
  {
    Serial.println("Didn't find AHT20");
    toggleLED(LED1, 300, 3);
  }
  displayDisplay("Done", 0, true);
  //---------------------------------------
  displayDisplay("RS485...", 100, false);
  Serial2.begin(9600, SERIAL_8O1, RS_RX, RS_TX);
  node.begin(1, Serial2);
  displayDisplay("Done", 0, true);
  //---------------------------------------
  displayDisplay("WiFi...", 100, false);
  WiFiManager wifiManager;
  wifiManager.autoConnect("smartAgriBox");
  // WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  isWifiConnected = true;
  displayDisplay("Done", 0, true);
  displayDisplay("SocketIO...", 100, false);
  // server address, port and URL
  socketIO.begin(SERVER, 80, "/socket.io/?EIO=4");

  // event handler
  socketIO.onEvent(socketIOEvent);
  displayDisplay("Done", 1000, true);
  //---------------------------------------
}

void loop()
{
  display.display();
  delay(10);
  socketIO.loop();
  buttonState1 = digitalRead(NUT1);
  buttonState2 = digitalRead(NUT2);
  buttonState3 = digitalRead(NUT3);

  if (buttonState1 == LOW && lastButtonState1 == HIGH)
  {
    lastPress1 = millis();
    Serial.println("Button 1 pressed");
  }
  if (buttonState2 == LOW && lastButtonState2 == HIGH)
  {
    Serial.println("Button 2 pressed");
    lastPress2 = millis();
  }
  if (buttonState3 == LOW && lastButtonState3 == HIGH)
  {
    Serial.println("Button 3 pressed");
    lastPress3 = millis();
  }

  // release button 1 for modeScreen change
  if (buttonState1 == HIGH && lastButtonState1 == LOW)
  {
    Serial.println("Button 1 released");

    unsigned long elapsedTime = millis() - lastPress1;
    if (elapsedTime > 1000 && elapsedTime < 3000)
    {
      dw(COI, HIGH);
      lastCoiHigh = millis();
      handleBtnPress(1);
    }
    else if (elapsedTime > 3000)
    {
      dw(COI, HIGH);
      lastCoiHigh = millis();
      handleBtnPress(2);
    }
  }
  // release button 2 for selected thing change
  if (buttonState2 == HIGH && lastButtonState2 == LOW)
  {
    Serial.println("Button 1 released");
    unsigned long elapsedTime = millis() - lastPress2;
    if (elapsedTime > 1000 && elapsedTime < 3000)
    {
      dw(COI, HIGH);
      lastCoiHigh = millis();
      handleBtnPress(3);
    }
  }
  // release button 3 for selected thing status change
  if (buttonState3 == HIGH && lastButtonState3 == LOW)
  {
    Serial.println("Button 1 released");
    unsigned long elapsedTime = millis() - lastPress3;
    if (elapsedTime > 1000)
    {
      dw(COI, HIGH);
      lastCoiHigh = millis();
      handleBtnPress(4);
    }
  }
  // DONE: add modeScreen 1 show info on oled
  if (modeScreen == 1)
  {
    display.fillRect(0, 0, 128, 55, SH110X_BLACK);
    // show info of relay 1 and 2 in the oled, the left side is relay 1 and the right side is relay 2, each will hold 1/2 of the width of the oled and 2/3 of the height of the oled
    if (dr(RELAY1) == LOW)
    {
      display.drawRect(0, 0, display.width() / 2 - 1, display.height() * 2 / 3, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
    }
    else
    {
      display.fillRect(0, 0, display.width() / 2 - 1, display.height() * 2 / 3, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    }
    display.setCursor(10, 20);
    display.print("Relay 1");
    if (dr(RELAY2) == LOW)
    {
      display.drawRect(display.width() / 2 + 1, 0, display.width() / 2, display.height() * 2 / 3, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
    }
    else
    {
      display.fillRect(display.width() / 2 + 1, 0, display.width() / 2 - 1, display.height() * 2 / 3, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    }
    display.setCursor(72, 20);
    display.print("Relay 2");
    display.setCursor(29 + 62 * selectedRelay, display.height() * 2 / 3 + 1);
    display.setTextColor(SH110X_WHITE);
    display.write(0x5e);
  }
  else if (modeScreen == 0)
  {
    updateDataOnDisplay(1);
    updateDataOnDisplay(2);
    // updateDataOnDisplay(3);
  }

  if (millis() - lastFillRectWhite > 500)
  {
    display.fillRect(0, 56, 128, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(80, 57);
    if (isWifiConnected)
    {
      display.print("W:1");
    }
    else
    {
      display.print("W:0");
    }
    display.setCursor(100, 57);
    if (isSocketIOConnected)
    {
      display.print("S:1");
    }
    else
    {
      display.print("S:0");
    }
    display.setTextColor(SH110X_WHITE);
    lastFillRectWhite = millis();
  }

  if (millis() - lastCoiHigh > 500 && dr(COI) == HIGH)
  {
    dw(COI, LOW);
  }
  if (millis() - lastSend > 5000)
  {
    lastSend = millis();
    // update infor on display
    display.setCursor(10, 57);
    display.setTextColor(SH110X_BLACK); // Draw white text
    display.print("S");
    display.setTextColor(SH110X_WHITE); // Draw white text
    sendDataToServer(1, 0, "");
  }
  if (millis() - lastCheckSensor > 3000)
  {
    readAHT();
    readRS485();
    lastCheckSensor = millis();
  }
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
  lastButtonState3 = buttonState3;
}
