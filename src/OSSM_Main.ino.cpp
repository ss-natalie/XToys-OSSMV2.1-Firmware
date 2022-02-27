# 1 "C:\\Users\\Admin\\AppData\\Local\\Temp\\tmp8v2qznrm"
#include <Arduino.h>
# 1 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP_FlexyStepper.h>
#include <Encoder.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <OSSM-BLE.h>
#include <list>

#include "FastLED.h"
#include "OSSM_Config.h"
#include "OSSM_PinDef.h"
#include "OssmUi.h"

const char* FIRMWARE_VERSION = "v1.1";
# 25 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
#define DEBUG 

#ifdef DEBUG
#define LogDebug(...) Serial.println(__VA_ARGS__)
#define LogDebugFormatted(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebug(...) ((void)0)
#define LogDebugFormatted(...) ((void)0)
#endif


volatile bool g_has_not_homed = true;
bool REMOTE_ATTACHED = false;


Encoder g_encoder(ENCODER_A, ENCODER_B);


OssmUi g_ui(REMOTE_ADDRESS, REMOTE_SDA, REMOTE_CLK);
# 53 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
IRAM_ATTR void encoderPushButton();
float getEncoderPercentage();
void ICACHE_RAM_ATTR stopSwitchHandler();
void setup();
void loop();
#line 53 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
IRAM_ATTR void encoderPushButton()
{



    LogDebug("Encoder Button Push");
}

float getEncoderPercentage()
{
    const int encoderFullScale = 100;
    int position = g_encoder.read();
    float positionPercentage;
    if (position < 0)
    {
        g_encoder.write(0);
        position = 0;
    }
    else if (position > encoderFullScale)
    {
        g_encoder.write(encoderFullScale);
        position = encoderFullScale;
    }

    positionPercentage = 100.0 * position / encoderFullScale;

    return positionPercentage;
}
# 91 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
WiFiManager wm;


ESP_FlexyStepper stepper;


volatile float strokePercentage = 0;
volatile float speedPercentage = 0;
volatile float deceleration = 0;
volatile int targetPosition;
volatile int targetDuration;
volatile int targetStepperPosition = 0;
volatile int remainingCommandTime = 0;
volatile float accelspeed = 0;




TaskHandle_t wifiTask = nullptr;
TaskHandle_t getInputTask = nullptr;
TaskHandle_t motionTask = nullptr;
TaskHandle_t estopTask = nullptr;
TaskHandle_t oledTask = nullptr;
TaskHandle_t bleTask = nullptr;
TaskHandle_t blemTask = nullptr;

#define BRIGHTNESS 170
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define LED_PIN 25
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];



void setLedRainbow(CRGB leds[]);
void getUserInputTask(void *pvParameters);
void motionCommandTask(void *pvParameters);
void wifiConnectionTask(void *pvParameters);
void bleConnectionTask(void *pvParameters);
void blemotionTask(void *pvParameters);
void estopResetTask(void *pvParameters);
float getAnalogAverage(int pinNumber, int samples);
bool setInternetControl(bool wifiControlEnable);
bool getInternetSettings();

bool stopSwitchTriggered = 0;







void ICACHE_RAM_ATTR stopSwitchHandler()
{
    stopSwitchTriggered = 1;
    vTaskSuspend(motionTask);
    vTaskSuspend(getInputTask);
    stepper.emergencyStop();
}
# 162 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
bool deviceConnected = false;
std::list<std::string> pendingCommands = {};
bool stepperMoving = false;
bool moveto = false;

void updateSettingsCharacteristic();
void processCommand(std::string msg);
void moveTo(int targetPosition, int targetDuration);


void processCommand(std::string msg) {

  char command = NULL;
  int channel = 0;
  int targetAmount = 0;
  int targetDuration = 0;
  int numBeingRead = NUM_NONE;

  for (char c : msg) {
    switch (c) {
      case 'l':
      case 'L':
        command = 'L';
        numBeingRead = NUM_CHANNEL;
        break;
      case 'i':
      case 'I':
        numBeingRead = NUM_DURATION;
        break;
      case 'D':
      case 'd':
        command = 'D';
        numBeingRead = NUM_CHANNEL;
        break;
      case 'v':
      case 'V':
        numBeingRead = NUM_VALUE;
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        int num = c - '0';
        switch (numBeingRead) {
          case NUM_CHANNEL:
            channel = num;
            numBeingRead = NUM_PERCENT;
            break;
          case NUM_PERCENT:
            targetAmount = targetAmount * 10 + num;
            break;
          case NUM_DURATION:
            targetDuration = targetDuration * 10 + num;
            break;
        }
        break;
    }
  }



  if (command == 'L' && channel == 1) {
    moveTo(targetAmount, targetDuration);

  } else if (command == 'D') {
  } else {
    Serial.print("Invalid command: ");
    Serial.println(msg.c_str());
  }

}

void moveTo(int targetPosition, int targetDuration){
        stepper.releaseEmergencyStop();
        float currentStepperPosition = stepper.getCurrentPositionInMillimeters();
        float targetxStepperPosition;
        targetPosition = map(targetPosition, 0, 100, 0, 1000);
        if(targetPosition != 0){
         targetxStepperPosition = (float(targetPosition) * 0.001) * maxStrokeLengthMm;
        } else {
         targetxStepperPosition = 0.0;
        }
        float targetspeed = map(targetDuration, 80, 1000, maxSpeedMmPerSecond, 0);

        float travelInMM = targetxStepperPosition -currentStepperPosition;
        targetspeed = (abs(travelInMM) / targetspeed) * (targetDuration);
        accelspeed = map(targetDuration, 80, 1000, 99, 0);
        Serial.print("targetspeed: ");
        Serial.println(targetspeed);
        Serial.print("targetxStepperPosition: ");
        Serial.println(targetxStepperPosition);
        stepper.setSpeedInMillimetersPerSecond(targetspeed);
        stepper.setAccelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * accelspeed * accelspeed / accelerationScaling);
        stepper.setTargetPositionInMillimeters(targetxStepperPosition);
}


class SettingsCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string msg = characteristic->getValue();

    LogDebug("Received command: ");
    Serial.println(msg.c_str());

    std::size_t pos = msg.find(':');
    std::string settingKey = msg.substr(0, pos);
    std::string settingValue = msg.substr(pos + 1, msg.length());

    if (settingKey == "maxIn") {
      maxInPosition = atoi(settingValue.c_str());
      preferences.putInt("maxIn", maxInPosition);
    }
    if (settingKey == "maxOut") {
      maxOutPosition = atoi(settingValue.c_str());
      preferences.putInt("maxOut", maxOutPosition);
    }
    if (settingKey == "maxSpeed") {
      maxSpeed = atoi(settingValue.c_str());
      preferences.putInt("maxSpeed", maxSpeed);
    }
    if (settingKey == "minSpeed") {
      minSpeed = atoi(settingValue.c_str());
      preferences.putInt("minSpeed", minSpeed);
    }
    Serial.print("Setting pref ");
    Serial.print(settingKey.c_str());
    Serial.print(" to ");
    Serial.println(settingValue.c_str());
    updateSettingsCharacteristic();
  }
};


class ControlCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string msg = characteristic->getValue();

    Serial.print("Received command: ");
    Serial.println(msg.c_str());


    if (msg == "DSTOP") {
      Serial.println("STOP");
      stepper.emergencyStop();
      pendingCommands.clear();
      stepperMoving = false;
      return;
    } else if (msg == "DENABLE") {
      Serial.println("ENABLE");
      stepper.releaseEmergencyStop();
      pendingCommands.clear();
      stepperMoving = false;

      return;
    } else if (msg == "DDISABLE") {
      Serial.println("DISABLE");
      stepper.emergencyStop();
      pendingCommands.clear();
      stepperMoving = false;
      return;
    }
    if (msg.front() == 'D') {
      return;
    }
    if (msg.back() == 'C') {
      pendingCommands.clear();
      pendingCommands.push_back(msg);
      return;
    }

    if (pendingCommands.size() < 100) {
      pendingCommands.push_back(msg);
      Serial.print("# of pending commands: ");
      Serial.println(pendingCommands.size());
    } else {
      Serial.print("Too many commands in queue. Dropping: ");
      Serial.println(msg.c_str());
    }
  }
};


class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Connected");
    vTaskSuspend(motionTask);
    vTaskSuspend(getInputTask);
    vTaskSuspend(estopTask);
    vTaskResume(blemTask);
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Disconnected");
    pServer->startAdvertising();
    vTaskResume(motionTask);
    vTaskResume(getInputTask);
    vTaskSuspend(estopTask);
  }
};


void updateSettingsCharacteristic() {
  String settingsInfo = String("maxIn:") + maxInPosition + ",maxOut:" + maxOutPosition + ",maxSpeed:" + maxSpeed + ",minSpeed:" + minSpeed;
  settingsCharacteristic->setValue(settingsInfo.c_str());
}
# 383 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
void setup()
{
    Serial.begin(115200);
    LogDebug("\n Starting");
    delay(200);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(150);
    setLedRainbow(leds);
    FastLED.show();
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

    preferences.begin("OSSM", false);
    maxSpeed = preferences.getInt("maxSpeed", DEFAULT_MAX_SPEED);
    minSpeed = preferences.getInt("minSpeed", DEFAULT_MIN_SPEED);

    float stepsPerMm = motorStepPerRevolution / (pulleyToothCount * beltPitchMm);
    stepper.setStepsPerMillimeter(stepsPerMm);


    stepper.setSpeedInStepsPerSecond(200);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(100);
    stepper.setDecelerationInStepsPerSecondPerSecond(100000);
    stepper.setLimitSwitchActive(LIMIT_SWITCH_PIN);




    stepper.startAsService();



    WiFi.mode(WIFI_STA);

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
    pinMode(WIFI_CONTROL_TOGGLE_PIN, WIFI_CONTROLLER);





    pinMode(STOP_PIN, INPUT_PULLUP);


    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopSwitchHandler, RISING);


    pinMode(SPEED_POT_PIN, INPUT);
    adcAttachPin(SPEED_POT_PIN);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);


    if (digitalRead(WIFI_RESET_PIN) == LOW)
    {

        wm.resetSettings();
        LogDebug("settings reset");
    }


    g_ui.Setup();
    g_ui.UpdateOnly();


    pinMode(ENCODER_SWITCH, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ENCODER_SWITCH), encoderPushButton, RISING);


    xTaskCreatePinnedToCore(wifiConnectionTask,
                            "wifiConnectionTask",
                            10000,
                            NULL,
                            1,
                            &wifiTask,
                            0);
    delay(100);

    if (g_has_not_homed == true)
    {
        LogDebug("OSSM will now home");
        g_ui.UpdateMessage("Finding Home");
        stepper.setSpeedInMillimetersPerSecond(20);
        stepper.moveToHomeInMillimeters(1, 30, 300, LIMIT_SWITCH_PIN);
        LogDebug("OSSM has homed, will now move out to max length");
        g_ui.UpdateMessage("Moving to Max");
        stepper.setSpeedInMillimetersPerSecond(20);
        stepper.moveToPositionInMillimeters((-1 * maxStrokeLengthMm) - strokeZeroOffsetmm);
        LogDebug("OSSM has moved out, will now set new home?");
        stepper.setCurrentPositionAsHomeAndStop();
        LogDebug("OSSM should now be home and happy");
        g_has_not_homed = false;
    }

    xTaskCreatePinnedToCore(blemotionTask,
                            "blemotionTask",
                            1000,
                            NULL,
                            1,
                            &blemTask,
                            0);
    vTaskSuspend(blemTask);
    delay(100);



    xTaskCreatePinnedToCore(bleConnectionTask,
                            "bleConnectionTask",
                            10000,
                            NULL,
                            1,
                            &bleTask,
                            0);
    delay(100);






    xTaskCreatePinnedToCore(getUserInputTask,
                            "getUserInputTask",
                            10000,
                            NULL,
                            1,
                            &getInputTask,
                            0);
    delay(100);
    xTaskCreatePinnedToCore(motionCommandTask,
                            "motionCommandTask",
                            10000,
                            NULL,
                            1,
                            &motionTask,
                            0);

    delay(100);
    xTaskCreatePinnedToCore(estopResetTask,
                            "estopResetTask",
                            10000,
                            NULL,
                            1,
                            &estopTask,
                            0);

    delay(100);

    g_ui.UpdateMessage("OSSM Ready to Play");
}
# 543 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
void loop()
{
    g_ui.UpdateState(static_cast<int>(speedPercentage), static_cast<int>(strokePercentage + 0.5f));
    g_ui.UpdateScreen();


    static bool is_connected = false;
    if (!is_connected && g_ui.DisplayIsConnected())
    {
        LogDebug("Display Connected");
        is_connected = true;
    }
    else if (is_connected && !g_ui.DisplayIsConnected())
    {
        LogDebug("Display Disconnected");
        is_connected = false;
    }
}
# 570 "D:/SM-Projekte/xtoy-stepper/src/OSSM_Main.ino"
void estopResetTask(void *pvParameters)
{
    for (;;)
    {
        if (stopSwitchTriggered == 1)
        {
            while ((getAnalogAverage(SPEED_POT_PIN, 50) > 2))
            {
                vTaskDelay(1);
            }
            stopSwitchTriggered = 0;
            vTaskResume(motionTask);
            vTaskResume(getInputTask);
        }
        vTaskDelay(100);
    }
}

void bleConnectionTask(void *pvParameters){

Serial.println("Initializing BLE Server...");
  BLEDevice::init("OSSM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  infoService = pServer->createService(BLEUUID((uint16_t) 0x180a));
  BLE2904* softwareVersionDescriptor = new BLE2904();
  softwareVersionDescriptor->setFormat(BLE2904::FORMAT_UINT8);
  softwareVersionDescriptor->setNamespace(1);
  softwareVersionDescriptor->setUnit(0x27ad);

  softwareVersionCharacteristic = infoService->createCharacteristic((uint16_t) 0x2a28, BLECharacteristic::PROPERTY_READ);
  softwareVersionCharacteristic->addDescriptor(softwareVersionDescriptor);
  softwareVersionCharacteristic->addDescriptor(new BLE2902());
  softwareVersionCharacteristic->setValue(FIRMWARE_VERSION);
  infoService->start();

  pService = pServer->createService(SERVICE_UUID);
  controlCharacteristic = pService->createCharacteristic(
                                         CONTROL_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  controlCharacteristic->addDescriptor(new BLE2902());
  controlCharacteristic->setValue("");
  controlCharacteristic->setCallbacks(new ControlCharacteristicCallback());

  settingsCharacteristic = pService->createCharacteristic(
                                         SETTINGS_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  settingsCharacteristic->addDescriptor(new BLE2902());
  settingsCharacteristic->setValue("");
  settingsCharacteristic->setCallbacks(new SettingsCharacteristicCallback());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  updateSettingsCharacteristic();
  vTaskDelete(NULL);
}


void wifiConnectionTask(void *pvParameters)
{
    wm.setConfigPortalTimeout(100);
    wm.setConfigPortalBlocking(false);


    if (!wm.autoConnect("OSSM-setup"))
    {

        LogDebug("failed to connect and hit timeout");
    }
    else
    {

        LogDebug("Connected!");
    }
    for (;;)
    {
        wm.process();
        vTaskDelay(1);


        if (WiFi.status() == WL_CONNECTED)
        {
            vTaskDelete(NULL);
        }
    }
}

void blemotionTask(void *pvParameters)
{
    for (;;)

    {
        while (stepper.getDistanceToTargetSigned() != 0)
        {
            vTaskDelay(5);
        }
        stepper.setDecelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * accelspeed * accelspeed / accelerationScaling);
        vTaskDelay(1);
        if (pendingCommands.size() > 0) {
        std::string command = pendingCommands.front();
        processCommand(command);
        pendingCommands.pop_front();
        }
        vTaskDelay(1);
    }
}


void getUserInputTask(void *pvParameters)
{
    bool wifiControlEnable = false;
    for (;;)

    {


        if (speedPercentage > 1)
        {
            stepper.releaseEmergencyStop();
        }
        else
        {
            stepper.emergencyStop();

        }

        if (digitalRead(WIFI_CONTROL_TOGGLE_PIN) == HIGH)
        {
            if (wifiControlEnable == false)
            {


                wifiControlEnable = true;
                if (WiFi.status() != WL_CONNECTED)
                {
                    delay(5000);
                }
                setInternetControl(wifiControlEnable);
            }
            getInternetSettings();

      }
       else
       {
            if (wifiControlEnable == true)
            {


                wifiControlEnable = false;
                setInternetControl(wifiControlEnable);
            }
            speedPercentage = getAnalogAverage(SPEED_POT_PIN,
                                               50);

            strokePercentage = getEncoderPercentage();
        }



        if (speedPercentage > commandDeadzonePercentage)
        {
            stepper.setSpeedInMillimetersPerSecond(maxSpeedMmPerSecond * speedPercentage / 100.0);
            stepper.setAccelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage *
                                                                   speedPercentage / accelerationScaling);



        }
        vTaskDelay(100);
    }
}

void motionCommandTask(void *pvParameters)
{
    for (;;)

    {

        while ((stepper.getDistanceToTargetSigned() != 0) || (strokePercentage < commandDeadzonePercentage) ||
               (speedPercentage < commandDeadzonePercentage))
        {
            vTaskDelay(5);
        }
        float targetPosition = (strokePercentage / 100.0) * maxStrokeLengthMm;
        LogDebugFormatted("Moving stepper to position %ld \n", static_cast<long int>(targetPosition));
        vTaskDelay(1);
        stepper.setDecelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage * speedPercentage /
                                                               accelerationScaling);
        stepper.setTargetPositionInMillimeters(targetPosition);
        vTaskDelay(1);

        while ((stepper.getDistanceToTargetSigned() != 0) || (strokePercentage < commandDeadzonePercentage) ||
               (speedPercentage < commandDeadzonePercentage))
        {
            vTaskDelay(5);

        }
        targetPosition = 0;

        vTaskDelay(1);
        stepper.setDecelerationInMillimetersPerSecondPerSecond(maxSpeedMmPerSecond * speedPercentage * speedPercentage /
                                                               accelerationScaling);
        stepper.setTargetPositionInMillimeters(targetPosition);
        vTaskDelay(1);
  }
}


float getAnalogAverage(int pinNumber, int samples)
{
    float sum = 0;
    float average = 0;
    float percentage = 0;
    for (int i = 0; i < samples; i++)
    {

        sum += analogRead(pinNumber);
    }
    average = sum / samples;

    percentage = 100.0 * average / 4096.0;
    return percentage;
}

bool setInternetControl(bool wifiControlEnable)
{




    String serverNameBubble = "http://d2g4f7zewm360.cloudfront.net/ossm-set-control";





    StaticJsonDocument<200> doc;
    doc["ossmId"] = ossmId;
    doc["wifiControlEnabled"] = wifiControlEnable;
    doc["stroke"] = strokePercentage;
    doc["speed"] = speedPercentage;
    String requestBody;
    serializeJson(doc, requestBody);


    HTTPClient http;
    http.begin(serverNameBubble);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(requestBody);
    String payload = "{}";
    payload = http.getString();
    http.end();


    StaticJsonDocument<200> bubbleResponse;
    deserializeJson(bubbleResponse, payload);




    const char *wifiEnabledStr = (wifiControlEnable ? "true" : "false");
    LogDebugFormatted("Setting Wifi Control: %s\n%s\n%s\n", wifiEnabledStr, requestBody.c_str(), payload.c_str());
    LogDebugFormatted("HTTP Response code: %d\n", httpResponseCode);

    return true;
}

bool getInternetSettings()
{




    String serverNameBubble = "http://d2g4f7zewm360.cloudfront.net/ossm-get-settings";






    StaticJsonDocument<200> doc;
    doc["ossmId"] = ossmId;
    String requestBody;
    serializeJson(doc, requestBody);


    HTTPClient http;
    http.begin(serverNameBubble);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(requestBody);
    String payload = "{}";
    payload = http.getString();
    http.end();


    StaticJsonDocument<200> bubbleResponse;
    deserializeJson(bubbleResponse, payload);



    strokePercentage = bubbleResponse["response"]["stroke"];
    speedPercentage = bubbleResponse["response"]["speed"];


    LogDebug(payload);
    LogDebugFormatted("HTTP Response code: %d\n", httpResponseCode);

    return true;
}

void setLedRainbow(CRGB leds[])
{


    for (int hueShift = 0; hueShift < 350; hueShift++)
    {
        int gHue = hueShift % 255;
        fill_rainbow(leds, NUM_LEDS, gHue, 25);
        FastLED.show();
        delay(4);
    }
}