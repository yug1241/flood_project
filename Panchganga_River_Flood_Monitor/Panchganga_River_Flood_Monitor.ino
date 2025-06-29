// Panchganga River flood surveillance system for Kolhapur city.

#define TINY_GSM_MODEM_SIM7600
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>


// ---------- Pin Definitions ----------

#define WAKEUP_BUTTON_PIN     12  


// ====== WATER SENSORCONFIGURATION ======
#define WATER_SENSOR1_PIN             11  // lower sensor  // kami dhoka water level ajun khali
#define WATER_SENSOR2_PIN           10  // Upper sensor // jast dhoka water level avrti ali ata danger 


// ====== ULTRASONIC CONFIGURATION ======
#define trigpin    15
#define echopin    16

const float SPEED_OF_SOUND_CM_PER_US = 0.0343;          // cm/us
const float CM_TO_FEET = 0.0328084;                     // Conversion factor

                  
#define SENSOR_MIN_RAW_CM 19.0                          // Sensor’s minimum measurable raw distance in cm

// ====== DHT11 CONFIGURATION ======
#define DHTPIN                  14
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


// ====== SIM7670G CONFIGURATION ======
#define MODEM_RX              17
#define MODEM_TX              18
#define MODEM_BAUD            115200
HardwareSerial SerialAT(1); 


// ====== CLOCK CONFIGURATION ======
#define TIMEZONE_OFFSET_SEC 19800 // +5:30 India
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);


// ====== LCD CONFIGURATION ======
LiquidCrystal_I2C lcd(0x27, 20, 4);


// -------------------- TELEGRAM CONFIGURATION --------------------
#define TELEGRAM_TOKEN               "7956471159:AAFjOAlxjtbRS4Et97kLZiIKeBHCtVxRX_Q"
#define TELEGRAM_PERSONAL_CHAT_ID    "7049226007"   
#define TELEGRAM_GROUP_CHAT_ID       "-1002576027880"     // this group_chat_id i got from   https://api.telegram.org/bot<YOUR_BOT_TOKEN>/getUpdates  


// -------------------- CLOUD CONFIGURATION --------------------
#define CLOUD_API_KEY           "6FYVA0W67B8W2D88"
#define CLOUD_SERVER            "api.thingspeak.com"



// -------------------- THRESHOLDS & TIMING --------------------
#define DISTANCE_THRESHOLD_FT     2.50
//#define TELEGRAM_INTERVAL_MS    600000  //10 minute  // yala variable banvle mhnun delete kela
#define CLOUD_INTERVAL_MS         300000 //5 minute
#define MAX_RETRIES               3

// -------------------- MObile Number on which alert messages want to send --------------------
#define MOBILE_NUMBER             "+917385340111"


// -------------------- NETWORK CONFIGURATION --------------------
#define NETWORK_APN             "internet"


// ---------- Sleep Durations ----------

#define AUTO_SLEEP_MINUTES        50
#define AUTO_SLEEP_TIMEOUT_MS     (AUTO_SLEEP_MINUTES * 60 * 1000UL)
#define AUTO_SLEEP_WAKEUP_MINUTES 2
#define AUTO_SLEEP_WAKEUP_USEC    (AUTO_SLEEP_WAKEUP_MINUTES * 60 * 1000000ULL)

// ---------- Globals ----------
bool buttonState = HIGH;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool isSleepCommandTriggered = false;
unsigned long bootTimeMillis = 0;
bool floodAlertMode = false;  // Set true when woken up by water sensor
unsigned long lastTelegramUpdate = 0;
unsigned long lastCloudUpdate = 0;
unsigned long lastDistanceCheck = 0;
unsigned long currentMillis = 0;

//tegram variable tym
unsigned long telegramIntervalMs = 1800000;  // default 30 min  
float previousDistance = 0.0;
unsigned long lastRateCheckMillis = 0;



// ---------- Function Declaration or Prototypes ----------
void initializeSerial();
void initializeWaterSensors();
void initializeModem();
bool checkSIMCard();
void configureSMSMode();
void sendSMS(String number, String text);
void enterDeepSleep(unsigned long sleepDurationUsec, bool isCommandTriggered);
void checkSleepTimer();
void printWakeupReason();
void configureWakeupButton();
void configureWakeupWaterSensors();
void checkButtonWakeLogic();
void configureWakeupSources();  
void initSIMModule();
String getTimeFromNTP();  
float getDistanceFeet(bool &errorFlag);
void sendTelegramsensordata(float distance, float tempC, float humidity);
void uploadToCloud(float distance);
bool executeATCommand(const String& command);
void printATCommand(const String& command);
void printSuccess(const String& message);
void printError(const String& message);
int extractHTTPResponseCode();
bool readTempHumidity(float &tempC, float &humidity);  
void adjustTelegramInterval(float distance);




void checkSuddenWaterChange(float currentDistance) {
  static float lastCheckedDistance = 0;
  static unsigned long lastCheckedTime = 0;
  unsigned long now = millis();

  if (now - lastCheckedTime >= 2000) {  //  every 2 seconds la check karnar
    float change = abs(currentDistance - lastCheckedDistance);
    if (change > 0.5)
    {  
         String alertMsg = "Sudden change detected in water level: " + String(currentDistance, 2) + " ft";
         sendSMS(MOBILE_NUMBER, alertMsg);
    }
    lastCheckedDistance = currentDistance;
    lastCheckedTime = now;
  }
}


void adjustTelegramInterval(float distance) {
  if (distance <= 3.00) {
     telegramIntervalMs = 180000; // 3 min // ikde sarvat akmi tym pahije karan he case mahnje flood ala water level cross threshold 
  //  telegramIntervalMs = 2000; // 2sec
     
  } else if (distance > 3 && distance >= DISTANCE_THRESHOLD_FT) {
   telegramIntervalMs = 900000; // 15 min
     //telegramIntervalMs = 60000; // 1 min 
  } else { // ikde water bharpoor dur ahe
  
    telegramIntervalMs = 1800000; // 30 min
      //telegramIntervalMs = 120000; // 2 min for testing
  }
}


// ---------- Setup ----------
void setup() {
  
   lcd.init();
   lcd.backlight();
   lcd.setCursor(0, 0);
   lcd.print("WELCOME TO IEEE.....");
   lcd.setCursor(0, 1);
   lcd.print("TECH4 GOOD PROJECT");
   lcd.setCursor(0, 2);
   lcd.print("Developed By........");
   lcd.setCursor(0, 3);
  lcd.print("DOLPHIN LABS PVT LTD");
  initializeSerial();
  printWakeupReason();
  initializeWaterSensors();
  configureWakeupButton();
  configureWakeupWaterSensors();
  initializeModem();

  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);

  dht.begin();
  

  bootTimeMillis = millis();
  isSleepCommandTriggered = false;

  Serial.println("ESP32 Woke up.");
  sendSMS(MOBILE_NUMBER, "ESP32 has Woken Up - Dolphin Labs");

  if (!checkSIMCard()) {
    Serial.println("No SIM card detected. Halting.");
    while (true);
  }

  configureSMSMode();
  sendSMS(MOBILE_NUMBER, "ESP32 is Ready. Send commands like STATUS, LED ON, LED OFF, DEEP SLEEP or SLEEP or S.");
  lcd.clear();
}

void loop() {
  currentMillis = millis();

  checkButtonWakeLogic();
  checkSleepTimer();

  static bool alertSent = false;
  static unsigned long lastSensorRead = 0;
  static unsigned long lastNTPUpdate = 0;
  static String cachedTime = "--:--:--";
  static float lastDistance = 0;
  static float lastTempC = 0;
  static float lastHumidity = 0;


  if (currentMillis - lastNTPUpdate >= 30000) { // 30 sec
    cachedTime = getTimeFromNTP();  
    lastNTPUpdate = currentMillis;
  }


  lcd.setCursor(0, 0);
  lcd.print("DT:");
  lcd.print(cachedTime);

  if (currentMillis - lastSensorRead >= 2000) { // 2sec
    lastSensorRead = currentMillis;

    bool error = false;
    lastDistance = getDistanceFeet(error);

    bool dhtStatus = readTempHumidity(lastTempC, lastHumidity);

    lcd.setCursor(0, 1);
    if (error) {
      lcd.print("NO Echo Check Pins ");
      sendSMS(MOBILE_NUMBER,"NO Echo Check Pins");
    } else {
      lcd.print("LEVEL = ");
      lcd.print(lastDistance, 2);
      lcd.print(" ft   ");
    }

    lcd.setCursor(0, 2);
    if (!dhtStatus) {
      lcd.print("DHT11 Error        ");
      sendSMS(MOBILE_NUMBER,"DHT11 Failed to read from readings");
    } else {
      lcd.print("TEMP = ");
      lcd.print(lastTempC);
      lcd.print(" C   ");
    }

    lcd.setCursor(0, 3);
    if (dhtStatus) {
      lcd.print("HUMD = ");
      lcd.print(lastHumidity);
      lcd.print(" %   ");
    } else {
      lcd.print("                  "); 
    }

    Serial.printf("📏 Distance: %.2f ft\n", lastDistance);

 
    if (lastDistance < DISTANCE_THRESHOLD_FT && !alertSent) 
     {
      String alertMsg = "High Alert! Water Level Crossed Threshold Value: " + String(lastDistance, 2) + " ft";
        sendSMS(MOBILE_NUMBER, alertMsg);
       
        alertSent = true;
    } else if (lastDistance <= DISTANCE_THRESHOLD_FT) {
      alertSent = false;
    }
  }

  // Telegram update
     adjustTelegramInterval(lastDistance);
    checkSuddenWaterChange(lastDistance);

  if (currentMillis - lastTelegramUpdate >= telegramIntervalMs) 
  {
    sendTelegramsensordata(lastDistance, lastTempC, lastHumidity);
    lastTelegramUpdate = currentMillis;
  }

  // Cloud update
  if (currentMillis - lastCloudUpdate >= CLOUD_INTERVAL_MS) {
    uploadToCloud(lastDistance);
    lastCloudUpdate = currentMillis;
  }
}



// ---------- Initialization ----------
void initializeSerial() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 ESP32 Booting...");
}



void initializeWaterSensors() {
  pinMode(WATER_SENSOR1_PIN, INPUT);
  pinMode(WATER_SENSOR2_PIN, INPUT);
}

void configureWakeupButton() {
  pinMode(WAKEUP_BUTTON_PIN, INPUT);
  rtc_gpio_deinit((gpio_num_t)WAKEUP_BUTTON_PIN);
  rtc_gpio_init((gpio_num_t)WAKEUP_BUTTON_PIN);
  rtc_gpio_set_direction((gpio_num_t)WAKEUP_BUTTON_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en((gpio_num_t)WAKEUP_BUTTON_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)WAKEUP_BUTTON_PIN);
}

void configureWakeupWaterSensors() {
  gpio_num_t sensors[] = { (gpio_num_t)WATER_SENSOR1_PIN, (gpio_num_t)WATER_SENSOR2_PIN };
  for (int i = 0; i < 2; ++i) {
    rtc_gpio_deinit(sensors[i]);
    rtc_gpio_init(sensors[i]);
    rtc_gpio_set_direction(sensors[i], RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(sensors[i]);
    rtc_gpio_pulldown_dis(sensors[i]);
    rtc_gpio_hold_en(sensors[i]);
  }
}

void configureWakeupSources() {
  
  configureWakeupButton();
  configureWakeupWaterSensors();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_BUTTON_PIN, 0);
  esp_sleep_enable_ext1_wakeup((1ULL << WATER_SENSOR1_PIN) | (1ULL << WATER_SENSOR2_PIN), ESP_EXT1_WAKEUP_ANY_HIGH);
}


void initializeModem() {
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  Serial.println("Initializing SIM Module...");
  initSIMModule();
}

bool checkSIMCard() {
  SerialAT.println("AT+CPIN?");
  delay(1000);
  return SerialAT.find("READY");
}

void configureSMSMode() {
  SerialAT.println("AT+CMGF=1");
  delay(500);
  SerialAT.println("AT+CNMI=1,2,0,0,0");
  delay(500);
}

// ---------- Core Functionalities ----------
void sendSMS(String number, String text) {
  Serial.println("Sending SMS...");
  SerialAT.println("AT+CMGF=1");
  delay(50);
  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(number);
  SerialAT.println("\"");
  delay(50);
  SerialAT.print(text);
  SerialAT.write(26);
 

  unsigned long startMillis = millis();
while (millis() - startMillis < 5000) {
 
  yield();  
}

  Serial.println("SMS Sent.");
}




void enterDeepSleep(unsigned long sleepDurationUsec, bool isCommandTriggered) {
  Serial.println("Entering Deep Sleep...");
  configureWakeupSources(); 

  esp_sleep_enable_timer_wakeup(sleepDurationUsec);
  delay(500);
  esp_deep_sleep_start();
}

void checkSleepTimer() {
if (floodAlertMode)
  {
    return;  
  }


  if (!isSleepCommandTriggered && (millis() - bootTimeMillis >= AUTO_SLEEP_TIMEOUT_MS))
   {
    Serial.println("Auto-sleep after " + String(AUTO_SLEEP_MINUTES) + " minutes.");
    sendSMS(MOBILE_NUMBER, "ESP32 going to sleep after inactivity.");
    enterDeepSleep(AUTO_SLEEP_WAKEUP_USEC, false);
  }
}

void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by button (ext0).\n");
      sendSMS(MOBILE_NUMBER,"Wakeup caused by PUSH button");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by water level sensor (ext1).\n");
          floodAlertMode = true;  


      if (digitalRead(WATER_SENSOR1_PIN) == HIGH)
       {
        sendSMS(MOBILE_NUMBER, "ALERT: Water level reached at Sensor 1.");
       }

      if (digitalRead(WATER_SENSOR2_PIN) == HIGH) {
        sendSMS(MOBILE_NUMBER, "HIGH ALERT: Water level reached at Sensor 2!");
      }

      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer.\n");
      break;
    default:
      Serial.println("Normal boot.\n");
      break;
  }
}

void checkButtonWakeLogic() {
  int reading = digitalRead(WAKEUP_BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW)
       {
        Serial.println("Button Press Detected: Wakeup");
      
      }
    }
  }

  lastButtonState = reading;

}


// Function definitions

String getTimeFromNTP() {
  modem.sendAT("+CNTP=\"pool.ntp.org\",0");
  if (!modem.waitResponse(5000L, "OK")) {
    Serial.println("Failed to set NTP server");
    return "NTP SET ERR";
  }

  
  modem.sendAT("+CNTP");
  if (!modem.waitResponse(10000L, "OK")) {
    Serial.println("Failed to sync time");
    return "NTP SYNC ERR";
  }

  delay(2000);


  modem.sendAT("+CCLK?");
  if (modem.waitResponse(5000L, "+CCLK:")) {
    String timeString = modem.stream.readStringUntil('\n');
    timeString.trim();

    int q1 = timeString.indexOf('"');
    int q2 = timeString.indexOf('"', q1 + 1);

    if (q1 != -1 && q2 != -1) {
      String dateTime = timeString.substring(q1 + 1, q2); 
      dateTime.replace(",", " ");

    
      int yy = dateTime.substring(0, 2).toInt();
      int MM = dateTime.substring(3, 5).toInt();
      int dd = dateTime.substring(6, 8).toInt();
      int hh = dateTime.substring(9, 11).toInt();
      int mm = dateTime.substring(12, 14).toInt();
      int ss = dateTime.substring(15, 17).toInt();

   
      mm += 30;
      hh += 5;
      if (mm >= 60) {
        mm -= 60;
        hh += 1;
      }
      if (hh >= 24) {
        hh -= 24;
        dd += 1; 
      }

    
      char buffer[25];
      sprintf(buffer, "%02d/%02d/%02d %02d:%02d:%02d", dd, MM, yy, hh, mm, ss);
      String istTime = String(buffer);
      return istTime;
    }
  }

  Serial.println("Failed to read time");
  return "TIME RD ERR";
}



float getDistanceFeet(bool &errorFlag) {
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  long duration = pulseIn(echopin, HIGH, 30000);  

  if (duration == 0) {
    errorFlag = true;
    return 0.0;   
  }

  errorFlag = false;

  float measuredDistanceCm = (duration * SPEED_OF_SOUND_CM_PER_US) / 2.0;

  if (measuredDistanceCm <= SENSOR_MIN_RAW_CM) {
    return 0.0;
  }

  float measuredDistanceFeet = measuredDistanceCm * CM_TO_FEET;

  return measuredDistanceFeet;
}

bool readTempHumidity(float &tempC, float &humidity) {
  tempC = dht.readTemperature();
  humidity = dht.readHumidity();

  if (isnan(tempC) || isnan(humidity)) {
    Serial.println("Failed to read from DHT11!");
    return false;
  }
  return true;
}


// -------------------- SEND MESSAGE TO TELEGRAM --------------------

void sendTelegramsensordata(float distance, float tempC, float humidity) {
  String message =  "📏 WATER_LEVEL : " + String(distance, 2) + " ft%0A"
                    "🌡 TEMPERATURE : " + String(tempC, 2) + "°C%0A"
                    " 💧 HUMIDITY : " + String(humidity, 2) + "%";
              

  message.replace(" ", "%20");
  Serial.println("Sending data to Telegram...");

    String chatIds[] = { TELEGRAM_PERSONAL_CHAT_ID, TELEGRAM_GROUP_CHAT_ID };

  for (int i = 0; i < 2; i++) {
     String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) + "/sendMessage?chat_id=" + chatIds[i] + "&text=" + message;
      


for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
      Serial.println("📨 Sending to Telegram (Attempt " + String(attempt) + ")");
      executeATCommand("AT+HTTPTERM");
      executeATCommand("AT+HTTPINIT");
      executeATCommand("AT+HTTPPARA=\"CID\",1");
      executeATCommand("AT+HTTPSSL=1");
      executeATCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"");
      executeATCommand("AT+HTTPACTION=0");
      delay(500);
      int responseCode = extractHTTPResponseCode();

      if (responseCode == 200) {
        printSuccess("Telegram message sent to chat ID: " + chatIds[i]);
        executeATCommand("AT+HTTPTERM");
        break;
      } else {
        printError("Telegram HTTP error to chat ID: " + chatIds[i] + " code: " + String(responseCode));
      }
    }
  }
}

// -------------------- SEND DATA TO CLOUD --------------------
void uploadToCloud(float distance) {
  String url = "http://" + String(CLOUD_SERVER) + "/update?api_key=" + CLOUD_API_KEY + "&field1=" + String(distance, 2);

  for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
    Serial.println("Sending to ThingSpeak (Attempt " + String(attempt) + ")");
    executeATCommand("AT+HTTPTERM");
    executeATCommand("AT+HTTPINIT");
    executeATCommand("AT+HTTPPARA=\"CID\",1");
    executeATCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"");
    executeATCommand("AT+HTTPACTION=0");

    delay(500);
    int responseCode = extractHTTPResponseCode();

    if (responseCode == 200) {
      printSuccess("Cloud update success");
      executeATCommand("AT+HTTPTERM");
      return;
    } else {
      printError("HTTP error code: " + String(responseCode));
    }
  }
}

bool executeATCommand(const String& command) {
  printATCommand(command);
SerialAT.println(command);

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialAT.available()) {
      String response = SerialAT.readStringUntil('\n');
      response.trim();
      if (response.length()) {
        if (response.indexOf("OK") >= 0 || response.indexOf("+HTTPACTION") >= 0) {
          printSuccess(response);
          return true;
        } else if (response.indexOf("ERROR") >= 0) {
          printError(response);
          return false;
        }
      }
    }
  }
  return false;
}



void printATCommand(const String& command) {
  Serial.println("🛠️ AT: " + command);
}

void printSuccess(const String& message) {
  Serial.println("✅ " + message);
}

void printError(const String& message) {
  Serial.println("❌ " + message);
}

// -------------------- EXTRACT HTTP RESPONSE CODE --------------------
int extractHTTPResponseCode() {
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (SerialAT.available()) {
      String line = SerialAT.readStringUntil('\n');
      line.trim();
      if (line.startsWith("+HTTPACTION:")) {
        int i1 = line.indexOf(',');
        int i2 = line.indexOf(',', i1 + 1);
        if (i1 > 0 && i2 > i1) {
          return line.substring(i1 + 1, i2).toInt();
        }
      }
    }
  }
  return -1;
}

// -------------------- INITIALIZE SIM MODULE --------------------
void initSIMModule() {
  executeATCommand("AT");
  delay(500);
  executeATCommand("ATE0");
  delay(500);
  executeATCommand("AT+CFUN=1");
  delay(500);
  executeATCommand("AT+CPIN?");
  delay(500);
  executeATCommand("AT+CSQ");
  delay(500);
  executeATCommand("AT+CREG?");
  delay(500);
  executeATCommand("AT+CGATT=1");
  delay(500);
  executeATCommand("AT+CGDCONT=1,\"IP\",\"" NETWORK_APN "\"");
  delay(500);
  executeATCommand("AT+NETOPEN");
  delay(3000);
  executeATCommand("AT+IPADDR");
  delay(500);
}

 

