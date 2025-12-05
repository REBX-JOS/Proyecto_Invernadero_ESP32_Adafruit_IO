/*
  medidor_invernadero_esp32_iotcloud (main.cpp)
  Versión con fix: usa WiFiConnectionHandler en lugar de ConnectionHandler abstracto.
  - Borra src/secrets.h y src/thingProperties.h si usas este archivo incrustado.
  - Reemplaza SECRET_SSID / SECRET_OPTIONAL_PASS / SECRET_DEVICE_KEY por tus valores reales.
*/


// Librerías de Cloud / ConnectionHandler primero
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <WiFiConnectionHandler.h> // <-- incluir el handler concreto

// Core / red y servidores
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// Incluir Keypad AL FINAL de las cabeceras que no deben verse afectadas
#include <Keypad.h>

// Evitar conflictos por macros definidas en Keypad.h (p. ej. CLOSED)
#ifdef CLOSED
  #undef CLOSED
#endif
#ifdef OPEN
  #undef OPEN
#endif
#ifdef PRESSED
  #undef PRESSED
#endif
#ifdef RELEASED
  #undef RELEASED
#endif

#include <ArduinoJson.h>

// ----------------- SECRETS / PROPIEDADES INCRUSTADAS -----------------
#define SECRET_SSID "TU_SSID7"
#define SECRET_OPTIONAL_PASS "TU_PASSWORD"
#define SECRET_DEVICE_KEY "lgZmCl3O7@1Ir#4JZT@M@6tjP" // Device Secret (Device Key)
#define DEVICE_LOGIN_NAME "acf26117-60ef-4f66-ac88-6bf4a2c4edd1"
#define THING_ID "10a75ce2-993a-41b2-962c-4af45d904417"

// ----------------- CONFIG -----------------
const char WIFI_SSID[] = SECRET_SSID;
const char WIFI_PASS[] = SECRET_OPTIONAL_PASS;

// ----------------- HARDWARE -----------------
#define PIN_LM35 34
#define PIN_SOIL 35
#define PIN_DHT  15

#define RELAY_FOCUS  26
#define RELAY_FAN    27
#define RELAY_PUMP   25

// I2C LCD
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

// Keypad
const byte ROWS = 4;
const byte COLS = 4;
byte rowPins[ROWS] = {32,33,14,12};
byte colPins[COLS] = {13,18,19,23};
char keys[ROWS][COLS] = { {'1','2','3','A'}, {'4','5','6','B'}, {'7','8','9','C'}, {'*','0','#','D'} };
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);

// ----------------- VARIABLES -----------------
float temp_LM35 = 0.0;
float temp_amb = 0.0;
float hum_amb = 0.0;
int soil_adc = 0;
int foco = 0;
int vent = 0;
int bomba = 0;
int modo_auto = 1; // 1 = AUTO por defecto

// Callbacks forward declarations
void onFocoChange();
void onVentChange();
void onBombaChange();
void onModoAutoChange();

// Inicialización de propiedades
void initProperties() {
  // Ajusta si tu versión de la librería requiere setThingId / setDeviceId.
  // ArduinoCloud.setThingId(THING_ID);
  // ArduinoCloud.setDeviceId(DEVICE_LOGIN_NAME);

  ArduinoCloud.addProperty(temp_LM35, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(temp_amb, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(hum_amb, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(soil_adc, READ, 1 * SECONDS, NULL);

  ArduinoCloud.addProperty(foco, READWRITE, ON_CHANGE, onFocoChange);
  ArduinoCloud.addProperty(vent, READWRITE, ON_CHANGE, onVentChange);
  ArduinoCloud.addProperty(bomba, READWRITE, ON_CHANGE, onBombaChange);
  ArduinoCloud.addProperty(modo_auto, READWRITE, ON_CHANGE, onModoAutoChange);
}

// ----------------- Connection handler CONCRETO -----------------
// Usar la clase WiFiConnectionHandler (provista por Arduino_ConnectionHandler)
WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_SSID, SECRET_OPTIONAL_PASS);

// ----------------- Resto del programa -----------------
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool foco_on = false;
volatile bool fan_on = false;
volatile bool pump_on = false;

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

float readLM35_C();
int readSoilADC();
void updateActuatorStateLocal(const char* device, bool state);
String getSystemJSON();
void notifyAllClients();
void imprimirEstadoLCD();

void onFocoChange() { updateActuatorStateLocal("foco", foco == 1); }
void onVentChange() { updateActuatorStateLocal("vent", vent == 1); }
void onBombaChange() { updateActuatorStateLocal("bomba", bomba == 1); }
void onModoAutoChange() { Serial.printf("Modo cambiado desde Cloud -> %s\n", modo_auto ? "AUTO":"MAN"); }

// Tareas (idénticas a tu implementación)
void sensorsTask(void* pvParameters) {
  (void)pvParameters;
  while (1) {
    float t = readLM35_C();
    float h = dht.readHumidity();
    float ta = dht.readTemperature();
    int soil = readSoilADC();

    portENTER_CRITICAL(&mux);
    temp_LM35 = t;
    if (!isnan(h)) hum_amb = h;
    if (!isnan(ta)) temp_amb = ta;
    soil_adc = soil;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void controlTask(void* pvParameters) {
  (void)pvParameters;
  while (1) {
    portENTER_CRITICAL(&mux);
    float temp = temp_LM35;
    int soil = soil_adc;
    bool autoMode = (modo_auto == 1);
    portEXIT_CRITICAL(&mux);

    if (autoMode) {
      if (temp < (20.0 - 0.5)) { updateActuatorStateLocal("foco", true); updateActuatorStateLocal("vent", false); }
      else if (temp > (24.0 + 0.5)) { updateActuatorStateLocal("vent", true); updateActuatorStateLocal("foco", false); }
      else { updateActuatorStateLocal("vent", false); updateActuatorStateLocal("foco", false); }
      if (soil <= 400) updateActuatorStateLocal("bomba", true);
      else if (soil >= 2000) updateActuatorStateLocal("bomba", false);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void webTask(void* pvParameters) {
  (void)pvParameters;
  unsigned long lastBroadcast = 0;
  const unsigned long interval = 1000;
  while (1) {
    server.handleClient();
    webSocket.loop();
    unsigned long now = millis();
    if (now - lastBroadcast >= interval) { notifyAllClients(); lastBroadcast = now; }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void lcdKeypadTask(void* pvParameters) {
  (void)pvParameters;
  unsigned long lastUpdate = 0;
  while (1) {
    char key = keypad.getKey();
    if (key) {
      if (key == 'A') {
        portENTER_CRITICAL(&mux);
        modo_auto = !modo_auto;
        portEXIT_CRITICAL(&mux);
        Serial.printf("Modo %s\n", modo_auto ? "AUTO":"MAN");
      } else if (key == 'B' && !modo_auto) {
        portENTER_CRITICAL(&mux);
        foco = foco ? 0 : 1;
        portEXIT_CRITICAL(&mux);
        onFocoChange();
      } else if (key == 'C' && !modo_auto) {
        portENTER_CRITICAL(&mux);
        vent = vent ? 0 : 1;
        portEXIT_CRITICAL(&mux);
        onVentChange();
      } else if (key == 'D' && !modo_auto) {
        portENTER_CRITICAL(&mux);
        bomba = bomba ? 0 : 1;
        portEXIT_CRITICAL(&mux);
        onBombaChange();
      }
    }
    if (millis() - lastUpdate >= 500) { imprimirEstadoLCD(); lastUpdate = millis(); }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void cloudTask(void* pvParameters) {
  (void)pvParameters;
  while (1) {
    ArduinoCloud.update();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(RELAY_FOCUS, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);
  digitalWrite(RELAY_FOCUS, LOW);
  digitalWrite(RELAY_FAN, LOW);
  digitalWrite(RELAY_PUMP, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Invernadero IoT");

  dht.begin();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LM35, ADC_11db);
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);

  // Inicializar propiedades e iniciar conexión con Arduino IoT Cloud
  initProperties();

  // ArduinoCloud.begin acepta un ConnectionHandler (aquí: WiFiConnectionHandler)
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // REST / WS
  server.on("/", [](){ server.send(200, "text/html", "<h1>ESP32</h1>"); });
  server.on("/api/data", HTTP_GET, [](){ server.send(200, "application/json", getSystemJSON()); });
  server.begin();

  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length){
    if (type == WStype_TEXT) Serial.printf("WS from %u: %s\n", num, payload);
  });

  // Crear tareas FreeRTOS
  xTaskCreatePinnedToCore(sensorsTask, "Sensors", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(controlTask, "Control", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(webTask, "Web", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(lcdKeypadTask, "LCDKey", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(cloudTask, "Cloud", 4096, NULL, 1, NULL, 1);

  lcd.clear();
  lcd.print("Sistema listo");
  delay(800);
}

float readLM35_C() {
  const int N = 8; long sum = 0;
  for (int i=0;i<N;i++){ sum += analogRead(PIN_LM35); delay(2); }
  float avg = (float)sum / N;
  float voltage = (avg / 4095.0) * 3.3;
  return voltage / 0.01;
}

int readSoilADC() {
  const int N = 6; long sum = 0;
  for (int i=0;i<N;i++){ sum += analogRead(PIN_SOIL); delay(2); }
  return sum / N;
}

void updateActuatorStateLocal(const char* device, bool state) {
  portENTER_CRITICAL(&mux);
  if (strcmp(device, "foco")==0) {
    foco_on = state;
    foco = state ? 1 : 0;
    digitalWrite(RELAY_FOCUS, state ? HIGH : LOW);
  } else if (strcmp(device, "vent")==0) {
    fan_on = state;
    vent = state ? 1 : 0;
    digitalWrite(RELAY_FAN, state ? HIGH : LOW);
  } else if (strcmp(device, "bomba")==0) {
    pump_on = state;
    bomba = state ? 1 : 0;
    digitalWrite(RELAY_PUMP, state ? HIGH : LOW);
  }
  portEXIT_CRITICAL(&mux);
  Serial.printf("Actuador %s -> %s\n", device, state ? "ON":"OFF");
}

String getSystemJSON() {
  portENTER_CRITICAL(&mux);
  float t = temp_LM35;
  float ta = temp_amb;
  float h = hum_amb;
  int soil = soil_adc;
  bool f = foco_on;
  bool v = fan_on;
  bool p = pump_on;
  bool m = (modo_auto == 1);
  portEXIT_CRITICAL(&mux);

  StaticJsonDocument<256> doc;
  doc["temp_LM35"] = t;
  doc["temp_amb"] = ta;
  doc["hum_amb"] = h;
  doc["soil_adc"] = soil;
  doc["foco"] = f ? 1 : 0;
  doc["vent"] = v ? 1 : 0;
  doc["bomba"] = p ? 1 : 0;
  doc["modo_auto"] = m ? 1 : 0;

  String out;
  serializeJson(doc, out);
  return out;
}

void notifyAllClients() {
  String payload = getSystemJSON();
  webSocket.broadcastTXT(payload);
}

void imprimirEstadoLCD() {
  portENTER_CRITICAL(&mux);
  float t = temp_LM35; float ta = temp_amb; float h = hum_amb; int soil = soil_adc;
  bool f = foco_on; bool v = fan_on; bool p = pump_on; bool m = (modo_auto==1);
  portEXIT_CRITICAL(&mux);

  char buf[21];
  lcd.clear();
  snprintf(buf, sizeof(buf), "T:%.1f A:%.1f", t, ta); lcd.setCursor(0,0); lcd.print(buf);
  snprintf(buf, sizeof(buf), "H:%.0f Soil:%4d", h, soil); lcd.setCursor(0,1); lcd.print(buf);
  snprintf(buf, sizeof(buf), "Modo:%s F:%d V:%d", m? "AUTO":"MAN", f?1:0, v?1:0); lcd.setCursor(0,2); lcd.print(buf);
  lcd.setCursor(0,3); lcd.print("Bomba:"); lcd.print(p? "ON":"OFF");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}