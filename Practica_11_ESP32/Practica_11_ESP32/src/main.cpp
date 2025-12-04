/*
  medidor_invernadero_esp32_adafruitio
  Adaptado para Adafruit IO Cloud
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <ArduinoJson.h>

// ==================== CONFIGURACIÓN ADAFRUIT IO ====================
// ---- WiFi Credentials ----
const char* WIFI_SSID = "TU_SSID7";
const char* WIFI_PASSWORD = "TU_PASSWORD";

// ---- Adafruit IO Config ----
const char* AIO_SERVER = "io.adafruit.com";      // Servidor de Adafruit IO
const int AIO_PORT = 1883;                       // Puerto MQTT (1883 sin SSL)
// Si quieres SSL, usa puerto 8883 y WiFiClientSecure
const char* AIO_USERNAME = "a22300913";           // Tu username de Adafruit IO
const char* AIO_KEY = "aio_Wzne23dFqt06K9VlBP4UreFSlbf6"; // Tu Active Key

// ---- Adafruit IO Feeds (Topics) ----
// Formato: username/feeds/feed_name
const char* FEED_TEMP_LM35 = "a22300913/feeds/temp-lm35";
const char* FEED_TEMP_AMB = "a22300913/feeds/temp-amb";
const char* FEED_HUM_AMB = "a22300913/feeds/hum-amb";
const char* FEED_SOIL = "a22300913/feeds/soil-moisture";
const char* FEED_FOCO = "a22300913/feeds/foco";
const char* FEED_VENT = "a22300913/feeds/ventilador";
const char* FEED_BOMBA = "a22300913/feeds/bomba";
const char* FEED_MODO = "a22300913/feeds/modo-auto";

// Topics para suscribirse (control)
const char* SUB_FOCO = "a22300913/feeds/foco";
const char* SUB_VENT = "a22300913/feeds/ventilador";
const char* SUB_BOMBA = "a22300913/feeds/bomba";

// ==================== CONFIGURACIÓN DE HARDWARE ====================
// Pines
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
byte rowPins[ROWS] = {32, 33, 14, 12};
byte colPins[COLS] = {13, 18, 19, 23};
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// DHT
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);

// ==================== VARIABLES GLOBALES ====================
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile float measured_temp = 0.0;
volatile float measured_temp_amb = 0.0;
volatile float measured_hum = 0.0;
volatile int measured_soil = 0;

volatile bool foco_on = false;
volatile bool fan_on = false;
volatile bool pump_on = false;
volatile bool modo_automatico = true;

volatile unsigned long foco_on_ts = 0;
volatile unsigned long fan_on_ts = 0;
volatile unsigned long pump_on_ts = 0;

// Umbrales para control automático
float temp_min = 20.0;
float temp_max = 24.0;
float temp_hysteresis = 0.5;
int soil_threshold_low = 400;
int soil_threshold_high = 2000;

// ==================== OBJETOS NETWORK ====================
WiFiClient wifiClient;  // Para puerto 1883 (sin SSL)
// Para SSL (puerto 8883), usa:
// WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// ==================== PROTOTIPOS DE FUNCIONES ====================
void sensorsTask(void* pvParameters);
void controlTask(void* pvParameters);
void webTask(void* pvParameters);
void lcdKeypadTask(void* pvParameters);
void cloudTask(void* pvParameters);

float readLM35_C();
int readSoilADC();
void updateActuatorState(String device, bool state);
String getSystemJSON();
void notifyAllClients();
void imprimirEstadoLCD();

// Funciones Adafruit IO
void aioConnect();
void aioCallback(char* topic, byte* payload, unsigned int length);
void publishToAIO();
bool aioPublish(const char* feed, const char* value);
bool aioPublish(const char* feed, float value);
bool aioPublish(const char* feed, int value);
bool aioPublish(const char* feed, bool value);

// ==================== FUNCIONES AUXILIARES ====================
float readLM35_C() {
  int raw = analogRead(PIN_LM35);
  float voltage = raw * 3.3 / 4095.0;
  return voltage * 100.0;  // LM35: 10mV por °C
}

int readSoilADC() {
  return analogRead(PIN_SOIL);  // 0-4095
}

void updateActuatorState(String device, bool state) {
  portENTER_CRITICAL(&mux);
  if (device == "foco") {
    foco_on = state;
    // INVERTIR: HIGH = APAGADO, LOW = ENCENDIDO
    digitalWrite(RELAY_FOCUS, state ? LOW : HIGH);
    if (state) foco_on_ts = millis();
  } else if (device == "vent") {
    fan_on = state;
    // INVERTIR: HIGH = APAGADO, LOW = ENCENDIDO
    digitalWrite(RELAY_FAN, state ? LOW : HIGH);
    if (state) fan_on_ts = millis();
  } else if (device == "bomba") {
    pump_on = state;
    // INVERTIR: HIGH = APAGADO, LOW = ENCENDIDO
    digitalWrite(RELAY_PUMP, state ? LOW : HIGH);
    if (state) pump_on_ts = millis();
  }
  portEXIT_CRITICAL(&mux);
  Serial.printf("Actuador %s -> %s\n", device.c_str(), state ? "ON" : "OFF");
  
  // Publicar cambio de estado en Adafruit IO
  if (mqttClient.connected()) {
    if (device == "foco") aioPublish(FEED_FOCO, state);
    else if (device == "vent") aioPublish(FEED_VENT, state);
    else if (device == "bomba") aioPublish(FEED_BOMBA, state);
  }
}

String getSystemJSON() {
  portENTER_CRITICAL(&mux);
  float t = measured_temp;
  float ta = measured_temp_amb;
  float h = measured_hum;
  int soil = measured_soil;
  bool f = foco_on;
  bool v = fan_on;
  bool p = pump_on;
  bool m = modo_automatico;
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
  float t = measured_temp;
  float ta = measured_temp_amb;
  float h = measured_hum;
  int soil = measured_soil;
  bool f = foco_on;
  bool v = fan_on;
  bool p = pump_on;
  bool m = modo_automatico;
  portEXIT_CRITICAL(&mux);

  char buf[21];
  lcd.clear();
  snprintf(buf, sizeof(buf), "T:%.1f A:%.1f", t, ta);
  lcd.setCursor(0, 0);
  lcd.print(buf);
  snprintf(buf, sizeof(buf), "H:%.0f Soil:%4d", h, soil);
  lcd.setCursor(0, 1);
  lcd.print(buf);
  snprintf(buf, sizeof(buf), "Modo:%s F:%d V:%d", m ? "AUTO" : "MAN", f ? 1 : 0, v ? 1 : 0);
  lcd.setCursor(0, 2);
  lcd.print(buf);
  lcd.setCursor(0, 3);
  lcd.print("Bomba:");
  lcd.print(p ? "ON" : "OFF");
}

// ==================== FUNCIONES ADAFRUIT IO ====================
void aioCallback(char* topic, byte* payload, unsigned int length) {
  // Convertir payload a string
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.printf("Adafruit IO [%s]: %s\n", topic, message);
  
  // Determinar qué feed es
  String topicStr = String(topic);
  String valueStr = String(message);
  
  // Convertir valor a booleano (Adafruit IO envía "ON"/"OFF" o "1"/"0")
  bool state = (valueStr == "1" || valueStr == "ON" || valueStr == "true");
  
  // Actualizar actuador correspondiente
  if (topicStr.endsWith("/foco")) {
    updateActuatorState("foco", state);
  } else if (topicStr.endsWith("/ventilador")) {
    updateActuatorState("vent", state);
  } else if (topicStr.endsWith("/bomba")) {
    updateActuatorState("bomba", state);
  }
}

bool aioPublish(const char* feed, const char* value) {
  if (mqttClient.connected()) {
    bool success = mqttClient.publish(feed, value);
    if (success) {
      Serial.printf("Publicado en %s: %s\n", feed, value);
    } else {
      Serial.printf("Error publicando en %s\n", feed);
    }
    return success;
  }
  return false;
}

bool aioPublish(const char* feed, float value) {
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%.2f", value);
  return aioPublish(feed, buffer);
}

bool aioPublish(const char* feed, int value) {
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%d", value);
  return aioPublish(feed, buffer);
}

bool aioPublish(const char* feed, bool value) {
  return aioPublish(feed, value ? "1" : "0");
}

void publishToAIO() {
  // Obtener valores actuales
  portENTER_CRITICAL(&mux);
  float t = measured_temp;
  float ta = measured_temp_amb;
  float h = measured_hum;
  int soil = measured_soil;
  bool f = foco_on;
  bool v = fan_on;
  bool p = pump_on;
  bool m = modo_automatico;
  portEXIT_CRITICAL(&mux);
  
  // Publicar todos los datos
  if (mqttClient.connected()) {
    aioPublish(FEED_TEMP_LM35, t);
    delay(50);
    aioPublish(FEED_TEMP_AMB, ta);
    delay(50);
    aioPublish(FEED_HUM_AMB, h);
    delay(50);
    aioPublish(FEED_SOIL, soil);
    delay(50);
    aioPublish(FEED_FOCO, f);
    delay(50);
    aioPublish(FEED_VENT, v);
    delay(50);
    aioPublish(FEED_BOMBA, p);
    delay(50);
    aioPublish(FEED_MODO, m);
    
    Serial.println("✅ Datos enviados a Adafruit IO");
  }
}

void aioConnect() {
  Serial.println("=== Conectando a Adafruit IO ===");
  
  // Configurar servidor MQTT
  mqttClient.setServer(AIO_SERVER, AIO_PORT);
  Serial.printf("Servidor: %s:%d\n", AIO_SERVER, AIO_PORT);
  Serial.printf("Usuario: %s\n", AIO_USERNAME);
  
  // Crear Client ID único
  String clientId = "ESP32-Invernadero-" + String(random(0xffff), HEX);
  
  if (mqttClient.connect(clientId.c_str(), AIO_USERNAME, AIO_KEY)) {
    Serial.println("✅ Conectado a Adafruit IO!");
    
    // Suscribirse a feeds de control
    if (mqttClient.subscribe(SUB_FOCO)) {
      Serial.println("✅ Suscrito a: foco");
    }
    if (mqttClient.subscribe(SUB_VENT)) {
      Serial.println("✅ Suscrito a: ventilador");
    }
    if (mqttClient.subscribe(SUB_BOMBA)) {
      Serial.println("✅ Suscrito a: bomba");
    }
    
    Serial.println("Listo para enviar/recepcionar datos");
  } else {
    Serial.print("❌ Error de conexión Adafruit IO: ");
    int state = mqttClient.state();
    Serial.println(state);
    
    // Mostrar error específico
    switch(state) {
      case -4: Serial.println("MQTT_CONNECTION_TIMEOUT"); break;
      case -3: Serial.println("MQTT_CONNECTION_LOST"); break;
      case -2: Serial.println("MQTT_CONNECT_FAILED"); break;
      case -1: Serial.println("MQTT_DISCONNECTED"); break;
      case 1: Serial.println("MQTT_CONNECT_BAD_PROTOCOL"); break;
      case 2: Serial.println("MQTT_CONNECT_BAD_CLIENT_ID"); break;
      case 3: Serial.println("MQTT_CONNECT_UNAVAILABLE"); break;
      case 4: Serial.println("MQTT_CONNECT_BAD_CREDENTIALS"); break;
      case 5: Serial.println("MQTT_CONNECT_UNAUTHORIZED"); break;
    }
  }
}

// ==================== TAREAS FreeRTOS MODIFICADAS ====================
void sensorsTask(void* pvParameters) {
  (void)pvParameters;
  while (1) {
    float t = readLM35_C();
    float h = dht.readHumidity();
    float ta = dht.readTemperature();
    int soil = readSoilADC();
    
    portENTER_CRITICAL(&mux);
    measured_temp = t;
    if (!isnan(h)) measured_hum = h;
    if (!isnan(ta)) measured_temp_amb = ta;
    measured_soil = soil;
    portEXIT_CRITICAL(&mux);
    
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void controlTask(void* pvParameters) {
  (void)pvParameters;
  while (1) {
    portENTER_CRITICAL(&mux);
    float temp = measured_temp;
    int soil = measured_soil;
    portEXIT_CRITICAL(&mux);

    if (modo_automatico) {
      if (temp < (temp_min - temp_hysteresis)) {
        updateActuatorState("foco", true);
        updateActuatorState("vent", false);
      } else if (temp > (temp_max + temp_hysteresis)) {
        updateActuatorState("vent", true);
        updateActuatorState("foco", false);
      } else {
        updateActuatorState("vent", false);
        updateActuatorState("foco", false);
      }
      
      if (soil <= soil_threshold_low) updateActuatorState("bomba", true);
      else if (soil >= soil_threshold_high) updateActuatorState("bomba", false);
    }

    unsigned long now = millis();
    if (foco_on && (now - foco_on_ts > 5UL * 60UL * 1000UL)) updateActuatorState("foco", false);
    if (fan_on && (now - fan_on_ts > 5UL * 60UL * 1000UL)) updateActuatorState("vent", false);
    if (pump_on && (now - pump_on_ts > 10UL * 60UL * 1000UL)) updateActuatorState("bomba", false);

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
    if (now - lastBroadcast >= interval) {
      notifyAllClients();
      lastBroadcast = now;
    }
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
        modo_automatico = !modo_automatico;
        Serial.printf("Modo %s\n", modo_automatico ? "AUTO" : "MAN");
        // Publicar cambio de modo en Adafruit IO
        if (mqttClient.connected()) {
          aioPublish(FEED_MODO, modo_automatico);
        }
      } else if (key == 'B' && !modo_automatico) {
        updateActuatorState("foco", !foco_on);
      } else if (key == 'C' && !modo_automatico) {
        updateActuatorState("vent", !fan_on);
      } else if (key == 'D' && !modo_automatico) {
        updateActuatorState("bomba", !pump_on);
      }
    }
    
    if (millis() - lastUpdate >= 500) {
      imprimirEstadoLCD();
      lastUpdate = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void cloudTask(void* pvParameters) {
  (void)pvParameters;
  unsigned long lastConnectAttempt = 0;
  unsigned long lastPublishTime = 0;
  unsigned long reconnectDelay = 5000;
  const unsigned long PUBLISH_INTERVAL = 60000; // 60 segundos (respetar límites de Adafruit IO)
  
  while (1) {
    // Manejar conexión MQTT
    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastConnectAttempt >= reconnectDelay) {
        Serial.println("Desconectado de Adafruit IO, reconectando...");
        aioConnect();
        lastConnectAttempt = now;
        reconnectDelay = min(60000UL, reconnectDelay * 2UL);
      }
    } else {
      reconnectDelay = 5000;
      
      // Publicar datos periódicamente
      unsigned long now = millis();
      if (now - lastPublishTime >= PUBLISH_INTERVAL) {
        publishToAIO();
        lastPublishTime = now;
      }
    }
    
    // Mantener conexión MQTT activa
    mqttClient.loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ==================== SETUP MODIFICADO ====================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Inicializar pines de relé
  pinMode(RELAY_FOCUS, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);
   digitalWrite(RELAY_FOCUS, HIGH);
  digitalWrite(RELAY_FAN, HIGH);
  digitalWrite(RELAY_PUMP, HIGH);

  // Inicializar LCD y DHT
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Adafruit IO ESP32");
  lcd.setCursor(0, 1);
  lcd.print("Invernadero IoT");
  dht.begin();

  // Configurar ADC
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LM35, ADC_11db);
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);

  // Conectar WiFi
  Serial.println("====================================");
  Serial.println("   INVERNADERO ESP32 - Adafruit IO  ");
  Serial.println("====================================");
  
  Serial.printf("Conectando a WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ WiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("❌ Error de conexión WiFi");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi fallo!");
  }

  // Configurar endpoints REST
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>Invernadero ESP32</title>";
    html += "<style>body{font-family:Arial,sans-serif;margin:20px;}</style>";
    html += "</head><body>";
    html += "<h1>🌱 Sistema de Invernadero ESP32</h1>";
    html += "<p>Conectado a Adafruit IO</p>";
    html += "<p><a href='/api/data'>Ver datos JSON</a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  });
  
  server.on("/api/data", HTTP_GET, []() {
    server.send(200, "application/json", getSystemJSON());
  });
  
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  
  server.begin();

  // Inicializar WebSocket
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
      Serial.printf("WebSocket desde cliente %u: %s\n", num, payload);
    }
  });

  // Configurar MQTT para Adafruit IO
  mqttClient.setCallback(aioCallback);
  mqttClient.setBufferSize(512);
  
  // Conectar a Adafruit IO
  aioConnect();

  // Crear tareas FreeRTOS
  xTaskCreatePinnedToCore(sensorsTask, "Sensors", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(controlTask, "Control", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(webTask, "Web", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(lcdKeypadTask, "LCDKey", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(cloudTask, "Cloud", 4096, NULL, 1, NULL, 1);

  lcd.clear();
  lcd.print("Sistema listo");
  lcd.setCursor(0, 1);
  lcd.print("Adafruit IO OK");
  delay(1000);
}

// ==================== LOOP ====================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}