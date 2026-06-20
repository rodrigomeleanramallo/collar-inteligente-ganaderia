#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_MLX90614.h>
#include <esp_sleep.h>
#include "ISGlyb.h"

// ==== PINES ====
#define CS_PIN             5
#define RESET_PIN         14
#define IRQ_PIN           26
#define RXD2              16
#define TXD2              17
#define PULSE_SENSOR_PIN  34
#define LED_PIN            2
#define SENSOR_POWER_PIN   4
#define VOLTAJE_PIN       35

// ==== OBJETOS ====
HardwareSerial neogps(1);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
const uint8_t Nsensor = 8;
ISGlyb SN(Nsensor);

// ==== VARIABLES ====
float latitud = 0.0, longitud = 0.0;
float temperatura = 0.0;
int bpm = 0;
float acel_x = 0.0, acel_y = 0.0, acel_z = 0.0;
float aceleracionTotal = 0.0;
float voltajeBateria = 0.0;
const int Id = 1;
bool alerta = false;

// ADC (voltaje batería)
const float Vref = 3.3;
const int resolution = 4095;
const float factorDivisor = 2.0;

// Control cardíaco
int pulseSignal = 0;
int threshold = 2000;
unsigned long lastBeatTime = 0;
int beatCount = 0;

// LoRa
byte localAddress = 0xE1;
byte centralAddress = 0xBB;

// Control de movimiento y frecuencia de envío
float ultimaLatitud = 0.0;
float ultimaLongitud = 0.0;
unsigned long tiempoUltimoMovimiento = 0;
unsigned long intervaloActual = 0.1 *60 * 1000000ULL; // 6 minutos

// ==== SETUP ====
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Inicializando sistema..."));

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(21, 22, 50000);

  if (!mpu.begin()) {
    Serial.println("Error al inicializar MPU6050."); while (true);
  }
  if (!mlx.begin()) {
    Serial.println("Error al inicializar MLX90614."); while (true);
  }

  LoRa.setPins(CS_PIN, RESET_PIN, IRQ_PIN);
  if (!LoRa.begin(915E6)) {
    Serial.println("Error al iniciar LoRa."); while (true);
  }

  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("✅ Sistema listo.");
}

// ==== LOOP ====
void loop() {
  Serial.println("🔄 Iniciando lectura...");

  leerGPS();
  medirFrecuenciaCardiaca();
  calcularSensores();
  leerVoltajeBateria();
  enviarTelemetria();

  digitalWrite(SENSOR_POWER_PIN, LOW);
  delay(100);

  Serial.println("💤 Sleep...");
  esp_sleep_enable_timer_wakeup(intervaloActual);
  esp_deep_sleep_start();
}

// ==== FUNCIONES ====

void leerGPS() {
  Serial.println("📡 GPS...");
  unsigned long t0 = millis();
  while (millis() - t0 < 0.1*60000) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) {
        if (gps.location.isValid()) {
          latitud = gps.location.lat();
          longitud = gps.location.lng();

          float distancia = sqrt(pow(latitud - ultimaLatitud, 2) + pow(longitud - ultimaLongitud, 2));
          if (distancia > 0.0001) {
            tiempoUltimoMovimiento = millis();
            intervaloActual = 6 * 60 * 1000000ULL; // 6 min
            ultimaLatitud = latitud;
            ultimaLongitud = longitud;
          } else if (millis() - tiempoUltimoMovimiento > 60 * 60 * 1000UL) {
            intervaloActual = 15 * 60 * 1000000ULL; // 15 min
            alerta = true;
          }
          return;
        }
      }
    }
  }
}

void medirFrecuenciaCardiaca() {
  beatCount = 0;
  unsigned long startMillis = millis();
  while (millis() - startMillis < 10000) {
    pulseSignal = analogRead(PULSE_SENSOR_PIN);
    if (pulseSignal > threshold && (millis() - lastBeatTime > 300)) {
      lastBeatTime = millis();
      beatCount++;
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    delay(10);
  }
  bpm = beatCount * 6;
  Serial.print("❤️ BPM: ");
  Serial.println(bpm);
}

void calcularSensores() {
  temperatura = mlx.readObjectTempC();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  acel_x = a.acceleration.x;
  acel_y = a.acceleration.y;
  acel_z = a.acceleration.z;
  aceleracionTotal = sqrt(acel_x * acel_x + acel_y * acel_y + acel_z * acel_z);

  Serial.print("🌡️ Temp: "); Serial.print(temperatura);
  Serial.print(" | 🚀 Acel: "); Serial.println(aceleracionTotal);
}

void leerVoltajeBateria() {
  int raw = analogRead(VOLTAJE_PIN);
  float voltajePin = (raw * Vref) / resolution;
  voltajeBateria = voltajePin * factorDivisor;
  Serial.print("🔋 Voltaje batería: ");
  Serial.print(voltajeBateria, 2);
  Serial.println(" V");
}

void enviarTelemetria() {
  SN.setDfloat(latitud, "Latitud");
  SN.setDfloat(longitud, "Longitud");
  SN.setDint(Id, "ID");
  SN.setDfloat(temperatura, "Temperatura");
  SN.setDint(bpm, "BPM");
  SN.setDfloat(aceleracionTotal, "Aceleracion");
  SN.setDbool(alerta, "Alerta");
  SN.setDfloat(voltajeBateria, "Voltaje");

  int tam = SN.getTam();
  byte* payload = SN.getData();

  String data = "";
  for (int i = 0; i < tam; i++) {
    data += String(payload[i], HEX) + ",";
  }

  if (LoRa.beginPacket() == 1) {
    LoRa.write(centralAddress);
    LoRa.write(localAddress);
    LoRa.write(data.length());
    LoRa.print(data);
    LoRa.endPacket();
    Serial.println("📡 Telemetría enviada correctamente.");
  } else {
    Serial.println("⚠️ Fallo en envío LoRa.");
  }
}
