#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>

HardwareSerial lidarSerial(2);
#define RXD2 16
#define TXD2 17

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

uint32_t lastSensorReadTime = 0;
const uint32_t sensorReadInterval = 2000; 
uint16_t failedReads = 0;
const uint16_t maxFailedReads = 10;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Dispositivo conectado!");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Dispositivo desconectado!");
    }
};

void setupTFMini() {
  // Reset sensor configuration
  uint8_t resetCommand[] = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x00, 0x41};
  lidarSerial.write(resetCommand, sizeof(resetCommand));
  delay(1000);  
  
  // Set offset to 0mm
  uint8_t setOffsetCommand[] = {0x42, 0x57, 0x02, 0x00, 0x08, 0x00, 0x00, 0xA3};
  lidarSerial.write(setOffsetCommand, sizeof(setOffsetCommand));
  delay(100);
  
  // Set to 1Hz output
  uint8_t setFrameRate[] = {0x42, 0x57, 0x02, 0x00, 0x01, 0x00, 0x00, 0x9C};
  lidarSerial.write(setFrameRate, sizeof(setFrameRate));
  delay(100);
  
  // Clear serial buffer
  while (lidarSerial.available()) {
    lidarSerial.read();
  }
  
  Serial.println("Sensor TF-Luna configurado para 1Hz e offset zerado");
}

uint16_t readDistance() {
  // Clear any old data
  while (lidarSerial.available() > 9) {
    lidarSerial.read();
  }

  if (lidarSerial.available() >= 9) {
    uint8_t buffer[9];
    lidarSerial.readBytes(buffer, 9);

    // Verify packet header
    if (buffer[0] == 0x59 && buffer[1] == 0x59) {
      // Calculate checksum
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) {
        checksum += buffer[i];
      }
      
      if (checksum == buffer[8]) {
        uint16_t distance = buffer[2] + (buffer[3] << 8);
        failedReads = 0;
        return distance;
      }
    }
  }
  
  failedReads++;
  return 0;
}

void checkSensorHealth() {
  if (failedReads >= maxFailedReads) {
    Serial.println("Reinicializando sensor devido a múltiplas falhas...");
    setupTFMini();
    failedReads = 0;
  }
}

void setup() {
  Serial.begin(115200);
  
  lidarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Inicializando sensor TF-Luna...");
  setupTFMini();
  delay(1000);
  
  BLEDevice::init("ESP32-LIDAR");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pSensorCharacteristic = pService->createCharacteristic(
      SENSOR_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );
  
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Aguardando conexão BLE...");
}

void loop() {
  static uint16_t lastValidDistance = 0;
  
  if (millis() - lastSensorReadTime >= sensorReadInterval) {
    uint16_t distance = readDistance();
    
    if (distance > 0) {
      lastValidDistance = distance;
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    } else {
      Serial.println("Leitura inválida do sensor");
      checkSensorHealth();
    }
    lastSensorReadTime = millis();
  }

  if (deviceConnected) {
    String sensorData = "{\"distance\":" + String(lastValidDistance) + "}";
    pSensorCharacteristic->setValue(sensorData.c_str());
    pSensorCharacteristic->notify();
  }

  // Handling BLE connection status
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Iniciando publicidade BLE");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  delay(10);
}
