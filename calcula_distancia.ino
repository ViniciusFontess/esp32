#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>

// --- Configuração do Sensor ---
HardwareSerial lidarSerial(2);
#define RXD2 16
#define TXD2 17

// Altura fixa do sensor até o chão (em cm).
// O cálculo será: Altura = 209 - Distância_Lida
const uint16_t SENSOR_FIXED_HEIGHT_CM = 208;

// --- Configuração BLE ---
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

// --- Controle de Tempo e Saúde do Sensor ---
uint32_t lastSensorReadTime = 0;
const uint32_t sensorReadInterval = 2000; // Leitura a cada 2 segundos 
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

/**
 * @brief Configura o sensor TF-Luna/Mini com comandos de reset,
 * offset (0mm) e taxa de frame (1Hz).
 */
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

/**
 * @brief Lê a distância do sensor TF-Luna (pacote de 9 bytes) e verifica o checksum.
 * @return Distância em cm (centímetros) ou 0 se a leitura falhar.
 */
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
        // Distance is in cm (bytes 2 and 3)
        uint16_t distance = buffer[2] + (buffer[3] << 8);
        failedReads = 0;
        return distance;
      }
    }
  }
  
  failedReads++;
  return 0; // Retorna 0 em caso de falha na leitura
}

/**
 * @brief Verifica o número de falhas e reinicializa o sensor se o limite for atingido.
 */
void checkSensorHealth() {
  if (failedReads >= maxFailedReads) {
    Serial.println("Reinicializando sensor devido a múltiplas falhas...");
    setupTFMini();
    failedReads = 0;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inicializa a comunicação serial com o sensor
  lidarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Inicializando sensor TF-Luna...");
  setupTFMini();
  delay(1000);
  
  // Configuração BLE
  BLEDevice::init("ESP32-LIDAR-HEIGHT-CALC"); 
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Característica para enviar os dados (apenas a Altura Calculada)
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
  static uint16_t lastCalculatedHeight = 0; // Armazena a altura calculada (209 - distância lida)
  
  if (millis() - lastSensorReadTime >= sensorReadInterval) {
    uint16_t distance = readDistance();
    
    if (distance > 0) {
      lastValidDistance = distance;
      
      // --- Implementação do Cálculo Solicitado: 209 - Distância Lida ---
      if (lastValidDistance <= SENSOR_FIXED_HEIGHT_CM) {
        // Altura = Altura Fixa do Sensor - Distância Lida
        lastCalculatedHeight = SENSOR_FIXED_HEIGHT_CM - lastValidDistance;
      } else {
        // Se a distância lida for maior que a altura fixa (209cm), algo está errado ou fora do alcance.
        lastCalculatedHeight = 0; 
      }
      // -----------------------------------------------------------------

      // Removida a impressão da Distância Bruta (lastValidDistance)
      Serial.print("Altura calculada (209 - Distância): ");
      Serial.print(lastCalculatedHeight);
      Serial.println(" cm");

    } else {
      Serial.println("Leitura inválida do sensor");
      checkSensorHealth();
    }
    lastSensorReadTime = millis();
  }

  if (deviceConnected) {
    // --- Criação do Payload JSON Atualizado ---
    // Envia SOMENTE a altura calculada (209 - Distância)
    String sensorData = "{\"distance\":" + String(lastCalculatedHeight) + "}";
    pSensorCharacteristic->setValue(sensorData.c_str());
    pSensorCharacteristic->notify();
    // ------------------------------------------
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
