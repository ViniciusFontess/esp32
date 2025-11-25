#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>

// Configurações do LIDAR
HardwareSerial lidarSerial(2);
#define RXD2 16
#define TXD2 17
#define numeroDeLeitura 300
float alturaFixa = 213.0;

// Variáveis Bluetooth
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool medicaoEmAndamento = false;
bool calibrarEmAndamento = false;

#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define COMMAND_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String receivedValue = pCharacteristic->getValue();
      
      if (receivedValue.length() > 0) {
        // Corrigido: Converter std::string para String do Arduino corretamente
        String command = String(receivedValue.c_str());
        
        if (command == "MEDIR") {
          if (!medicaoEmAndamento && !calibrarEmAndamento) {
            medicaoEmAndamento = true;
          }
        } else if (command == "CALIBRAR") {
          if (!calibrarEmAndamento && !medicaoEmAndamento) {
            calibrarEmAndamento = true;
          }
        }
      }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);
  
  // Inicializa LIDAR
  lidarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Sistema LIDAR + Bluetooth iniciado");
  
  // Inicializa Bluetooth
  BLEDevice::init("ESP32-Altura");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pSensorCharacteristic = pService->createCharacteristic(
      SENSOR_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pSensorCharacteristic->addDescriptor(new BLE2902());
  
  pCommandCharacteristic = pService->createCharacteristic(
      COMMAND_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());
  
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("Sistema pronto! Use CALIBRAR primeiro, depois MEDIR");
}

uint16_t readTFminiPacket() {
  uint8_t buf[9];

  // Busca primeiro byte do header (0x59)
  int ch = -1;
  do {
    if (lidarSerial.available()) ch = lidarSerial.read();
    else delay(1);
  } while (ch != 0x59);

  // Lê segundo byte e verifica se é também 0x59
  while (!lidarSerial.available()) delay(1);
  int ch2 = lidarSerial.read();
  if (ch2 != 0x59) {
    return 0;
  }

  // Temos 0x59 0x59, monta o pacote
  buf[0] = 0x59;
  buf[1] = 0x59;
  int needed = 7;
  int received = 0;
  while (received < needed) {
    if (lidarSerial.available()) {
      received += lidarSerial.readBytes(&buf[2 + received], needed - received);
    } else {
      delay(1);
    }
  }

  // valida checksum
  uint8_t checksum = 0;
  for (int i = 0; i < 8; ++i) checksum += buf[i];
  if (checksum != buf[8]) return 0;

  // recombina low/high em uint16_t
  uint16_t distance_cm = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
  return distance_cm;
}

void purgeLidarInput() {
  while (lidarSerial.available()) {
    lidarSerial.read();
  }
}

void calibrateHeight() {
  String progresso = "{\"tipo\":\"calibragem\", \"status\":\"iniciando\"}";
  pSensorCharacteristic->setValue(progresso.c_str());
  pSensorCharacteristic->notify();
  
  Serial.println("Iniciando calibragem da altura fixa...");
  
  purgeLidarInput();
  delay(5);
  readTFminiPacket(); // descarta primeira amostra

  uint32_t sum = 0;
  uint16_t collected = 0;
  unsigned long start = millis();
  const unsigned long timeoutMs = 15000;

  while (collected < numeroDeLeitura && (millis() - start) < timeoutMs) {
    uint16_t d = readTFminiPacket();
    if (d > 0) {
      sum += d;
      collected++;
      
      // Envia progresso a cada 10%
      if (collected % (numeroDeLeitura / 10) == 0) {
        int percentual = (collected * 100) / numeroDeLeitura;
        progresso = "{\"tipo\":\"calibragem\", \"progresso\":" + String(percentual) + "}";
        pSensorCharacteristic->setValue(progresso.c_str());
        pSensorCharacteristic->notify();
      }
    }
    delay(5);
  }

  if (collected == 0) {
    progresso = "{\"tipo\":\"calibragem\", \"status\":\"erro\", \"mensagem\":\"Nenhuma amostra válida\"}";
    pSensorCharacteristic->setValue(progresso.c_str());
    pSensorCharacteristic->notify();
    Serial.println("Calibragem falhou: nenhuma amostra válida coletada.");
    return;
  }

  // média em float para melhor precisão
  float avg = (float)sum / (float)collected;
  alturaFixa = avg;

  progresso = "{\"tipo\":\"calibragem\", \"status\":\"concluido\", \"alturaFixa\":" + String(alturaFixa, 1) + "}";
  pSensorCharacteristic->setValue(progresso.c_str());
  pSensorCharacteristic->notify();
  
  Serial.print("Calibragem concluída. Nova altura fixa = ");
  Serial.print(alturaFixa);
  Serial.println(" cm");
}

uint16_t medirAltura() {
  String progresso = "{\"tipo\":\"medicao\", \"status\":\"iniciando\"}";
  pSensorCharacteristic->setValue(progresso.c_str());
  pSensorCharacteristic->notify();
  
  purgeLidarInput();
  delay(5);

  uint32_t somaDistancias = 0;
  int distanciasColetadas = 0;
  unsigned long start = millis();
  const unsigned long timeoutMs = 10000;

  while (distanciasColetadas < numeroDeLeitura && (millis() - start) < timeoutMs) {
    uint16_t d = readTFminiPacket();
    if (d > 0) {
      somaDistancias += d;
      distanciasColetadas++;
      
      // Envia progresso a cada 10%
      if (distanciasColetadas % (numeroDeLeitura / 10) == 0) {
        int percentual = (distanciasColetadas * 100) / numeroDeLeitura;
        progresso = "{\"tipo\":\"medicao\", \"progresso\":" + String(percentual) + "}";
        pSensorCharacteristic->setValue(progresso.c_str());
        pSensorCharacteristic->notify();
      }
    }
    delay(5);
  }

  if (distanciasColetadas == 0) {
    progresso = "{\"tipo\":\"medicao\", \"status\":\"erro\", \"mensagem\":\"Nenhuma amostra válida\"}";
    pSensorCharacteristic->setValue(progresso.c_str());
    pSensorCharacteristic->notify();
    Serial.println("Nenhuma amostra válida coletada (timeout)");
    return 0;
  }

  float media = (float)(somaDistancias) / (float)(distanciasColetadas);
  float distancia = alturaFixa - media;
  uint16_t distanciaInt = (uint16_t)(distancia + 0.5f); // arredonda para inteiro
  
  Serial.print("Média Distancia (cm): ");
  Serial.println(distanciaInt);
  
  return distanciaInt;
}

void loop() {
  // Processa calibragem quando solicitada via Bluetooth
  if (calibrarEmAndamento) {
    calibrateHeight();
    calibrarEmAndamento = false;
  }

  // Processa medição quando solicitada via Bluetooth
  if (medicaoEmAndamento) {
    uint16_t altura = medirAltura();
    
    if (altura > 0) {
      String resultado = "{\"tipo\":\"medicao\", \"status\":\"concluido\", \"altura\":" + String(altura) + ", \"alturaFixa\":" + String(alturaFixa, 1) + "}";
      pSensorCharacteristic->setValue(resultado.c_str());
      pSensorCharacteristic->notify();
    }
    
    medicaoEmAndamento = false;
  }

  // Gerenciamento de conexão BLE
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  delay(100);
}
