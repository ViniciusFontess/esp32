#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>

// Configurações otimizadas para medição de altura
#define NUM_AMOSTRAS 100        // 100 amostras para ~1 segundo de medição
#define INTERVALO_LEITURA 10    // 10ms entre leituras
#define JANELA_FILTRO 3         // Margem de filtro em cm

HardwareSerial lidarSerial(2);
#define RXD2 16
#define TXD2 17

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool medicaoEmAndamento = false;
uint32_t tempoInicioMedicao = 0;

#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define COMMAND_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

uint16_t failedReads = 0;
const uint16_t maxFailedReads = 10;
uint16_t amostras[NUM_AMOSTRAS];
const float ALTURA_SENSOR = 216.0;
uint16_t ultimaAlturaMedida = 0;

// Função de comparação para qsort
int compare(const void *a, const void *b) {
  uint16_t fa = *(const uint16_t*)a;
  uint16_t fb = *(const uint16_t*)b;
  
  if (fa < fb) return -1;
  else if (fa > fb) return 1;
  else return 0;
}

// Callback para comandos BLE - VERSÃO CORRIGIDA
class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string receivedValue = pCharacteristic->getValue();
      
      if (receivedValue.length() > 0) {
        // Converte std::string para String do Arduino de forma segura
        String command = "";
        for (int i = 0; i < receivedValue.length(); i++) {
          command += (char)receivedValue[i];
        }
        
        Serial.println("Comando recebido: " + command);
        
        if (command == "MEDIR") {
          if (!medicaoEmAndamento) {
            medicaoEmAndamento = true;
            tempoInicioMedicao = millis();
            Serial.println(">>> Iniciando medição por comando <<<");
          }
        } else if (command == "STATUS") {
          // Envia status atual
          String status = "{\"status\":\"" + String(medicaoEmAndamento ? "MEDINDO" : "PRONTO") + 
                         "\", \"altura\":" + String(ultimaAlturaMedida) + "}";
          pSensorCharacteristic->setValue(status.c_str());
          pSensorCharacteristic->notify();
        }
      }
    }
};

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
  // Reset configuration
  uint8_t resetCommand[] = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x00, 0x41};
  lidarSerial.write(resetCommand, sizeof(resetCommand));
  delay(1000);  
  
  // Set offset to 0mm
  uint8_t setOffsetCommand[] = {0x42, 0x57, 0x02, 0x00, 0x08, 0x00, 0x00, 0xA3};
  lidarSerial.write(setOffsetCommand, sizeof(setOffsetCommand));
  delay(100);
  
  // Set to 10Hz output para melhor resposta
  uint8_t setFrameRate[] = {0x42, 0x57, 0x02, 0x00, 0x01, 0x0A, 0x00, 0xA5};
  lidarSerial.write(setFrameRate, sizeof(setFrameRate));
  delay(100);
  
  // Clear serial buffer
  while (lidarSerial.available()) {
    lidarSerial.read();
  }
  
  Serial.println("Sensor TF-Luna configurado para 10Hz e offset zerado");
}

uint16_t readDistance() {
  // Limpa dados antigos mas mantém buffer para nova leitura
  while (lidarSerial.available() > 9) {
    lidarSerial.read();
  }

  if (lidarSerial.available() >= 9) {
    uint8_t buffer[9];
    lidarSerial.readBytes(buffer, 9);

    // Verifica header do pacote
    if (buffer[0] == 0x59 && buffer[1] == 0x59) {
      // Calcula checksum
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) {
        checksum += buffer[i];
      }
      
      if (checksum == buffer[8]) {
        uint16_t distance = buffer[2] + (buffer[3] << 8);
        // Filtro básico - descarta leituras obviamente erradas
        if (distance > 50 && distance < 250) {
          failedReads = 0;
          return distance;
        }
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

uint16_t calcularAlturaRobusta() {
  Serial.println("Coletando amostras...");
  
  // Coleta todas as amostras
  for(int i = 0; i < NUM_AMOSTRAS; i++){
    uint16_t leitura = readDistance();
    
    // Se leitura falhou, tenta novamente
    if (leitura == 0) {
      i--;
      delay(INTERVALO_LEITURA);
      continue;
    }
    
    amostras[i] = leitura;
    
    // Envia progresso via BLE (opcional)
    if (deviceConnected && (i % 20 == 0)) {
      int progresso = (i * 100) / NUM_AMOSTRAS;
      String progressData = "{\"progresso\":" + String(progresso) + "}";
      pSensorCharacteristic->setValue(progressData.c_str());
      pSensorCharacteristic->notify();
    }
    
    delay(INTERVALO_LEITURA);
  }

  // Ordena as amostras para encontrar a mediana
  qsort(amostras, NUM_AMOSTRAS, sizeof(uint16_t), compare);
  
  // Calcula a mediana (valor do meio)
  uint16_t mediana = amostras[NUM_AMOSTRAS / 2];
  
  // Filtro inteligente: considera apenas valores próximos da mediana
  uint32_t soma = 0;
  uint16_t count = 0;
  
  for(int i = 0; i < NUM_AMOSTRAS; i++) {
    if(abs(amostras[i] - mediana) <= JANELA_FILTRO) {
      soma += amostras[i];
      count++;
    }
  }
  
  uint16_t distanciaFiltrada;
  
  // Se não encontrou valores consistentes, usa a mediana pura
  if (count < NUM_AMOSTRAS * 0.6) {
    Serial.println("Aviso: Baixa consistência nas medições. Usando mediana.");
    distanciaFiltrada = mediana;
  } else {
    distanciaFiltrada = soma / count;
  }
  
  uint16_t alturaPessoa = ALTURA_SENSOR - distanciaFiltrada;
  
  Serial.print("Mediana: ");
  Serial.print(mediana);
  Serial.print("cm, Filtrada: ");
  Serial.print(distanciaFiltrada);
  Serial.print("cm, Altura: ");
  Serial.print(alturaPessoa);
  Serial.print("cm, Amostras válidas: ");
  Serial.print(count);
  Serial.println("/100");
  
  return alturaPessoa;
}

void setup() {
  Serial.begin(115200);
  
  lidarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Inicializando sensor TF-Luna para medição de altura...");
  setupTFMini();
  delay(1000);
  
  BLEDevice::init("ESP32-Altura");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Característica para dados do sensor (notify)
  pSensorCharacteristic = pService->createCharacteristic(
      SENSOR_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pSensorCharacteristic->addDescriptor(new BLE2902());
  
  // Característica para comandos (write)
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
  
  Serial.println("Sistema de medição de altura pronto!");
  Serial.println("Envie 'MEDIR' via BLE para iniciar a medição");
}

void loop() {
  // Processa medição se estiver em andamento
  if (medicaoEmAndamento) {
    Serial.println("=== PROCESSANDO MEDIÇÃO ===");
    
    uint16_t altura = calcularAlturaRobusta();
    ultimaAlturaMedida = altura;
    
    // Envia resultado via BLE
    String resultado = "{\"altura\":" + String(altura) + "}";
    pSensorCharacteristic->setValue(resultado.c_str());
    pSensorCharacteristic->notify();
    
    Serial.print(">>> ALTURA MEDIDA: ");
    Serial.print(altura);
    Serial.println(" cm <<<");
    Serial.println();
    
    medicaoEmAndamento = false;
  }

  // Gerenciamento de conexão BLE
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Dispositivo desconectado. Reiniciando publicidade BLE...");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  checkSensorHealth();
  delay(100);
}
