#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <TFMPI2C.h>
#include <algorithm>
#include <cmath>

TFMPI2C tfmp;
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool sensorAvailable = false;

#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

const int BUFFER_SIZE = 100;
uint16_t distanceBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

uint16_t calculateMedian(uint16_t values[], int size) {
    // Ordena os valores
    std::sort(values, values + size);
    
    // Calcula a mediana
    if (size % 2 == 0) {
        return (values[size / 2 - 1] + values[size / 2]) / 2;
    } else {
        return values[size / 2];
    }
}

float calculateStandardDeviation(uint16_t values[], int size, float mean) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += pow(values[i] - mean, 2);
    }
    return sqrt(sum / size);
}

int filterOutliers(uint16_t source[], uint16_t destination[], int size, float threshold) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += source[i];
    }
    float mean = sum / size;
    
    float stdDev = calculateStandardDeviation(source, size, mean);
    
    int filteredCount = 0;
    for (int i = 0; i < size; i++) {
        if (fabs(source[i] - mean) <= threshold * stdDev) {
            destination[filteredCount++] = source[i];
        }
    }
    
    return filteredCount;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Inicializa o sensor TFMPI2C
    if (tfmp.begin() == false) {
        Serial.println("Sensor TFMP não detectado. Verifique a conexão.");
        sensorAvailable = false;
    } else {
        Serial.println("Sensor TFMP inicializado com sucesso!");
        sensorAvailable = true;
    }

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
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    Serial.println("Aguardando conexão BLE...");
}

void loop() {
    static uint16_t dist = 0;
    static uint16_t filteredBuffer[BUFFER_SIZE];

    if (deviceConnected && sensorAvailable) {
        if (tfmp.getData(dist)) {
            // Adiciona a leitura ao buffer
            distanceBuffer[bufferIndex] = dist;
            bufferIndex++;
            
            // Quando o buffer estiver cheio, processa os dados
            if (bufferIndex >= BUFFER_SIZE) {
                bufferIndex = 0;
                
                // Filtra outliers (usando 2 desvios padrão como threshold)
                int filteredCount = filterOutliers(distanceBuffer, filteredBuffer, BUFFER_SIZE, 2.0);
                
                if (filteredCount > 0) {
                    // Calcula a mediana dos valores filtrados
                    uint16_t medianDistance = calculateMedian(filteredBuffer, filteredCount);
                    
                    // Envia a mediana via BLE
                    pSensorCharacteristic->setValue(String(medianDistance).c_str());
                    pSensorCharacteristic->notify();
                    
                    Serial.print("Distacia enviada: ");
                    Serial.println(medianDistance);
                }
            }
        } else {
            Serial.println("Falha na leitura do sensor");
        }
        delay(10); // Pequeno delay entre leituras
    }

    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Iniciando publicidade");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("Dispositivo conectado");
    }
}
