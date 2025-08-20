#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <TFMPI2C.h>

TFMPI2C tfmp;
BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool sensorAvailable = false;

#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

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
        
        // Configura o sensor para modo de distância apenas
        tfmp.sendCommand(0x01, 0x02); // Modo de distância
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

    if (deviceConnected) {
        if (sensorAvailable) {
            // Lê apenas a distância
            if (tfmp.getData(dist)) {
                pSensorCharacteristic->setValue(String(dist).c_str());
                pSensorCharacteristic->notify();
                Serial.print("Distância: ");
                Serial.print(dist);
                Serial.println(" cm");
            } else {
                Serial.println("Falha na leitura do sensor");
            }
        } else {
            // Modo simulado apenas se o sensor não estiver disponível
            dist = random(50, 200);
            pSensorCharacteristic->setValue(String(dist).c_str());
            pSensorCharacteristic->notify();
            Serial.print("Distância simulada: ");
            Serial.println(dist);
        }
        delay(100);
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
