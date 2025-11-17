#include <HardwareSerial.h>

// Porta serial usada para o LIDAR (UART2 do ESP32)
HardwareSerial lidarSerial(2);
#define RXD2 16
#define TXD2 17
#define numeroDeLeitura 300
#define erro 1
float alturaFixa = 213;

void setup() {
  // Serial para console (PC)
  Serial.begin(115200);
  delay(100);

  // Inicializa UART do LIDAR (TF‑Luna normalmente usa 115200)
  lidarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("LIDAR minimal - iniciando");
  
  // Opcional: enviar comando para ajustar frame rate do módulo
  // Se quiser mudar taxa, calcule checksum e escreva o comando aqui.
  // Exemplo comentado (não enviado):
  // uint8_t frameCmd[8] = {0x42,0x57,0x02,0x00,0x0A,0x00,0x00,0x00}; // 10 Hz (placeholder)
  // lidarSerial.write(frameCmd, sizeof(frameCmd));
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
    // não é header duplo, reinicia busca
    return 0;
  }

  // Temos 0x59 0x59, monta o pacote
  buf[0] = 0x59;
  buf[1] = 0x59;
  int needed = 7; // restam 7 bytes do pacote (9 - 2)
  int received = 0;
  while (received < needed) {
    if (lidarSerial.available()) {
      received += lidarSerial.readBytes(&buf[2 + received], needed - received);
    } else {
      delay(1);
    }
  }

  // valida checksum (soma bytes 0..7 deve ser igual a buf[8])
  uint8_t checksum = 0;
  for (int i = 0; i < 8; ++i) checksum += buf[i];
  if (checksum != buf[8]) return 0;

  // recombina low/high em uint16_t (little-endian)
  uint16_t distance_cm = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
  return distance_cm;
}
void purgeLidarInput() {
  while (lidarSerial.available()) {
    lidarSerial.read();
  }
}

void calibrateHeight(uint16_t sampleCount = numeroDeLeitura, uint32_t timeoutMs = 15000, uint32_t minIntervalMs = 5) {
  Serial.println("Iniciando calibragem da altura fixa...");
  // remove amostras antigas
  
  purgeLidarInput();
  delay(5);
  // descarta a primeira amostra nova para alinhar ao fluxo atual
  readTFminiPacket();

  uint32_t sum = 0;
  uint16_t collected = 0;
  unsigned long start = millis();

  while (collected < sampleCount && (millis() - start) < timeoutMs) {
    uint16_t d = readTFminiPacket();
    if (d > 0) {
      sum += d;
      collected++;
    }
    // espera mínima entre leituras para não sobrecarregar
    if (minIntervalMs) delay(minIntervalMs);
  }

  if (collected == 0) {
    Serial.println("Calibragem falhou: nenhuma amostra válida coletada.");
    return;
  }

  // média em float para melhor precisão, depois arredonda para inteiro
  float avg = (float)sum / (float)collected;
  float novaAltura = avg;
  //uint16_t novaAltura = (uint16_t)(avg + 0.5f);
  alturaFixa = novaAltura;

  Serial.print("Calibragem concluída. Nova altura fixa = ");
  Serial.print(alturaFixa);
  Serial.println(" cm");
}


void loop() {
  // coleciona numeroDeLeitura amostras válidas (com sincronização e checksum)
  uint32_t somaDistancias = 0;
  int distanciasColetadas = 0;
  int Calibrar = 1;
  uint16_t distance = 0;
  unsigned long start = millis();
  const unsigned long timeoutMs = 5000; // timeout total de coleta (opcional)
  if(Calibrar == 1)
    calibrateHeight();
  purgeLidarInput();
  delay(5); // aguarda dados chegarem
  while (distanciasColetadas < numeroDeLeitura && (millis() - start) < timeoutMs) {
    uint16_t d = readTFminiPacket();
    if (d > 0) {
      somaDistancias += d;
      distanciasColetadas++;
    } else {
      // pacote inválido ou não sincronizado — apenas tenta de novo
    }
    // pequeno delay para evitar tight-loop agressivo; ajuste se necessário
    delay(5);
  }

  if (distanciasColetadas > 0) {
    float media = (float)(somaDistancias) / (float)(distanciasColetadas);
    float distance = alturaFixa - media;
    uint16_t distanceInt = (uint16_t)(distance + 0.5f); // arredonda para inteiro
    Serial.print("Média Distancia (cm):  ");
    Serial.println(distanceInt);
  } else {
        Serial.println("Nenhuma amostra válida coletada (timeout)");
  }

  // espere antes da próxima rodada de médias
  delay(1000);
    purgeLidarInput();
}