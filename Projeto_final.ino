#include <Wire.h>
#include "SSD1306Wire.h"
#include <WiFi.h>
#include <esp_now.h>
#include "esp_mac.h"  // required - exposes esp_mac_type_t values

SSD1306Wire display(0x3c, 5, 4);

// =====================================================
// LEITURA DA BATERIA (validada com multímetro)
// =====================================================
#define POT_PIN 34
#define MULT (6.6 / 4095.0)

#define N_AMOSTRAS 20
long leituras[N_AMOSTRAS];
int indiceLeitura = 0;
long somaLeituras = 0;
bool bufferCheio = false;

// =====================================================
// MÉDIA MÓVEL DO RSSI (20 amostras)
// =====================================================
#define N_AMOSTRAS_RSSI 20
int leiturasRSSI[N_AMOSTRAS_RSSI];
int indiceRSSI = 0;
long somaRSSI = 0;
bool bufferRSSICheio = false;

int sinalMedio = -100; // valor suavizado, usado no display (barras + texto)

// =====================================================
// BOTÃO PTT (PUSH-TO-TALK) - GPIO 21
// =====================================================
#define BOTAO_PIN 21

int leituraBotaoAtual = HIGH;
int leituraBotaoAnterior = HIGH;
unsigned long ultimoDebounce = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// =====================================================
// STATUS DE CONEXÃO (AGORA 100% AUTOMÁTICO VIA ESP-NOW)
// Não depende mais de botão físico (GPIO 33 removido)
// =====================================================
bool conectado = false;
unsigned long ultimoPacoteRecebido = 0;
const unsigned long TIMEOUT_CONEXAO = 10000; // 10s sem receber pacote = desconectado

float anguloSpinner = 0;
unsigned long ultimaAtualizacaoSpinner = 0;
const unsigned long INTERVALO_SPINNER = 60; // ms -> velocidade da rotação do "procurando"

// Posição do círculo: Quadrante 1 (superior esquerdo) do display 128x64
#define CENTRO_X_CONEXAO 18
#define CENTRO_Y_CONEXAO 14
#define RAIO_CONEXAO 12

// =====================================================
// VARIÁVEIS
// =====================================================
float tensaoBateria = 0;
int sinal = -100;   // RSSI bruto em dBm (último pacote recebido). -100 = "sem sinal ainda"
bool microfone = false;

unsigned long ultimaAtualizacao = 0;
const unsigned long INTERVALO_ATUALIZACAO = 5000;

bool precisaRedesenhar = true;

// Controle de Ping-Pong e Latência
unsigned long tempoUltimoEnvio = 0;
int contadorPacotes = 0;

// =====================================================
// VU-METER (SIMULADO - trocar por leitura real do I2S no Sprint 3)
// =====================================================
int nivelVU = 0; // 0 a 100
unsigned long ultimaAtualizacaoVU = 0;
const unsigned long INTERVALO_VU = 80; // ms (~12 atualizações por segundo)

// =====================================================
// ESP-NOW (Struct atualizada para TX/RX e Ping-Pong)
// =====================================================
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xF7, 0x17, 0xE0}; // MAC do colega

typedef struct struct_message {
  float bateria;
  bool microfone;
  bool estaFalando; // true = Falando (TX), false = Ouvindo (RX)
  int idPacote;     // Controle de Ping-Pong
} struct_message;

struct_message dadosEnviar;
struct_message dadosRecebidos;

// Estado atual de transmissão do outro dispositivo (para o display)
bool outroEstaFalando = false;

// =====================================================
// ÍCONE DA BATERIA - 16x16 PIXELS
// =====================================================
const uint8_t bateria_16x16[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xFE, 0x1F, 0xFE, 0x1F, 0xFE, 0x7F, 0xFE, 0x7F,
  0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x1F, 0xFE, 0x1F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// =====================================================
// ÍCONE DO MICROFONE - PREENCHIDO (ON) - 16x16 PIXELS
// =====================================================
const uint8_t microfone_16x16[] PROGMEM = {
  0xE0, 0x01, 0xF0, 0x03, 0xF0, 0x03, 0xF0, 0x03, 0xF0, 0x03, 0xF0, 0x03,
  0xF0, 0x03, 0xE0, 0x01, 0xE0, 0x01, 0xC0, 0x00, 0xC0, 0x00, 0xE0, 0x01,
  0xF0, 0x03, 0xF0, 0x03, 0xF8, 0x07, 0x00, 0x00
};

// =====================================================
// ÍCONE DO MICROFONE - CONTORNO/SILHUETA (OFF) - 16x16 PIXELS
// =====================================================
const uint8_t microfone_outline_16x16[] PROGMEM = {
  0xE0, 0x01, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02, 0x10, 0x02,
  0x10, 0x02, 0x20, 0x01, 0x20, 0x01, 0xC0, 0x00, 0xC0, 0x00, 0x20, 0x01,
  0x10, 0x02, 0x10, 0x02, 0xF8, 0x07, 0x00, 0x00
};

// =====================================================
// FUNÇÃO: Lê ADC com média móvel e retorna a TENSÃO (V)
// =====================================================
float lerTensaoBateria() {
  int leituraADC = analogRead(POT_PIN);

  somaLeituras -= leituras[indiceLeitura];
  leituras[indiceLeitura] = leituraADC;
  somaLeituras += leituraADC;
  indiceLeitura = (indiceLeitura + 1) % N_AMOSTRAS;
  if (indiceLeitura == 0) bufferCheio = true;

  int amostrasValidas = bufferCheio ? N_AMOSTRAS : max(indiceLeitura, 1);
  float mediaADC = (float)somaLeituras / amostrasValidas;

  return mediaADC * MULT;
}

// =====================================================
// FUNÇÃO: Atualiza a média móvel do RSSI (20 amostras)
// =====================================================
void atualizarMediaRSSI(int novoValor) {
  somaRSSI -= leiturasRSSI[indiceRSSI];
  leiturasRSSI[indiceRSSI] = novoValor;
  somaRSSI += novoValor;
  indiceRSSI = (indiceRSSI + 1) % N_AMOSTRAS_RSSI;
  if (indiceRSSI == 0) bufferRSSICheio = true;

  int amostrasValidas = bufferRSSICheio ? N_AMOSTRAS_RSSI : max(indiceRSSI, 1);
  sinalMedio = somaRSSI / amostrasValidas;
}

// =====================================================
// ESP-NOW: envia telemetria e estado PTT
// =====================================================
void enviarTelemetria() {
  contadorPacotes++;
  dadosEnviar.bateria = tensaoBateria;
  dadosEnviar.microfone = microfone;
  dadosEnviar.estaFalando = microfone;
  dadosEnviar.idPacote = contadorPacotes;

  tempoUltimoEnvio = millis();
  esp_now_send(broadcastAddress, (uint8_t *) &dadosEnviar, sizeof(dadosEnviar));
}

// =====================================================
// ESP-NOW: callback de ENVIO (Validação de Ping-Pong)
// =====================================================
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    unsigned long latencia = millis() - tempoUltimoEnvio;
    Serial.printf("[PING-PONG] Pacote #%d entregue com sucesso! Latencia: %lu ms\n", contadorPacotes, latencia);
  } else {
    Serial.println("[PING-PONG] Falha na entrega do pacote!");
  }
}

// =====================================================
// ESP-NOW: callback de RECEBIMENTO
// Agora também é responsável por marcar "conectado = true"
// =====================================================
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  memcpy(&dadosRecebidos, incomingData, sizeof(dadosRecebidos));

  sinal = recvInfo->rx_ctrl->rssi;
  atualizarMediaRSSI(sinal);

  // Atualiza o estado de quem está falando do outro lado
  outroEstaFalando = dadosRecebidos.estaFalando;

  // ---- CONEXÃO AUTOMÁTICA ----
  // Se chegou pacote do colega, consideramos conectado
  ultimoPacoteRecebido = millis();
  if (!conectado) {
    conectado = true;
    Serial.println("Status: CONECTADO (pacote recebido do colega)");
  }
  // -----------------------------

  Serial.printf("[ESPNOW] De: %02X:%02X:%02X:%02X:%02X:%02X | RSSI médio: %d dBm | Bat: %.2fV | Outro Falando: %s\n",
    recvInfo->src_addr[0], recvInfo->src_addr[1], recvInfo->src_addr[2],
    recvInfo->src_addr[3], recvInfo->src_addr[4], recvInfo->src_addr[5],
    sinalMedio, dadosRecebidos.bateria, outroEstaFalando ? "SIM (FALANDO)" : "NÃO (OUVINDO)");

  precisaRedesenhar = true;
}

// =====================================================
// FUNÇÃO: Verifica o botão com debounce (estilo PTT)
// =====================================================
void verificarBotao() {
  int leitura = digitalRead(BOTAO_PIN);

  if (leitura != leituraBotaoAnterior) {
    ultimoDebounce = millis();
  }

  if ((millis() - ultimoDebounce) > DEBOUNCE_DELAY) {
    if (leitura != leituraBotaoAtual) {
      leituraBotaoAtual = leitura;

      bool novoEstado = (leituraBotaoAtual == LOW);

      if (novoEstado != microfone) {
        microfone = novoEstado;
        precisaRedesenhar = true;
        Serial.println(microfone ? "Estado: FALANDO (TX)" : "Estado: OUVINDO (RX)");

        // Dispara o pacote imediatamente ao alterar o PTT para sincronizar os estados
        enviarTelemetria();
      }
    }
  }

  leituraBotaoAnterior = leitura;
}

// =====================================================
// FUNÇÃO: Verifica se a conexão com o colega caiu (timeout)
// Substitui o antigo botão da GPIO 33
// =====================================================
void verificarTimeoutConexao() {
  if (conectado && (millis() - ultimoPacoteRecebido > TIMEOUT_CONEXAO)) {
    conectado = false;
    precisaRedesenhar = true;
    Serial.println("Status: CONEXAO PERDIDA (timeout)");
  }
}

// =====================================================
// FUNÇÃO: Anima o "spinner" de busca
// =====================================================
void atualizarSpinner() {
  if (conectado) return;

  if (millis() - ultimaAtualizacaoSpinner < INTERVALO_SPINNER) return;
  ultimaAtualizacaoSpinner = millis();

  anguloSpinner += 25;
  if (anguloSpinner >= 360) anguloSpinner -= 360;

  precisaRedesenhar = true;
}

// =====================================================
// VU-METER SIMULADO
// =====================================================
void atualizarVUMeterSimulado() {
  if (millis() - ultimaAtualizacaoVU < INTERVALO_VU) return;
  ultimaAtualizacaoVU = millis();

  if (microfone) {
    int variacao = random(-25, 26);
    nivelVU += variacao;
    nivelVU = constrain(nivelVU, 10, 100);
  } else {
    nivelVU -= 8;
    if (nivelVU < 0) nivelVU = 0;
  }

  precisaRedesenhar = true;
}

// =====================================================
// ESP-NOW: configuração inicial
// =====================================================
void configurarEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Falha ao adicionar peer broadcast");
  }
}

// =====================================================
// FUNÇÃO: Desenha barras de sinal (1 a 4)
// =====================================================
void desenharBarrasSinal(int x, int y) {
  int nivelBarras = 0;

  if (sinalMedio >= -50)      nivelBarras = 4;
  else if (sinalMedio >= -65) nivelBarras = 3;
  else if (sinalMedio >= -75) nivelBarras = 2;
  else if (sinalMedio >= -85) nivelBarras = 1;
  else                        nivelBarras = 0;

  const int larguraBarra = 3;
  const int espacoBarra  = 1;
  const int alturaMaxima = 16;

  for (int i = 0; i < 4; i++) {
    int alturaBarra = (i + 1) * 4;
    int xBarra = x + i * (larguraBarra + espacoBarra);
    int yBarra = y + (alturaMaxima - alturaBarra);

    if (i < nivelBarras) {
      display.fillRect(xBarra, yBarra, larguraBarra, alturaBarra);
    } else {
      display.drawRect(xBarra, yBarra, larguraBarra, alturaBarra);
    }
  }
}

// =====================================================
// FUNÇÃO: Desenha o ícone do microfone
// =====================================================
void desenharIconeMicrofone(int x, int y) {
  if (microfone) {
    display.drawXbm(x, y, 16, 16, microfone_16x16);
  } else {
    display.drawXbm(x, y, 16, 16, microfone_outline_16x16);
  }
}

// =====================================================
// FUNÇÃO: Desenha a barra do VU-Meter
// =====================================================
void desenharVUMeterSimulado(int x, int y, int largura, int altura) {
  display.drawRect(x, y, largura, altura);

  int larguraPreenchida = map(nivelVU, 0, 100, 0, largura - 2);
  if (larguraPreenchida > 0) {
    display.fillRect(x + 1, y + 1, larguraPreenchida, altura - 2);
  }
}

// =====================================================
// FUNÇÃO: Desenha o indicador de status (Quadrante 1)
// =====================================================
void desenharStatusConexao() {
  display.drawCircle(CENTRO_X_CONEXAO, CENTRO_Y_CONEXAO, RAIO_CONEXAO);

  if (conectado) {
    display.fillCircle(CENTRO_X_CONEXAO, CENTRO_Y_CONEXAO, 3);
    display.drawString(0, CENTRO_Y_CONEXAO + RAIO_CONEXAO + 2, "Conectado");
  } else {
    float rad = anguloSpinner * PI / 180.0;
    int xPonta = CENTRO_X_CONEXAO + (int)(RAIO_CONEXAO * cos(rad));
    int yPonta = CENTRO_Y_CONEXAO + (int)(RAIO_CONEXAO * sin(rad));

    display.drawLine(CENTRO_X_CONEXAO, CENTRO_Y_CONEXAO, xPonta, yPonta);
    display.drawString(0, CENTRO_Y_CONEXAO + RAIO_CONEXAO + 2, "Conectando");
  }
}

// =====================================================
// FUNÇÃO: Redesenha a tela inteira com Status TX/RX
// =====================================================
void atualizarDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  desenharStatusConexao();

  display.drawXbm(112, 0, 16, 16, bateria_16x16);
  display.drawString(80, 3, String(tensaoBateria, 2) + "V");

  desenharBarrasSinal(112, 20);
  display.drawString(74, 26, String(sinalMedio) + "dBm");

  desenharIconeMicrofone(112, 40);

  // ---- LÓGICA DE STATUS: FALANDO / OUVINDO / OFF ----
  // FALANDO  -> eu apertei o botão (PTT local ativo)
  // OUVINDO  -> o COLEGA apertou o botão dele (recebi estaFalando = true)
  // OFF      -> nenhum dos dois está apertando
  if (microfone) {
    display.drawString(70, 43, "FALANDO");
  } else if (outroEstaFalando) {
    display.drawString(70, 43, "OUVINDO");
  } else {
    display.drawString(82, 43, "OFF");
  }
  // -----------------------------------------------------

  desenharVUMeterSimulado(2, 57, 104, 7);

  display.display();
}

// =====================================================
// MAC via eFuse
// =====================================================
String getDefaultMacAddress() {
  String mac = "";
  unsigned char mac_base[6] = {0};

  if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
    char buffer[18];
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }

  return mac;
}

void printMAC() {
  Serial.print("MAC (eFuse, default): ");
  Serial.println(getDefaultMacAddress());
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  printMAC();

  pinMode(POT_PIN, INPUT);
  pinMode(BOTAO_PIN, INPUT_PULLUP);
  // GPIO 33 removida - conexão agora é automática via ESP-NOW

  randomSeed(analogRead(POT_PIN));

  for (int i = 0; i < N_AMOSTRAS; i++) {
    leituras[i] = analogRead(POT_PIN);
    somaLeituras += leituras[i];
    delay(5);
  }
  bufferCheio = true;

  for (int i = 0; i < N_AMOSTRAS_RSSI; i++) {
    leiturasRSSI[i] = -100;
    somaRSSI += -100;
  }

  configurarEspNow();

  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(10, 8,  "Bem vindo");
  display.drawString(10, 26, "MAC Wi-Fi:");
  display.drawString(10, 42, getDefaultMacAddress());
  display.display();
  delay(3000);

  tensaoBateria = lerTensaoBateria();
  atualizarDisplay();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  while (Serial.available()) {
    Serial.read();
    printMAC();
  }

  verificarBotao();
  verificarTimeoutConexao(); // substitui verificarConexao()
  atualizarSpinner();
  atualizarVUMeterSimulado();

  // Envio periódico de telemetria (a cada 5 segundos)
  if (millis() - ultimaAtualizacao >= INTERVALO_ATUALIZACAO) {
    ultimaAtualizacao = millis();
    tensaoBateria = lerTensaoBateria();
    precisaRedesenhar = true;

    enviarTelemetria();

    Serial.printf("ON: %lu ms | ADC bruto: %d bits | BAT: %4.2f V | RSSI bruto: %d dBm | RSSI médio: %d dBm\n",
                   millis(), analogRead(POT_PIN), tensaoBateria, sinal, sinalMedio);
  }

  if (precisaRedesenhar) {
    atualizarDisplay();
    precisaRedesenhar = false;
  }
}