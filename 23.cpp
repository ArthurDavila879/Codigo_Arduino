#include <IRremote.hpp>

// =====================================================
// PINOS
// =====================================================

// L298N
const uint8_t IN1 = 2;
const uint8_t IN2 = 3;
const uint8_t IN3 = 4;
const uint8_t IN4 = 12;

const uint8_t ENA = 5;
const uint8_t ENB = 6;

// HC-SR04
const uint8_t TRIG_ESQ = 8;
const uint8_t ECHO_ESQ = 9;

const uint8_t TRIG_DIR = 10;
const uint8_t ECHO_DIR = 11;

// TCRT5000
const uint8_t IR_BORDA_ESQ = A2;
const uint8_t IR_BORDA_DIR = A1;

// Receptor IR
const uint8_t RECV_PIN = A0;


// =====================================================
// CONTROLE DO JUIZ
// =====================================================

const uint32_t CODIGO_READY = 3994155780UL;
const uint32_t CODIGO_START = 3977444100UL;
const uint32_t CODIGO_STOP  = 3960732420UL;


// =====================================================
// ESTADO DA PARTIDA
// =====================================================

enum EstadoPartida : uint8_t {
  AGUARDANDO,
  READY,
  START,
  STOP_PERMANENTE
};

EstadoPartida estadoPartida = AGUARDANDO;


// =====================================================
// VELOCIDADES
// =====================================================

const uint8_t VELOCIDADE_ATAQUE = 255;
const uint8_t VELOCIDADE_BUSCA  = 170;
const uint8_t VELOCIDADE_RECUO  = 200;
const uint8_t VELOCIDADE_GIRO   = 200;


// =====================================================
// CONFIGURAÇÕES DOS SENSORES
// =====================================================

// Metros
const float DISTANCIA_ATAQUE_MINIMA = 0.02f;
const float DISTANCIA_ATAQUE_MAXIMA = 0.80f;

// IMPORTANTE:
// Esse valor ainda precisa ser calibrado no robô real.
const int LIMIAR_BORDA = 450;

// Atualmente:
// valor > 450 = borda.
//
// Se no Serial você descobrir que BRANCO gera valor MENOR,
// troque ">" por "<" na função lerBorda().


// =====================================================
// TEMPOS
// =====================================================

const unsigned long TEMPO_RECUO = 250;
const unsigned long TEMPO_GIRO = 300;

const unsigned long INTERVALO_ULTRASSOM = 80;

// 7 ms é suficiente para a região que estamos utilizando.
const unsigned long TIMEOUT_ULTRASSOM_US = 7000;


// =====================================================
// FUGA DA BORDA
// =====================================================

enum EstadoFuga : uint8_t {
  FUGA_INATIVA,
  FUGA_RECUANDO,
  FUGA_GIRANDO
};

enum DirecaoFuga : uint8_t {
  SEM_DIRECAO,
  FUGA_DIREITA,
  FUGA_ESQUERDA
};

EstadoFuga estadoFuga = FUGA_INATIVA;
DirecaoFuga direcaoFuga = SEM_DIRECAO;

unsigned long tempoInicioFuga = 0;


// =====================================================
// ULTRASSÔNICOS
// =====================================================

float distanciaEsq = -1.0f;
float distanciaDir = -1.0f;

unsigned long ultimoUltrassom = 0;


// =====================================================
// DEBUG
// =====================================================

unsigned long ultimoDebug = 0;


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // ---------------------------
  // Motores
  // ---------------------------

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  parar();


  // ---------------------------
  // Ultrassônicos
  // ---------------------------

  pinMode(TRIG_ESQ, OUTPUT);
  pinMode(ECHO_ESQ, INPUT);

  pinMode(TRIG_DIR, OUTPUT);
  pinMode(ECHO_DIR, INPUT);

  digitalWrite(TRIG_ESQ, LOW);
  digitalWrite(TRIG_DIR, LOW);


  // ---------------------------
  // Sensores de borda
  // ---------------------------

  pinMode(IR_BORDA_ESQ, INPUT);
  pinMode(IR_BORDA_DIR, INPUT);


  // ---------------------------
  // Controle IR
  // ---------------------------

  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);


  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F(" MINI SUMO - INOVAWEEK 2026"));
  Serial.println(F("=============================="));
  Serial.println(F("Estado: AGUARDANDO"));
  Serial.println(F("Aguardando READY..."));
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // ===================================================
  // 1 - CONTROLE DO JUIZ
  // ===================================================

  verificarControle();


  // ===================================================
  // 2 - DEBUG DOS SENSORES
  // ===================================================

  debugSensores();


  // ===================================================
  // 3 - STOP PERMANENTE
  // ===================================================

  if (estadoPartida == STOP_PERMANENTE) {

    parar();

    return;
  }


  // ===================================================
  // 4 - AGUARDANDO / READY
  // ===================================================

  if (estadoPartida != START) {

    parar();

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    return;
  }


  // ===================================================
  // 5 - BORDA
  // ===================================================

  gerenciarFugaBorda();


  if (estadoFuga != FUGA_INATIVA) {
    return;
  }


  // ===================================================
  // 6 - ULTRASSÔNICOS
  // ===================================================

  atualizarUltrassonicos();


  // Verifica novamente o controle.
  verificarControle();


  if (estadoPartida != START) {

    parar();

    return;
  }


  // ===================================================
  // 7 - DECISÃO
  // ===================================================

  decidirMovimento();
}


// =====================================================
// CONTROLE DO JUIZ
// =====================================================

void verificarControle() {

  if (!IrReceiver.decode()) {
    return;
  }


  uint32_t codigo =
    IrReceiver.decodedIRData.decodedRawData;


  IrReceiver.resume();


  Serial.print(F("IR RAW: 0x"));
  Serial.println(codigo, HEX);


  // ===================================================
  // STOP
  // ===================================================

  if (codigo == CODIGO_STOP) {

    estadoPartida = STOP_PERMANENTE;

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    parar();

    Serial.println(F(""));
    Serial.println(F(">>> STOP PERMANENTE <<<"));
    Serial.println(F("Reinicie fisicamente o robo."));

    return;
  }


  // Depois de STOP nada funciona.
  if (estadoPartida == STOP_PERMANENTE) {

    parar();

    return;
  }


  // ===================================================
  // READY
  // ===================================================

  if (codigo == CODIGO_READY) {

    estadoPartida = READY;

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    parar();

    Serial.println(F(""));
    Serial.println(F(">>> READY <<<"));
    Serial.println(F("Robo parado."));

    return;
  }


  // ===================================================
  // START
  // ===================================================

  if (codigo == CODIGO_START) {

    // START somente inicia se estiver READY.
    //
    // Se receber START novamente enquanto já está
    // lutando, nada acontece.

    if (estadoPartida == READY) {

      estadoPartida = START;

      ultimoUltrassom = 0;

      Serial.println(F(""));
      Serial.println(F(">>> START <<<"));
      Serial.println(F("Luta iniciada."));
    }

    return;
  }
}


// =====================================================
// DEBUG DOS SENSORES
// =====================================================

void debugSensores() {

  if (millis() - ultimoDebug < 300) {
    return;
  }

  ultimoDebug = millis();


  int tcrtEsq = analogRead(IR_BORDA_ESQ);
  int tcrtDir = analogRead(IR_BORDA_DIR);


  Serial.print(F("TCRT E: "));
  Serial.print(tcrtEsq);

  Serial.print(F(" |