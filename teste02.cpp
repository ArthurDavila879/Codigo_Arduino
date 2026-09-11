#include <IRremote.hpp>

// =====================================================
// PINOS
// =====================================================

// ---------- L298N ----------
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;

const int ENA = 6;  // PWM - Motor esquerdo
const int ENB = 7;  // PWM - Motor direito

// ---------- Ultrassônico esquerdo ----------
const int TRIG_ESQ = 8;
const int ECHO_ESQ = 9;

// ---------- Ultrassônico direito ----------
const int TRIG_DIR = 10;
const int ECHO_DIR = 11;

// ---------- TCRT5000 ----------
const int IR_BORDA_ESQ = 12;
const int IR_BORDA_DIR = 13;

// ---------- Receptor IR ----------
const int RECV_PIN = A0;


// =====================================================
// CONFIGURAÇÕES
// =====================================================

// Velocidades dos motores (0 - 255)
const int VELOCIDADE_ATAQUE = 255;
const int VELOCIDADE_BUSCA = 170;
const int VELOCIDADE_RECUO = 200;
const int VELOCIDADE_GIRO = 200;

// Distância para considerar que encontrou o adversário
const float DISTANCIA_ATAQUE = 0.60; // metros

// Tempos da reação à borda
const unsigned long TEMPO_RECUO = 250;
const unsigned long TEMPO_GIRO = 300;

// Intervalo entre medições dos ultrassônicos
const unsigned long INTERVALO_ULTRASSOM = 80;


// =====================================================
// VARIÁVEIS
// =====================================================

bool roboAtivo = false;

float distanciaEsq = -1;
float distanciaDir = -1;

unsigned long ultimoUltrassom = 0;


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // -----------------------------
  // Motores
  // -----------------------------

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  parar();

  // -----------------------------
  // Ultrassônicos
  // -----------------------------

  pinMode(TRIG_ESQ, OUTPUT);
  pinMode(ECHO_ESQ, INPUT);

  pinMode(TRIG_DIR, OUTPUT);
  pinMode(ECHO_DIR, INPUT);

  digitalWrite(TRIG_ESQ, LOW);
  digitalWrite(TRIG_DIR, LOW);

  // -----------------------------
  // Sensores de borda
  // -----------------------------

  pinMode(IR_BORDA_ESQ, INPUT);
  pinMode(IR_BORDA_DIR, INPUT);

  // -----------------------------
  // Receptor IR
  // -----------------------------

  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);

  Serial.println("=================================");
  Serial.println("      MINI SUMO - V1");
  Serial.println("=================================");
  Serial.println("Aguardando comando...");
}


// =====================================================
// LOOP PRINCIPAL
// =====================================================

void loop() {

  verificarControle();

  if (!roboAtivo) {
    parar();
    return;
  }

  // --------------------------------
  // 1. PRIORIDADE: BORDA
  // --------------------------------

  if (detectarBorda()) {

    evitarBorda();

    return;
  }

  // --------------------------------
  // 2. LER ULTRASSÔNICOS
  // --------------------------------

  if (millis() - ultimoUltrassom >= INTERVALO_ULTRASSOM) {

    ultimoUltrassom = millis();

    distanciaEsq = medirDistancia(TRIG_ESQ, ECHO_ESQ);
    distanciaDir = medirDistancia(TRIG_DIR, ECHO_DIR);
  }

  // --------------------------------
  // 3. DECIDIR MOVIMENTO
  // --------------------------------

  decidirMovimento();
}


// =====================================================
// CONTROLE REMOTO
// =====================================================

void verificarControle() {

  if (IrReceiver.decode()) {

    unsigned long codigo =
      IrReceiver.decodedIRData.decodedRawData;

    Serial.print("Codigo IR: 0x");
    Serial.println(codigo, HEX);

    // Mesmo botão usado no código original
    if (codigo == 0x5DA2FF00) {

      roboAtivo = !roboAtivo;

      if (roboAtivo) {

        Serial.println(">>> ROBO INICIADO <<<");

      } else {

        Serial.println(">>> ROBO PARADO <<<");

        parar();
      }
    }

    IrReceiver.resume();
  }
}


// =====================================================
// DETECÇÃO DA BORDA
// =====================================================

bool detectarBorda() {

  int esquerda = digitalRead(IR_BORDA_ESQ);
  int direita = digitalRead(IR_BORDA_DIR);

  /*
     ATENÇÃO:

     Esta versão considera LOW como detecção
     de borda.

     Dependendo do módulo TCRT5000, pode ser
     necessário inverter para HIGH.
  */

  bool bordaEsquerda = (esquerda == LOW);
  bool bordaDireita = (direita == LOW);

  return bordaEsquerda || bordaDireita;
}


// =====================================================
// EVITAR BORDA
// =====================================================

void evitarBorda() {

  int esquerda = digitalRead(IR_BORDA_ESQ);
  int direita = digitalRead(IR_BORDA_DIR);

  bool bordaEsquerda = (esquerda == LOW);
  bool bordaDireita = (direita == LOW);

  Serial.println("!!! BORDA DETECTADA !!!");

  // --------------------------------
  // Ambos detectaram borda
  // --------------------------------

  if (bordaEsquerda && bordaDireita) {

    recuar();

    delay(TEMPO_RECUO);

    return;
  }

  // --------------------------------
  // Borda esquerda
  // --------------------------------

  if (bordaEsquerda) {

    recuar();

    delay(TEMPO_RECUO);

    girarDireita();

    delay(TEMPO_GIRO);

    return;
  }

  // --------------------------------
  // Borda direita
  // --------------------------------

  if (bordaDireita) {

    recuar();

    delay(TEMPO_RECUO);

    girarEsquerda();

    delay(TEMPO_GIRO);

    return;
  }
}


// =====================================================
// DECISÃO DO MOVIMENTO
// =====================================================

void decidirMovimento() {

  bool encontrouEsquerda =
    distanciaValida(distanciaEsq) &&
    distanciaEsq <= DISTANCIA_ATAQUE;

  bool encontrouDireita =
    distanciaValida(distanciaDir) &&
    distanciaDir <= DISTANCIA_ATAQUE;


  // --------------------------------
  // Adversário nos dois sensores
  // --------------------------------

  if (encontrouEsquerda && encontrouDireita) {

    Serial.println(">>> ADVERSARIO A FRENTE <<<");

    atacar();

    return;
  }


  // --------------------------------
  // Adversário à esquerda
  // --------------------------------

  if (encontrouEsquerda) {

    Serial.println(">>> ADVERSARIO ESQUERDA <<<");

    atacarEsquerda();

    return;
  }


  // --------------------------------
  // Adversário à direita
  // --------------------------------

  if (encontrouDireita) {

    Serial.println(">>> ADVERSARIO DIREITA <<<");

    atacarDireita();

    return;
  }


  // --------------------------------
  // Nenhum adversário
  // --------------------------------

  procurar();
}


// =====================================================
// ULTRASSÔNICO
// =====================================================

float medirDistancia(int trig, int echo) {

  // Dispara pulso
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig, LOW);

  // Timeout de 25 ms
  unsigned long tempo =
    pulseIn(echo, HIGH, 25000);

  // Nenhum retorno
  if (tempo == 0) {
    return -1;
  }

  // Velocidade aproximada do som:
  // 0,000343 m/us
  float distancia =
    (tempo * 0.000343) / 2.0;

  return distancia;
}


bool distanciaValida(float distancia) {

  return distancia > 0;
}


// =====================================================
// MOVIMENTAÇÃO
// =====================================================

void frente() {

  // Motor esquerdo
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor direito
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_ATAQUE);
  analogWrite(ENB, VELOCIDADE_ATAQUE);
}


void atacar() {

  analogWrite(ENA, VELOCIDADE_ATAQUE);
  analogWrite(ENB, VELOCIDADE_ATAQUE);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void recuar() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, VELOCIDADE_RECUO);
  analogWrite(ENB, VELOCIDADE_RECUO);
}


void girarEsquerda() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
}


void girarDireita() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
}


// =====================================================
// ATAQUE DIRECIONAL
// =====================================================

void atacarEsquerda() {

  analogWrite(ENA, VELOCIDADE_BUSCA);
  analogWrite(ENB, VELOCIDADE_ATAQUE);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void atacarDireita() {

  analogWrite(ENA, VELOCIDADE_ATAQUE);
  analogWrite(ENB, VELOCIDADE_BUSCA);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


// =====================================================
// PROCURAR ADVERSÁRIO
// =====================================================

void procurar() {

  /*
     Enquanto não encontra o adversário,
     o robô gira procurando.
  */

  analogWrite(ENA, VELOCIDADE_BUSCA);
  analogWrite(ENB, VELOCIDADE_BUSCA);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


// =====================================================
// PARAR
// =====================================================

void parar() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
