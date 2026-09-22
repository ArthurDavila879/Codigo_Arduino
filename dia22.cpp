#include <IRremote.hpp>

// =====================================================
// PINOS
// =====================================================

// L298N
const uint8_t IN1 = 2;
const uint8_t IN2 = 3;
const uint8_t IN3 = 4;
const uint8_t IN4 = 7;

const uint8_t ENA = 5;  // PWM - Motor esquerdo
const uint8_t ENB = 6;  // PWM - Motor direito

// Ultrassônicos
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
//
// IMPORTANTE:
// Estes valores RAW são os que você estava utilizando.
// Antes da competição, confirme no Serial com o controle
// oficial da competição.
//
// Regulamento:
// A = FF30CF -> READY
// B = FF18E7 -> START
// C = FF7A85 -> STOP
//
// =====================================================

const uint32_t CODIGO_READY = 3994155780UL;
const uint32_t CODIGO_START = 3977444100UL;
const uint32_t CODIGO_STOP  = 3960732420UL;


// =====================================================
// ESTADOS DA PARTIDA
// =====================================================

enum EstadoPartida : uint8_t {
  AGUARDANDO = 0,
  READY = 1,
  START = 2,
  STOP_PERMANENTE = 3
};

EstadoPartida estadoPartida = AGUARDANDO;


// =====================================================
// CONFIGURAÇÕES DOS MOTORES
// =====================================================

const uint8_t VELOCIDADE_ATAQUE = 255;
const uint8_t VELOCIDADE_BUSCA  = 170;
const uint8_t VELOCIDADE_RECUO  = 200;
const uint8_t VELOCIDADE_GIRO   = 200;


// =====================================================
// CONFIGURAÇÕES DOS SENSORES
// =====================================================

// Distâncias em metros
const float DISTANCIA_ATAQUE_MAXIMA = 0.80f;
const float DISTANCIA_ATAQUE_MINIMA = 0.02f;

// Diferença máxima entre duas leituras para confirmar alvo
const float DIFERENCA_MAXIMA_ULTRASSOM = 0.10f;

// TCRT5000
// PRECISA ser calibrado no dojô real.
const int LIMIAR_BORDA = 250;


// =====================================================
// TEMPOS
// =====================================================

const unsigned long TEMPO_RECUO = 250;
const unsigned long TEMPO_GIRO  = 300;

const unsigned long INTERVALO_ULTRASSOM = 80;

// 80 cm correspondem a aproximadamente 4,7 ms de eco.
// 7 ms deixa uma margem sem bloquear 25 ms desnecessariamente.
const unsigned long TIMEOUT_ULTRASSOM_US = 7000;


// =====================================================
// ESTADO DA FUGA
// =====================================================

enum EstadoFuga : uint8_t {
  FUGA_INATIVA = 0,
  FUGA_RECUANDO = 1,
  FUGA_GIRANDO = 2
};

enum DirecaoFuga : uint8_t {
  SEM_DIRECAO = 0,
  FUGA_DIREITA = 1,
  FUGA_ESQUERDA = 2
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
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // Motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  parar();

  // Ultrassônico esquerdo
  pinMode(TRIG_ESQ, OUTPUT);
  pinMode(ECHO_ESQ, INPUT);

  // Ultrassônico direito
  pinMode(TRIG_DIR, OUTPUT);
  pinMode(ECHO_DIR, INPUT);

  digitalWrite(TRIG_ESQ, LOW);
  digitalWrite(TRIG_DIR, LOW);

  // Sensores de borda
  pinMode(IR_BORDA_ESQ, INPUT);
  pinMode(IR_BORDA_DIR, INPUT);

  // Receptor IR
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);

  Serial.println();
  Serial.println(F("================================="));
  Serial.println(F(" MINI SUMO - INOVAWEEK 2026"));
  Serial.println(F("================================="));
  Serial.println(F("Estado: AGUARDANDO"));
  Serial.println(F("Aguardando READY (A)..."));
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // -------------------------------------------------
  // PRIORIDADE ABSOLUTA:
  // controle do juiz
  // -------------------------------------------------

  verificarControle();

  // STOP é permanente.
  if (estadoPartida == STOP_PERMANENTE) {
    parar();
    return;
  }

  // READY ou AGUARDANDO:
  // robô obrigatoriamente imóvel.
  if (estadoPartida != START) {
    parar();

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    return;
  }


  // -------------------------------------------------
  // PRIORIDADE 2:
  // borda
  // -------------------------------------------------

  gerenciarFugaBorda();

  if (estadoFuga != FUGA_INATIVA) {
    return;
  }


  // -------------------------------------------------
  // PRIORIDADE 3:
  // adversário
  // -------------------------------------------------

  atualizarUltrassonicos();


  // O controle pode ter recebido algum comando
  // durante o processamento anterior.
  verificarControle();

  if (estadoPartida != START) {
    parar();
    return;
  }


  // -------------------------------------------------
  // DECISÃO
  // -------------------------------------------------

  decidirMovimento();
}


// =====================================================
// CONTROLE DO JUIZ
// =====================================================

void verificarControle() {

  if (!IrReceiver.decode()) {
    return;
  }

  uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;

  // Libera imediatamente o receptor.
  IrReceiver.resume();


  // -------------------------------------------------
  // DEBUG
  // -------------------------------------------------

  Serial.print(F("IR RAW: 0x"));
  Serial.println(codigo, HEX);


  // -------------------------------------------------
  // STOP
  //
  // Maior prioridade.
  // Depois disso somente reset/desligar-ligar.
  // -------------------------------------------------

  if (codigo == CODIGO_STOP) {

    estadoPartida = STOP_PERMANENTE;

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    parar();

    Serial.println(F(">>> STOP PERMANENTE <<<"));
    Serial.println(F("Reinicie fisicamente o robo."));

    return;
  }


  // -------------------------------------------------
  // Se já recebeu STOP, absolutamente nada
  // pode reativar o robô.
  // -------------------------------------------------

  if (estadoPartida == STOP_PERMANENTE) {
    parar();
    return;
  }


  // -------------------------------------------------
  // READY
  //
  // Pode:
  // - preparar inicialmente;
  // - pausar temporariamente durante START.
  // -------------------------------------------------

  if (codigo == CODIGO_READY) {

    estadoPartida = READY;

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    parar();

    Serial.println(F(">>> READY <<<"));
    Serial.println(F("Robo imovel aguardando START."));

    return;
  }


  // -------------------------------------------------
  // START
  //
  // Só entra em START se estiver em READY.
  //
  // Se receber START enquanto já está START,
  // simplesmente continua lutando.
  // -------------------------------------------------

  if (codigo == CODIGO_START) {

    if (estadoPartida == READY) {

      estadoPartida = START;

      // Força uma nova leitura rapidamente.
      ultimoUltrassom = 0;

      Serial.println(F(">>> START <<<"));
      Serial.println(F("Luta iniciada."));
    }

    return;
  }
}


// =====================================================
// BORDA
// =====================================================

bool lerBorda(uint8_t pino) {

  int valor = analogRead(pino);

  // ATENÇÃO:
  // confirme experimentalmente se branco realmente
  // produz valor MAIOR no seu sensor.
  return valor > LIMIAR_BORDA;
}


// =====================================================
// FUGA DA BORDA
// =====================================================

void gerenciarFugaBorda() {

  // Controle do juiz continua tendo prioridade.
  verificarControle();

  if (estadoPartida != START) {
    parar();
    estadoFuga = FUGA_INATIVA;
    return;
  }


  // -------------------------------------------------
  // FUGA JÁ ESTÁ EM ANDAMENTO
  // -------------------------------------------------

  if (estadoFuga == FUGA_RECUANDO) {

    if (millis() - tempoInicioFuga >= TEMPO_RECUO) {

      estadoFuga = FUGA_GIRANDO;
      tempoInicioFuga = millis();

      if (direcaoFuga == FUGA_DIREITA) {
        girarDireita();
      } else {
        girarEsquerda();
      }
    }

    return;
  }


  if (estadoFuga == FUGA_GIRANDO) {

    if (millis() - tempoInicioFuga >= TEMPO_GIRO) {

      estadoFuga = FUGA_INATIVA;
      direcaoFuga = SEM_DIRECAO;

      parar();
    }

    return;
  }


  // -------------------------------------------------
  // NÃO ESTÁ FUGINDO:
  // verificar borda.
  // -------------------------------------------------

  bool bordaEsquerda = lerBorda(IR_BORDA_ESQ);
  bool bordaDireita  = lerBorda(IR_BORDA_DIR);


  if (!bordaEsquerda && !bordaDireita) {
    return;
  }


  Serial.print(F("BORDA: "));

  if (bordaEsquerda && bordaDireita) {

    Serial.println(F("AMBOS"));

    // Padrão quando ambos detectarem.
    direcaoFuga = FUGA_DIREITA;

  } else if (bordaEsquerda) {

    Serial.println(F("ESQUERDA"));

    // Borda à esquerda -> fugir para direita.
    direcaoFuga = FUGA_DIREITA;

  } else {

    Serial.println(F("DIREITA"));

    // Borda à direita -> fugir para esquerda.
    direcaoFuga = FUGA_ESQUERDA;
  }


  estadoFuga = FUGA_RECUANDO;
  tempoInicioFuga = millis();

  recuar();
}


// =====================================================
// ULTRASSÔNICOS
// =====================================================

void atualizarUltrassonicos() {

  if (millis() - ultimoUltrassom < INTERVALO_ULTRASSOM) {
    return;
  }

  ultimoUltrassom = millis();


  // -------------------------------------------------
  // ESQUERDO
  // -------------------------------------------------

  distanciaEsq = medirDistancia(TRIG_ESQ, ECHO_ESQ);

  verificarControle();

  if (estadoPartida != START) {
    return;
  }


  // -------------------------------------------------
  // DIREITO
  // -------------------------------------------------

  distanciaDir = medirDistancia(TRIG_DIR, ECHO_DIR);
}


// =====================================================
// MEDIÇÃO ULTRASSÔNICA
// =====================================================

float medirDistancia(uint8_t trig, uint8_t echo) {

  float dist1 = leituraUltrassom(trig, echo);

  if (dist1 < 0.0f) {
    return -1.0f;
  }


  // Fora da região de ataque:
  // não precisamos confirmar.
  if (dist1 > DISTANCIA_ATAQUE_MAXIMA) {
    return dist1;
  }


  // -------------------------------------------------
  // Segunda leitura para confirmar alvo.
  //
  // Não usamos delay(5).
  // -------------------------------------------------

  float dist2 = leituraUltrassom(trig, echo);

  if (dist2 < 0.0f) {
    return -1.0f;
  }


  // Diferença absoluta entre floats.
  float diferenca = dist1 - dist2;

  if (diferenca < 0.0f) {
    diferenca = -diferenca;
  }


  if (diferenca <= DIFERENCA_MAXIMA_ULTRASSOM) {

    return (dist1 + dist2) / 2.0f;
  }


  // Leituras incompatíveis = provável ruído.
  return -1.0f;
}


// =====================================================
// LEITURA BÁSICA HC-SR04
// =====================================================

float leituraUltrassom(uint8_t trig, uint8_t echo) {

  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig, LOW);


  unsigned long tempo =
    pulseIn(echo, HIGH, TIMEOUT_ULTRASSOM_US);


  if (tempo == 0) {
    return -1.0f;
  }


  // velocidade do som ≈ 343 m/s
  return (tempo * 0.000343f) / 2.0f;
}


// =====================================================
// DISTÂNCIA VÁLIDA
// =====================================================

bool distanciaValida(float distancia) {

  return distancia >= DISTANCIA_ATAQUE_MINIMA &&
         distancia <= DISTANCIA_ATAQUE_MAXIMA;
}


// =====================================================
// DECISÃO DE MOVIMENTO
// =====================================================

void decidirMovimento() {

  bool encontrouEsquerda = distanciaValida(distanciaEsq);
  bool encontrouDireita  = distanciaValida(distanciaDir);


  // Os dois sensores detectaram.
  if (encontrouEsquerda && encontrouDireita) {

    atacar();

    return;
  }


  // Somente esquerda.
  if (encontrouEsquerda) {

    atacarEsquerda();

    return;
  }


  // Somente direita.
  if (encontrouDireita) {

    atacarDireita();

    return;
  }


  // Nenhum adversário.
  procurar();
}


// =====================================================
// MOTORES
// =====================================================

void atacar() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_ATAQUE);
  analogWrite(ENB, VELOCIDADE_ATAQUE);
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

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_BUSCA);
  analogWrite(ENB, VELOCIDADE_ATAQUE);
}


void atacarDireita() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_ATAQUE);
  analogWrite(ENB, VELOCIDADE_BUSCA);
}


// =====================================================
// BUSCA
// =====================================================

void procurar() {

  // Giro no próprio eixo.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, VELOCIDADE_BUSCA);
  analogWrite(ENB, VELOCIDADE_BUSCA);
}


// =====================================================
// PARAR
// =====================================================

void parar() {

  // Primeiro remove PWM.
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}