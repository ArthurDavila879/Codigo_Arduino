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
const uint8_t TRIG_DIR = 10;
const uint8_t ECHO_DIR = 11;

const uint8_t TRIG_ESQ = 8;
const uint8_t ECHO_ESQ = 9;

// TCRT5000
// SENSOR DE BORDA ESQUERDO REMOVIDO/COMENTADO
// const uint8_t IR_BORDA_ESQ = A2;

// ÚNICO SENSOR DE BORDA UTILIZADO: DIREITO
const uint8_t IR_BORDA_DIR = A1;

// Receptor IR
const uint8_t RECV_PIN = A0;


// =====================================================
// CONTROLE DO JUIZ
// =====================================================

const uint32_t CODIGO_READY = FF30CF;
const uint32_t CODIGO_START = FF18E7;
const uint32_t CODIGO_STOP  = FF7A85;


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

const float DISTANCIA_ATAQUE_MINIMA = 0.02f;
const float DISTANCIA_ATAQUE_MAXIMA = 0.80f;


// =====================================================
// TEMPOS
// =====================================================

const unsigned long TEMPO_RECUO = 250;
const unsigned long TEMPO_GIRO = 300;

const unsigned long INTERVALO_ULTRASSOM = 80;

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
  // Sensor de borda
  // SOMENTE DIREITO
  // ---------------------------

  pinMode(IR_BORDA_DIR, INPUT);

  // O sensor esquerdo foi desativado.
  // pinMode(IR_BORDA_ESQ, INPUT);


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
  Serial.println(F("Sensor de borda ativo: DIREITO"));
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // ===================================================
  // 1 - CONTROLE DO JUIZ
  // ===================================================

  verificarControle();

  debugSensores();


  // ===================================================
  // 2 - STOP PERMANENTE
  // ===================================================

  if (estadoPartida == STOP_PERMANENTE) {
    parar();
    return;
  }


  // ===================================================
  // 3 - AGUARDANDO / READY
  // ===================================================

  if (estadoPartida != START) {
    parar();
    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;
    return;
  }


  // ===================================================
  // 4 - BORDA
  // ===================================================

  gerenciarFugaBorda();

  if (estadoFuga != FUGA_INATIVA) {
    return;
  }


  // ===================================================
  // 5 - ULTRASSÔNICOS
  // ===================================================

  atualizarUltrassonicos();
  verificarControle();

  if (estadoPartida != START) {
    parar();
    return;
  }


  // ===================================================
  // 6 - DECISÃO
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

    if (estadoPartida == READY) {

      estadoPartida = START;

      ultimoUltrassom = 0;

      Serial.println(F(""));
      Serial.println(F(">>> START <<<"));
      Serial.println(F("Luta iniciada."));
    }

    else if (estadoPartida == START) {

      estadoPartida = READY;

      estadoFuga = FUGA_INATIVA;
      direcaoFuga = SEM_DIRECAO;

      parar();

      Serial.println(F(""));
      Serial.println(F(">>> START NOVAMENTE <<<"));
      Serial.println(F("Voltando para READY."));
      Serial.println(F("Aguardando nova partida."));
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

  // SOMENTE O TCRT DIREITO
  int tcrtDir = analogRead(IR_BORDA_DIR);

  Serial.print(F("TCRT DIREITO: "));
  Serial.print(tcrtDir);

  Serial.print(F(" | ULTRA E: "));

  if (distanciaEsq < 0) {
    Serial.print(F("SEM ECO"));
  } else {
    Serial.print(distanciaEsq * 100.0f, 1);
    Serial.print(F("cm"));
  }


  Serial.print(F(" | ULTRA D: "));

  if (distanciaDir < 0) {
    Serial.print(F("SEM ECO"));
  } else {
    Serial.print(distanciaDir * 100.0f, 1);
    Serial.print(F("cm"));
  }

  Serial.println();
}


// =====================================================
// FILTRO DO SENSOR DE BORDA DIREITO
// =====================================================

// Ajuste conforme os testes do sensor direito.
const int LIMIAR_BORDA_DIR = 767;

// Margem para evitar ficar alternando perto do limiar.
const int HISTERESE = 3;

// Quantas confirmações consecutivas são necessárias.
const uint8_t CONFIRMACOES_BORDA = 3;

uint8_t contadorBordaDir = 0;
bool estadoBordaDir = false;


// =====================================================
// MEDIANA DE 9 LEITURAS
// =====================================================

int lerSensorFiltrado(uint8_t pino) {

  const uint8_t N = 9;

  int valores[N];

  for (uint8_t i = 0; i < N; i++) {

    valores[i] = analogRead(pino);

    delayMicroseconds(150);
  }


  // Ordenação simples
  for (uint8_t i = 0; i < N - 1; i++) {

    for (uint8_t j = i + 1; j < N; j++) {

      if (valores[j] < valores[i]) {

        int temp = valores[i];

        valores[i] = valores[j];

        valores[j] = temp;
      }
    }
  }


  // Retorna valor central
  return valores[N / 2];
}


// =====================================================
// LEITURA DA BORDA
// SOMENTE SENSOR DIREITO
// =====================================================

bool lerBordaDireita() {

  int valor = lerSensorFiltrado(IR_BORDA_DIR);


  // Se já está detectando borda,
  // só sai do estado quando o valor subir
  // acima do limite + histerese.
  if (estadoBordaDir) {

    if (valor > LIMIAR_BORDA_DIR + HISTERESE) {

      estadoBordaDir = false;
      contadorBordaDir = 0;
    }

    return estadoBordaDir;
  }


  // Branco = valor menor
  if (valor <= 771) {

    if (contadorBordaDir < CONFIRMACOES_BORDA) {
      contadorBordaDir++;
    }

  } else {

    contadorBordaDir = 0;
  }


  if (contadorBordaDir >= CONFIRMACOES_BORDA) {

    estadoBordaDir = true;
    contadorBordaDir = 0;

    Serial.println(F(">>> BORDA DIREITA DETECTADA <<<"));
  }


  return estadoBordaDir;
}


// =====================================================
// GERENCIAMENTO DA FUGA
// =====================================================

void gerenciarFugaBorda() {

  verificarControle();


  if (estadoPartida != START) {

    parar();

    estadoFuga = FUGA_INATIVA;

    return;
  }


  // ===================================================
  // RECUANDO
  // ===================================================

  if (estadoFuga == FUGA_RECUANDO) {

    if (millis() - tempoInicioFuga >= TEMPO_RECUO) {

      estadoFuga = FUGA_GIRANDO;

      tempoInicioFuga = millis();

      // Como somente o sensor direito está ativo,
      // quando ele encontra a borda o robô gira para a esquerda.
      girarEsquerda();
    }

    return;
  }


  // ===================================================
  // GIRANDO
  // ===================================================

  if (estadoFuga == FUGA_GIRANDO) {

    if (millis() - tempoInicioFuga >= TEMPO_GIRO) {

      estadoFuga = FUGA_INATIVA;
      direcaoFuga = SEM_DIRECAO;

      parar();
    }

    return;
  }


  // ===================================================
  // VERIFICAÇÃO DA BORDA
  // SOMENTE DIREITA
  // ===================================================

  bool bordaDireita = lerBordaDireita();


  if (!bordaDireita) {
    return;
  }


  Serial.println(F(">>> BORDA DIREITA -> RECUANDO E GIRANDO ESQUERDA <<<"));

  direcaoFuga = FUGA_ESQUERDA;

  estadoFuga = FUGA_RECUANDO;

  tempoInicioFuga = millis();

  recuar();
}


// =====================================================
// ATUALIZAÇÃO DOS ULTRASSÔNICOS
// =====================================================

void atualizarUltrassonicos() {

  if (millis() - ultimoUltrassom < INTERVALO_ULTRASSOM) {
    return;
  }

  ultimoUltrassom = millis();


  // ===================================================
  // ESQUERDO
  // ===================================================

  distanciaEsq =
    medirDistancia(TRIG_ESQ, ECHO_ESQ);


  verificarControle();


  if (estadoPartida != START) {
    return;
  }


  delayMicroseconds(3000);


  // ===================================================
  // DIREITO
  // ===================================================

  distanciaDir =
    medirDistancia(TRIG_DIR, ECHO_DIR);
}


// =====================================================
// MEDIR DISTÂNCIA
// =====================================================

float medirDistancia(
  uint8_t trig,
  uint8_t echo
) {

  digitalWrite(trig, LOW);

  delayMicroseconds(2);

  digitalWrite(trig, HIGH);

  delayMicroseconds(10);

  digitalWrite(trig, LOW);


  unsigned long tempo =
    pulseIn(
      echo,
      HIGH,
      TIMEOUT_ULTRASSOM_US
    );


  if (tempo == 0) {
    return -1.0f;
  }


  float distancia =
    (tempo * 0.000343f) / 2.0f;


  return distancia;
}


// =====================================================
// DISTÂNCIA VÁLIDA PARA ATAQUE
// =====================================================

bool distanciaValida(float distancia) {

  return
    distancia >= DISTANCIA_ATAQUE_MINIMA &&
    distancia <= DISTANCIA_ATAQUE_MAXIMA;
}


// =====================================================
// DECISÃO
// =====================================================

void decidirMovimento() {

  bool encontrouEsquerda =
    distanciaValida(distanciaEsq);

  bool encontrouDireita =
    distanciaValida(distanciaDir);


  // ===================================================
  // AMBOS DETECTARAM
  // ===================================================

  if (encontrouEsquerda &&
      encontrouDireita) {

    Serial.println("ATACAR RETO");
    delay(1000);
    atacar();

    return;
  }


  // ===================================================
  // ESQUERDA
  // ===================================================

  if (encontrouEsquerda) {

    Serial.println("ATACAR ESQUERDA");
    delay(1000);
    atacarEsquerda();

    return;
  }


  // ===================================================
  // DIREITA
  // ===================================================

  if (encontrouDireita) {

    Serial.println("ATACAR DIREITA");
    delay(1000);
    atacarDireita();

    return;
  }


  // ===================================================
  // NENHUM
  // ===================================================

  procurar();
}


// =====================================================
// ATAQUE RETO
// =====================================================

void atacar() {

  // MOTOR ESQUERDO
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // MOTOR DIREITO INVERTIDO
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, VELOCIDADE_ATAQUE);
  analogWrite(ENB, VELOCIDADE_ATAQUE);
}


// =====================================================
// RECUAR
// =====================================================

void recuar() {

  // MOTOR ESQUERDO
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // MOTOR DIREITO INVERTIDO
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_RECUO);
  analogWrite(ENB, VELOCIDADE_RECUO);
}


// =====================================================
// GIRAR ESQUERDA
// =====================================================

void girarEsquerda() {

  // Motor esquerdo para trás
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Motor direito para frente
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
}


// =====================================================
// GIRAR DIREITA
// =====================================================

void girarDireita() {

  // Motor esquerdo para frente
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor direito para trás
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
}


// =====================================================
// ATAQUE PARA ESQUERDA
// =====================================================

void atacarEsquerda() {

  // Motor esquerdo mais lento
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor direito mais rápido
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, VELOCIDADE_BUSCA);
  analogWrite(ENB, VELOCIDADE_ATAQUE);
}


// =====================================================
// ATAQUE PARA DIREITA
// =====================================================

void atacarDireita() {

  // Motor esquerdo mais rápido
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor direito mais lento
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, VELOCIDADE_ATAQUE);
  analogWrite(ENB, VELOCIDADE_BUSCA);
}


// =====================================================
// PROCURAR ADVERSÁRIO
// =====================================================

void procurar() {

  // Motor esquerdo para frente
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor direito para trás
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, VELOCIDADE_BUSCA);
  analogWrite(ENB, VELOCIDADE_BUSCA);
}


// =====================================================
// PARAR
// =====================================================

void parar() {

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}