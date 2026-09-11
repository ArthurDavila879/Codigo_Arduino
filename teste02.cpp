#include <IRremote.hpp>

// =====================================================
// PINOS
// =====================================================

// ---------- L298N ----------
// L298N
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 7;

const int ENA = 5;  // PWM - Motor esquerdo
const int ENB = 6;  // PWM - Motor direito

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
// VARIÁVEIS DE ESTADO
// =====================================================

// Máquina de estados da partida (Regras Oficiais RoboCore)
// 0 = Aguardando, 1 = Ready (Tecla A), 2 = Start (Tecla B), 3 = Stop Permanente (Tecla C)
int estadoPartida = 0; 

// Variáveis para controle da fuga de borda sem delay
unsigned long tempoInicioFuga = 0;
int estadoFuga = 0;     // 0 = inativo, 1 = recuando, 2 = girando
int direcaoFuga = 0;    // 1 = girar direita, 2 = girar esquerda

// Variáveis dos ultrassônicos
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
  Serial.println(" MINI SUMO - V3 (REGRAS 2026) ");
  Serial.println("=================================");
  Serial.println("Aguardando comando READY (Tecla A)...");
}


// =====================================================
// LOOP PRINCIPAL
// =====================================================

void loop() {
  // 1. Sempre escuta o controle remoto
  verificarControle();

  // Se o robô NÃO estiver em estado de START (lutando), ele não faz nada
  if (estadoPartida != 2) {
    parar();
    estadoFuga = 0; // Cancela qualquer fuga em andamento se o juiz parar a luta
    return;
  }

  // --------------------------------
  // 2. PRIORIDADE MAX: GERENCIAR BORDA
  // --------------------------------
  gerenciarFugaBorda();

  // Se o robô está no meio de uma manobra de fuga, ele IGNORA o resto do loop.
  if (estadoFuga > 0) {
    return; 
  }

  // --------------------------------
  // 3. LER ULTRASSÔNICOS
  // --------------------------------
  if (millis() - ultimoUltrassom >= INTERVALO_ULTRASSOM) {
    ultimoUltrassom = millis();
    distanciaEsq = medirDistancia(TRIG_ESQ, ECHO_ESQ);
    distanciaDir = medirDistancia(TRIG_DIR, ECHO_DIR);
  }

  // --------------------------------
  // 4. DECIDIR MOVIMENTO
  // --------------------------------
  decidirMovimento();
}


// =====================================================
// CONTROLE REMOTO (PADRÃO ROBOCORE / REGRAS 2026)
// =====================================================

void verificarControle() {
  if (IrReceiver.decode()) {
    unsigned long codigo = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume(); // Libera para o próximo sinal

    // -----------------------------------------
    // REGRA 1: TECLA C (STOP) - 0xA15EFF00
    // O robô deve parar de forma permanente.
    // -----------------------------------------
    if (codigo == 0xA15EFF00) {
      estadoPartida = 3; // Stop Permanente
      Serial.println(">>> ESTADO: STOP (PARADA PERMANENTE) <<<");
      parar();
      return;
    }

    // Se estiver em Stop Permanente, ignora os outros botões (exige reset físico)
    if (estadoPartida == 3) {
      return; 
    }

    // -----------------------------------------
    // REGRA 2: TECLA A (READY) - 0xF30CFF00
    // O robô fica pronto e imóvel aguardando.
    // -----------------------------------------
    if (codigo == 0xF30CFF00) {
      estadoPartida = 1; // Ready
      Serial.println(">>> ESTADO: READY (PRONTO PARA LUTAR) <<<");
      parar();
    }
    
    // -----------------------------------------
    // REGRA 3: TECLA B (START) - 0xE718FF00
    // Inicia a luta (só funciona se estiver Ready)
    // -----------------------------------------
    else if (codigo == 0xE718FF00 && estadoPartida == 1) {
      estadoPartida = 2; // Start
      Serial.println(">>> ESTADO: START (LUTA INICIADA!) <<<");
    }
  }
}


// =====================================================
// NOVA LÓGICA DE FUGA DA BORDA (SEM DELAY)
// =====================================================

void gerenciarFugaBorda() {
  // LÓGICA 1: O robô já está no meio de uma fuga?
  if (estadoFuga > 0) {
    
    // Etapa 1: Está recuando
    if (estadoFuga == 1) { 
      if (millis() - tempoInicioFuga >= TEMPO_RECUO) {
        estadoFuga = 2; // Acabou de recuar, vai girar
        tempoInicioFuga = millis(); // Zera o cronômetro para o giro
        
        if (direcaoFuga == 1) girarDireita();
        else girarEsquerda();
      }
    } 
    // Etapa 2: Está girando
    else if (estadoFuga == 2) { 
      if (millis() - tempoInicioFuga >= TEMPO_GIRO) {
        estadoFuga = 0; // Acabou a fuga! Robô livre.
        parar();
      }
    }
    return; // Sai da função para não ler os sensores de linha de novo.
  }

  // LÓGICA 2: O robô NÃO está fugindo, vamos ler a linha branca
  bool bordaEsquerda = (digitalRead(IR_BORDA_ESQ) == LOW);
  bool bordaDireita = (digitalRead(IR_BORDA_DIR) == LOW);

  if (bordaEsquerda || bordaDireita) {
    Serial.println("!!! BORDA DETECTADA !!!");
    
    estadoFuga = 1; // Inicia a manobra (Recuo)
    tempoInicioFuga = millis(); // Marca o tempo que começou
    recuar();

    // Decide para onde vai girar DEPOIS de recuar
    if (bordaEsquerda && bordaDireita) {
      direcaoFuga = 1; // Padrão: Gira pra direita se os dois pegarem
    } else if (bordaEsquerda) {
      direcaoFuga = 1; // Gira pra direita
    } else if (bordaDireita) {
      direcaoFuga = 2; // Gira pra esquerda
    }
  }
}


// =====================================================
// DECISÃO DO MOVIMENTO
// =====================================================

void decidirMovimento() {
  bool encontrouEsquerda = distanciaValida(distanciaEsq) && distanciaEsq <= DISTANCIA_ATAQUE;
  bool encontrouDireita = distanciaValida(distanciaDir) && distanciaDir <= DISTANCIA_ATAQUE;

  // --------------------------------
  // Adversário nos dois sensores
  // --------------------------------
  if (encontrouEsquerda && encontrouDireita) {
    atacar();
    return;
  }

  // --------------------------------
  // Adversário à esquerda
  // --------------------------------
  if (encontrouEsquerda) {
    atacarEsquerda();
    return;
  }

  // --------------------------------
  // Adversário à direita
  // --------------------------------
  if (encontrouDireita) {
    atacarDireita();
    return;
  }

  // --------------------------------
  // Nenhum adversário
  // --------------------------------
  procurar();
}


// =====================================================
// ULTRASSÔNICO (COM FILTRO ANTI-RUÍDO)
// =====================================================

float medirDistancia(int trig, int echo) {
  // --- PRIMEIRA LEITURA ---
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long tempo1 = pulseIn(echo, HIGH, 25000);
  if (tempo1 == 0) return -1;

  float dist1 = (tempo1 * 0.000343) / 2.0;

  // Se a primeira leitura diz que é hora de atacar, vamos confirmar!
  if (dist1 > 0 && dist1 <= DISTANCIA_ATAQUE) {
    delay(5); // Pausa curtinha (5ms) para dissipar o som antigo
    
    // --- SEGUNDA LEITURA (CONFIRMAÇÃO) ---
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    unsigned long tempo2 = pulseIn(echo, HIGH, 25000);
    float dist2 = (tempo2 * 0.000343) / 2.0;

    // Se a diferença entre as duas leituras for menor que 10cm, é real!
    if (abs(dist1 - dist2) < 0.10) {
      return (dist1 + dist2) / 2.0; // Retorna a média
    } else {
      return -1; // Era um fantasma (ruído), ignorar.
    }
  }

  return dist1; // Se for longe, retorna normal
}

bool distanciaValida(float distancia) {
  return distancia > 0;
}


// =====================================================
// MOVIMENTAÇÃO
// =====================================================

void frente() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
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
