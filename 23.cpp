#include <IRremote.hpp>

// =====================================================
// MINI SUMO - INOVAWEEK 2026
// =====================================================
//
// PRIORIDADES:
//
// 1. STOP / READY / START
// 2. BORDA
// 3. ADVERSARIO
// 4. BUSCA
//
// =====================================================


// =====================================================
// DEBUG
// =====================================================
//
// true  = mostra sensores no Serial
// false = recomendado para competição
//

const bool DEBUG_SERIAL = true;


// =====================================================
// PINOS
// =====================================================

// ----------------------
// L298N
// ----------------------

const uint8_t IN1 = 2;
const uint8_t IN2 = 3;

const uint8_t IN3 = 4;
const uint8_t IN4 = 12;

const uint8_t ENA = 5;   // PWM motor esquerdo
const uint8_t ENB = 6;   // PWM motor direito


// ----------------------
// HC-SR04 ESQUERDO
// ----------------------

const uint8_t TRIG_ESQ = 8;
const uint8_t ECHO_ESQ = 9;


// ----------------------
// HC-SR04 DIREITO
// ----------------------

const uint8_t TRIG_DIR = 10;
const uint8_t ECHO_DIR = 11;


// ----------------------
// TCRT5000
// ----------------------

const uint8_t IR_BORDA_ESQ = A2;
const uint8_t IR_BORDA_DIR = A1;


// ----------------------
// Receptor IR
// ----------------------

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

// Ataque frontal máximo
const uint8_t VELOCIDADE_ATAQUE = 255;

// Ataque quando somente um sensor vê o adversário
const uint8_t VELOCIDADE_CURVA_ATAQUE = 190;

// Busca propositalmente mais lenta.
// Antes estava 170.
const uint8_t VELOCIDADE_BUSCA = 110;

// Recuo ao detectar borda
const uint8_t VELOCIDADE_RECUO = 200;

// Giro depois de recuar da borda
const uint8_t VELOCIDADE_GIRO = 180;


// =====================================================
// CONFIGURAÇÃO DOS ULTRASSÔNICOS
// =====================================================

// HC-SR04 possui região muito próxima pouco confiável.
// 4 cm evita falsos alvos extremamente próximos.
const float DISTANCIA_ATAQUE_MINIMA = 0.04f;

// 80 cm
const float DISTANCIA_ATAQUE_MAXIMA = 0.80f;


// -----------------------------------------------------
// Os sensores NÃO são mais disparados juntos.
//
// ESQ
// 30 ms
// DIR
// 30 ms
// ESQ
//
// Portanto cada HC-SR04 individualmente recebe
// aproximadamente 60 ms entre disparos.
// -----------------------------------------------------

const unsigned long INTERVALO_PING_MS = 30;


// Se uma leitura tiver mais que isso, ela deixa
// de ser considerada válida.
const unsigned long VALIDADE_LEITURA_MS = 150;


// Mantém a direção do último adversário visto por
// alguns milissegundos caso um eco seja perdido.
const unsigned long MEMORIA_ALVO_MS = 120;


// Timeout do pulseIn.
//
// 80 cm precisam de aproximadamente 4,7 ms.
// 7 ms dá uma boa margem.
const unsigned long TIMEOUT_ULTRASSOM_US = 7000;


// =====================================================
// CONFIGURAÇÃO DA BORDA
// =====================================================

const int LIMIAR_BORDA = 450;


// true:
//
// valor > 450 = BORDA
//
// Como você relatou que a borda já está funcionando,
// mantive essa lógica.
//
// Se algum dia ficar invertido:
//
// true -> false
//

const bool BORDA_QUANDO_VALOR_MAIOR = true;


// =====================================================
// TEMPOS DA FUGA
// =====================================================

const unsigned long TEMPO_RECUO_MS = 250;
const unsigned long TEMPO_GIRO_MS  = 300;


// =====================================================
// ESTADO DA FUGA
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

unsigned long inicioFuga = 0;


// =====================================================
// DIREÇÃO DO ÚLTIMO ADVERSÁRIO
// =====================================================

enum DirecaoAlvo : uint8_t {

  ALVO_NENHUM,
  ALVO_ESQUERDA,
  ALVO_DIREITA,
  ALVO_CENTRO

};


DirecaoAlvo ultimaDirecaoAlvo = ALVO_NENHUM;

unsigned long ultimoAlvoVisto = 0;


// =====================================================
// ULTRASSÔNICOS
// =====================================================

float distanciaEsq = -1.0f;
float distanciaDir = -1.0f;


// Momento em que cada leitura foi feita.
unsigned long tempoDistanciaEsq = 0;
unsigned long tempoDistanciaDir = 0;


// Controle da alternância dos HC-SR04.
bool proximoUltrassomEsquerdo = true;

unsigned long ultimoPingUltrassom = 0;


// =====================================================
// DEBUG
// =====================================================

unsigned long ultimoDebug = 0;


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);


  // ===================================================
  // MOTORES
  // ===================================================

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  parar();


  // ===================================================
  // ULTRASSÔNICOS
  // ===================================================

  pinMode(TRIG_ESQ, OUTPUT);
  pinMode(ECHO_ESQ, INPUT);

  pinMode(TRIG_DIR, OUTPUT);
  pinMode(ECHO_DIR, INPUT);

  digitalWrite(TRIG_ESQ, LOW);
  digitalWrite(TRIG_DIR, LOW);


  // ===================================================
  // TCRT5000
  // ===================================================

  pinMode(IR_BORDA_ESQ, INPUT);
  pinMode(IR_BORDA_DIR, INPUT);


  // ===================================================
  // RECEPTOR IR
  // ===================================================

  IrReceiver.begin(
    RECV_PIN,
    ENABLE_LED_FEEDBACK
  );


  if (DEBUG_SERIAL) {

    Serial.println();
    Serial.println(F("==============================="));
    Serial.println(F(" MINI SUMO - INOVAWEEK 2026"));
    Serial.println(F("==============================="));

    Serial.println(F("Estado: AGUARDANDO"));
    Serial.println(F("Aguardando READY..."));
  }
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // ===================================================
  // PRIORIDADE 1
  // CONTROLE DO JUIZ
  // ===================================================

  verificarControle();


  // ===================================================
  // DEBUG
  // ===================================================

  debugSensores();


  // ===================================================
  // STOP PERMANENTE
  // ===================================================

  if (estadoPartida == STOP_PERMANENTE) {

    parar();

    return;
  }


  // ===================================================
  // READY / AGUARDANDO
  // ===================================================

  if (estadoPartida != START) {

    parar();

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    return;
  }


  // ===================================================
  // PRIORIDADE 2
  // BORDA
  // ===================================================

  //
  // Se a função retornar true,
  // este ciclo do loop pertence exclusivamente
  // à fuga da borda.
  //

  if (gerenciarFugaBorda()) {

    return;
  }


  // ===================================================
  // PRIORIDADE 3
  // ULTRASSÔNICOS
  // ===================================================

  atualizarUltrassonicos();


  // Um comando pode ter chegado durante pulseIn().
  verificarControle();


  if (estadoPartida != START) {

    parar();

    return;
  }


  // ===================================================
  // DECISÃO
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


  if (DEBUG_SERIAL) {

    Serial.print(F("IR RAW: 0x"));
    Serial.println(codigo, HEX);
  }


  // ===================================================
  // STOP
  // ===================================================

  if (codigo == CODIGO_STOP) {

    estadoPartida = STOP_PERMANENTE;

    estadoFuga = FUGA_INATIVA;
    direcaoFuga = SEM_DIRECAO;

    limparAlvo();

    parar();


    if (DEBUG_SERIAL) {

      Serial.println();
      Serial.println(F(">>> STOP PERMANENTE <<<"));
      Serial.println(F("Reinicie fisicamente o robo."));
    }

    return;
  }


  // Depois de STOP absolutamente nada
  // pode reativar o robô.

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

    limparAlvo();

    parar();


    if (DEBUG_SERIAL) {

      Serial.println();
      Serial.println(F(">>> READY <<<"));
      Serial.println(F("Robo parado."));
    }

    return;
  }


  // ===================================================
  // START
  // ===================================================

  if (codigo == CODIGO_START) {

    //
    // Só sai do READY para START.
    //
    // START recebido durante START não faz nada.
    //

    if (estadoPartida == READY) {

      estadoPartida = START;

      estadoFuga = FUGA_INATIVA;
      direcaoFuga = SEM_DIRECAO;

      limparAlvo();


      // Faz o primeiro HC-SR04 ser lido rapidamente.
      ultimoPingUltrassom = 0;

      proximoUltrassomEsquerdo = true;


      if (DEBUG_SERIAL) {

        Serial.println();
        Serial.println(F(">>> START <<<"));
        Serial.println(F("Luta iniciada."));
      }
    }

    return;
  }
}


// =====================================================
// LIMPAR ALVO
// =====================================================

void limparAlvo() {

  distanciaEsq = -1.0f;
  distanciaDir = -1.0f;

  tempoDistanciaEsq = 0;
  tempoDistanciaDir = 0;

  ultimaDirecaoAlvo = ALVO_NENHUM;
  ultimoAlvoVisto = 0;
}


// =====================================================
// LEITURA ANALÓGICA MAIS ESTÁVEL
// =====================================================
//
// O Arduino troca o canal do ADC entre A1 e A2.
//
// Descartamos uma primeira leitura e usamos
// a segunda para reduzir efeito da troca de canal.
//

int analogReadEstavel(uint8_t pino) {

  analogRead(pino);

  return analogRead(pino);
}


// =====================================================
// VALOR É BORDA?
// =====================================================

bool valorIndicaBorda(int valor) {

  if (BORDA_QUANDO_VALOR_MAIOR) {

    return valor > LIMIAR_BORDA;
  }

  return valor < LIMIAR_BORDA;
}


// =====================================================
// LEITURA CONFIRMADA DA BORDA
// =====================================================
//
// Exige duas leituras indicando borda.
//
// Isso reduz falsos disparos produzidos por:
//
// - ruído dos motores
// - vibração
// - valor próximo do limiar
//

bool lerBordaConfirmada(uint8_t pino) {

  int leitura1 = analogReadEstavel(pino);


  if (!valorIndicaBorda(leitura1)) {

    return false;
  }


  delayMicroseconds(400);


  int leitura2 = analogReadEstavel(pino);


  return valorIndicaBorda(leitura2);
}


// =====================================================
// GERENCIAR FUGA
// =====================================================
//
// Retorna:
//
// true  = fuga ocupou este loop
// false = pode continuar para ultrassônicos
//

bool gerenciarFugaBorda() {

  verificarControle();


  if (estadoPartida != START) {

    parar();

    estadoFuga = FUGA_INATIVA;

    return true;
  }


  // ===================================================
  // RECUANDO
  // ===================================================

  if (estadoFuga == FUGA_RECUANDO) {

    if (
      millis() - inicioFuga >= TEMPO_RECUO_MS
    ) {

      estadoFuga = FUGA_GIRANDO;

      inicioFuga = millis();


      if (direcaoFuga == FUGA_DIREITA) {

        girarDireita();

      } else {

        girarEsquerda();
      }
    }


    return true;
  }


  // ===================================================
  // GIRANDO
  // ===================================================

  if (estadoFuga == FUGA_GIRANDO) {

    if (
      millis() - inicioFuga >= TEMPO_GIRO_MS
    ) {

      estadoFuga = FUGA_INATIVA;

      direcaoFuga = SEM_DIRECAO;


      // Não queremos atacar usando uma leitura
      // ultrassônica feita antes da fuga.
      limparAlvo();


      //
      // IMPORTANTE:
      //
      // NÃO chamamos parar() aqui.
      //
      // Retornamos e, no próximo loop,
      // a borda será verificada novamente.
      //
    }


    return true;
  }


  // ===================================================
  // VERIFICAR BORDA
  // ===================================================

  bool bordaEsquerda =
    lerBordaConfirmada(IR_BORDA_ESQ);


  bool bordaDireita =
    lerBordaConfirmada(IR_BORDA_DIR);


  // Nenhuma borda.
  if (
    !bordaEsquerda &&
    !bordaDireita
  ) {

    return false;
  }


  // Ao entrar em fuga,
  // invalida qualquer adversário antigo.

  limparAlvo();


  if (DEBUG_SERIAL) {

    Serial.print(F(">>> BORDA: "));
  }


  // ===================================================
  // DOIS SENSORES
  // ===================================================

  if (
    bordaEsquerda &&
    bordaDireita
  ) {

    if (DEBUG_SERIAL) {

      Serial.println(F("AMBOS"));
    }


    //
    // Se os dois detectarem,
    // recua e escolhe direita como padrão.
    //

    direcaoFuga = FUGA_DIREITA;
  }


  // ===================================================
  // BORDA ESQUERDA
  // ===================================================

  else if (bordaEsquerda) {

    if (DEBUG_SERIAL) {

      Serial.println(F("ESQUERDA"));
    }


    //
    // Borda na esquerda:
    // gira para direita.
    //

    direcaoFuga = FUGA_DIREITA;
  }


  // ===================================================
  // BORDA DIREITA
  // ===================================================

  else {

    if (DEBUG_SERIAL) {

      Serial.println(F("DIREITA"));
    }


    //
    // Borda direita:
    // gira para esquerda.
    //

    direcaoFuga = FUGA_ESQUERDA;
  }


  // ===================================================
  // COMEÇAR RECUO
  // ===================================================

  estadoFuga = FUGA_RECUANDO;

  inicioFuga = millis();

  recuar();


  return true;
}


// =====================================================
// ATUALIZAR ULTRASSÔNICOS
// =====================================================
//
// NOVA ESTRATÉGIA:
//
// T = 0ms    esquerdo
// T = 30ms   direito
// T = 60ms   esquerdo
// T = 90ms   direito
//
// Isso reduz interferência entre os HC-SR04.
//

void atualizarUltrassonicos() {

  unsigned long agora = millis();


  if (
    agora - ultimoPingUltrassom <
    INTERVALO_PING_MS
  ) {

    return;
  }


  ultimoPingUltrassom = agora;


  // ===================================================
  // ESQUERDO
  // ===================================================

  if (proximoUltrassomEsquerdo) {

    distanciaEsq =
      medirDistancia(
        TRIG_ESQ,
        ECHO_ESQ
      );


    tempoDistanciaEsq = millis();


    proximoUltrassomEsquerdo = false;
  }


  // ===================================================
  // DIREITO
  // ===================================================

  else {

    distanciaDir =
      medirDistancia(
        TRIG_DIR,
        ECHO_DIR
      );


    tempoDistanciaDir = millis();


    proximoUltrassomEsquerdo = true;
  }
}


// =====================================================
// MEDIR DISTÂNCIA
// =====================================================

float medirDistancia(
  uint8_t trig,
  uint8_t echo
) {

  // Garante TRIG LOW antes do pulso.

  digitalWrite(trig, LOW);

  delayMicroseconds(2);


  // Pulso de 10 us.

  digitalWrite(trig, HIGH);

  delayMicroseconds(10);

  digitalWrite(trig, LOW);


  // Aguarda ECHO.

  unsigned long tempo =
    pulseIn(
      echo,
      HIGH,
      TIMEOUT_ULTRASSOM_US
    );


  // ===================================================
  // SEM ECO
  // ===================================================

  if (tempo == 0) {

    return -1.0f;
  }


  // ===================================================
  // DISTÂNCIA
  // ===================================================

  float distancia =
    (tempo * 0.000343f) / 2.0f;


  return distancia;
}


// =====================================================
// DISTÂNCIA VÁLIDA
// =====================================================

bool distanciaValida(
  float distancia,
  unsigned long momentoLeitura
) {

  // Nunca foi lida.

  if (momentoLeitura == 0) {

    return false;
  }


  // Leitura muito antiga.

  if (
    millis() - momentoLeitura >
    VALIDADE_LEITURA_MS
  ) {

    return false;
  }


  // Sem eco.

  if (distancia < 0.0f) {

    return false;
  }


  // Fora da faixa de ataque.

  return
    distancia >= DISTANCIA_ATAQUE_MINIMA &&
    distancia <= DISTANCIA_ATAQUE_MAXIMA;
}


// =====================================================
// DECISÃO
// =====================================================

void decidirMovimento() {

  bool encontrouEsquerda =
    distanciaValida(
      distanciaEsq,
      tempoDistanciaEsq
    );


  bool encontrouDireita =
    distanciaValida(
      distanciaDir,
      tempoDistanciaDir
    );


  // ===================================================
  // OS DOIS ENXERGARAM
  // ===================================================

  if (
    encontrouEsquerda &&
    encontrouDireita
  ) {

    ultimaDirecaoAlvo = ALVO_CENTRO;

    ultimoAlvoVisto = millis();

    atacar();

    return;
  }


  // ===================================================
  // SOMENTE ESQUERDA
  // ===================================================

  if (encontrouEsquerda) {

    ultimaDirecaoAlvo = ALVO_ESQUERDA;

    ultimoAlvoVisto = millis();

    atacarEsquerda();

    return;
  }


  // ===================================================
  // SOMENTE DIREITA
  // ===================================================

  if (encontrouDireita) {

    ultimaDirecaoAlvo = ALVO_DIREITA;

    ultimoAlvoVisto = millis();

    atacarDireita();

    return;
  }


  // ===================================================
  // NENHUM ECO
  // ===================================================
  //
  // HC-SR04 pode perder um único eco.
  //
  // Por isso mantemos o movimento anterior por apenas
  // alguns milissegundos.
  //

  if (
    ultimaDirecaoAlvo != ALVO_NENHUM &&
    millis() - ultimoAlvoVisto <= MEMORIA_ALVO_MS
  ) {

    if (
      ultimaDirecaoAlvo == ALVO_CENTRO
    ) {

      atacar();

      return;
    }


    if (
      ultimaDirecaoAlvo == ALVO_ESQUERDA
    ) {

      atacarEsquerda();

      return;
    }


    if (
      ultimaDirecaoAlvo == ALVO_DIREITA
    ) {

      atacarDireita();

      return;
    }
  }


  // ===================================================
  // PERDEU COMPLETAMENTE O ADVERSÁRIO
  // ===================================================

  ultimaDirecaoAlvo = ALVO_NENHUM;

  procurar();
}


// =====================================================
// ATAQUE RETO
// =====================================================
//
// Mantive o sentido que está funcionando
// na sua versão atual.
//

void atacar() {

  // Motor esquerdo para frente

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);


  // Motor direito para frente

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  analogWrite(
    ENA,
    VELOCIDADE_ATAQUE
  );

  analogWrite(
    ENB,
    VELOCIDADE_ATAQUE
  );
}


// =====================================================
// RECUAR
// =====================================================

void recuar() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);


  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  analogWrite(
    ENA,
    VELOCIDADE_RECUO
  );

  analogWrite(
    ENB,
    VELOCIDADE_RECUO
  );
}


// =====================================================
// GIRAR PARA ESQUERDA
// =====================================================

void girarEsquerda() {

  // Esquerdo para frente
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);


  // Direito para trás
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  analogWrite(
    ENA,
    VELOCIDADE_GIRO
  );

  analogWrite(
    ENB,
    VELOCIDADE_GIRO
  );
}


// =====================================================
// GIRAR PARA DIREITA
// =====================================================

void girarDireita() {

  // Esquerdo para trás
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);


  // Direito para frente
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  analogWrite(
    ENA,
    VELOCIDADE_GIRO
  );

  analogWrite(
    ENB,
    VELOCIDADE_GIRO
  );
}


// =====================================================
// ATAQUE PARA ESQUERDA
// =====================================================

void atacarEsquerda() {

  // Ambos para frente.

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  // Motor esquerdo mais lento.
  // Motor direito mais rápido.
  //
  // Robô curva para esquerda.

  analogWrite(
    ENA,
    VELOCIDADE_CURVA_ATAQUE
  );

  analogWrite(
    ENB,
    VELOCIDADE_ATAQUE
  );
}


// =====================================================
// ATAQUE PARA DIREITA
// =====================================================

void atacarDireita() {

  // Ambos para frente.

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  // Motor esquerdo mais rápido.
  // Motor direito mais lento.
  //
  // Robô curva para direita.

  analogWrite(
    ENA,
    VELOCIDADE_ATAQUE
  );

  analogWrite(
    ENB,
    VELOCIDADE_CURVA_ATAQUE
  );
}


// =====================================================
// BUSCAR ADVERSÁRIO
// =====================================================

void procurar() {

  //
  // Giro no próprio eixo.
  //
  // Mais lento do que antes para permitir
  // que o HC-SR04 consiga encontrar o adversário.
  //

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);


  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  analogWrite(
    ENA,
    VELOCIDADE_BUSCA
  );

  analogWrite(
    ENB,
    VELOCIDADE_BUSCA
  );
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


// =====================================================
// DEBUG DOS SENSORES
// =====================================================

void debugSensores() {

  if (!DEBUG_SERIAL) {

    return;
  }


  if (
    millis() - ultimoDebug < 300
  ) {

    return;
  }


  ultimoDebug = millis();


  int tcrtEsq =
    analogReadEstavel(IR_BORDA_ESQ);


  int tcrtDir =
    analogReadEstavel(IR_BORDA_DIR);


  // ===================================================
  // TCRT
  // ===================================================

  Serial.print(F("TCRT E: "));
  Serial.print(tcrtEsq);

  Serial.print(F(" | TCRT D: "));
  Serial.print(tcrtDir);


  // ===================================================
  // ULTRASSOM ESQUERDO
  // ===================================================

  Serial.print(F(" | ULTRA E: "));


  if (distanciaEsq < 0.0f) {

    Serial.print(F("SEM ECO"));

  } else {

    Serial.print(
      distanciaEsq * 100.0f,
      1
    );

    Serial.print(F("cm"));
  }


  // ===================================================
  // ULTRASSOM DIREITO
  // ===================================================

  Serial.print(F(" | ULTRA D: "));


  if (distanciaDir < 0.0f) {

    Serial.print(F("SEM ECO"));

  } else {

    Serial.print(
      distanciaDir * 100.0f,
      1
    );

    Serial.print(F("cm"));
  }


  // ===================================================
  // ESTADO
  // ===================================================

  Serial.print(F(" | ESTADO: "));


  switch (estadoPartida) {

    case AGUARDANDO:

      Serial.print(F("AGUARDANDO"));
      break;


    case READY:

      Serial.print(F("READY"));
      break;


    case START:

      Serial.print(F("START"));
      break;


    case STOP_PERMANENTE:

      Serial.print(F("STOP"));
      break;
  }


  Serial.println();
}
