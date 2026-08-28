#include <IRremote.hpp>

int RECV_PIN = 11;  // Arduino pino D11 conectado no Receptor IR

int leituraSensor;       // variavel de leitura do sensor
int led = 8;             // led indicador = D8 do Arduino
int fotoTransistor = 7;  // coletor do fototransistor = D7 do Arduino

int PinTrigger = 2;
int PinEcho = 3;
float TempoEcho = 0;
const float velocidadeSom_mpus = 0.000340;

// Variáveis para controle de tempo sem travar o código (millis)
unsigned long tempoAnteriorUltrassom = 0; 
const long intervaloUltrassom = 2000; // Intervalo de 2 segundos para o ultrassônico

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);

  pinMode(PinTrigger, OUTPUT);
  digitalWrite(PinTrigger, LOW);
  pinMode(PinEcho, INPUT);
  delay(100);

  pinMode(led, OUTPUT);            // pino do led indicador = saida
  pinMode(fotoTransistor, INPUT);  // pino do coletor do fototransistor = entrada
}

void loop() {
  leituraSensor = digitalRead(fotoTransistor);

  if (IrReceiver.decode()) {
    Serial.print("Codigo recebido: 0x");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);

    if (IrReceiver.decodedIRData.decodedRawData == 0xBA45FF00) {
      IrReceiver.resume();  // Libera o receptor IR IMEDIATAMENTE

      Serial.println(">>> Modo INICIADO <<<");
      bool executarUltrassom = true;
      tempoAnteriorUltrassom = millis(); // Inicializa a contagem de tempo

      while (executarUltrassom) {
        
        // --- 1. LEITURA DO FOTOTRANSISTOR (TEMPO REAL) ---
        // Roda instantaneamente a cada ciclo do while, sem nenhum delay
        leituraSensor = digitalRead(fotoTransistor);  
        if (leituraSensor == 0) {
          digitalWrite(led, HIGH);  // acende LED indicador
        } else {
          digitalWrite(led, LOW);   // apaga LED indicador
        }

        // --- 2. LEITURA DO ULTRASSÔNICO (A CADA 2 SEGUNDOS) ---
        // Verifica se já se passaram 2000ms desde a última leitura
        unsigned long tempoAtual = millis();
        if (tempoAtual - tempoAnteriorUltrassom >= intervaloUltrassom) {
          tempoAnteriorUltrassom = tempoAtual; // Atualiza o cronômetro

          DisparaPulsoUltrassonico();
          TempoEcho = pulseIn(PinEcho, HIGH);
          Serial.print("Distancia em metros: ");
          Serial.println(CalculaDistancia(TempoEcho));
        }

        // --- 3. VERIFICAÇÃO DO BOTÃO DE SAÍDA (TEMPO REAL) ---
        if (IrReceiver.decode()) {
          if (IrReceiver.decodedIRData.decodedRawData == 0xBA45FF00) {
            executarUltrassom = false;
            Serial.println(">>> Modo ENCERRADO <<<");
          }
          IrReceiver.resume();  // Libera o receptor após verificar
        }
      }
    }
    IrReceiver.resume();
  }
}

void DisparaPulsoUltrassonico() {
  digitalWrite(PinTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(PinTrigger, LOW);
}

float CalculaDistancia(float tempo_us) {
  return ((tempo_us * velocidadeSom_mpus) / 2);
}
