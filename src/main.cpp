#include <Arduino.h>
#include "portaSerial.h"

// Teste de loopback: conecte D17 (TX) ao D16 (RX) com um fio jumper.
// Tudo que for enviado vai aparecer de volta no RX.

static portaSerial* mySerial = nullptr;
// static: limita a visibilidade desse ponteiro a este arquivo
// nullptr: inicializa com valor nulo (boa prática — evita ponteiro com valor lixo)

void setup() {
    Serial.begin(115200); // liga a UART de debug (pinos RX0/TX0) para imprimir no monitor serial
    delay(1000);          // aguarda o monitor serial conectar antes de começar a imprimir

    Serial.println("=== Porta Serial por Software ===");

    // Cria o objeto aqui (e não como global) para garantir que
    // o hardware do ESP32 já está completamente inicializado
    mySerial = new portaSerial(17, 16, 9600); // TX=D17, RX=D16, 9600 baud

    Serial.println("Enviando 'A'...");
    mySerial->envia('A'); // envia um único caractere

    delay(20); // aguarda o byte percorrer o fio de loopback e ser recebido

    Serial.println("Enviando \"Oi mundo\"...");
    mySerial->envia("Oi mundo"); // envia uma string completa (percorre até o '\0')
}

void loop() {
    // Verifica continuamente se algum byte foi recebido no buffer
    if (mySerial->disponivel()) {
        char c = mySerial->le(); // retira o byte mais antigo do buffer circular

        // Imprime em hexadecimal e como caractere para facilitar o debug
        Serial.print("Recebido: 0x");
        Serial.print((uint8_t)c, HEX);
        Serial.print(" '");
        Serial.print(c);
        Serial.println("'");
    }
    delay(10); // evita que o loop consuma 100% do processador
}
