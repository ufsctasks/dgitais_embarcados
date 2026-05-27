#pragma once
// Garante que esse arquivo seja incluído uma única vez na compilação,
// evitando erros de declaração duplicada.

#include "driver/gpio.h"   // API do ESP-IDF para controlar pinos digitais
#include "esp_timer.h"     // API do ESP-IDF para timers de alta precisão (resolução de 1 µs)

class portaSerial {
public:
    portaSerial(int tx_pin, int rx_pin, int baud);
    ~portaSerial();

    void envia(char c);          // envia um único caractere
    void envia(const char* str); // envia uma string (percorre até o '\0')
    char le();                   // retira e retorna o próximo byte do buffer (bloqueia se vazio)
    bool disponivel();           // retorna true se há bytes esperando no buffer

private:
    static const int RX_BUF_SIZE = 64; // buffer circular de recepção: até 64 bytes na fila

    gpio_num_t _tx;        // número do pino TX
    gpio_num_t _rx;        // número do pino RX
    int64_t    _period_us; // duração de 1 bit em µs (ex: 9600 baud → 104 µs)

    // ── Estado da transmissão ──────────────────────────────────────────────────
    esp_timer_handle_t _tx_timer;
    volatile uint8_t   _tx_byte; // byte que está sendo transmitido agora
    volatile int       _tx_idx;  // qual bit estamos enviando (1-8=dados, 9=stop, 10=fim)
    volatile bool      _tx_busy; // true enquanto uma transmissão está em andamento
    // volatile avisa o compilador que essas variáveis podem mudar dentro de um timer/ISR,
    // então ele não pode "cachear" o valor num registrador — deve sempre ler da memória.

    // ── Estado da recepção ────────────────────────────────────────────────────
    esp_timer_handle_t _rx_timer;
    volatile uint8_t   _rx_byte;              // byte sendo montado bit a bit
    volatile int       _rx_idx;               // -2=ocioso, -1=verificando start bit, 0-7=bit de dado
    volatile uint8_t   _rx_buf[RX_BUF_SIZE]; // buffer circular
    volatile int       _rx_head;              // posição onde o próximo byte entra
    volatile int       _rx_tail;              // posição onde o próximo byte sai

    // Funções static porque o ESP-IDF não consegue passar 'this' diretamente para
    // callbacks de timer/ISR. O objeto é passado via void* arg e recuperado por cast.
    static void           tx_cb(void* arg);  // callback do timer de TX
    static void           rx_cb(void* arg);  // callback do timer de RX
    static void IRAM_ATTR rx_isr(void* arg); // ISR do pino RX
    // IRAM_ATTR força a função ficar na RAM interna — obrigatório para ISRs,
    // pois o cache de flash pode estar indisponível durante uma interrupção.

    void rx_push(uint8_t b); // insere byte no buffer circular
};
