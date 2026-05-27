#include "portaSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// ─── Construtor ──────────────────────────────────────────────────────────────

portaSerial::portaSerial(int tx_pin, int rx_pin, int baud)
    : _tx((gpio_num_t)tx_pin),       // converte o inteiro para o tipo gpio_num_t do ESP-IDF
      _rx((gpio_num_t)rx_pin),
      _period_us(1000000LL / baud),  // período de 1 bit em µs (ex: 9600 baud → 104 µs)
      _tx_byte(0),
      _tx_idx(1),
      _tx_busy(false),
      _rx_byte(0),
      _rx_idx(-2),   // -2 = ocioso, nao esta recebendo nada
      _rx_head(0),
      _rx_tail(0)
{
    // ── Pino TX: saída, nível alto (idle) ────────────────────────────────────
    gpio_reset_pin(_tx);
    gpio_set_direction(_tx, GPIO_MODE_OUTPUT);
    gpio_set_level(_tx, 1); // linha UART fica HIGH quando não está transmitindo (padrão do protocolo)

    // ── Pino RX: entrada com pull-up ─────────────────────────────────────────
    gpio_reset_pin(_rx);
    gpio_set_direction(_rx, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_rx, GPIO_PULLUP_ONLY); // pull-up garante HIGH quando nada está chegando

    // ── Cria timer de TX ─────────────────────────────────────────────────────
    esp_timer_create_args_t args;
    memset(&args, 0, sizeof(args)); // zera a struct para não deixar lixo nos campos não usados
    args.callback = tx_cb;
    args.arg      = this;           // passa o objeto como argumento — permite tx_cb acessar os membros
    args.name     = "uart_tx";
    esp_timer_create(&args, &_tx_timer);

    // ── Cria timer de RX ─────────────────────────────────────────────────────
    args.callback = rx_cb;
    args.name     = "uart_rx";
    esp_timer_create(&args, &_rx_timer);

    // ── Aquece o subsistema de timer ─────────────────────────────────────────
    // O ESP-IDF tem latência extra na PRIMEIRA chamada de timer da vida do programa.
    // Sem esse warmup, o primeiro byte recebido sai deslocado em 1 bit.
    // Como _rx_idx == -2, o rx_cb retorna imediatamente sem fazer nada.
    esp_timer_start_once(_rx_timer, 100); // dispara em 100 µs só para "acordar" o timer
    vTaskDelay(pdMS_TO_TICKS(1));         // aguarda o disparo de warmup completar

    // ── Configura interrupção no pino RX ─────────────────────────────────────
    gpio_install_isr_service(0);               // ativa o serviço de ISR por pino do ESP32
                                               // (retorna ESP_ERR_INVALID_STATE se já instalado, tudo bem)
    gpio_set_intr_type(_rx, GPIO_INTR_NEGEDGE);// NEGEDGE = borda HIGH→LOW = exatamente o start bit UART
    gpio_isr_handler_add(_rx, rx_isr, this);   // registra a ISR para esse pino, passando o objeto via arg
}

// ─── Destrutor ───────────────────────────────────────────────────────────────

portaSerial::~portaSerial() {
    // Libera todos os recursos de hardware na ordem inversa da criação
    gpio_isr_handler_remove(_rx);
    esp_timer_stop(_tx_timer);
    esp_timer_delete(_tx_timer);
    esp_timer_stop(_rx_timer);
    esp_timer_delete(_rx_timer);
}

// ─── Transmissão ─────────────────────────────────────────────────────────────

void portaSerial::envia(char c) {
    // Espera a transmissão anterior terminar antes de começar uma nova
    while (_tx_busy) {
        vTaskDelay(1); // libera o processador por 1 tick do FreeRTOS em vez de travar o core
    }

    _tx_byte = (uint8_t)c;
    _tx_idx  = 1;      // começa pelo D0 (LSB — bit menos significativo)
    _tx_busy = true;

    gpio_set_level(_tx, 0);                          // coloca o start bit (LOW) imediatamente no pino
    esp_timer_start_periodic(_tx_timer, _period_us); // timer periódico vai cuidar dos 9 bits restantes
}

void portaSerial::envia(const char* str) {
    // Percorre a string caractere a caractere até encontrar o '\0' (fim de string em C)
    while (*str) {
        envia(*str++); // *str lê o caractere atual, str++ avança o ponteiro para o próximo
    }
}

// ─── Callback do timer de TX ─────────────────────────────────────────────────
// Chamado automaticamente a cada _period_us (104 µs para 9600 baud).
//
// Linha do tempo de um byte ('A' = 0x41):
//   t=0       104µs   208µs  ...  832µs   936µs   1040µs
//  [START=0] [D0=1]  [D1=0] ...  [D7=0]  [STOP=1] [para]
//             idx=1   idx=2       idx=8   idx=9    idx=10

void portaSerial::tx_cb(void* arg) {
    portaSerial* self = static_cast<portaSerial*>(arg); // recupera o objeto a partir do void* arg
    int idx = self->_tx_idx;

    if (idx <= 8) {
        // Envia bit de dado — LSB primeiro, como exige o protocolo UART.
        // >> (idx-1) desloca o byte até o bit desejado chegar na posição 0.
        // & 1 extrai apenas esse bit (resulta em 0 ou 1).
        gpio_set_level(self->_tx, (self->_tx_byte >> (idx - 1)) & 1);
    } else if (idx == 9) {
        gpio_set_level(self->_tx, 1); // stop bit: linha volta para HIGH
    } else {
        // idx == 10: quadro completo — para o timer e libera para próxima transmissão
        esp_timer_stop(self->_tx_timer);
        self->_tx_busy = false;
        return;
    }
    self->_tx_idx = idx + 1; // avança para o próximo bit
}

// ─── ISR de recepção ─────────────────────────────────────────────────────────
// Disparada automaticamente quando o pino RX detecta borda de descida (start bit).
// IRAM_ATTR: função fica na RAM interna — obrigatório pois o cache de flash
// pode estar indisponível durante uma interrupção.

void IRAM_ATTR portaSerial::rx_isr(void* arg) {
    portaSerial* self = static_cast<portaSerial*>(arg);

    gpio_intr_disable(self->_rx); // desabilita a interrupção para não disparar de novo no meio do quadro
    self->_rx_idx = -1;           // estado: aguardando confirmação do start bit

    // Agenda a primeira leitura para o MEIO do start bit (period/2 = 52 µs).
    // Ler no centro do intervalo maximiza a distância das bordas de transição,
    // onde o sinal pode estar instável.
    esp_timer_start_once(self->_rx_timer, self->_period_us / 2);
}

// ─── Callback do timer de RX ─────────────────────────────────────────────────
// _rx_idx < -1 : ocioso (descarta o disparo de warmup)
// _rx_idx == -1: meio do start bit — confirma se é válido ou ruído
// _rx_idx 0..7 : meio de cada bit de dado — amostra e monta o byte

void portaSerial::rx_cb(void* arg) {
    portaSerial* self = static_cast<portaSerial*>(arg);

    if (self->_rx_idx < -1) return; // ocioso — disparo de warmup do construtor, ignora

    if (self->_rx_idx == -1) {
        // Estamos no meio do start bit — verifica se a linha ainda está LOW
        if (gpio_get_level(self->_rx) == 0) {
            // Start bit confirmado — prepara para receber os 8 bits de dado
            self->_rx_byte = 0;
            self->_rx_idx  = 0;
            // Timer PERIÓDICO para amostrar D0 até D7 sem acumular latência.
            // Se usássemos start_once repetido, cada chamada adicionaria ~5-10 µs
            // de atraso, deslocando a amostragem para fora do centro do bit.
            esp_timer_start_periodic(self->_rx_timer, self->_period_us);
        } else {
            // Linha voltou para HIGH: foi ruído, não um start bit real
            gpio_intr_enable(self->_rx); // rearma a interrupção para o próximo byte
        }
        return;
    }

    // ── Amostragem de bit de dado ─────────────────────────────────────────────
    int bit = gpio_get_level(self->_rx); // lê o estado atual do pino (0 ou 1)

    // Encaixa o bit na posição correta do byte sendo montado.
    // (bit << _rx_idx) cria uma máscara com o bit na posição _rx_idx.
    // |= coloca esse bit no _rx_byte sem apagar os bits já recebidos.
    // Exemplo: _rx_idx=3, bit=1 → 00001000 → encaixa D3 no byte.
    self->_rx_byte |= (uint8_t)(bit << self->_rx_idx);
    self->_rx_idx++;

    if (self->_rx_idx == 8) {
        // Todos os 8 bits recebidos — byte completo
        esp_timer_stop(self->_rx_timer);
        self->rx_push(self->_rx_byte); // salva no buffer circular
        self->_rx_idx = -2;            // volta ao estado ocioso
        gpio_intr_enable(self->_rx);   // rearma a interrupção para o próximo byte
    }
    // Se _rx_idx < 8, o timer periódico já dispara novamente para o próximo bit
}

// ─── Buffer circular de recepção ─────────────────────────────────────────────

void portaSerial::rx_push(uint8_t b) {
    int next = (_rx_head + 1) % RX_BUF_SIZE; // próxima posição do head em anel (volta ao 0 após 63)
    if (next != _rx_tail) { // se next == tail o buffer está cheio — descarta o byte
        _rx_buf[_rx_head] = b;
        _rx_head = next;
    }
    // Visualização:
    //   posições: [ ][ ][ ][ ][ ][ ]...[ ]
    //                  ^tail      ^head
    //              (mais antigo)  (próximo vazio)
}

bool portaSerial::disponivel() {
    return _rx_head != _rx_tail; // head == tail significa buffer vazio
}

char portaSerial::le() {
    while (!disponivel()) {
        vTaskDelay(1); // bloqueia liberando o processador até chegar algum dado
    }
    uint8_t b = _rx_buf[_rx_tail];
    _rx_tail  = (_rx_tail + 1) % RX_BUF_SIZE; // avança o tail em anel
    return (char)b;
}
