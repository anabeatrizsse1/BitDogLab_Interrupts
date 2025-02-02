#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"             // Biblioteca para funcionalidades básicas do Raspberry Pi Pico
#include "hardware/pio.h"            // Biblioteca para manipulação do PIO (Programmable I/O)
#include "hardware/clocks.h"         // Biblioteca para manipulação de clocks
#include "pico/multicore.h"          // Biblioteca para uso de múltiplos núcleos
#include "ws2812.pio.h"              // Biblioteca para controle de LEDs WS2812

// Definição dos pinos
const uint MAT_PIN = 7;    // Pino para controle da matriz de LEDs WS2812
const uint BTN_B = 6;      // Botão B - Decrementa o número exibido
const uint LED_R = 13;     // LED RGB - Pino do LED vermelho
const uint LED_G = 12;     // LED RGB - Pino do LED verde
const uint LED_B = 11;     // LED RGB - Pino do LED azul
const uint BTN_A = 5;      // Botão A - Incrementa o número exibido

// Definindo o tamanho da matriz de LEDs
#define NUM_LEDS 25

// PIO e máquina de estado para controle do WS2812
PIO pio = pio0;
uint sm = 0;
uint offset;

// Padrões de números (0-9) para a matriz de LEDs
const uint8_t numeros[10][25] = {

    {1,1,1,1,1, 1,0,0,0,1, 1,0,0,0,1, 1,0,0,0,1, 1,1,1,1,1}, // 0
    {1,1,1,1,1, 0,0,1,0,0, 0,0,1,0,0, 0,0,1,0,0, 0,0,1,1,1}, // 1
    {1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1}, // 2
    {1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1}, // 3
    {1,0,0,0,0, 0,0,0,0,1, 1,1,1,1,1, 1,0,0,0,1, 1,0,0,0,1}, // 4
    {1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,1}, // 5
    {1,1,1,1,1, 1,0,0,0,1, 1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,1}, // 6
    {0,0,1,0,0, 0,0,1,0,0, 0,1,0,0,0, 0,0,0,0,1, 1,1,1,1,1}, // 7
    {1,1,1,1,1, 1,0,0,0,1, 1,1,1,1,1, 1,0,0,0,1, 1,1,1,1,1}, // 8
    {1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1, 1,0,0,0,1, 1,1,1,1,1}, // 9
};

// Variáveis para controle da interrupção e debouncing
volatile int numero_atual = 0;            // Número que será exibido na matriz (0-9)
volatile uint32_t last_interrupt_time_A = 0;  // Armazena o último tempo de interrupção do botão A
volatile uint32_t last_interrupt_time_B = 0;  // Armazena o último tempo de interrupção do botão B
const uint32_t DEBOUNCE_DELAY = 200000;     // Atraso de 200ms para debouncing

// Função que configura a cor de um LED da matriz
void set_pixel_color(uint index, uint8_t r, uint8_t g, uint8_t b) {
    // Fator de brilho para atenuar a intensidade das cores
    float brilho = 0.3;  // Brilho de 30%

    // Aplica o brilho nas cores RGB
    r = (uint8_t)(r * brilho);
    g = (uint8_t)(g * brilho);
    b = (uint8_t)(b * brilho);

    // Envia a cor modificada para o WS2812 (em formato RGB)
    uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    pio_sm_put_blocking(pio, sm, color << 8);
}

// Função executada para piscar um LED
void piscar_led_task() {
    while (true) {
        gpio_put(LED_R, true);  // Acende o LED vermeho
        sleep_ms(100);          // Espera 100ms
        gpio_put(LED_R, false); // Apaga o LED vermelho
        sleep_ms(100);          // Espera 100ms
    }
}

// Função que atualiza a matriz com o número atual
void atualizar_matriz() {
    for (int i = 0; i < NUM_LEDS; i++) {
        if (numeros[numero_atual][i])  // Se o LED da posição i deve acender
            set_pixel_color(i, 0, 0, 255);  // Cor verde (vermelho, verde, azul)
        else
            set_pixel_color(i, 0, 0, 0);    // Apaga o LED
    }
}

// Função de interrupção para os botões (debouncing)
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());  // Obtém o tempo atual em microssegundos

    // Se o botão A foi pressionado e passou o tempo de debouncing
    if (gpio == BTN_A && (now - last_interrupt_time_A) > DEBOUNCE_DELAY) {
        last_interrupt_time_A = now;  // Atualiza o tempo da última interrupção
        numero_atual = (numero_atual + 1) % 10;  // Incrementa o número atual e retorna para 0 após 9
        atualizar_matriz();  // Atualiza a matriz para mostrar o novo número
    }

    // Se o botão B foi pressionado e passou o tempo de debouncing
    if (gpio == BTN_B && (now - last_interrupt_time_B) > DEBOUNCE_DELAY) {
        last_interrupt_time_B = now;  // Atualiza o tempo da última interrupção
        numero_atual = (numero_atual - 1 + 10) % 10;  // Decrementa o número atual e volta para 9 após 0
        atualizar_matriz();  // Atualiza a matriz para mostrar o novo número
    }
}

// Função de configuração dos pinos e inicialização
void setup() {
    stdio_init_all();  // Inicializa a comunicação serial

    // Configuração dos pinos de LEDs
    gpio_init(LED_R);  
    gpio_set_dir(LED_R, GPIO_OUT);  // Define o pino como saída

    gpio_init(LED_G);  
    gpio_set_dir(LED_G, GPIO_OUT);  // Define o pino como saída

    gpio_init(LED_B);  
    gpio_set_dir(LED_B, GPIO_OUT);  // Define o pino como saída
    
    // Configuração dos pinos de botões
    gpio_init(BTN_A);  
    gpio_init(BTN_B);  
    gpio_set_dir(BTN_A, GPIO_IN);  // Define o pino como entrada
    gpio_set_dir(BTN_B, GPIO_IN);  // Define o pino como entrada
    gpio_pull_up(BTN_A);  // Habilita o resistor pull-up no botão A
    gpio_pull_up(BTN_B);  // Habilita o resistor pull-up no botão B
    
    // Inicializa o PIO para controle do WS2812
    offset = pio_add_program(pio, &ws2812_program);
    sm = pio_claim_unused_sm(pio, true);  // Requisita um estado de máquina não utilizado
    ws2812_program_init(pio, sm, offset, MAT_PIN, 800000, false);  // Inicializa o WS2812
    
    // Configura interrupções para os botões com debouncing
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_B, GPIO_IRQ_EDGE_FALL, true, gpio_irq_handler);

    // Exibe inicialmente o número 0 na matriz de LEDs
    atualizar_matriz();
}

// Função principal
int main() {
    setup();  // Chama a função de configuração
    multicore_launch_core1(piscar_led_task);  // Lança a função de piscar LED no núcleo 1

    while (true) {
        sleep_ms(1000);  // Aguarda 1 segundo (loop principal não faz nada enquanto as interrupções ocorrem)
    }
}
