#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "pico/bootrom.h"
#include "hardware/pwm.h"

// Include the PIO program header
#include "display.pio.h"

// Bibliotecas referentes à configuração do display
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"

// Definições de constantes
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define DISPLAY_ADDRESS 0x3C
#define NUM_PIXELS 25
#define OUT_PIN 7
#define DEBOUNCE_TIME_US 200000
#define CLOCK_SPEED_KHZ 100000

// Definição dos LEDs RGB
typedef enum {
    RLED_PIN = 13,
    GLED_PIN = 11,
    BLED_PIN = 12
} LedPins;

// Definição dos botões
typedef enum {
    BTNA_PIN = 5,
    BTNB_PIN = 6
} ButtonPins;

// Enumeração para dígitos de 0 a 9
typedef enum {
    DIGIT_0 = 0,
    DIGIT_1 = 1,
    DIGIT_2 = 2,
    DIGIT_3 = 3,
    DIGIT_4 = 4,
    DIGIT_5 = 5,
    DIGIT_6 = 6,
    DIGIT_7 = 7,
    DIGIT_8 = 8,
    DIGIT_9 = 9
} Digit;

// Variável de mudança de string para inteiro
static volatile int ic = 0;

// Variável ligada ao debounce dos botões
static volatile uint32_t last_time = 0;

// Inicializa a estrutura do display
ssd1306_t ssd;

// Matriz com todos os dígitos
const double digits[10][25] = {
    { // Digito 0
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0
    },
    { // Digito 1
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 1.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0
    },
    { // Digito 2
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0
    },
    { // Digito 3
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0
    },
    { // Digito 4
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0
    },
    { // Digito 5
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0
    },
    { // Digito 6
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0
    },
    { // Digito 7
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0
    },
    { // Digito 8
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0
    },
    { // Digito 9
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 1.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0
    }
};

// Inicialização dos LEDs e Botões
void init_all() {
    gpio_init(RLED_PIN);
    gpio_set_dir(RLED_PIN, GPIO_OUT);
    gpio_put(RLED_PIN, 0);

    gpio_init(GLED_PIN);
    gpio_set_dir(GLED_PIN, GPIO_OUT);
    gpio_put(GLED_PIN, 0);

    gpio_init(BLED_PIN);
    gpio_set_dir(BLED_PIN, GPIO_OUT);
    gpio_put(BLED_PIN, 0);

    gpio_init(BTNA_PIN);
    gpio_set_dir(BTNA_PIN, GPIO_IN);
    gpio_pull_up(BTNA_PIN);

    gpio_init(BTNB_PIN);
    gpio_set_dir(BTNB_PIN, GPIO_IN);
    gpio_pull_up(BTNB_PIN);
}

// Rotina para definição da intensidade de cores do LED
uint32_t matrix_rgb(double b, double r, double g) {
    const unsigned char R = r * 255;
    const unsigned char G = g * 255;
    const unsigned char B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

// Inicializa o sistema de clock
void clock_init() {
    bool ok = set_sys_clock_khz(CLOCK_SPEED_KHZ, false);
    if (ok) {
        printf("Clock em %ld Hz\n", clock_get_hz(clk_sys));
    } else {
        printf("Falha ao configurar o clock\n");
    }
}

// Configura a PIO
void pio_config(PIO pio, uint *offset, uint *sm) {
    *offset = pio_add_program(pio, &display_program);
    *sm = pio_claim_unused_sm(pio, true);
    display_program_init(pio, *sm, *offset, OUT_PIN);
}

// Função para imprimir um número na matriz de LEDs
void print_digit(Digit digit, PIO pio, uint sm, double r, double g, double b) {
    const double intensity = 0.01; // Valor para intensidade dos LEDs

    if (digit >= DIGIT_0 && digit <= DIGIT_9) {
        for (int16_t i = 0; i < NUM_PIXELS; i++) {
            uint32_t led_value = matrix_rgb(b * intensity * (digits[digit][24 - i]),
                                            r * intensity * (digits[digit][24 - i]),
                                            g * intensity * (digits[digit][24 - i]));
            pio_sm_put_blocking(pio, sm, led_value); // Envia o valor para o LED
        }
    } else {
        printf("Valor incompatível.\n");
    }
}

// Função que é chamada quando ocorre a interrupção
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - last_time > DEBOUNCE_TIME_US) {
        last_time = current_time;

        if (gpio == BTNA_PIN) {
            gpio_put(GLED_PIN, !gpio_get(GLED_PIN)); // Alterna o estado do LED verde
            printf("Estado do LED Verde Alternado.\n");
            ssd1306_fill(&ssd, false); // Limpa o display
            ssd1306_draw_string(&ssd, "LED VERDE ALT", 2, 48); // Desenha uma string 
            ssd1306_send_data(&ssd); // Atualiza o display  
        } else if (gpio == BTNB_PIN) {
            gpio_put(BLED_PIN, !gpio_get(BLED_PIN)); // Alterna o estado do LED azul
            printf("Estado do LED Azul Alternado.\n");
            ssd1306_fill(&ssd, false); // Limpa o display
            ssd1306_draw_string(&ssd, "LED AZUL ALT", 2, 48); // Desenha uma string 
            ssd1306_send_data(&ssd); // Atualiza o display
        }
    }
}

// Função principal
int main() {
    // Inicializa clock, stdio e configurações
    stdio_init_all();
    init_all();
    clock_init();

    PIO pio = pio0;
    uint offset, sm;
    pio_config(pio, &offset, &sm);

    printf("Sistema inicializado.\n");

    // Configuração dos botões como interrupções
    gpio_set_irq_enabled_with_callback(BTNA_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTNB_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA); // Pull up the data line
    gpio_pull_up(I2C_SCL); // Pull up the clock line
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, DISPLAY_ADDRESS, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    while (true) {
        if (stdio_usb_connected()) { // Certifica-se de que o USB está conectado
            char c;
            if (scanf("%c", &c) == 1) { // Lê caractere da entrada padrão
                printf("Recebido: '%c'\n", c);

                ssd1306_fill(&ssd, false); // Limpa o display
                ssd1306_draw_string(&ssd, "DIGITADO:", 2, 38);
                ssd1306_draw_char(&ssd, c, 80, 38); // Desenha o caractere digitado
                ssd1306_send_data(&ssd); // Manda a informação para o display

                if (c >= '0' && c <= '9') { // Se o caractere digitado for um número de 0 a 9, imprime na matriz
                    ic = c - '0'; // Conversão de string para inteiro
                    print_digit((Digit)ic, pio, sm, 1, 1, 1);
                }
            }
        }
        sleep_ms(1000);
    }
    return 0;
}