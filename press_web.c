// - BIBLIOTECAS

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include <math.h>

                         // Biblioteca padrão para entrada e saída
#include <string.h>              // Biblioteca manipular strings
#include <stdlib.h>              // funções para realizar várias operações, incluindo alocação de memória dinâmica (malloc)

                        // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "hardware/adc.h"        // Biblioteca da Raspberry Pi Pico para manipulação do conversor ADC
#include "pico/cyw43_arch.h"     // Biblioteca para arquitetura Wi-Fi da Pico com CYW43  

#include "lwip/pbuf.h"           // Lightweight IP stack - manipulação de buffers de pacotes de rede
#include "lwip/tcp.h"            // Lightweight IP stack - fornece funções e estruturas para trabalhar com o protocolo TCP
#include "lwip/netif.h"          // Lightweight IP stack - fornece funções e estruturas para trabalhar com interfaces de rede (netif)

#include "hardware/pwm.h"       //Para usar periféricos como o Buzzer
#include "hardware/clocks.h"

#include "hardware/pio.h"       // Para a matriz de leds

// Biblioteca gerada pelo arquivo .pio durante compilação.
#include "ws2818b.pio.h"


// - VARIÁVEIS

#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa
// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C



// Definição do número de LEDs e pino.
#define LED_COUNT 25
#define LED_PINX 7

// Credenciais WIFI - Tome cuidado se publicar no github!
#define WIFI_SSID "SEUSSID"
#define WIFI_PASSWORD "SUASENHA"

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43
#define LED_BLUE_PIN 12                 // GPIO12 - LED azul
#define LED_GREEN_PIN 11                // GPIO11 - LED verde
#define LED_RED_PIN 13                  // GPIO13 - LED vermelho

// Configuração do pino do buzzer
#define BUZZER_PIN 21

// Configuração da frequência do buzzer (em Hz)
#define BUZZER_FREQUENCY 4000

// - FUNÇÕES INICIAIS BÁSICAS

// Definição de pixel GRB
struct pixel_t {
  uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

// Declaração do buffer de pixels que formam a matriz.
npLED_t leds[LED_COUNT];

// Variáveis para uso da máquina PIO.
PIO np_pio;
uint sm;

/**
 * Inicializa a máquina PIO para controle da matriz de LEDs.
 */
void npInit(uint pin) {

  // Cria programa PIO.
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  // Toma posse de uma máquina PIO.
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
  }

  // Inicia programa na máquina PIO obtida.
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Limpa buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  leds[index].R = r;
  leds[index].G = g;
  leds[index].B = b;
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite() {
  // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}


// Inicializa o PWM no pino do buzzer
void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta divisor de clock
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0); // Desliga o PWM inicialmente
}

void beep(uint pin, uint duration_ms) {
    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o duty cycle para 50% (ativo)
    pwm_set_gpio_level(pin, 2048);

    // Temporização
    sleep_ms(duration_ms);

    // Desativar o sinal PWM (duty cycle 0)
    pwm_set_gpio_level(pin, 0);

    // Pausa entre os beeps
    sleep_ms(100); // Pausa de 100ms
}




// Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
void gpio_led_bitdog(void);

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Leitura da temperatura interna
float temp_read(void);

// Tratamento do request do usuário
void user_request(char **request);



// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

// Variáveis globais para armazenar os últimos valores lidos
float last_temp_bmp = 0;
float last_temp_aht = 0;
float last_humidity = 0;
float last_pressure = 0;
float last_altitude = 0;

#define MAX_POINTS 50
float altitude_data[MAX_POINTS];
float humidity_data[MAX_POINTS];
int data_index = 0;

float altitude_offset = 0.0f;
float calibrated_altitude = 0.0f;


// - CÓDIGO PRINCIPAL



int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
   // Fim do trecho para modo BOOTSEL com botão B

   
    stdio_init_all();

    pwm_init_buzzer(BUZZER_PIN);

     gpio_init(LED_RED_PIN);
    gpio_init(LED_GREEN_PIN);
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);

    // Inicializa matriz de LEDs NeoPixel.
    npInit(LED_PINX);
    npClear();

    // I2C do Display funcionando em 400Khz.
    i2c_init(I2C_PORT_DISP, 400 * 1000);

    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA_DISP);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL_DISP);                                        // Pull up the clock line
    ssd1306_t ssd;                                                     // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP); // Inicializa o display
    ssd1306_config(&ssd);                                              // Configura o display
    ssd1306_send_data(&ssd);                                           // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa o I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o BMP280
    bmp280_init(I2C_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C_PORT);
    aht20_init(I2C_PORT);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    char str_tmp1[5];  // Buffer para armazenar a string
    char str_alt[5];  // Buffer para armazenar a string  
    char str_tmp2[5];  // Buffer para armazenar a string
    char str_umi[5];  // Buffer para armazenar a string      

    bool cor = true;

    gpio_led_bitdog();


    //Inicializa a arquitetura do cyw43
    while (cyw43_arch_init())
    {
        printf("Falha ao inicializar Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }

    // GPIO do CI CYW43 em nível baixo
    cyw43_arch_gpio_put(LED_PIN, 0);

    // Ativa o Wi-Fi no modo Station, de modo a que possam ser feitas ligações a outros pontos de acesso Wi-Fi.
    cyw43_arch_enable_sta_mode();

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    printf("Conectando ao Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000))
    {
        printf("Falha ao conectar ao Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }
    printf("Conectado ao Wi-Fi\n");

    // Caso seja a interface de rede padrão - imprimir o IP do dispositivo.
    if (netif_default)
    {
        printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    // Configura o servidor TCP - cria novos PCBs TCP. É o primeiro passo para estabelecer uma conexão TCP.
    struct tcp_pcb *server = tcp_new();
    if (!server)
    {
        printf("Falha ao criar servidor TCP\n");
        return -1;
    }

    //vincula um PCB (Protocol Control Block) TCP a um endereço IP e porta específicos.
    if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Falha ao associar servidor TCP à porta 80\n");
        return -1;
    }

    // Coloca um PCB (Protocol Control Block) TCP em modo de escuta, permitindo que ele aceite conexões de entrada.
    server = tcp_listen(server);

    // Define uma função de callback para aceitar conexões TCP de entrada. É um passo importante na configuração de servidores TCP.
    tcp_accept(server, tcp_server_accept);
    printf("Servidor ouvindo na porta 80\n");

    beep(BUZZER_PIN, 250); // - SINAL SONORO DE INICIALIZAÇÃO

    // Inicializa o conversor ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);




    while (1)
    {

        cyw43_arch_poll(); // Necessário para manter o Wi-Fi ativo
        sleep_ms(100);      // Reduz o uso da CPU

        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        double altitude = calculate_altitude(pressure);

        // Atualiza variáveis globais
    last_temp_bmp = temperature / 100.0;
    last_pressure = pressure;
    last_altitude = altitude;

        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20
        if (aht20_read(I2C_PORT, &data))
        {
             last_temp_aht = data.temperature;
            last_humidity = data.humidity;

            printf("Temperatura AHT: %.2f C\n", data.temperature);
            printf("Umidade: %.2f %%\n\n\n", data.humidity);
            gpio_put(LED_RED_PIN, 1);
            gpio_put(LED_BLUE_PIN, 0);
            
        }
        else
        {
            printf("Erro na leitura do AHT10!\n\n\n");
             gpio_put(LED_RED_PIN, 0);
            gpio_put(LED_BLUE_PIN, 1);
            
        }
        // - CONDIÇÕES DE SEGURANÇA

        if(last_temp_aht > 45.0){
            beep(BUZZER_PIN, 250);
            npSetLED(2, 255, 0, 0);
            npSetLED(12, 255, 0, 0);
            npSetLED(17, 255, 0, 0);
            npSetLED(22, 255, 0, 0);
            npWrite();
            npClear();

        } else if (last_humidity >50.0){
            beep(BUZZER_PIN, 250);
            npSetLED(2, 255, 0, 0);
            npSetLED(12, 255, 0, 0);
            npSetLED(17, 255, 0, 0);
            npSetLED(22, 255, 0, 0);
            npWrite();
            npClear();

        } else if (last_temp_bmp > 45.0){
            beep(BUZZER_PIN, 250);
            npSetLED(2, 255, 0, 0);
            npSetLED(12, 255, 0, 0);
            npSetLED(17, 255, 0, 0);
            npSetLED(22, 255, 0, 0);
            npWrite();
            npClear();

        } else if (last_pressure < 100){
            beep(BUZZER_PIN, 250);
            npSetLED(2, 255, 0, 0);
            npSetLED(12, 255, 0, 0);
            npSetLED(17, 255, 0, 0);
            npSetLED(22, 255, 0, 0);
            npWrite();
            npClear();

        } else if (last_altitude < 100){
            beep(BUZZER_PIN, 250);
            npSetLED(2, 255, 0, 0);
            npSetLED(12, 255, 0, 0);
            npSetLED(17, 255, 0, 0);
            npSetLED(22, 255, 0, 0);
            npWrite();
            npClear();

        }

        altitude_data[data_index] = last_altitude;
        humidity_data[data_index] = last_humidity;
        data_index = (data_index + 1) % MAX_POINTS;
        //float calibrated_altitude = last_altitude + altitude_offset;
        // No loop principal, onde você calcula a altitude:
        //double altitude = calculate_altitude(pressure) + altitude_offset;
        last_altitude = altitude;



        sprintf(str_tmp1, "%.1fC", temperature / 100.0);  // Converte o inteiro em string
        sprintf(str_alt, "%.0fm", altitude);  // Converte o inteiro em string
        sprintf(str_tmp2, "%.1fC", data.temperature);  // Converte o inteiro em string
        sprintf(str_umi, "%.1f%%", data.humidity);  // Converte o inteiro em string    
        
        float temperatura = temperature / 100.0;
    
        //  Atualiza o conteúdo do display com animações
        ssd1306_fill(&ssd, !cor);                           // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);       // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);            // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);            // Desenha uma linha
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6);  // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);   // Desenha uma string
        ssd1306_draw_string(&ssd, "BMP280  AHT10", 10, 28); // Desenha uma string
        ssd1306_line(&ssd, 63, 25, 63, 60, cor);            // Desenha uma linha vertical
        ssd1306_draw_string(&ssd, str_tmp1, 14, 41);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_alt, 14, 52);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_tmp2, 73, 41);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_umi, 73, 52);            // Desenha uma string
        ssd1306_send_data(&ssd);                            // Atualiza o display

        sleep_ms(500);
    }

    


     cyw43_arch_deinit();
    return 0;
}





// -------------------------------------- Funções ---------------------------------

// Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
void gpio_led_bitdog(void){
    // Configuração dos LEDs como saída
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);
    
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);
    
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);
}

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Tratamento do request do usuário - digite aqui
void user_request(char **request){


    

    if (strstr(*request, "GET /music_01") != NULL)
    {
        //play_star_wars(BUZZER_PIN);
    }
    else if (strstr(*request, "GET /display_01") != NULL)
    {
         
       /* sleep_ms(200);
    // Limpa o display. O display inicia com todos os pixels apagados.
        ssd1306_fill(&ssd, false);
        ssd1306_send_data(&ssd);

        ssd1306_draw_string(&ssd, "O que o 0", 16, 10);
        ssd1306_draw_string(&ssd, "disse pro 8?", 8, 20);
        ssd1306_draw_string(&ssd, "Belo cinto!", 32, 40);

        ssd1306_send_data(&ssd);


        sleep_ms(4000);
    // Limpa o display. O display inicia com todos os pixels apagados.
        ssd1306_fill(&ssd, false);
        ssd1306_send_data(&ssd);*/

    }
    else if (strstr(*request, "GET /green_on") != NULL)
    {
        gpio_put(LED_GREEN_PIN, 1);
    }
    else if (strstr(*request, "GET /green_off") != NULL)
    {
        gpio_put(LED_GREEN_PIN, 0);
    }
    else if (strstr(*request, "GET /animation_01") != NULL)
    {
       // anima_one();
    }
    else if (strstr(*request, "GET /red_off") != NULL)
    {
       // gpio_put(LED_RED_PIN, 0);
    }
    else if (strstr(*request, "GET /on") != NULL)
    {
        cyw43_arch_gpio_put(LED_PIN, 1);
    }
    else if (strstr(*request, "GET /off") != NULL)
    {
        cyw43_arch_gpio_put(LED_PIN, 0);
    } // Verifica se há calibração na URL
    

};

// Leitura da temperatura interna
/*float temp_read(void){
    adc_select_input(4);
    uint16_t raw_value = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float temperature = 27.0f - ((raw_value * conversion_factor) - 0.706f) / 0.001721f;
        return temperature;
}*/

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // Alocação do request na memória dinámica
    char *request = (char *)malloc(p->len + 1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    printf("Request: %s\n", request);

    // Tratamento de request - Controle dos LEDs
    user_request(&request);
    
    // Leitura da temperatura interna
    //float temperatura = temperature / 100.0;


    // - CÓDIGO HTML
   char html[8000];
snprintf(html, sizeof(html),
"HTTP/1.1 200 OK\r\n"
"Content-Type: text/html\r\n"
"Connection: close\r\n\r\n"
"<!DOCTYPE html>"
"<html>"
"<head>"
"<title>Dados dos Sensores</title>"
"<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"
"<style>"
"body { font-family: Arial, sans-serif; background: #f5f5f5; padding: 20px; }"
".container { max-width: 900px; margin: auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }"
"h1 { text-align: center; color: #333; }"
".sensor-data { margin-bottom: 30px; }"
".sensor { margin-bottom: 10px; }"
"canvas { margin-top: 30px; }"
"button { margin: 5px; padding: 8px 16px; font-size: 16px; }"
"</style>"
"</head>"
"<body>"
"<div class='container'>"
"<h1>Dados dos Sensores</h1>"

"<div class='sensor-data'>"
"<div class='sensor'><strong>BMP280</strong><br>"
"Temperatura: %.2f °C<br>"
"Pressao: %.2f kPa<br>"
"Altitude: <span id='altitude' data-raw='%.2f'>%.2f</span> m</div>"

"<div class='sensor'><strong>AHT20</strong><br>"
"Temperatura: %.2f °C<br>"
"Umidade: <span id='humidity'>%.2f</span> %%</div>"
"</div>"

"<div style='text-align:center;'>"
"<button onclick='ajustarAltitude(2)'>+2 m</button>"
"<button onclick='ajustarAltitude(-2)'>-2 m</button>"
"</div>"

"<canvas id='altitudeChart' width='800' height='300'></canvas>"
"<canvas id='humidityChart' width='800' height='300'></canvas>"

"<footer style='text-align:center; margin-top:20px;'>Atualizado a cada 5 segundos</footer>"
"</div>"

"<script>"
"let altitudeData = [%.2f];"
"let originalAltitudeData = [%.2f];"
"let humidityData = [%.2f];"
"let timeLabels = [new Date().toLocaleTimeString()];"
"let altitudeOffset = 0;"

"const altitudeChart = new Chart(document.getElementById('altitudeChart').getContext('2d'), {"
"  type: 'line',"
"  data: {"
"    labels: timeLabels,"
"    datasets: [{"
"      label: 'Altitude (m)',"
"      data: altitudeData,"
"      borderColor: 'rgba(75, 192, 192, 1)',"
"      backgroundColor: 'rgba(75, 192, 192, 0.2)',"
"      tension: 0.1"
"    }]"
"  },"
"  options: { scales: { y: { beginAtZero: false } } }"
"});"

"const humidityChart = new Chart(document.getElementById('humidityChart').getContext('2d'), {"
"  type: 'line',"
"  data: {"
"    labels: timeLabels,"
"    datasets: [{"
"      label: 'Umidade (%%)',"
"      data: humidityData,"
"      borderColor: 'rgba(255, 99, 132, 1)',"
"      backgroundColor: 'rgba(255, 99, 132, 0.2)',"
"      tension: 0.1"
"    }]"
"  },"
"  options: { scales: { y: { beginAtZero: false } } }"
"});"

"function ajustarAltitude(valor) {"
"  altitudeOffset += valor;"
"  const altSpan = document.getElementById('altitude');"
"  const raw = parseFloat(altSpan.dataset.raw);"
"  altSpan.textContent = (raw + altitudeOffset).toFixed(2);"
"  altitudeChart.data.datasets[0].data = originalAltitudeData.map(val => val + altitudeOffset);"
"  altitudeChart.update();"
"}"

"setInterval(() => {"
"  fetch('/altitude')"
"    .then(response => response.text())"
"    .then(value => {"
"      let alt = parseFloat(value);"
"      if (!isNaN(alt)) {"
"        if (altitudeData.length >= 10) {"
"          altitudeData.shift();"
"          originalAltitudeData.shift();"
"          timeLabels.shift();"
"        }"
"        altitudeData.push(alt + altitudeOffset);"
"        originalAltitudeData.push(alt);"
"        timeLabels.push(new Date().toLocaleTimeString());"
"        altitudeChart.update();"
"        const altSpan = document.getElementById('altitude');"
"        altSpan.dataset.raw = alt.toFixed(2);"
"        altSpan.textContent = (alt + altitudeOffset).toFixed(2);"
"      }"
"    });"

"  fetch('/humidity')"
"    .then(response => response.text())"
"    .then(value => {"
"      let hum = parseFloat(value);"
"      if (!isNaN(hum)) {"
"        if (humidityData.length >= 10) {"
"          humidityData.shift();"
"        }"
"        humidityData.push(hum);"
"        humidityChart.update();"
"        document.getElementById('humidity').textContent = hum.toFixed(2);"
"      }"
"    });"
"}, 5000);"
"</script>"

"</body>"
"</html>",
last_temp_bmp, last_pressure, last_altitude, last_altitude, // Altitude com data-raw
last_temp_aht, last_humidity,
last_altitude, last_altitude, last_humidity  // Altitude inicial, original e umidade
);



    

    // Escreve dados para envio (mas não os envia imediatamente).
    tcp_write(tpcb, html, strlen(html), TCP_WRITE_FLAG_COPY);

    // Envia a mensagem
    tcp_output(tpcb);

    //libera memória alocada dinamicamente
    free(request);
    
    //libera um buffer de pacote (pbuf) que foi alocado anteriormente
    pbuf_free(p);

    return ERR_OK;
}