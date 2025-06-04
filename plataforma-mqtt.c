// VERSÃO CORRIGIDA - FORÇA ENVIO DO JOYSTICK SEM ESPERAR HELLO_ACK
// COM ADIÇÃO DE DISPLAY OLED E MATRIZ RGB
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/mqtt.h"
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Novas bibliotecas para os componentes
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "lib/ssd1306.h"    // Biblioteca do SSD1306
#include "lib/font.h"       // Fonte para o OLED
#include "ws2812.pio.h"     // Biblioteca para Matriz WS2812

// Configurações
#define WIFI_SSID     "SUA_SSID"
#define WIFI_PASSWORD "SUA_SENHA"
#define BROKER_IP     "192.168.2.107" 
#define BROKER_PORT   1885

// Configuração para o debug seguro
#define SEGURO_DEBUG 1  // Defina como 0 para desativar recursos que podem causar problemas

// Configurações dos botões da BitDogLab
#define BUTTON_CAPTURE 5   
#define BUTTON_LIGHTS  6   
#define BUTTON_CAMERA  22  

// Configurações do joystick analógico
#define ADC_X_PIN  26  
#define ADC_Y_PIN  27  
#define DEADZONE   0.05f   // Zona morta pequena
#define MAX_SPEED  100.0f  

// CALIBRAÇÃO BASEADA NOS SEUS LOGS:
// X: 19-4080, Y: 17-4086
// Centro real: aproximadamente X=2050, Y=2050
#define ADC_CENTER_X  2050  // Baseado nos seus dados
#define ADC_CENTER_Y  2050  // Baseado nos seus dados  
#define ADC_RANGE     2000  // Range útil

#define DEBOUNCE_TIME  100

// MQTT Configurations
#define MQTT_CLIENT_ID       "pico_w_client"
#define MQTT_TOPIC_CONTROL   "rover/joystick"
#define MQTT_TOPIC_STATUS    "rover/status"
#define MQTT_TOPIC_CAPTURE   "rover/capture"
#define MQTT_TOPIC_HELLO     "rover/hello"
#define MQTT_TOPIC_COMMANDS  "rover/commands"
#define MQTT_TOPIC_SCORE     "rover/score"

// OLED Display via I2C
#define SDA_PIN 14
#define SCL_PIN 15
#define I2C_ADDR 0x3C
#define I2C_PORT i2c1
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

// RGB LED (PWM)
#define R_LED_PIN 13
#define G_LED_PIN 11
#define B_LED_PIN 12
#define PWM_WRAP 255

// Matriz WS2812
#define NUM_PIXELS 25
#define WS2812_PIN 7
#define IS_RGBW false

// Estados do rover para exibição
#define ESTADO_NORMAL 0
#define ESTADO_CAPTURANDO 1
#define ESTADO_CONECTANDO 2

// Estados do sistema
static bool connection_test_done = false;
static bool connection_successful = false;
static bool mqtt_connected = false;

// Estado dos botões
static bool lights_on = false;
static bool camera_on = false;
static bool capture_active = false;
static uint32_t capture_time = 0;

// Variáveis para debounce de botões
static uint32_t last_btn_capture_time = 0;
static uint32_t last_btn_lights_time = 0;
static uint32_t last_btn_camera_time = 0;

// Variáveis MQTT
static mqtt_client_t *mqtt_client;
static u8_t inbuf[256];
static bool hello_ack_received = false;
static bool hello_sent = false;

// Display OLED
ssd1306_t display;

// Estado do rover (para exibição)
static int rover_estado = ESTADO_CONECTANDO;
static uint32_t ultima_captura = 0;
static int pontos_capturados = 0;
static int score_atual = 0;

// Buffer para a matriz de LEDs
bool buffer_leds[NUM_PIXELS] = {false};

// Padrões para matriz de LEDs (5x5)
const bool padrao_normal[5][5] = {
    {false, true, false, true, false},
    {true, false, true, false, true},
    {false, true, false, true, false},
    {true, false, true, false, true},
    {false, true, false, true, false}
};

const bool padrao_captura[5][5] = {
    {false, false, true, false, false},
    {false, true, true, true, false},
    {true, true, true, true, true},
    {false, true, true, true, false},
    {false, false, true, false, false}
};

// Protótipos de funções
bool test_tcp_connection(const char* ip, u16_t port);
bool init_wifi(void);
void print_network_info(void);
bool init_mqtt(void);
void configurar_gpio(void);
void enviar_comando_captura(void);
void enviar_comando_luzes(bool estado);
void enviar_comando_camera(bool estado);
void ler_joystick(float *x, float *y);
void enviar_comandos_joystick(float joy_x, float joy_y);

// Novos protótipos para os componentes adicionais
void inicializar_display(void);
void inicializar_led_rgb(void);
void inicializar_matriz_leds(void);
void atualizar_display(void);
void definir_cor_rgb(uint8_t r, uint8_t g, uint8_t b);
void definir_leds(uint8_t r, uint8_t g, uint8_t b);
void atualizar_buffer_matriz(const bool padrao[5][5]);

// Callback para teste de conexão TCP
static err_t tcp_connected_callback(void *arg, struct tcp_pcb *pcb, err_t err) {
    printf("🎉 CONEXÃO TCP ESTABELECIDA!\n");
    connection_successful = true;
    connection_test_done = true;
    tcp_close(pcb);
    return ERR_OK;
}

static void tcp_error_callback(void *arg, err_t err) {
    printf("❌ ERRO NA CONEXÃO TCP: %d\n", err);
    connection_successful = false;
    connection_test_done = true;
}

// Callback para mensagens recebidas via MQTT
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("📨 MQTT mensagem recebida no tópico: %s\n", topic);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    if (len > 0 && len < sizeof(inbuf)) {
        memcpy(inbuf, data, len);
        inbuf[len] = 0;
        
        printf("   Payload: %s\n", inbuf);
        
        if (strstr((char*)inbuf, "hello_ack=true")) {
            printf("✅ Hello confirmado pelo simulador!\n");
            hello_ack_received = true;
            
            // Atualiza o estado do rover
            rover_estado = ESTADO_NORMAL;
            atualizar_display();
            
            // Atualiza a matriz de LEDs
            atualizar_buffer_matriz(padrao_normal);
            definir_leds(0, 0, 50); // Azul
            
            // Atualiza o LED RGB
            definir_cor_rgb(0, 0, 255); // Azul
        }
        
        // Verifica por informações de score
        if (strstr((char*)inbuf, "score=")) {
            char *score_str = strstr((char*)inbuf, "score=");
            if (score_str) {
                int novo_score = atoi(score_str + 6); // Pula "score="
                
                // Verifica se o score aumentou (capturou ponto)
                if (novo_score > score_atual) {
                    rover_estado = ESTADO_CAPTURANDO;
                    ultima_captura = to_ms_since_boot(get_absolute_time());
                    pontos_capturados++;
                    
                    // Atualiza matriz de LEDs com padrão de captura
                    atualizar_buffer_matriz(padrao_captura);
                    definir_leds(0, 255, 0); // Verde brilhante
                    
                    // LED RGB em verde
                    definir_cor_rgb(0, 255, 0);
                }
                
                score_atual = novo_score;
                atualizar_display();
            }
        }
    }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("✅ MQTT conectado!\n");
        mqtt_connected = true;
        
        mqtt_publish(client, MQTT_TOPIC_STATUS, "online", 6, 0, 0, NULL, NULL);
        mqtt_subscribe(client, MQTT_TOPIC_COMMANDS, 0, NULL, NULL);
        mqtt_subscribe(client, MQTT_TOPIC_SCORE, 0, NULL, NULL);
        
        if (!hello_sent) {
            mqtt_publish(client, MQTT_TOPIC_HELLO, "HELLO", 5, 0, 0, NULL, NULL);
            printf("👋 HELLO enviado!\n");
            hello_sent = true;
        }
        
        // Atualiza display ao conectar no MQTT
        rover_estado = ESTADO_CONECTANDO;
        atualizar_display();
        
        // Matriz LED azul para indicar conectado
        atualizar_buffer_matriz(padrao_normal);
        definir_leds(0, 0, 50);
    } else {
        printf("❌ Falha na conexão MQTT: %d\n", status);
        mqtt_connected = false;
        
        // Matriz LED vermelho para indicar falha
        atualizar_buffer_matriz(padrao_normal);
        definir_leds(50, 0, 0);
        
        // LED RGB vermelho
        definir_cor_rgb(255, 0, 0);
    }
}

// FUNÇÃO CORRIGIDA DO JOYSTICK - BASEADA NOS SEUS DADOS REAIS
void ler_joystick(float *x, float *y) {
    // Lê ADC para eixo X
    adc_select_input(0); // ADC0 - Pino 26
    uint16_t raw_x = adc_read();
    
    // Lê ADC para eixo Y  
    adc_select_input(1); // ADC1 - Pino 27
    uint16_t raw_y = adc_read();
    
    // Debug periódico dos valores brutos
    static uint32_t last_debug = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_debug > 1000) { // A cada 1 segundo
        printf("📊 ADC RAW: X=%u, Y=%u\n", raw_x, raw_y);
        last_debug = now;
    }
    
    // Converte para faixa -1.0 a 1.0 usando dados reais
    *x = ((float)raw_x - ADC_CENTER_X) / ADC_RANGE;
    *y = ((float)raw_y - ADC_CENTER_Y) / ADC_RANGE;
    
    // INVERSÃO BASEADA NO QUE VIMOS NOS LOGS - Inverte Y
    *y = -*y;
    
    // Aplica zona morta
    if (fabs(*x) < DEADZONE) *x = 0.0f;
    if (fabs(*y) < DEADZONE) *y = 0.0f;
    
    // Limita os valores
    *x = (*x > 1.0f) ? 1.0f : (*x < -1.0f) ? -1.0f : *x;
    *y = (*y > 1.0f) ? 1.0f : (*y < -1.0f) ? -1.0f : *y;
    
    // Debug quando há movimento significativo
    if (fabs(*x) > 0.1f || fabs(*y) > 0.1f) {
        static uint32_t last_movement_debug = 0;
        if (now - last_movement_debug > 300) {
            printf("🎮 JOYSTICK ATIVO! X=%.3f, Y=%.3f (Raw: %u, %u)\n", *x, *y, raw_x, raw_y);
            last_movement_debug = now;
        }
    }
}

// FUNÇÃO CORRIGIDA - ENVIA JOYSTICK SEMPRE QUE MQTT ESTIVER CONECTADO
void enviar_comandos_joystick(float joy_x, float joy_y) {
    if (!mqtt_connected) {
        printf("⚠️ MQTT não conectado - não enviando joystick\n");
        return;
    }
    
    float speed = joy_y * MAX_SPEED;
    float steering = joy_x * 100.0f;
    
    char cmd[128];
    snprintf(cmd, sizeof(cmd), 
            "{\"speed\":%.1f,\"steering\":%.1f,\"mode\":0,\"lights\":\"%s\",\"camera\":\"%s\"}",
            speed, steering,
            lights_on ? "on" : "off", 
            camera_on ? "on" : "off");
    
    // ENVIA SEMPRE via MQTT no tópico rover/joystick
    err_t result = mqtt_publish(mqtt_client, MQTT_TOPIC_CONTROL, cmd, strlen(cmd), 0, 0, NULL, NULL);
    
    // Debug melhorado
    static uint32_t last_print = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (fabs(joy_x) > 0.05f || fabs(joy_y) > 0.05f || now - last_print > 2000) {
        if (now - last_print > 200) { // Limita frequência do debug
            if (result == ERR_OK) {
                printf("📤 MQTT ENVIADO! speed=%.1f, steering=%.1f\n", speed, steering);
            } else {
                printf("❌ ERRO ao enviar MQTT: %d\n", result);
            }
            last_print = now;
        }
    }
}

// Callback para GPIOs (botões) - VERSÃO CORRIGIDA
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (gpio == BUTTON_CAPTURE && now - last_btn_capture_time > DEBOUNCE_TIME) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            printf("🟢 BOTÃO CAPTURA pressionado!\n");
            if (mqtt_connected) {
                const char* msg = "{\"capture\":true}";
                mqtt_publish(mqtt_client, MQTT_TOPIC_CAPTURE, msg, strlen(msg), 0, 0, NULL, NULL);
                printf("📤 Captura enviada via MQTT\n");
                
                // Apenas marca para atualização visual no loop principal
                // Não fazer operações de I/O complexas dentro do handler de interrupção
                capture_active = true;
                capture_time = now;
            }
        }
        last_btn_capture_time = now;
    }
    else if (gpio == BUTTON_LIGHTS && now - last_btn_lights_time > DEBOUNCE_TIME) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            lights_on = !lights_on;
            printf("💡 BOTÃO LUZES: %s\n", lights_on ? "ON" : "OFF");
            if (mqtt_connected) {
                char msg[64];
                snprintf(msg, sizeof(msg), "{\"lights\":\"%s\"}", lights_on ? "on" : "off");
                mqtt_publish(mqtt_client, MQTT_TOPIC_STATUS, msg, strlen(msg), 0, 0, NULL, NULL);
                printf("📤 Luzes enviadas via MQTT\n");
                
                // Atualiza o LED RGB conforme estado das luzes
                if (lights_on)
                    definir_cor_rgb(255, 255, 150); // Amarelo claro
                else
                    definir_cor_rgb(0, 0, 255); // Azul
                
                atualizar_display();
            }
        }
        last_btn_lights_time = now;
    }
    else if (gpio == BUTTON_CAMERA && now - last_btn_camera_time > DEBOUNCE_TIME) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            camera_on = !camera_on;
            printf("📷 BOTÃO CÂMERA: %s\n", camera_on ? "ON" : "OFF");
            if (mqtt_connected) {
                char msg[64];
                snprintf(msg, sizeof(msg), "{\"camera\":\"%s\"}", camera_on ? "on" : "off");
                mqtt_publish(mqtt_client, MQTT_TOPIC_STATUS, msg, strlen(msg), 0, 0, NULL, NULL);
                printf("📤 Câmera enviada via MQTT\n");
                atualizar_display();
            }
        }
        last_btn_camera_time = now;
    }
}

void configurar_gpio(void) {
    // Inicializa ADC para joystick
    adc_init();
    adc_gpio_init(ADC_X_PIN);
    adc_gpio_init(ADC_Y_PIN);
    
    // Configura botões
    gpio_init(BUTTON_CAPTURE);
    gpio_set_dir(BUTTON_CAPTURE, GPIO_IN);
    gpio_pull_up(BUTTON_CAPTURE);
    
    gpio_init(BUTTON_LIGHTS);
    gpio_set_dir(BUTTON_LIGHTS, GPIO_IN);
    gpio_pull_up(BUTTON_LIGHTS);
    
    gpio_init(BUTTON_CAMERA);
    gpio_set_dir(BUTTON_CAMERA, GPIO_IN);
    gpio_pull_up(BUTTON_CAMERA);
    
    // Configura interrupções
    gpio_set_irq_enabled_with_callback(BUTTON_CAPTURE, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(BUTTON_LIGHTS, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON_CAMERA, GPIO_IRQ_EDGE_FALL, true);
    
    printf("✅ GPIOs configurados!\n");
    printf("🎮 Joystick: X=Pino%d(ADC0), Y=Pino%d(ADC1)\n", ADC_X_PIN, ADC_Y_PIN);
    printf("🔘 Botões: Captura=%d, Luzes=%d, Câmera=%d\n", BUTTON_CAPTURE, BUTTON_LIGHTS, BUTTON_CAMERA);
}

bool test_tcp_connection(const char* ip, u16_t port) {
    printf("🔍 Testando TCP %s:%d\n", ip, port);
    
    ip_addr_t target_ip;
    if (!ipaddr_aton(ip, &target_ip)) {
        printf("❌ IP inválido\n");
        return false;
    }
    
    struct tcp_pcb *pcb = tcp_new();
    if (pcb == NULL) {
        printf("❌ Falha ao criar TCP PCB\n");
        return false;
    }
    
    tcp_err(pcb, tcp_error_callback);
    connection_test_done = false;
    connection_successful = false;
    
    err_t err = tcp_connect(pcb, &target_ip, port, tcp_connected_callback);
    if (err != ERR_OK) {
        printf("❌ Falha ao conectar TCP\n");
        tcp_close(pcb);
        return false;
    }
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (!connection_test_done && (to_ms_since_boot(get_absolute_time()) - start_time) < 10000) {
        cyw43_arch_poll();
        sleep_ms(100);
    }
    
    return connection_successful;
}

bool init_wifi(void) {
    printf("🌐 Conectando Wi-Fi...\n");
    
    if (cyw43_arch_init()) {
        printf("❌ Falha na inicialização Wi-Fi\n");
        return false;
    }
    
    cyw43_arch_enable_sta_mode();
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000)) {
        printf("❌ Falha ao conectar Wi-Fi\n");
        return false;
    }
    
    printf("✅ Wi-Fi conectado!\n");
    return true;
}

void print_network_info(void) {
    printf("📊 IP do Pico: %s\n", ipaddr_ntoa(&cyw43_state.netif[0].ip_addr));
    printf("📊 Broker: %s:%d\n", BROKER_IP, BROKER_PORT);
}

bool init_mqtt(void) {
    ip_addr_t broker_addr;
    
    if (!ipaddr_aton(BROKER_IP, &broker_addr)) {
        printf("❌ IP do broker inválido\n");
        return false;
    }
    
    printf("📡 Inicializando MQTT...\n");
    
    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("❌ Falha ao criar cliente MQTT\n");
        return false;
    }
    
    mqtt_set_inpub_callback(mqtt_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
    
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;
    ci.client_user = "torlee";
    ci.client_pass = "123456";
    ci.keep_alive = 60;
    ci.will_topic = MQTT_TOPIC_STATUS;
    ci.will_msg = "offline";
    ci.will_qos = 0;
    ci.will_retain = 0;
    
    err_t err = mqtt_client_connect(mqtt_client, &broker_addr, BROKER_PORT, 
                                   mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("❌ Falha ao iniciar conexão MQTT\n");
        return false;
    }
    
    return true;
}

// ===== NOVAS FUNÇÕES PARA DISPLAY E LEDS =====

// Inicialização do display OLED
void inicializar_display() {
    // Inicialização do I2C 
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    sleep_ms(100); // Aguarda estabilização do I2C
    
    // Inicialização do display usando a biblioteca ssd1306
    // A função ssd1306_init não retorna valor, é void
    ssd1306_init(&display, SSD1306_WIDTH, SSD1306_HEIGHT, false, I2C_ADDR, I2C_PORT);
    ssd1306_config(&display);
    ssd1306_fill(&display, 0);
    
    // Tela de boas-vindas
    ssd1306_draw_string(&display, "Rover MQTT", 25, 5);
    ssd1306_draw_string(&display, "BitDogLab", 25, 25);
    ssd1306_draw_string(&display, "Inicializando...", 10, 45);
    ssd1306_send_data(&display);
    
    printf("✅ Display OLED inicializado!\n");
}

// Atualização do display OLED
void atualizar_display() {
    // Limpa o display
    ssd1306_fill(&display, 0);
    
    // Desenha título
    ssd1306_draw_string(&display, "Rover MQTT", 25, 0);
    
    // Status da conexão
    if (!mqtt_connected) {
        ssd1306_draw_string(&display, "Status: Desconectado", 0, 16);
        ssd1306_draw_string(&display, "Tentando conectar...", 0, 28);
    } else {
        ssd1306_draw_string(&display, "Status: Conectado", 0, 16);
        
        // Exibe informações adicionais quando conectado
        char linha_score[32];
        sprintf(linha_score, "Score: %d", score_atual);
        ssd1306_draw_string(&display, linha_score, 10, 28);
        
        char linha_pontos[32];
        sprintf(linha_pontos, "Pontos: %d", pontos_capturados);
        ssd1306_draw_string(&display, linha_pontos, 10, 40);
    }
    
    // Mostra estados dos botões na parte inferior
    char status_line[32];
    sprintf(status_line, "L:%s C:%s", 
            lights_on ? "ON" : "OFF", 
            camera_on ? "ON" : "OFF");
    ssd1306_draw_string(&display, status_line, 10, 52);
    
    // Atualiza o display
    ssd1306_send_data(&display);
}

// Inicialização do LED RGB
void inicializar_led_rgb() {
    gpio_set_function(R_LED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(B_LED_PIN, GPIO_FUNC_PWM);
    
    uint slice_r = pwm_gpio_to_slice_num(R_LED_PIN);
    uint slice_b = pwm_gpio_to_slice_num(B_LED_PIN);
    
    pwm_set_wrap(slice_r, PWM_WRAP);
    pwm_set_clkdiv(slice_r, 125.0f);
    pwm_set_enabled(slice_r, true);
    
    if (slice_b != slice_r) {
        pwm_set_wrap(slice_b, PWM_WRAP);
        pwm_set_clkdiv(slice_b, 125.0f);
        pwm_set_enabled(slice_b, true);
    }
    
    // LED Verde como saída digital 
    gpio_init(G_LED_PIN);
    gpio_set_dir(G_LED_PIN, GPIO_OUT);
    
    printf("✅ LED RGB inicializado!\n");
}

// Configuração de cores para o LED RGB
void definir_cor_rgb(uint8_t r, uint8_t g, uint8_t b) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(R_LED_PIN), pwm_gpio_to_channel(R_LED_PIN), r);
    gpio_put(G_LED_PIN, g > 10); // Digital on/off baseado na intensidade
    pwm_set_chan_level(pwm_gpio_to_slice_num(B_LED_PIN), pwm_gpio_to_channel(B_LED_PIN), b);
}

// Função auxiliar para formatar cores para a matriz de LEDs
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 8) | ((uint32_t)g << 16) | (uint32_t)b;
}

// Função auxiliar para enviar um pixel para a matriz
static inline void enviar_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// Define os LEDs da matriz com base no buffer
void definir_leds(uint8_t r, uint8_t g, uint8_t b) {
    // Segurança: verificar se os valores estão em faixas razoáveis
    // Limita brilho para evitar picos de corrente
    r = (r > 100) ? 100 : r;
    g = (g > 100) ? 100 : g;
    b = (b > 100) ? 100 : b;
    
    uint32_t cor = urgb_u32(r, g, b);
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (buffer_leds[i])
            enviar_pixel(cor);
        else
            enviar_pixel(0);
    }
    
    // Não usar sleep_us aqui, pois pode causar problemas em handlers de interrupção
    // Use apenas no loop principal
}

// Atualiza o buffer com um padrão específico
void atualizar_buffer_matriz(const bool padrao[5][5]) {
    for (int linha = 0; linha < 5; linha++) {
        for (int coluna = 0; coluna < 5; coluna++) {
            int indice = linha * 5 + coluna;
            buffer_leds[indice] = padrao[linha][coluna];
        }
    }
}

// Inicialização da matriz de LEDs WS2812
void inicializar_matriz_leds() {
    try_pio: 
    {
        PIO pio = pio0;
        uint sm = 0;
        uint offset = pio_add_program(pio, &ws2812_program);
        
        // Adiciona verificação de erro
        if (offset == 0) {
            printf("❌ Erro ao adicionar programa PIO para WS2812\n");
            return;
        }
        
        ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
        
        // Inicialização simplificada para evitar problemas
        // Primeiro limpa todos os LEDs
        for (int i = 0; i < NUM_PIXELS; i++) {
            buffer_leds[i] = false;
        }
        definir_leds(0, 0, 0);
        sleep_ms(100);
        
        // Efeito de inicialização mais simples
        for (int i = 0; i < NUM_PIXELS; i++) {
            buffer_leds[i] = true;
        }
        definir_leds(10, 10, 10); // Brilho baixo para evitar picos de corrente
        sleep_ms(500);
        
        // Define padrão inicial
        atualizar_buffer_matriz(padrao_normal);
        definir_leds(0, 0, 30); // Azul
    }
    
    printf("✅ Matriz de LEDs inicializada!\n");
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("🔧═══════════════════════════════════🔧\n");
    printf("🔧      VERSÃO AMPLIADA FINAL       🔧\n");
    printf("🔧  COM DISPLAY OLED E MATRIZ RGB   🔧\n");
    printf("🔧═══════════════════════════════════🔧\n");
    
    printf("🎯 ADIÇÕES IMPLEMENTADAS:\n");
    printf("   - Display OLED para status visual\n");
    printf("   - Matriz de LEDs para feedback\n");
    printf("   - LED RGB para indicação de status\n");
    printf("   - Centro ajustado para 2050 (baseado nos seus dados)\n");
    printf("   - Joystick enviado sempre que MQTT conectado\n");
    
    // Configura GPIOs
    configurar_gpio();
    
    // Inicializa os componentes adicionais
    inicializar_display();
    inicializar_led_rgb();
    inicializar_matriz_leds();
    
    // Inicializa com estado desconectado
    definir_cor_rgb(255, 0, 0); // Vermelho para desconectado
    
    if (!init_wifi()) {
        printf("💀 Falha crítica no Wi-Fi\n");
        
        // Atualiza display com erro
        ssd1306_fill(&display, 0);
        ssd1306_draw_string(&display, "ERRO Wi-Fi", 25, 10);
        ssd1306_draw_string(&display, "Verifique as", 20, 25);
        ssd1306_draw_string(&display, "configuracoes", 15, 40);
        ssd1306_send_data(&display);
        
        return 1;
    }
    
    print_network_info();
    
    // Atualiza display com info de conexão
    ssd1306_fill(&display, 0);
    ssd1306_draw_string(&display, "Wi-Fi Conectado", 10, 0);
    ssd1306_draw_string(&display, "IP:", 0, 16);
    ssd1306_draw_string(&display, ipaddr_ntoa(&cyw43_state.netif[0].ip_addr), 20, 16);
    ssd1306_draw_string(&display, "Testando conexao...", 0, 32);
    ssd1306_send_data(&display);
    
    sleep_ms(3000);
    
    if (test_tcp_connection(BROKER_IP, BROKER_PORT)) {
        printf("✅ TCP OK! Inicializando MQTT...\n");
        
        // Atualiza display
        ssd1306_fill(&display, 0);
        ssd1306_draw_string(&display, "TCP OK", 40, 0);
        ssd1306_draw_string(&display, "Conectando MQTT...", 0, 25);
        ssd1306_draw_string(&display, BROKER_IP, 30, 40);
        ssd1306_send_data(&display);
        
        if (init_mqtt()) {
            uint32_t start_time = to_ms_since_boot(get_absolute_time());
            while (!mqtt_connected && (to_ms_since_boot(get_absolute_time()) - start_time) < 10000) {
                cyw43_arch_poll();
                sleep_ms(100);
            }
            
            if (mqtt_connected) {
                printf("✅ MQTT conectado!\n");
                printf("🎮 AGORA O JOYSTICK SERÁ ENVIADO AUTOMATICAMENTE!\n");
                printf("🔍 Observe as mensagens '📤 MQTT ENVIADO!' no console\n");
                
                // LED azul para conectado
                definir_cor_rgb(0, 0, 255);
                
                // Atualiza display
                rover_estado = ESTADO_NORMAL;
                atualizar_display();
            }
        }
    } else {
        printf("❌ TCP falhou!\n");
        
        // Atualiza display com erro
        ssd1306_fill(&display, 0);
        ssd1306_draw_string(&display, "ERRO TCP", 30, 10);
        ssd1306_draw_string(&display, "Verifique o", 20, 25);
        ssd1306_draw_string(&display, "endereco do broker", 0, 40);
        ssd1306_send_data(&display);
    }
    
    printf("\n🔄 Iniciando loop principal...\n");
    
    uint32_t last_status_time = 0;
    uint32_t last_joystick_time = 0;
    uint32_t last_display_time = 0;
    
    while (true) {
        cyw43_arch_poll();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Status a cada 10 segundos
        if (mqtt_connected && now - last_status_time > 10000) {
            mqtt_publish(mqtt_client, MQTT_TOPIC_STATUS, "rover_active", 12, 0, 0, NULL, NULL);
            printf("📊 Status enviado\n");
            last_status_time = now;
        }
        
        // *** JOYSTICK ENVIADO A CADA 100ms SEMPRE QUE MQTT CONECTADO ***
        if (mqtt_connected && now - last_joystick_time > 100) {
            float joy_x, joy_y;
            ler_joystick(&joy_x, &joy_y);
            
            // ENVIA SEMPRE, SEM ESPERAR HELLO_ACK!
            enviar_comandos_joystick(joy_x, joy_y);
            
            last_joystick_time = now;
        }
        
        // Atualiza o display a cada 500ms
        if (now - last_display_time > 500) {
            atualizar_display();
            last_display_time = now;
        }
        
        // Verifica se botão de captura foi pressionado
        if (capture_active && now - capture_time < 1000) {
            // Atualiza o estado para capturando (agora no loop principal, não na interrupção)
            rover_estado = ESTADO_CAPTURANDO;
            ultima_captura = now;
            
            // Atualiza matriz de LEDs com padrão de captura
            atualizar_buffer_matriz(padrao_captura);
            definir_leds(0, 255, 0); // Verde brilhante
            
            // LED RGB em verde
            definir_cor_rgb(0, 255, 0);
            
            // Desativa a flag depois de processar
            if (now - capture_time > 500) {
                capture_active = false;
            }
        }
        
        // Retorna ao estado normal após 500ms de animação de captura
        if (rover_estado == ESTADO_CAPTURANDO && now - ultima_captura > 500) {
            rover_estado = ESTADO_NORMAL;
            
            // Retorna matriz de LEDs para o padrão normal
            atualizar_buffer_matriz(padrao_normal);
            definir_leds(0, 0, 30); // Azul
            
            // Retorna LED RGB para azul ou amarelo (dependendo das luzes)
            if (lights_on)
                definir_cor_rgb(255, 255, 150); // Amarelo claro
            else
                definir_cor_rgb(0, 0, 255); // Azul
        }
        
        sleep_ms(10);
    }
    
    return 0;
}