cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# CONFIGURAÇÃO CRÍTICA: Board deve ser definido antes do SDK
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Inclui o SDK
include(pico_sdk_import.cmake)

# Nome do projeto
project(plataforma-mqtt C CXX ASM)

# Inicializa o SDK
pico_sdk_init()

# Executável principal
add_executable(plataforma-mqtt 
    plataforma-mqtt.c 
    lib/ssd1306.c
)

# Configuração do programa
pico_set_program_name(plataforma-mqtt "Rover MQTT v2.0")
pico_set_program_version(plataforma-mqtt "2.0")

# Gerar cabeçalho PIO para WS2812
pico_generate_pio_header(plataforma-mqtt ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# CONFIGURAÇÃO CRÍTICA: USB para debug
pico_enable_stdio_usb(plataforma-mqtt 1)
pico_enable_stdio_uart(plataforma-mqtt 0)

# Diretórios de inclusão
target_include_directories(plataforma-mqtt PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/lib
)

# CONFIGURAÇÕES CRÍTICAS PARA RESOLVER TIMEOUT
target_compile_definitions(plataforma-mqtt PRIVATE
    # Configurações básicas MQTT
    LWIP_MQTT=1
    LWIP_ALTCP=1
    
    # CONFIGURAÇÕES CRÍTICAS DE MEMÓRIA
    MEMP_NUM_SYS_TIMEOUT=64        # QUADRUPLICA o pool (padrão: 16)
    MEMP_NUM_TCP_PCB=16            # Mais conexões TCP
    MEMP_NUM_TCP_PCB_LISTEN=8      # Mais listeners
    MEMP_NUM_NETBUF=32             # Mais buffers de rede
    MEMP_NUM_NETCONN=16            # Mais conexões de rede
    
    # Configurações de buffers
    MEMP_NUM_PBUF=64               # QUADRUPLICA buffers de pacotes
    PBUF_POOL_SIZE=48              # Pool ainda maior
    TCP_SND_BUF=16384              # Buffer de envio maior (16KB)
    TCP_WND=16384                  # Janela TCP maior (16KB)
    
    # Configurações de memória geral
    MEM_SIZE=32768                 # 32KB de heap
    
    # Configurações de timer
    TCP_TMR_INTERVAL=50            # Timer TCP mais frequente (50ms)
    LWIP_SO_RCVTIMEO=1
    LWIP_SO_SNDTIMEO=1
    
    # MQTT específico
    MQTT_OUTPUT_RINGBUF_SIZE=512   # Buffer MQTT maior
    MQTT_REQ_MAX_IN_FLIGHT=2       # APENAS 2 requests simultâneos
    MQTT_CYCLIC_TIMER_INTERVAL=10  # Timer MQTT mais conservativo
    
    # Threading
    LWIP_TCPIP_CORE_LOCKING=1
    LWIP_CALLBACK_API=1
    
    # Desabilita features desnecessárias para economizar recursos
    LWIP_STATS=0
    LWIP_DEBUG=0
    LWIP_NETIF_STATUS_CALLBACK=1
    LWIP_NETIF_LINK_CALLBACK=1
)

# BIBLIOTECAS - ORDEM CRÍTICA
target_link_libraries(plataforma-mqtt
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_adc
    hardware_pwm
    # MUDANÇA CRÍTICA: Usar threadsafe ao invés de poll
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

# Gerar saídas
pico_add_extra_outputs(plataforma-mqtt)