extern "C" 
{
    #include "driver/i2c_master.h"
    #include "driver/gpio.h"
    #include <stdio.h>
    #include "esp_err.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"    
    #include "esp_lvgl_port.h"
    #include "lvgl.h"
    #include "displaySSD1306.h"   
    #include "esp_console.h" 
    #include "driver/uart.h"
    #include "freertos/queue.h"
    #include "esp_timer.h"
    #include "driver/uart.h"
    #include <string.h>
    #include <stdlib.h>
}


#include "cSMP3011.h"
//#include "CBMP280.h"
#include "CGlobalResources.h"

#define LED_PIN     gpio_num_t::GPIO_NUM_16  
#define COMP        gpio_num_t::GPIO_NUM_25
#define V1FE        gpio_num_t::GPIO_NUM_26
#define V2FE        gpio_num_t::GPIO_NUM_27

int pressao_ideal_f_pesado = 35;
int pressao_ideal_t_pesado = 35;
int pressao_ideal_f_leve = 28;
int pressao_ideal_t_leve = 28;
int pressao_ideal_f_final = 0;
int pressao_ideal_t_final = 0;

#define QNT_CARACTERES 100
char caracteres_recebidos[QNT_CARACTERES];
char caracteres_temporarios[QNT_CARACTERES];
bool informacao_nova = false;

#define TOLERANCIA 1

//--------------------------------------------------Botões e sensores de assento-----------------------------------//
#define DEBOUNCE_TIME_MS 50
#define NUM_BOTOES 8
// ------------------ DEFINIÇÃO DOS GPIOs ------------------
#define BE_GPIO  GPIO_NUM_23
#define BS_GPIO  GPIO_NUM_22
#define BD_GPIO  GPIO_NUM_21
#define BI_GPIO  GPIO_NUM_19
#define BC_GPIO  GPIO_NUM_18
#define B1_GPIO  GPIO_NUM_17
#define B2_GPIO  GPIO_NUM_0
#define B3_GPIO  GPIO_NUM_2

// ------------------ ENUM PARA ACESSO NOMINAL ------------------
typedef enum {
    BE = 0,
    BS,
    BD,
    BI,
    BC,
    B1,
    B2,
    B3
} bot_index_t;

// ------------------ TABELAS DE REFERÊNCIA ------------------
static const gpio_num_t botoes[NUM_BOTOES] = {
    BE_GPIO, BS_GPIO, BD_GPIO, BI_GPIO, BC_GPIO, B1_GPIO, B2_GPIO, B3_GPIO
};

// ------------------ NOMES DOS BOTÕES ------------------
static const char *nomes_botoes[NUM_BOTOES] = {
    "BE", "BS", "BD", "BI", "BC", "B1", "B2", "B3"
};

// ------------------ ESTRUTURAS E VARIÁVEIS ------------------
typedef struct {
    bot_index_t indice;
    uint8_t estado;
} botao_event_t;

static QueueHandle_t fila_botoes;
static volatile uint8_t ultimo_estado[NUM_BOTOES];
static uint64_t ultimo_evento_ts[NUM_BOTOES];

// ------------------ ISR (filtra mudanças, sem debounce) ------------------
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    bot_index_t indice = (bot_index_t)(uintptr_t)arg;
    uint8_t estado_atual = !gpio_get_level(botoes[indice]);

    if (estado_atual != ultimo_estado[indice])
    {
        botao_event_t evento = {
        .indice = indice,
        .estado = estado_atual
    };
    /*
    uint64_t agora = esp_timer_get_time() / 1000; // tempo em ms

    // Debounce: ignora eventos muito próximos
    if ((agora - ultimo_evento_ts[indice]) >= DEBOUNCE_TIME_MS) {
        // Atualiza o timestamp do último evento
        ultimo_evento_ts[indice] = agora;
        // Atualiza o estado estável
        ultimo_estado[indice] = estado_atual;
    }
    */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(fila_botoes, &evento, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();

    }
}

// ------------------ TASK DE TRATAMENTO (faz debounce) ------------------
static void tarefa_botoes(void *arg) {
    botao_event_t evento;

    while (1) {
        if (xQueueReceive(fila_botoes, &evento, portMAX_DELAY)) {
            uint64_t agora = esp_timer_get_time() / 1000; // tempo em ms

            // Debounce: ignora eventos muito próximos
            if ((agora - ultimo_evento_ts[evento.indice]) >= DEBOUNCE_TIME_MS) {
                // Atualiza o timestamp do último evento
                ultimo_evento_ts[evento.indice] = agora;
                // Atualiza o estado estável
                ultimo_estado[evento.indice] = evento.estado;

                if (evento.indice == B1 || evento.indice == B2 || evento.indice == B3)
                {
                    if (ultimo_estado[B1] + ultimo_estado[B2] + ultimo_estado[B3] >= 2)
                    {
                        pressao_ideal_f_final = pressao_ideal_f_pesado;
                        pressao_ideal_t_final = pressao_ideal_t_pesado;
                    }
                    else
                    {
                        pressao_ideal_f_final = pressao_ideal_f_leve;
                        pressao_ideal_t_final = pressao_ideal_t_leve;
                    }
                }
            }
        }
    }
}

// ------------------ INICIALIZAÇÃO ------------------
void inicia_botoes(void) {
    fila_botoes = xQueueCreate(1, sizeof(botao_event_t)); ///////////Isso resolveu nao sei como
    gpio_install_isr_service(0);

    uint64_t pin_mask = 0;
    for (int i = 0; i < NUM_BOTOES; i++) {
        pin_mask |= (1ULL << botoes[i]);
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    for (int i = 0; i < NUM_BOTOES; i++) {
        ultimo_estado[i] = !gpio_get_level(botoes[i]);
        ultimo_evento_ts[i] = esp_timer_get_time() / 1000;
        gpio_isr_handler_add(botoes[i], gpio_isr_handler, (void *)(uintptr_t)i);
    }

    xTaskCreate(tarefa_botoes, "tarefa_botoes", 2048, NULL, 10, NULL);
}
//--------------------------------------------------Fim Botões e sensores de assento-----------------------------------//

//---------------------------------------------------Receber dados serial---------------------------------------//
#define UART_PORT_NUM      UART_NUM_0
#define BUF_SIZE           128
#define QNT_CARACTERES     100

// Task única para ler e processar a UART
void uart_task(void* arg)
{
    uint8_t c;
    char buffer[QNT_CARACTERES];
    uint8_t ndx = 0;
    bool recebendo = false;

    while (1)
    {
        int len = uart_read_bytes(UART_PORT_NUM, &c, 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            if (recebendo)
            {
                if (c != '>')
                {
                    buffer[ndx++] = c;
                    if (ndx >= QNT_CARACTERES - 1) ndx = QNT_CARACTERES - 1;
                }
                else
                {
                    // Mensagem completa recebida
                    buffer[ndx] = '\0';
                    ndx = 0;
                    recebendo = false;

                    // Processa imediatamente a string
                    char *token = strtok(buffer, ",");
                    if (token) pressao_ideal_f_leve = atoi(token);

                    token = strtok(NULL, ",");
                    if (token) pressao_ideal_f_pesado = atoi(token);

                    token = strtok(NULL, ",");
                    if (token) pressao_ideal_t_leve = atoi(token);

                    token = strtok(NULL, ",");
                    if (token) pressao_ideal_t_pesado = atoi(token);

                    // DEBUG: printa valores recebidos
                    printf("Valores recebidos: F_leve=%d, F_pesado=%d, T_leve=%d, T_pesado=%d\n",
                           pressao_ideal_f_leve, pressao_ideal_f_pesado,
                           pressao_ideal_t_leve, pressao_ideal_t_pesado);
                }
            }
            else if (c == '<')
            {
                recebendo = true;
                ndx = 0;
            }
        }

        // Pequena pausa para liberar CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Inicializa UART e cria task
void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL);
}

//---------------------------------------------------Fim receber dados serial---------------------------------------//

static void task_serial(void* arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        printf("%d*%d*%d*%d*%d*%d*%d*%d*%d*%d*%d*\n",
               ultimo_estado[BE],
               ultimo_estado[BS],
               ultimo_estado[BD],
               ultimo_estado[BI],
               ultimo_estado[BC],
               SMP3011.getPressure(),
               pressao_ideal_f_final,
               pressao_ideal_t_final,
               pressao_ideal_t_final,
               0,
               0);
    }
}

//Calibragem
void calibragemTask(void *pvParameters)
{
    while (1)
    {
        int leitura_sfe_psi = SMP3011.getPressure();

        // Lógica de controle
        if (leitura_sfe_psi < pressao_ideal_f_final - TOLERANCIA)
        {
            gpio_set_level(V2FE, 0);      // LOW
            gpio_set_level(V1FE, 1);      // HIGH
            gpio_set_level(COMP, 1); // HIGH
        }
        else if (leitura_sfe_psi > pressao_ideal_f_final + 1)
        {
            gpio_set_level(V1FE, 0);       // LOW
            gpio_set_level(COMP, 0); // LOW
            gpio_set_level(V2FE, 1);       // HIGH
        }
        else if ((leitura_sfe_psi >= pressao_ideal_f_final) &&
                 (leitura_sfe_psi <= pressao_ideal_f_final + TOLERANCIA))
        {
            gpio_set_level(V1FE, 0);       // LOW
            gpio_set_level(COMP, 0); // LOW
            gpio_set_level(V2FE, 0);       // LOW
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequena pausa
    }
}

/*
    PROTOTYPES
*/
void sensorSMP3011Task(void *pvParameters);
//void sensorBMP280Task(void *pvParameters);

void statusLedTask(void *pvParameters);

/*
    VARIABLES
*/
cSMP3011    SMP3011;
//CBMP280     BMP280;
//SemaphoreHandle_t sensorMutex;

/**
 * @brief Entry point of the application.
 *
 * This function configures the I2C master mode and scans the bus for devices.
 * The bus is configured to use GPIO 5 for SDA and GPIO 4 for SCL, and the
 * clock speed is set to 100000 Hz. The scan starts from address 1 and goes
 * to address 126 (inclusive). If a device is found at an address, a message
 * is printed to the console with the address of the device.
 */
extern "C" void app_main() 
{

    uart_set_baudrate(UART_NUM_0, 9600); //Seta baudrate do terminal serial para 9600
    esp_log_level_set("*", ESP_LOG_NONE); //Tira os logs do esp-idf

    inicia_botoes(); //Inicia botões e sensores de assento

    if (ultimo_estado[B1] + ultimo_estado[B2] + ultimo_estado[B3] >= 2)
    {
        pressao_ideal_f_final = pressao_ideal_f_pesado;
        pressao_ideal_t_final = pressao_ideal_t_pesado;
    }
    else
    {
        pressao_ideal_f_final = pressao_ideal_f_leve;
        pressao_ideal_t_final = pressao_ideal_t_leve;
    }
                

    //------------------------------------------------
    // Status LED
    //------------------------------------------------
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(COMP, GPIO_MODE_OUTPUT);
    gpio_set_direction(V1FE, GPIO_MODE_OUTPUT);
    gpio_set_direction(V2FE, GPIO_MODE_OUTPUT);



    xTaskCreate(statusLedTask, "statusLedTask", 4096, NULL, 1, NULL);

    //------------------------------------------------
    // I2C Initialization
    //------------------------------------------------    
    I2C.init();

    //------------------------------------------------
    // SMP3011 Initialization
    //------------------------------------------------ 
    SMP3011.init();
    //BMP280.init();
    //sensorMutex = xSemaphoreCreateMutex();
    xTaskCreate(sensorSMP3011Task, "sensorSMP3011Task", 4096, NULL, 4, NULL);
    //xTaskCreate(sensorBMP280Task, "sensorBMP280Task", 4096, NULL, 1, NULL);

    uart_init();

    xTaskCreatePinnedToCore(task_serial, "serial_task", 2048, NULL, 3, NULL, 1);

    xTaskCreate(calibragemTask, "calibragemTask", 2048, NULL, 5, NULL);

    //------------------------------------------------
    // LVGL
    //------------------------------------------------
    displayInit();
    
    //------------------------------------------------
    // Create a Label
    //------------------------------------------------
    lvgl_port_lock(portMAX_DELAY);
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);
        
    lv_obj_t *lblPressure = lv_label_create(scr);    
    lv_label_set_text_fmt(lblPressure, "P: %5.1f", SMP3011.getPressure());    
    lv_obj_set_width(lblPressure, LCD_H_RES);
    lv_obj_align(lblPressure, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblPressure, 0);

    lv_obj_t *lblTemperature = lv_label_create(scr);    
    lv_label_set_text_fmt(lblTemperature, "T: %3.0f", SMP3011.getTemperature());    
    lv_obj_set_width(lblTemperature, LCD_H_RES);
    lv_obj_align(lblTemperature, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblTemperature, 16);

/*
    lv_obj_t *lblPressure2 = lv_label_create(scr);    
    lv_label_set_text_fmt(lblPressure2, "P: %5.1f", BMP280.getPressure());    
    lv_obj_set_width(lblPressure2, LCD_H_RES);
    lv_obj_align(lblPressure2, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblPressure2, 32);

    lv_obj_t *lblTemperature2 = lv_label_create(scr);    
    lv_label_set_text_fmt(lblTemperature2, "T: %3.0f", BMP280.getTemperature());    
    lv_obj_set_width(lblTemperature2, LCD_H_RES);
    lv_obj_align(lblTemperature2, LV_ALIGN_TOP_MID, 0, 0);    
    lv_obj_set_y(lblTemperature2, 48);    
*/
    lvgl_port_unlock();

    while(1)
    {
        lvgl_port_lock(portMAX_DELAY);        
        lv_label_set_text_fmt(lblPressure     , "P1: %5.1f PSI", SMP3011.getPressure());    
        lv_label_set_text_fmt(lblTemperature  , "T1: %3.0f oC" , SMP3011.getTemperature());    
        //lv_label_set_text_fmt(lblPressure2    , "P2: %5.1f kPa", BMP280.getPressure());    
        //lv_label_set_text_fmt(lblTemperature2 , "T2: %3.0f oC" , BMP280.getTemperature());  
        lvgl_port_unlock();
        vTaskDelay(100/portTICK_PERIOD_MS);
    }    
}

void sensorSMP3011Task(void *pvParameters) 
{
    while(1)
    {
        //xSemaphoreTake( sensorMutex, portMAX_DELAY );
        SMP3011.poll(); 
        //xSemaphoreGive( sensorMutex );
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}
/*
void sensorBMP280Task(void *pvParameters) 
{
    while(1)
    {
        //xSemaphoreTake( sensorMutex, portMAX_DELAY );
        BMP280.poll();
        //xSemaphoreGive( sensorMutex );
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}
*/
void statusLedTask(void *pvParameters) 
{
    while(1)
    {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(250/portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(250/portTICK_PERIOD_MS);
    }
}   