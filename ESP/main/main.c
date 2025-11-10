#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "correct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "lora.h"
#include "Stellarsat-frames.h"

//Definiciones pines GPIO
#define MOSI 35
#define MISO 37
#define SCK 36
#define CS 39
#define CS2 2
#define IRQPIN 5
#define RESETPIN 6 
#define IRQ2PIN 7
#define GPIO_OUTS ((1ULL << RESETPIN))
#define GPIO_INS ((1ULL << IRQPIN))
#define SPI_HOST    SPI2_HOST 

//Definiciones GPIO y SPI
const gpio_config_t *GPIO;
static const char *TAG = "STELLARSAT-1";

spi_device_handle_t spi_device1; // Handle for Slave 1
spi_device_handle_t spi_device2; // Handle for Slave 2
//Definiciones interrupciones y semáforos
SemaphoreHandle_t spi_mutex;
SemaphoreHandle_t tx_mutex;
SemaphoreHandle_t tellemetry_mutex;
const TaskHandle_t Task1;
const TaskHandle_t Task2;
const TaskHandle_t Task3;

//Definiciones de las características de la SC
#define HEADER_SIZE 15
#define HEADER_2_SIZE 13
#define MAX_SIZE_TC 255
#define BEACON_SIZE 215
#define CONV_BEACON_SIZE 417
#define RS_BEACON_SIZE 270
#define RS_CONV_BEACON_SIZE 549
#define TELEMETRY_SIZE 684
#define CONV_TELEMETRY_SIZE 1355
#define RS_TELEMETRY_SIZE 780
#define RS_CONV_TELEMETRY_SIZE 1553

uint8_t modulationScheme=0;
uint8_t codeScheme=3;
uint8_t interleaver=1;
uint8_t BS_ID[2]={0xAA,0xAA};
uint8_t SC_ID[2]= {0xD0, 0xD0}; //Spacecraft identifier(SANA assigned)
uint8_t BS_modulationScheme=0;
uint8_t BS_codeScheme=0;
uint8_t BS_interleaver=0;

//Definiciones de métodos
static esp_err_t gpio_init();
static esp_err_t spi_config();
static uint8_t spi_write(uint8_t reg, uint8_t value);
static uint8_t spi_read(uint8_t reg) ;
static int sx1276_begin();
static int sx1276_2_begin();
static int sx1276_transmit(uint8_t data[], int size);
static int sx1276_receive(uint8_t data[]);//, uint8_t message[]
static void telemetry(void *pvParameters);
static void housekeeping(void *pvParameters);
static uint8_t spi_write_tx(uint8_t reg, uint8_t value);
static void RX_task(void *pvParameters);
static void print_bits(uint8_t *data, size_t len);
static int conv(uint8_t *data,size_t len, uint8_t *encoded);
static int rs(uint8_t *data,size_t len, uint8_t *encoded);
void block_interleave(uint8_t *input, uint8_t *output, int rows, int cols);
void block_deinterleave(uint8_t *input, uint8_t *output, int rows, int cols);
static int deconv(uint8_t *data,size_t len, uint8_t *decoded);
static int ders(uint8_t *data,size_t len, uint8_t *decoded);

//paquetes dependiendo de los tamaños de codificación
uint8_t PAYLOAD[MAX_SIZE_TC];
uint8_t DECODED_PL[MAX_SIZE_TC];
uint8_t DATA_TX[200];
uint8_t DATA_BEACON[200];
uint8_t DATA_BEACON_INTERLEAVE[200];
uint8_t DATA_BEACON_CNV[402];
uint8_t DATA_BEACON_RS[255];
uint8_t DATA_BEACON_RS_256[256];
uint8_t DATA_BEACON_RS_INTERLEAVE[256];
uint8_t DATA_BEACON_CNV_RS[514];
uint8_t DATA_HDR[HEADER_SIZE];
uint8_t DATA_BEACON_PACKET[BEACON_SIZE];
uint8_t DATA_BEACON_PACKET_CNV[CONV_BEACON_SIZE];
uint8_t DATA_BEACON_PACKET_RS[RS_BEACON_SIZE];
uint8_t DATA_BEACON_PACKET_CNV_RS[RS_CONV_BEACON_SIZE];
uint8_t DATA_TELEMETRY[669];
uint8_t DATA_TELEMETRY_INTERLEAVE[669];
uint8_t DATA_TELEMETRY_CNV[1340];
uint8_t DATA_TELEMETRY_I_RS[223];
uint8_t DATA_TELEMETRY_O_RS[255];
uint8_t DATA_TELEMETRY_RS[765];
uint8_t DATA_TELEMETRY_RS_768[768];
uint8_t DATA_TELEMETRY_RS_INTERLEAVE[768];
uint8_t DATA_TELEMETRY_CNV_RS[1538];
uint8_t DATA_TELEMETRY_PACKET[TELEMETRY_SIZE];
uint8_t DATA_TELEMETRY_PACKET_CNV[CONV_TELEMETRY_SIZE];
uint8_t DATA_TELEMETRY_PACKET_RS[RS_TELEMETRY_SIZE];
uint8_t DATA_TELEMETRY_PACKET_CNV_RS[RS_CONV_TELEMETRY_SIZE];

//datos modificados por los tc
extern uint8_t start_pay_experiment;
extern uint8_t pay_experiment_configuration;
extern uint8_t ttc_rf;
extern uint8_t obc_actual_mode;
extern uint8_t eps_reboot;
extern uint8_t adcs_reboot;
extern uint8_t obc_ttc_reboot;
extern uint8_t pay_reboot;
extern uint8_t eol_skyfall_procedure;
extern uint8_t solution_experiment_abort;
extern uint8_t solution_experiment_queue_abort;
extern uint8_t new_tle[];
extern uint8_t adcs_solution;
extern uint8_t adcs_baseline;
extern uint8_t com_rssi;
extern uint32_t com_tc_count;
extern uint32_t com_tm_count;
//medida de la telemetría
extern uint8_t stellar_measure[15300];
typedef struct {
    int byte_start;
    int byte_end;
    int configuration;
} TelemetryParams_t;

void app_main(void)
{   
    spi_mutex = xSemaphoreCreateMutex();
    tx_mutex = xSemaphoreCreateMutex();
    tellemetry_mutex = xSemaphoreCreateMutex();
    if(spi_mutex==NULL){
        printf("Error al crear mutex");
        esp_restart();
    }
    if(tx_mutex==NULL){
        printf("Error al crear mutex");
        esp_restart();
    }
    if(tellemetry_mutex==NULL){
        printf("Error al crear mutex");
        esp_restart();
    }
    ESP_ERROR_CHECK(gpio_init());
    printf("Pines configurados\n");
    ESP_ERROR_CHECK(spi_config());
    printf("SPI configurada\n");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    if(!sx1276_begin()){
        esp_restart();
    }
    printf("SX1276 configurado\n");
    if(!sx1276_2_begin()){
        esp_restart();
    }
    printf("SX1276 - 2 configurado\n");

    // uint8_t data_prueba[10]={0x00,0x01,0x02,0x03,0x00,0x00,0x00,0x04,0x05,0x06};
    // uint8_t data_prueba_encoded[36]={0,0,0,0,0,0,0,0,0,0};
    // conv(data_prueba,10, data_prueba_encoded);
    // printf("\n");
    // printf("Encoded data (first 10 bytes):\n");
    // for (int i = 0; i < sizeof(data_prueba_encoded); i++) {
    //     printf("%d ", data_prueba_encoded[i]);
    // }
    // printf("\n");

    xTaskCreatePinnedToCore(housekeeping, "Beacon", 4096, NULL, 5, &Task3, 1);
    xTaskCreatePinnedToCore(RX_task, "Recepción continua", 4096, NULL, 5, &Task1, 0);
    while(1){vTaskDelay(1000/portTICK_PERIOD_MS);};
}

static esp_err_t spi_config(){
    esp_err_t ret;

    // 1. SPI Bus Configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = MISO,
        .mosi_io_num = MOSI,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1, // Not used for standard SPI
        .quadhd_io_num = -1, // Not used for standard SPI
        .max_transfer_sz = 32, // Max transfer size in bytes. Adjust as needed.
    };



    // // 3. SPI Device Configuration for Slave 1
    spi_device_interface_config_t devcfg_slave1 = {
        .clock_speed_hz = 1000000, // Clock speed for Slave 1.  Check slave datasheet!
        .mode = 0,                 // SPI mode (0-3).  Check slave datasheet!
        .spics_io_num = CS, // CS pin for Slave 1
        .queue_size = 7,           // Max queued transactions.
        .flags = 0,                // Various flags.  0 is usually fine.
    };

    // 4. SPI Device Configuration for Slave 2
    spi_device_interface_config_t devcfg_slave2 = {
        .clock_speed_hz = 1000000, // Clock speed for Slave 2. Check slave datasheet!
        .mode = 0,                 // SPI mode (0-3). Check slave datasheet!
        .spics_io_num = CS2, // CS pin for Slave 2
        .queue_size = 7,
        .flags = 0,
    }; 
    // Print pin configuration
    // ESP_LOGI(TAG, "MOSI Pin: %d", MOSI);
    // ESP_LOGI(TAG, "MISO Pin: %d", MISO);
    // ESP_LOGI(TAG, "CLK Pin: %d", SCK);
    // ESP_LOGI(TAG, "CS1 Pin: %d", CS);
    // ESP_LOGI(TAG, "CS2 Pin: %d", CS2);
    // 5. Initialize the SPI bus
    ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO); // Use DMA for efficiency
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret; // Handle error appropriately
    }
    //ESP_LOGI(TAG, "SPI bus initialized");

    // 6. Add Slave Device 1 to the bus
    //ESP_LOGI(TAG, "Slave 1 CS Pin: %d, Clock Speed: %d", devcfg_slave1.spics_io_num, devcfg_slave1.clock_speed_hz);
    ret = spi_bus_add_device(SPI_HOST, &devcfg_slave1, &spi_device1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add slave 1: %s", esp_err_to_name(ret));
        return ret; // Handle error
    }
    //ESP_LOGI(TAG, "Slave device 1 added to SPI bus");

    // 7. Add Slave Device 2 to the bus
    //ESP_LOGI(TAG, "Slave 2 CS Pin: %d, Clock Speed: %d", devcfg_slave2.spics_io_num, devcfg_slave2.clock_speed_hz);
    ret = spi_bus_add_device(SPI_HOST, &devcfg_slave2, &spi_device2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add slave 2: %s", esp_err_to_name(ret));
        return ret; // Handle error
    }
    //ESP_LOGI(TAG, "Slave device 2 added to SPI bus"); 
    return ESP_OK;
}

static uint8_t spi_write(uint8_t reg, uint8_t value){
    esp_err_t ret;
    uint8_t data[2]={reg | 0x80,value};
    spi_transaction_t transaction={
        .tx_buffer = data,
        .rx_buffer = NULL,
        .length = 16,
        .rxlength = 0,
        .flags = 0
    };
    if(xSemaphoreTake(spi_mutex,portMAX_DELAY)){
        ret = spi_device_transmit(spi_device1, &transaction);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit/receive from slave 1: %s", esp_err_to_name(ret));
            return -1; // Handle error
        }
        ESP_ERROR_CHECK(ret);
        //ESP_LOGI(TAG, "Sent and received data from Slave 1:");
        xSemaphoreGive(spi_mutex);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);

    return 1;
}

static uint8_t spi_read(uint8_t reg) {
    esp_err_t ret;
    uint8_t txdata[2] = { reg & 0x7F, 0x00 };  // Bit 7 = 0 para lectura
    uint8_t rxdata[2] = { 0 };
    
    spi_transaction_t transaction = {
        .tx_buffer = txdata,
        .length=16,
        .rx_buffer = rxdata,
        .rxlength=16,  // 16 bits = 2 bytes
        .flags = 0
    };
    if(xSemaphoreTake(spi_mutex,portMAX_DELAY)){
        ret = spi_device_transmit(spi_device1, &transaction);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit/receive from slave 2: %s", esp_err_to_name(ret));
            return 0; // Handle error
        }
        ESP_ERROR_CHECK(ret);
        //ESP_LOGI(TAG, "Reciving data from Slave 2:");
        xSemaphoreGive(spi_mutex);
    }
    return rxdata[1];  // El dato leído es el segundo byte
}

static uint8_t spi_write_tx(uint8_t reg, uint8_t value){
    esp_err_t ret;
    uint8_t data[2]={reg | 0x80,value};
    spi_transaction_t transaction={
        .tx_buffer = data,
        .rx_buffer = NULL,
        .length = 16,
        .rxlength = 0,
        .flags = 0
    };
    if(xSemaphoreTake(spi_mutex,portMAX_DELAY)){
        ret = spi_device_transmit(spi_device2, &transaction);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit/receive from slave 1: %s", esp_err_to_name(ret));
            return 0; // Handle error
        }
        ESP_ERROR_CHECK(ret);
        //ESP_LOGI(TAG, "Sent and received data from Slave 1:");
        xSemaphoreGive(spi_mutex);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);

    return 1;
}

static uint8_t spi_read_tx(uint8_t reg){
    esp_err_t ret;
    uint8_t txdata[2] = { reg & 0x7F, 0x00 };  // Bit 7 = 0 para lectura
    uint8_t rxdata[2] = { 0 };
    
    spi_transaction_t transaction = {
        .tx_buffer = txdata,
        .length=16,
        .rx_buffer = rxdata,
        .rxlength=16,  // 16 bits = 2 bytes
        .flags = 0
    };
    if(xSemaphoreTake(spi_mutex,portMAX_DELAY)){
        ret = spi_device_transmit(spi_device2, &transaction);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit/receive from slave 2: %s", esp_err_to_name(ret));
            return 0; // Handle error
        }
        ESP_ERROR_CHECK(ret);
        //ESP_LOGI(TAG, "Reciving data from Slave 2:");
        xSemaphoreGive(spi_mutex);
    }
    return rxdata[1];  // El dato leído es el segundo byte
}

static esp_err_t gpio_init(){
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTS,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf); //Configuramos los OUTPUTS

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INS;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);  //Configuramos INPUTS


/*     if (gpio_install_isr_service(0) != ESP_OK) { INTERRUPTIONS NOT WORKING
        printf("Error instalando el servicio de interrupciones\n");
    }
    printf("Servicio de interrupciones Ok\n");
    gpio_isr_handler_remove(IRQPIN);
    if (gpio_isr_handler_add(IRQPIN, onTxDone, NULL) != ESP_OK) {
        printf("Error añadiendo el manejador de interrupciones\n");
    }
    printf("Interrupción configurada Ok\n"); */


    return ESP_OK;
}

static int sx1276_begin(){
    gpio_set_level(RESETPIN, 1); //TRANCIVER RESET FOR STARTING IN A WELLKNOWN STATE
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RESETPIN, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RESETPIN, 1);
    //Registers for Common settings
    spi_write(REG_OPMODE, SLEEP_MODE);
    spi_write(REG_BITRATE_MSB, BITRATE_MSB_2);
    spi_write(REG_BITRATE_LSB, BITRATE_LSB_2);
    spi_write(REG_FDEV_MSB,0x00);   //0x0148=328, lo que según nuestra formula equivale a 20kHz----164!!!! 10kHz para arriba y para abajo.
    spi_write(REG_FDEV_LSB,0xA4);
    spi_write(REG_FRF_MSB,0x6C);   //   Fc=438.3MHz-----UPLINK
    spi_write(REG_FRF_MID,0xD3);  
    spi_write(REG_FRF_LSB,0x33);    
    //Registers for the Transmitter
    //spi_write(REG_PA_CONFIG,0x4F);
    spi_write(REG_PA_RAMP,0b00000100);  //filtro con BT de 0 y un raise/fall time de 250us para evitar problemas
    spi_write(REG_OCP, 0b00110010);   //protección de 150mA
    //Registers for the Receiver
    spi_write(REG_LNA, 0b00100000);
    spi_write(REG_RX_CONFIG,0b00111111);
    spi_write(REG_RSSI_CONFIG,00000011); //No toco el RSSI Offset, pero dejo en vez de 8 muestras de RSSI a 16 para tener mayor resolución. +-3dB
    //spi_write(REG_RSSI_COLLISION,);  //Por defecto, 10dB considerado como una colisión
    //spi_write(REG_RRS_THRESH,);  También lo dejo por defecto (0xFF)
    //spi_write(REG_RSSI_VAUE,);    //Solo se lee!!
    spi_write(REG_RX_BW,0b00001100);    //Teóricamente channel filter bandwidth, tengo que ver si es unilateral. Está a 25kHz, si es unilateral bajar bastante!
    //spi_write(REG_AFC_BW,);  //El filtro de la frecuencia del bucle de control de frecuencia. De momento se queda default
    spi_write(REG_AFC_FEI,0b0001001);
    spi_write(REG_PREAMBLE_DETECT, 0b11001010);  //Detectar 3 bytes de preámbulo. Tolerancia por defecto (10 chips creo de 4 chips por bit)
    //spi_write(REG_RX_TIMEOUT_1,); Por defecto mientras no funcionen las interrupciones.
    //spi_write(REG_RX_TIMEOUT_2,);
    //spi_write(REG_RX_TIMEOUT_3,);
    //spi_write(REG_RX_DELAY,);
    //RC Oscillator registers VALUES DEFAULT
    //  //  //  //  //  //  //  //  //  //
    //Packet Handling registers
    spi_write(REG_PREAMBLE_MSB, 0x00);
    spi_write(REG_PREAMBLE_LSB, 0x08);
    spi_write(REG_SYNC_CONFIG, 0b01010111);
    spi_write(REG_SYNC_VALUE_1,0b00000000);//2D
    spi_write(REG_SYNC_VALUE_2,0b01000101);//D4
    spi_write(REG_SYNC_VALUE_3,0b10100110);//2D
    spi_write(REG_SYNC_VALUE_4,0b11100010);//D4
    spi_write(REG_SYNC_VALUE_5,0b00110101);//2D
    spi_write(REG_SYNC_VALUE_6,0b00111001);//D4
    spi_write(REG_SYNC_VALUE_7,0b00010111);//2D
    spi_write(REG_SYNC_VALUE_8,0b11010101);//D4
    spi_write(REG_PACKET_CONFIG_1,0b10011000);   //tamaño variable[1], NO codi manchester[00], CRC [1], No borra paquete malo pero da interrup[1], sin filtrar por address[00], CRC CCITT[0]
    spi_write(REG_PACKET_CONFIG_2,0b01000000);
    spi_write(REG_PAYLOAD_LENGTH,0b11111111);   //64Bytes ahora mismo

    
    //MAS COSAS ENTRE MEDIAS QUE NO HE CONFIGURADO AÚN
    spi_write(REG_DIO_MAPPING_1, 0b00100100);
    //spi_write(REG_DIO_MAPPING_1, 0b00000000);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    spi_write(REG_OPMODE, STBY_MODE);

    return 1;

}

static int sx1276_2_begin(){
    gpio_set_level(RESETPIN, 1); //TRANCIVER RESET FOR STARTING IN A WELLKNOWN STATE
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RESETPIN, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RESETPIN, 1);
    //Registers for Common settings
    spi_write_tx(REG_OPMODE, SLEEP_MODE);
    spi_write_tx(REG_BITRATE_MSB, BITRATE_MSB_2);
    spi_write_tx(REG_BITRATE_LSB, BITRATE_LSB_2);
    spi_write_tx(REG_FDEV_MSB,0x00);   //0x0148=328, lo que según nuestra formula equivale a 20kHz----164!!!! 10kHz para arriba y para abajo.
    spi_write_tx(REG_FDEV_LSB,0xA4);
    spi_write_tx(REG_FRF_MSB,0x6D);   //   Ahora mismo 435.3MHz-->Frecuencia en la que transmito 0x7E
    spi_write_tx(REG_FRF_MID,0x93);   //0x13
    spi_write_tx(REG_FRF_LSB,0x33);    //0x33
    //Registers for the Transmitter
    spi_write_tx(REG_PA_CONFIG,0xFF);
    spi_write_tx(REG_PA_RAMP,0b00000100);  //filtro con BT de 0 y un raise/fall time de 250us para evitar problemas
    spi_write_tx(REG_OCP, 0b001001011);   //protección de 150mA
    //Registers for the Receiver
    spi_write_tx(REG_LNA, 0b00100000);
    spi_write_tx(REG_RX_CONFIG,0b00111111);
    spi_write_tx(REG_RSSI_CONFIG,00000011); //No toco el RSSI Offset, pero dejo en vez de 8 muestras de RSSI a 16 para tener mayor resolución. +-3dB
    //spi_write_tx(REG_RSSI_COLLISION,);  //Por defecto, 10dB considerado como una colisión
    //spi_write_tx(REG_RRS_THRESH,);  También lo dejo por defecto (0xFF)
    //spi_write_tx(REG_RSSI_VAUE,);    //Solo se lee!!
    spi_write_tx(REG_RX_BW,0b00001100);    //Teóricamente channel filter bandwidth, tengo que ver si es unilateral. Está a 25kHz, si es unilateral bajar bastante!
    //spi_write_tx(REG_AFC_BW,);  //El filtro de la frecuencia del bucle de control de frecuencia. De momento se queda default
    spi_write_tx(REG_AFC_FEI,0b0001001);
    spi_write_tx(REG_PREAMBLE_DETECT, 0b11001010);  //Detectar 3 bytes de preámbulo. Tolerancia por defecto (10 chips creo de 4 chips por bit)
    //spi_write_tx(REG_RX_TIMEOUT_1,); Por defecto mientras no funcionen las interrupciones.
    //spi_write_tx(REG_RX_TIMEOUT_2,);
    //spi_write_tx(REG_RX_TIMEOUT_3,);
    //spi_write_tx(REG_RX_DELAY,);
    //RC Oscillator registers VALUES DEFAULT
    //  //  //  //  //  //  //  //  //  //
    //Packet Handling registers
    spi_write_tx(REG_PREAMBLE_MSB, 0x00);
    spi_write_tx(REG_PREAMBLE_LSB, 0x08);
    spi_write_tx(REG_SYNC_CONFIG, 0b01010111);
    spi_write_tx(REG_SYNC_VALUE_1,0b00000000);//2D
    spi_write_tx(REG_SYNC_VALUE_2,0b01000101);//D4
    spi_write_tx(REG_SYNC_VALUE_3,0b10100110);//2D
    spi_write_tx(REG_SYNC_VALUE_4,0b11100010);//D4
    spi_write_tx(REG_SYNC_VALUE_5,0b00110101);//2D
    spi_write_tx(REG_SYNC_VALUE_6,0b00111001);//D4
    spi_write_tx(REG_SYNC_VALUE_7,0b00010111);//2D
    spi_write_tx(REG_SYNC_VALUE_8,0b11010101);//D4
    spi_write_tx(REG_PACKET_CONFIG_1,0b00011000);   //tamaño concreto[0], NO codi manchester[00], CRC [1], No borra paquete malo pero da interrup[1], sin filtrar por address[00], CRC CCITT[0]
    spi_write_tx(REG_PACKET_CONFIG_2,0b01000000);
    spi_write_tx(REG_PAYLOAD_LENGTH,0b11100001);   //200Bytes ahora mismo 0b11001000
    spi_write_tx(REG_FIFO_THRESH, 0b00100000);
    
    //MAS COSAS ENTRE MEDIAS QUE NO HE CONFIGURADO AÚN
    spi_write_tx(REG_DIO_MAPPING_1, 0b00100100);
    //spi_write_tx(REG_DIO_MAPPING_1, 0b00000000);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    spi_write_tx(REG_OPMODE, STBY_MODE);
    return 1;
}

static int sx1276_transmit(uint8_t data[], int size){
    //printf("FLAGS previos a transmitir: 0x%02X\n", spi_read_tx(REG_IRQ_FLAGS_1));
    
    for(int i=0; i<64; i++){
        spi_write_tx(REG_FIFO, data[i]);
        //vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    spi_write_tx(REG_OPMODE, TX_MODE);
    //printf("Entrando a transmitir\n");
    for(int i=64; i<size; i++){
        while((spi_read_tx(REG_IRQ_FLAGS_2) & 0b00100000)){vTaskDelay(0.001 / portTICK_PERIOD_MS);};
        spi_write_tx(REG_FIFO, data[i]);
        //vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    //printf("Datos en buffer\n");


    //spi_write_tx(REG_OPMODE, FSTx_MODE);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    //uint8_t mode = spi_read_tx(REG_OPMODE);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    //printf("Modo actual del SX1276: 0x%02X\n", mode);

    //spi_write_tx(REG_OPMODE, TX_MODE);

    //vTaskDelay(10 / portTICK_PERIOD_MS);
    //mode = spi_read_tx(REG_OPMODE);
    //printf("Modo actual del SX1276: 0x%02X\n", mode);

    while(!(spi_read_tx(REG_IRQ_FLAGS_2) & 0b00001000)){vTaskDelay(0.1 / portTICK_PERIOD_MS);};
    uint8_t irq_flags = spi_read_tx(REG_IRQ_FLAGS_2);
    if (irq_flags & 0b00001000) { // Check bit 3 (TxDone)
        printf("Packet sent successfully!\n");
        //printf("FLAGS: 0x%02X\n", spi_read_tx(REG_IRQ_FLAGS_1));
        spi_write_tx(REG_OPMODE, STBY_MODE);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        //printf("FLAGS standby: 0x%02X\n", spi_read_tx(REG_IRQ_FLAGS_1));
    }

    return 1;
}

static int sx1276_receive(uint8_t data[]){
    int message_type=0;
    int message_size=0;
    message_size = (spi_read(REG_FIFO));
    //printf("Message size: %d\n", message_size);
    for(int i=0; i<message_size; i++){
        data[i] = spi_read(REG_FIFO);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    uint8_t type_inter=data[0];
    if(((type_inter&0xF0)>>4)!=2){
        printf("No es TC");
        return 0;
    }
    BS_modulationScheme=data[1];
    BS_codeScheme=data[2];
    BS_interleaver=type_inter&0x0F;
    if((SC_ID[0]!=data[3]) | (SC_ID[1]!=data[4])){
        printf("No es para este SC");
        return 0;
    }
    BS_ID[0]=data[5];
    BS_ID[1]=data[6];
    uint8_t telecommand[message_size-13];
    memcpy(telecommand, &data[12], message_size-12);

    if(BS_codeScheme==1){///////////////////////TERMINAR//////////////////////////
        deconv(telecommand, sizeof(telecommand)*8, DECODED_PL);
        memcpy(telecommand, DECODED_PL, sizeof(telecommand));
        if(BS_interleaver){

        }
    }else if(BS_codeScheme==2){
        ders(telecommand, sizeof(telecommand)*8,DECODED_PL);
        if(BS_interleaver){

        }
    }else if(BS_codeScheme==3){
        
        if(BS_interleaver){

        }
    }

    // for(int i=0; i<message_size; i++){
    //     printf("%02X ", data[i]);
    //     vTaskDelay(1 / portTICK_PERIOD_MS);
    // }

    for(int i=0; i<sizeof(telecommand); i++){
        printf("%02X ", telecommand[i]);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    if(telecommand[1-1]==0x00 && telecommand[1]==02){
        if(telecommand[1+1]==0X00 && telecommand[1+2]==0X02){
            start_pay_experiment=1; //start pay experiment with configuration 1
            pay_experiment_configuration=1;
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X04){
            start_pay_experiment=1; //start pay experiment with configuration 2
            pay_experiment_configuration=2;
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X06){   
            start_pay_experiment=1; //start pay experiment with configuration 3
            pay_experiment_configuration=3;
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X08){
            message_type=1; //downlink the complete pay experiment with configuration 1-- pueden estar en distintas zonas de memoria los distintos experimentos/configuraciones
            TelemetryParams_t telemetry_params;
            telemetry_params.byte_start=0;
            telemetry_params.byte_end=sizeof(stellar_measure)/TELEMETRY_SIZE;
            telemetry_params.configuration=1;
            xTaskCreatePinnedToCore(telemetry, "Enviar Telemetria", 8192, &telemetry_params, 5, &Task2, 1);
            printf("Telemetría 1");
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X0A){
            message_type=1; //downlink the complete pay experiment with configuration 2
            TelemetryParams_t telemetry_params;
            telemetry_params.byte_start=0;
            telemetry_params.byte_end=sizeof(stellar_measure)/TELEMETRY_SIZE;
            telemetry_params.configuration=2;
            xTaskCreatePinnedToCore(telemetry, "Enviar Telemetria", 8192, &telemetry_params, 5, &Task2, 1);
            printf("Telemetría 2");
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X0C){
            message_type=1; //downlink the complete pay experiment with configuration 3
            TelemetryParams_t telemetry_params;
            telemetry_params.byte_start=0;
            telemetry_params.byte_end=sizeof(stellar_measure)/TELEMETRY_SIZE;
            telemetry_params.configuration=3;
            xTaskCreatePinnedToCore(telemetry, "Enviar Telemetria", 8192, &telemetry_params, 5, &Task2, 1);
            printf("Telemetría 3");
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X0E){
            message_type=2; //downlink the complete pay experiment with configuration 1 from paquet n to m
            TelemetryParams_t telemetry_params;
            telemetry_params.byte_start=(int)(telecommand[1+3]);
            telemetry_params.byte_end=(int)(telecommand[1+4]);
            telemetry_params.configuration=1;
            xTaskCreatePinnedToCore(telemetry, "Enviar Telemetria", 8192, &telemetry_params, 5, &Task2, 1);
            printf("Telemetría 1NM");
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X10){
            message_type=2; //downlink the complete pay experiment with configuration 2 from paquet n to m
            TelemetryParams_t telemetry_params;
            telemetry_params.byte_start=(int)(telecommand[1+3]);
            telemetry_params.byte_end=(int)(telecommand[1+4]);
            telemetry_params.configuration=2;
            xTaskCreatePinnedToCore(telemetry, "Enviar Telemetria", 8192, &telemetry_params, 5, &Task2, 1);
            printf("Telemetría 2NM");
        }else if(telecommand[1+1]==0X00 && telecommand[1+2]==0X12){
            message_type=2; //downlink the complete pay experiment with configuration 3 from paquet n to m
            TelemetryParams_t telemetry_params;
            telemetry_params.byte_start=(int)(telecommand[1+3]);
            telemetry_params.byte_end=(int)(telecommand[1+4]);
            telemetry_params.configuration=3;
            xTaskCreatePinnedToCore(telemetry, "Enviar Telemetria", 8192, &telemetry_params, 5, &Task2, 1);
            printf("Telemetría 3NM");
        }
    }else if(telecommand[1-1]==0x00 && telecommand[1]==0x04){
        if(telecommand[1+1]==0x00 && telecommand[1+2]==0x02){
            ttc_rf=0;
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x04){
            ttc_rf=1;
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x06){
            //handshake
            printf("holis\n");
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x08){
            //goodbye
            printf("adios\n");
        }
    }else if(telecommand[1-1]==0x00 && telecommand[1]==0x06){
        if(telecommand[1+1]==0x00 && telecommand[1+2]==0x02){
            if(obc_actual_mode==OPERATION_MODE){   //change from operation to leop mode
                obc_actual_mode=LEOP_MODE;
            }
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x04){
            if(obc_actual_mode==LEOP_MODE){ //change from LEOP to operation mode
                obc_actual_mode=OPERATION_MODE;
            }
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x06){
            if(obc_actual_mode==OPERATION_MODE){    //change from op mode to safe mode
                obc_actual_mode=SAFE_MODE;
            }
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x08){
            if(obc_actual_mode==SAFE_MODE){ //change from safe mode to operation mode
                obc_actual_mode=OPERATION_MODE;
            }
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x0A){
            //get the SC Events
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x0C){
            eps_reboot=1;   //perform the eps reboot
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x0E){
            adcs_reboot=1;  //perform the adcs reboot
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x10){
            obc_ttc_reboot=1;  //perform the ttc reboot
            esp_restart();
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x12){
            pay_reboot=1;  //perform the pay reboot
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x14){
            //return the obsw baseline
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x16){
            eol_skyfall_procedure=1;    //activate the EOL-Skyfall Procedure
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x18){
            eol_skyfall_procedure=2;    //abort the EOL-Skyfall Procedure
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x1A){
            solution_experiment_abort=1;    //abort solution/experiment
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x1C){
            solution_experiment_queue_abort=1;
        }
    }else if(telecommand[1-1]==0x00 && telecommand[1]==0x08){
        if(telecommand[1+1]==0x00 && telecommand[1+2]==0x02){
            //upload the TLE from GS to the SC
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x04){
            adcs_baseline=1;    //return to ADCS baseline
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x06){
            adcs_solution=1;    //start the ADCS solution 1
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x08){
            adcs_solution=2;    //start the ADCS solution 2
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x0A){
            adcs_solution=3;    //start the ADCS solution 3
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x0C){
            //download TBD hours of ADCS Solution 1
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x0E){
            //download TBD hours of ADCS Solution 2
        }else if(telecommand[1+1]==0x01 && telecommand[1+2]==0x00){
            //download TBD hours of ADCS Solution 3
        }else if(telecommand[1+1]==0x01 && telecommand[1+2]==0x02){
            //download TBD hours of ADCS Solution 1 from package N to M
        }else if(telecommand[1+1]==0x01 && telecommand[1+2]==0x04){
            //download TBD hours of ADCS Solution 2 from package N to M
        }else if(telecommand[1+1]==0x01 && telecommand[1+2]==0x06){
            //download TBD hours of ADCS Solution 3 from package N to M
        }
    }else if(telecommand[1-1]==0x00 && telecommand[1]==0x0A){
        if(telecommand[1+1]==0x00 && telecommand[1+2]==0x02){
            if((telecommand[1+3]==0x00) | (telecommand[1+3]==0x01) | (telecommand[1+3]==0x02) | (telecommand[1+3]==0x03)){
                spi_write_tx(REG_OPMODE, STBY_MODE);
                codeScheme=(uint8_t)telecommand[1+3];
                printf("New code scheme: %d\n", codeScheme);
                if(codeScheme==1){
                    spi_write_tx(REG_PACKET_CONFIG_2, 0b01000001);
                    spi_write_tx(REG_PAYLOAD_LENGTH,0b10101011);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }else if(codeScheme==2){
                    spi_write_tx(REG_PACKET_CONFIG_2, 0b01000001);
                    spi_write_tx(REG_PAYLOAD_LENGTH,0b00011000);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }else if(codeScheme==3){
                    spi_write_tx(REG_PACKET_CONFIG_2, 0b01000010);
                    spi_write_tx(REG_PAYLOAD_LENGTH,0b00011010);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }else{
                    spi_write_tx(REG_PACKET_CONFIG_2, 0b01000000);
                    spi_write_tx(REG_PAYLOAD_LENGTH,0b11100001);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x04){
            if(telecommand[1+3]==(0x00)){
                modulationScheme=(uint8_t)telecommand[1+3];
                printf("New modu scheme scheme: %d\n", modulationScheme);
            }
        }else if(telecommand[1+1]==0x00 && telecommand[1+2]==0x06){
            if((telecommand[1+3]== 0x01)| (telecommand[1+3]== 0x00)){
                interleaver=(uint8_t)telecommand[1+3];
                printf("New interleaver scheme: %d\n", interleaver);
            }
        }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);

    return 1;
}

static void telemetry(void *pvParameters){
    TelemetryParams_t *params = (TelemetryParams_t *)pvParameters;
    int byte_start = (params->byte_start)*TELEMETRY_SIZE;
    int byte_end = (params->byte_end)*TELEMETRY_SIZE;

    if(xSemaphoreTake(tellemetry_mutex,portMAX_DELAY)){
        spi_write_tx(REG_OPMODE, STBY_MODE);
        printf("%d ",codeScheme);
        printf("%d ",interleaver);
        //cambiar tamaños de telemetría y poner semáforo
        if(((int)codeScheme)==0){
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000010);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b10110110);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else if(((int)codeScheme)==1){
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000101);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b01010101);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else if(((int)codeScheme)==2){
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000011);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b00010110);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else if(((int)codeScheme)==3){
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000110);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b00011011);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else{
            xSemaphoreGive(tellemetry_mutex);
            return;
        }

        for(int i=byte_start;i<byte_end;i+=612){
            if(byte_end>sizeof(stellar_measure)){
                byte_end=sizeof(stellar_measure);
            }
            uint16_t packet=i/612;
            printf("%d", packet);
            printf("\n");
            create_telemetry(DATA_TELEMETRY, i, i+612, packet);
            if(((int)codeScheme)==1){
                printf(" Telemetry 1 \n");
                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int j=0;i<sizeof(DATA_TELEMETRY);j+=8){
                        block_interleave(&DATA_TELEMETRY[j], &DATA_TELEMETRY_INTERLEAVE[j], 2, 4);
                    }
                    memcpy(DATA_TELEMETRY,DATA_TELEMETRY_INTERLEAVE,sizeof(DATA_TELEMETRY_INTERLEAVE));
                }
                uint8_t telemetry_size[3]={(CONV_TELEMETRY_SIZE>>16)&0xFF,(CONV_TELEMETRY_SIZE>>8)&0xFF,CONV_TELEMETRY_SIZE&0xFF};
                create_header(DATA_HDR, telemetry_size);
                conv(DATA_TELEMETRY,sizeof(DATA_TELEMETRY), DATA_TELEMETRY_CNV);
                memcpy(DATA_TELEMETRY_PACKET_CNV,DATA_HDR,sizeof(DATA_HDR));
                memcpy(DATA_TELEMETRY_PACKET_CNV+sizeof(DATA_HDR),DATA_TELEMETRY_CNV,sizeof(DATA_TELEMETRY_CNV));

                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_TELEMETRY_PACKET_CNV, sizeof(DATA_TELEMETRY_PACKET_CNV));
                    xSemaphoreGive(tx_mutex);
                }
            }
            else if(((int)codeScheme)==2){
                printf(" telemetry 2 \n");
                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int j=0;j<sizeof(DATA_TELEMETRY);j+=8){
                        block_interleave(&DATA_TELEMETRY[j], &DATA_TELEMETRY_INTERLEAVE[j], 2, 4);
                    }
                    memcpy(DATA_TELEMETRY,DATA_TELEMETRY_INTERLEAVE,sizeof(DATA_TELEMETRY_INTERLEAVE));
                }
                uint8_t telemetry_size[3]={(RS_TELEMETRY_SIZE>>16)&0xFF,(RS_TELEMETRY_SIZE>>8)&0xFF,RS_TELEMETRY_SIZE&0xFF};
                create_header(DATA_HDR, telemetry_size);
                for(int j=0; j<3; j++){
                    memcpy(DATA_TELEMETRY_I_RS, &DATA_TELEMETRY[223*j], 223);
                    rs(DATA_TELEMETRY_I_RS,sizeof(DATA_TELEMETRY_I_RS), DATA_TELEMETRY_O_RS);
                    memcpy(&DATA_TELEMETRY_RS[223*j], DATA_TELEMETRY_O_RS, sizeof(DATA_TELEMETRY_O_RS));
                }
                memcpy(DATA_TELEMETRY_PACKET_RS,DATA_HDR,sizeof(DATA_HDR));
                memcpy(DATA_TELEMETRY_PACKET_RS+sizeof(DATA_HDR),DATA_TELEMETRY_RS,sizeof(DATA_TELEMETRY_RS));
                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_TELEMETRY_PACKET_RS, sizeof(DATA_TELEMETRY_PACKET_RS));
                    xSemaphoreGive(tx_mutex);
                }
            }else if(((int)codeScheme)==3){
                uint8_t telemetry_size[3]={(RS_CONV_TELEMETRY_SIZE>>16)&0xFF,(RS_CONV_TELEMETRY_SIZE>>8)&0xFF,RS_CONV_TELEMETRY_SIZE&0xFF};
                create_header(DATA_HDR, telemetry_size);
                printf(" telemetry 3 \n");
                for(int j=0; j<3; j++){
                    memcpy(DATA_TELEMETRY_I_RS, &DATA_TELEMETRY[223*j], 223);
                    rs(DATA_TELEMETRY_I_RS,sizeof(DATA_TELEMETRY_I_RS), DATA_TELEMETRY_O_RS);
                    memcpy(&DATA_TELEMETRY_RS[223*j], DATA_TELEMETRY_O_RS, sizeof(DATA_TELEMETRY_O_RS));
                }            
                memcpy(DATA_TELEMETRY_RS_768,DATA_TELEMETRY_RS,sizeof(DATA_TELEMETRY_RS));
                DATA_TELEMETRY_RS_768[765]=0;
                DATA_TELEMETRY_RS_768[766]=0;
                DATA_TELEMETRY_RS_768[767]=0;

                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int j=0; j<sizeof(DATA_TELEMETRY_RS_768);j+=8){
                        block_interleave(&DATA_TELEMETRY_RS_768[j], &DATA_TELEMETRY_RS_INTERLEAVE[j], 2, 4);
                    }
                    memcpy(DATA_TELEMETRY_RS_768,DATA_TELEMETRY_RS_INTERLEAVE,sizeof(DATA_TELEMETRY_RS_INTERLEAVE));
                }
                conv(DATA_TELEMETRY_RS_768,sizeof(DATA_TELEMETRY_RS_768), DATA_TELEMETRY_CNV_RS);
                memcpy(DATA_TELEMETRY_PACKET_CNV_RS,DATA_HDR,sizeof(DATA_HDR));
                memcpy(DATA_TELEMETRY_PACKET_CNV_RS+sizeof(DATA_HDR),DATA_TELEMETRY_CNV_RS,sizeof(DATA_TELEMETRY_CNV_RS));
                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_TELEMETRY_PACKET_CNV_RS, sizeof(DATA_TELEMETRY_PACKET_CNV_RS));
                    xSemaphoreGive(tx_mutex);
                }

            }else{
                printf(" Telemetry 0 \n");
                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int j=0;j<sizeof(DATA_TELEMETRY);j+=8){
                        block_interleave(&DATA_TELEMETRY[j], &DATA_TELEMETRY_INTERLEAVE[j], 2, 4);
                    }
                    memcpy(DATA_TELEMETRY,DATA_TELEMETRY_INTERLEAVE,sizeof(DATA_TELEMETRY_INTERLEAVE));
                }
                uint8_t telemetry_size[3]={(TELEMETRY_SIZE>>16)&0xFF,(TELEMETRY_SIZE>>8)&0xFF,TELEMETRY_SIZE&0xFF};
                create_header(DATA_HDR, telemetry_size);
                mempcpy(DATA_TELEMETRY_PACKET,DATA_HDR,sizeof(DATA_HDR));
                mempcpy(DATA_TELEMETRY_PACKET+sizeof(DATA_HDR),DATA_TELEMETRY,sizeof(DATA_TELEMETRY));

                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_TELEMETRY_PACKET, sizeof(DATA_TELEMETRY_PACKET));
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    xSemaphoreGive(tx_mutex);
                }
            }
        }
        printf("telemetry sended!\n");
        com_tm_count=com_tm_count+1;
        if(codeScheme==1){
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000001);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b10101011);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }else if(codeScheme==2){
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000001);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b00011000);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }else if(codeScheme==3){
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000010);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b00011010);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }else{
            spi_write_tx(REG_PACKET_CONFIG_2, 0b01000000);
            spi_write_tx(REG_PAYLOAD_LENGTH,0b11100001);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    xSemaphoreGive(tellemetry_mutex);
    vTaskDelete(Task2);
    return;
}

static void housekeeping(void *pvParameters){
    while(1){
        if(xSemaphoreTake(tellemetry_mutex,portMAX_DELAY)){
            //TAMAÑO CON CABEZERA 233 BYTES- CON CONVO 435 BYTES - CON RS 288 BYTES - CON AMBOS - 
            printf("%d ",codeScheme);
            printf("%d ",interleaver);
            create_housekeeping(DATA_BEACON);
            if(((int)codeScheme)==1){
                printf(" housekeeping 1 ");
                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int i=0;i<sizeof(DATA_BEACON);i+=8){
                        block_interleave(&DATA_BEACON[i], &DATA_BEACON_INTERLEAVE[i], 2, 4);
                    }
                    memcpy(DATA_BEACON,DATA_BEACON_INTERLEAVE,sizeof(DATA_BEACON_INTERLEAVE));
                }
                uint8_t housekeeping_size[3]={(CONV_BEACON_SIZE>>16)&0xFF,(CONV_BEACON_SIZE>>8)&0xFF,CONV_BEACON_SIZE&0xFF};
                create_header(DATA_HDR, housekeeping_size);
                conv(DATA_BEACON,sizeof(DATA_BEACON), DATA_BEACON_CNV);
                memcpy(DATA_BEACON_PACKET_CNV,DATA_HDR,sizeof(DATA_HDR));
                memcpy(DATA_BEACON_PACKET_CNV+sizeof(DATA_HDR),DATA_BEACON_CNV,sizeof(DATA_BEACON_CNV));

                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_BEACON_PACKET_CNV, sizeof(DATA_BEACON_PACKET_CNV));
                    xSemaphoreGive(tx_mutex);
                }
            }
            else if(((int)codeScheme)==2){
                printf(" housekeeping 2 ");
                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int i=0;i<sizeof(DATA_BEACON);i+=8){
                        block_interleave(&DATA_BEACON[i], &DATA_BEACON_INTERLEAVE[i], 2, 4);
                    }
                    memcpy(DATA_BEACON,DATA_BEACON_INTERLEAVE,sizeof(DATA_BEACON_INTERLEAVE));
                }
                uint8_t housekeeping_size[3]={(RS_BEACON_SIZE>>16)&0xFF,(RS_BEACON_SIZE>>8)&0xFF,RS_BEACON_SIZE&0xFF};
                create_header(DATA_HDR, housekeeping_size);
                rs(DATA_BEACON,sizeof(DATA_BEACON), DATA_BEACON_RS);
                memcpy(DATA_BEACON_PACKET_RS,DATA_HDR,sizeof(DATA_HDR));
                memcpy(DATA_BEACON_PACKET_RS+sizeof(DATA_HDR),DATA_BEACON_RS,sizeof(DATA_BEACON_RS));
                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_BEACON_PACKET_RS, sizeof(DATA_BEACON_PACKET_RS));
                    xSemaphoreGive(tx_mutex);
                }
            }else if(((int)codeScheme)==3){
                uint8_t housekeeping_size[3]={(RS_CONV_BEACON_SIZE>>16)&0xFF,(RS_CONV_BEACON_SIZE>>8)&0xFF,RS_CONV_BEACON_SIZE&0xFF};
                create_header(DATA_HDR, housekeeping_size);
                printf(" housekeeping 3 ");
                rs(DATA_BEACON,sizeof(DATA_BEACON), DATA_BEACON_RS);
                memcpy(DATA_BEACON_RS_256,DATA_BEACON_RS,sizeof(DATA_BEACON_RS));
                DATA_BEACON_RS_256[255]=0;

                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int i=0; i<sizeof(DATA_BEACON_RS_256);i+=8){
                        block_interleave(&DATA_BEACON_RS_256[i], &DATA_BEACON_RS_INTERLEAVE[i], 2, 4);
                    }
                    memcpy(DATA_BEACON_RS_256,DATA_BEACON_RS_INTERLEAVE,sizeof(DATA_BEACON_RS_INTERLEAVE));
                }
                conv(DATA_BEACON_RS_256,sizeof(DATA_BEACON_RS_256), DATA_BEACON_CNV_RS);
                memcpy(DATA_BEACON_PACKET_CNV_RS,DATA_HDR,sizeof(DATA_HDR));
                memcpy(DATA_BEACON_PACKET_CNV_RS+sizeof(DATA_HDR),DATA_BEACON_CNV_RS,sizeof(DATA_BEACON_CNV_RS));
                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_BEACON_PACKET_CNV_RS, sizeof(DATA_BEACON_PACKET_CNV_RS));
                    xSemaphoreGive(tx_mutex);
                }

            }else{
                printf(" housekeeping 0 ");
                if(((int)interleaver)==1){
                    printf(" interleaver 1 ");
                    for(int i=0;i<sizeof(DATA_BEACON);i+=8){
                        block_interleave(&DATA_BEACON[i], &DATA_BEACON_INTERLEAVE[i], 2, 4);
                    }
                    memcpy(DATA_BEACON,DATA_BEACON_INTERLEAVE,sizeof(DATA_BEACON_INTERLEAVE));
                }
                uint8_t housekeeping_size[3]={(BEACON_SIZE>>16)&0xFF,(BEACON_SIZE>>8)&0xFF,BEACON_SIZE&0xFF};
                create_header(DATA_HDR, housekeeping_size);
                mempcpy(DATA_BEACON_PACKET,DATA_HDR,sizeof(DATA_HDR));
                mempcpy(DATA_BEACON_PACKET+sizeof(DATA_HDR),DATA_BEACON,sizeof(DATA_BEACON));
                if(xSemaphoreTake(tx_mutex,portMAX_DELAY)){
                    sx1276_transmit(DATA_BEACON_PACKET, 234);
                    xSemaphoreGive(tx_mutex);
                }
            }
        }
        xSemaphoreGive(tellemetry_mutex);

        vTaskDelay(7000 / portTICK_PERIOD_MS);
    }
}

static void RX_task(void *pvParameters){
    spi_write(REG_OPMODE, RX_MODE);

    while(1){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        printf("Esperando paquete: \n");
        while(!(spi_read(REG_IRQ_FLAGS_2) & 0b00000100)){vTaskDelay(10 / portTICK_PERIOD_MS);};
        printf("Packet recevied! ");
        com_rssi=spi_read(REG_RSSI_VAUE);
        com_tc_count=com_tc_count+1;
        printf("RSSI: -%ddBm\n", com_rssi/2);
        sx1276_receive(PAYLOAD);//, DATA_TX


    } 
}

static void print_bits(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        for (int j = 7; j >= 0; j--) {
            printf("%d", (data[i] >> j) & 1);
        }
        printf(" ");
    }
    printf("\n");
}

static int conv(uint8_t *data,size_t len, uint8_t *encoded) {
    const correct_convolutional_polynomial_t poly[] = {0b111, 0b101};

    correct_convolutional *conv = correct_convolutional_create(2, 9, poly);
    if (!conv) {
        fprintf(stderr, "Error: No se pudo crear el codificador convolucional\n");
        return 0;
    }
    //printf("Conv encoder Input lenght: %d\n", len);


    // Calcular el tamaño de salida esperado
    size_t encoded_max_len = correct_convolutional_encode_len(conv, len);

    //Codificar datos
    size_t encoded_len = correct_convolutional_encode(conv, data, len, encoded);
    if (encoded_len == 0) {
        fprintf(stderr, "Error: La codificación falló\n");
        correct_convolutional_destroy(conv);
        return 0;
    }

    //printf("Encode lenght: %d\n", encoded_max_len/8);


    // Liberar memoria
    correct_convolutional_destroy(conv);


    return 1;
}

static int rs(uint8_t *data,size_t len, uint8_t *encoded){
    int parity = 32;
    int msg_len = (len + parity);  
    correct_reed_solomon *rs = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds,1,1,parity);
    if (!rs) {
        fprintf(stderr, "Error: No se pudo crear el codificador Reed-Solomon\n");
        return 0;
    }
    //printf("Conv encoder Input lenght: %d\n", len);
    ssize_t encoded_len = correct_reed_solomon_encode(rs, data, len, encoded);
    if (encoded_len <= 0) {
        fprintf(stderr, "Error: La codificación falló\n");
        correct_reed_solomon_destroy(rs);
        return 0;
    }
    //printf("Encode lenght: %d\n", encoded_len);


    correct_reed_solomon_destroy(rs);
    return 1;


    // Codificar datos
    
}

static int deconv(uint8_t *data,size_t len, uint8_t *decoded){
    const correct_convolutional_polynomial_t poly[] = {0b111, 0b101};

    correct_convolutional *conv = correct_convolutional_create(2, 9, poly);
    if (!conv) {
        fprintf(stderr, "Error: No se pudo crear el codificador convolucional\n");
        return 0;
    }
    uint8_t convde[(len/8-2)/2];
    size_t decoded_len = correct_convolutional_decode(conv, data, len, convde);
    if (decoded_len == 0) {
        fprintf(stderr, "Error: La decodificación falló\n");
        correct_convolutional_destroy(conv);
        return 1;
    }
    // printf("Decoded data: ");
    // printf("%d ",sizeof(convde));
    // for(int i=0; i<sizeof(convde);i++){
    //     printf("%d ",convde[i]);
        
    // }
    //printf("\n");
    memcpy(decoded,convde,sizeof(convde));
    correct_convolutional_destroy(conv);
    return 1;
}

static int ders(uint8_t *data,size_t len, uint8_t *decoded){
    int parity = 32;
    int msg_len = (len + parity);  
    correct_reed_solomon *rs = correct_reed_solomon_create(correct_rs_primitive_polynomial_ccsds,1,1,parity);
    if (!rs) {
        fprintf(stderr, "Error: No se pudo crear el codificador Reed-Solomon\n");
        return 0;
    }
    uint8_t rsde[223];
    ssize_t decoded_len = correct_reed_solomon_decode(rs, data, len, rsde);
    if (decoded_len <= 0) {
        fprintf(stderr, "Error: La decodificación falló\n");
        correct_reed_solomon_destroy(rs);
        return 1;
    }
    memcpy(decoded,rsde,sizeof(rsde));
    correct_reed_solomon_destroy(rs);
    return 1;
}

void block_interleave(uint8_t *input, uint8_t *output, int rows, int cols) {
    for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
            output[col * rows + row] = input[row * cols + col];
        }
    }
}

void block_deinterleave(uint8_t *input, uint8_t *output, int rows, int cols) {
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            output[row * cols + col] = input[col * rows + row];
        }
    }
}