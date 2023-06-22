#include <stdio.h>
#include "string.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "math.h"
#include "time.h"

#include "simcom.h"

#define TAG "4G"

static const int RX_BUF_SIZE = 12000;

//++ UART TX and RX Pins for Communication between simcom & esp
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

//++ to enable the simcom module
#define EN_GPIO 32

//++ to store the time
struct timespec start;

//++ Task Handler of the simcom uart receive
TaskHandle_t xHandle_rx_task;

//++ character array to store firmware data in parts
char ota_bin_chuck_data[11560];

int bin_chuck_data_length = 0;
int bin_chuck_data_length1 = 0;

//++ AT command for initiaze simcom setup
char AT_COMMANDS[3][30] = {{"AT+CPIN?\r\n"},
                           {"AT+CPSI?\r\n"}};

//++ AT command to establish http connection & get binary file size
char AT_HTTP_ESTABLISH_COMMAND[4][250] = {{"AT+HTTPINIT\r\n"},
                                          {"AT+HTTPPARA=\"URL\",\"https://www.alitersolutions.com/beta/iot/blink.bin\"\r\n"},
                                          {"AT+HTTPACTION=0\r\n"},
                                          {"AT+HTTPREAD=0,1156\r\n"}};

//++ Variable to store the AT command retry count
uint8_t command_retry_count = 0;

//++ variabel to store Firmware file size
int bin_size = 0;
int total_chuck_downloaded = 0;

//++ Flags
bool ok_received = false;
bool http_action = false;
bool next_command = true;
bool http_data_read = false;
bool Is_sim_available = false;
bool Is_LTE_got_network = false;

void simcomm_uart_init(void)
{
    const uart_config_t uart_config =
        {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void send_cmd_to_simcomm(const char *tag, const char *data)
{
    ESP_LOGW(tag, "Sending Command %s", data);
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGW(tag, "--[ wrote %d bytes]--", txBytes);
}

//++ Function to extract the useful data from the response
void simcomm_response_parser(char *data, int data_length)
{
    int http_data_index = 0;
    bin_chuck_data_length = 0;

    for (int i = 0; i <= data_length; i++)
    {
        if (data[i] == 'O' && data[i + 1] == 'K') //++ AT Response for OK
        {
            ESP_LOGI(TAG, "AT res success");
            ok_received = true;
        }

        else if (data[i] == '+' && data[i + 1] == 'C' && data[i + 2] == 'P' && data[i + 3] == 'I' && data[i + 4] == 'N' && data[i + 5] == ':') //++ AT Response to check sim status
        {
            if (data[i + 7] == 'R' && data[i + 8] == 'E' && data[i + 9] == 'A' && data[i + 10] == 'D' && data[i + 11] == 'Y')
            {
                ESP_LOGW(TAG, "\n[ activity : SIM Availability Checkup ] [ Status : Found ]\n");
                Is_sim_available = true;
            }
            else if (data[i + 7] == 'S' && data[i + 8] == 'I' && data[i + 9] == 'M' && data[i + 11] == 'R' && data[i + 12] == 'E' && data[i + 13] == 'M' && data[i + 14] == 'O' && data[i + 15] == 'V' && data[i + 16] == 'E' && data[i + 17] == 'D')
            {
                ESP_LOGE(TAG, "[ Activity : SIM Availability Checkup ] [ status : Please Put simcard ]");
            }
        }

        else if (data[i] == '+' && data[i + 1] == 'C' && data[i + 2] == 'P' && data[i + 3] == 'S' && data[i + 4] == 'I' && data[i + 5] == ':') //++ AT Response to check UE System Information
        {
            int index = 0;
            int net_mode = i + 7;
            char network_mode_4g[12];
            memset(network_mode_4g, 0, sizeof(network_mode_4g));

            do
            {
                network_mode_4g[index] = data[net_mode];
                index++;
                net_mode++;
            } while (data[net_mode] != ',');

            ESP_LOGW(TAG, "[ Network Mode : %s ]", network_mode_4g);

            if (strcmp(network_mode_4g, "LTE") == 0)
            {
                Is_LTE_got_network = true;
            }
            index = 0;
            int status = net_mode + 1;

            char sim_network_status[6];
            memset(sim_network_status, 0, sizeof(sim_network_status));

            if (network_mode_4g[0] == 'N' && network_mode_4g[1] == 'O')
            {
                do
                {
                    sim_network_status[index] = data[status];
                    index++;
                    status++;
                } while (data[status] != '\r');

                sim_network_status[status] = 0;
                ESP_LOGW(TAG, "[ Network Status : %s ]", sim_network_status);
                Is_LTE_got_network = false;
            }
            else
            {
                do
                {
                    sim_network_status[index] = data[status];
                    index++;
                    status++;
                } while (data[status] != ',');

                ESP_LOGW(TAG, "[ Network Status : %s ]", sim_network_status);
                Is_LTE_got_network = true;
            }
        }

        // +HTTPACTION: 0,200,binary_length
        else if (data[i] == '+' && data[i + 1] == 'H' && data[i + 2] == 'T' && data[i + 3] == 'T' && data[i + 4] == 'P' && data[i + 5] == 'A' && data[i + 6] == 'C' && data[i + 7] == 'T' && data[i + 8] == 'I' && data[i + 9] == 'O' && data[i + 10] == 'N' && data[i + 11] == ':')
        {
            if (data[i + 15] == '2' && data[i + 16] == '0' && data[i + 17] == '0')
            {
                ESP_LOGI(TAG, "valid http response: %d", i);

                char length_string[10];
                memset(length_string, 0, sizeof(length_string));

                int start_index = i + 19, index = 0;

                while (data[start_index] != '\0')
                {
                    length_string[index] = data[start_index];
                    start_index++;
                    index++;
                }
                // 172144
                bin_size = atoi(length_string);
                ESP_LOGI(TAG, "ota bin size: %d", bin_size);

                http_action = true;
            }
        }

        // +HTTPREAD:
        if (data[i] == '+' && data[i + 1] == 'H' && data[i + 2] == 'T' && data[i + 3] == 'T' && data[i + 4] == 'P' && data[i + 5] == 'R' && data[i + 6] == 'E' && data[i + 7] == 'A' && data[i + 8] == 'D' && data[i + 9] == ':')
        {
            // ESP_LOGI(TAG, "http read index: %d", i);

            char length_string[4];
            memset(length_string, 0, sizeof(length_string));

            int start_index = i + 11, index1 = 0;

            while (data[start_index] != '\n')
            {
                length_string[index1] = data[start_index];
                start_index++;
                index1++;
            }

            int http_data_start_index = start_index + 1;

            if (atoi(length_string) == 0)
            {
                total_chuck_downloaded = total_chuck_downloaded + 1;
                ESP_LOGI(TAG, "bin_chuck_data_length: %d, total_chuck_downloaded: %d", bin_chuck_data_length, total_chuck_downloaded);
                http_data_read = true;
                break;
            }

            //++ http read length
            sscanf(length_string, "%d", &bin_chuck_data_length1);
            bin_chuck_data_length = bin_chuck_data_length + bin_chuck_data_length1;

            int range = http_data_start_index + bin_chuck_data_length1 - 1;

            // ESP_LOGI(TAG, "http data index: %d", http_data_index);

            for (int i = http_data_start_index; i <= range; i++)
            {
                ota_bin_chuck_data[http_data_index] = data[i];
                http_data_index++;
            }

            // ESP_LOGI(TAG, "ota_bin_chuck_data[0]: %x, ota_bin_chuck_data[1024]: %x", ota_bin_chuck_data[0], ota_bin_chuck_data[1024]);
        }
    }
}

void sim_rx_task()
{
    static const char *RX_TASK_TAG = "SIM_RX_TASK";
    // esp_log_level_set(RX_TASK_TAG, ESP_LOG_WARN);

    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 500 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGW(RX_TASK_TAG, "--[ received %d bytes: %s ]--", rxBytes, data);

            //++ parser for AT reponse
            simcomm_response_parser((char *)data, rxBytes);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    free(data);
}

void simcom_start()
{
    ESP_LOGI(TAG, "\n\nxx -- INITIALISING 4G -- xx\n");

    gpio_set_direction(EN_GPIO, GPIO_MODE_OUTPUT);

    simcomm_uart_init(); //++ Call UART Initializing Functio

    xTaskCreatePinnedToCore(sim_rx_task, "uart_rx_task", 2048 * 6, NULL, 2, &xHandle_rx_task, 0);
}

void simcom_stop()
{
    if (xHandle_rx_task != NULL)
    {
        vTaskDelete(xHandle_rx_task);
        xHandle_rx_task = NULL;
    }
}

//++ Function to check response of the executed AT command
void response_checker_for_intial_setup(long time, int index, char command[])
{
    command_retry_count = 0;
start:
    clock_gettime(CLOCK_REALTIME, &start);
    time = start.tv_sec;
    long current_timestamp = 0;
    Is_sim_available = false;
    Is_LTE_got_network = false;

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &start);
        current_timestamp = start.tv_sec;

        int AT_COMMAND_INDEX = index;

        if (AT_COMMAND_INDEX == 0)
        {
            if (Is_sim_available == true)
            {
                Is_sim_available = false;
                break;
            }
        }

        if (AT_COMMAND_INDEX == 1)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            if (Is_LTE_got_network == true)
            {
                Is_LTE_got_network = false;
                break;
            }
            else
            {
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        }

        if (command_retry_count > 2) //++ after 3 retry restart simcom
        {
            ESP_LOGW(TAG, "try count : %d", command_retry_count);
            next_command = false;
            break;
        }

        if (current_timestamp - time > 20)
        {
            send_cmd_to_simcomm(TAG, command);
            command_retry_count++;
            goto start;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

//++ Function to check response of the executed AT command
void response_checker_http_establish_commands(long time, int index, char command[])
{
    command_retry_count = 0;
start:
    clock_gettime(CLOCK_REALTIME, &start);
    time = start.tv_sec;
    long current_timestamp = 0;
    ok_received = false;
    http_action = false;

    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &start);
        current_timestamp = start.tv_sec;

        int AT_HTTP_COMMAND_INDEX = index;

        if (AT_HTTP_COMMAND_INDEX == 0 || AT_HTTP_COMMAND_INDEX == 1)
        {
            if (ok_received == true)
            {
                ok_received = false;
                break;
            }
        }

        if (AT_HTTP_COMMAND_INDEX == 2)
        {
            if (http_action == true)
            {
                http_action = false;
                break;
            }
        }

        if (command_retry_count > 2) //++ after 3 retry restart simcom
        {
            ESP_LOGW(TAG, "try count : %d", command_retry_count);
            next_command = false;
            break;
        }

        if (current_timestamp - time > 20)
        {
            send_cmd_to_simcomm(TAG, command);
            command_retry_count++;
            goto start;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

int get_binary_file_size()
{
    gpio_set_level(EN_GPIO, 1); //++ Restarting Simcomm via ENABLE pin
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(EN_GPIO, 0); //++ Restarting Simcomm via ENABLE pin

    for (int command_num = 0; command_num < 2; command_num++)
    {
        send_cmd_to_simcomm("TX_TASK_TAG", AT_COMMANDS[command_num]);

        clock_gettime(CLOCK_REALTIME, &start);

        //++ wait for AT response
        response_checker_for_intial_setup(start.tv_sec, command_num, AT_COMMANDS[command_num]);
    }

    for (int command_num = 0; command_num < 3; command_num++)
    {
        send_cmd_to_simcomm("TX_TASK_TAG", AT_HTTP_ESTABLISH_COMMAND[command_num]);

        clock_gettime(CLOCK_REALTIME, &start);

        //++ wait for AT response
        response_checker_http_establish_commands(start.tv_sec, command_num, AT_HTTP_ESTABLISH_COMMAND[command_num]);
    }

    return bin_size;
}