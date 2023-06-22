#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "string.h"
#include "simcom.h"

#include "nvs.h"
#include "nvs_flash.h"
#include <sys/socket.h>

#define TAG "ota_over_4G"
#define HASH_LEN 32

void ota_init(void *pvParameter)
{
    ESP_LOGI(TAG, "-----xx Initializing OTA xx-----");

    esp_https_ota_handle_t https_ota_handle = NULL;

    //++ get the size of the binary file
    uint16_t binary_length = get_binary_file_size();

    esp_err_t ret = ota_through_simcom(&https_ota_handle, binary_length);

    if (ret == ESP_OK)
    {
        while (1)
        {
            esp_err_t err = esp_simcom_ota_perform(https_ota_handle);

            if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
            {
                break;
            }
        }

        esp_err_t ota_finish_err = esp_https_ota_finish(https_ota_handle);

        if (ota_finish_err == ESP_OK)
        {
            ESP_LOGI(TAG, "[ activity : Firmware Update ] [ status : Updated ]");
        }
        else
        {
            ESP_LOGI(TAG, "[ activity : Firmware Update ] [ status : Update Fail ]");
        }
    }
    else
    {
        ESP_LOGE(TAG, "[ activity : Firmware Update ] [ status : Updated Fail ]");
    }
    
    ESP_LOGI(TAG, "[ activity : Rsetarting the ESP ]");
    esp_restart();
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void)
{
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
}

void app_main(void)
{
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    get_sha256_of_partitions();
    
    //++ start simcom 
    simcom_start();

    xTaskCreate(&ota_init, "ota_task", 8192, NULL, 5, NULL);
}

