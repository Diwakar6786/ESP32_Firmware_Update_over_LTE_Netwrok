# Requirement

### Software Tools
ESP-IDF for ESP32

### Hardware Components
- ESP WROOM32
- SIMCOM A7672S
- SIMACARD

# ESP-IDF Development Environment Changes
- Replace the **esp_https_ota.c** & **esp_https_ota.h** components file (your local development environment) with the files given in folder **idf_internal_file**.
- Location of the file (esp_https_ota.c): C:\ **own esp working directory** \components\esp_https_ota\src\esp_https_ota.c.
- Location of the file (esp_https_ota.h) : C:\ **own esp working directory** \components\esp_https_ota\includes\esp_https_ota.h.

# Hardware Interfacing
- GPIO 17 (ESP) : UART TX (A7627S)
- GPIO 16 : UART RX
- GPIO 32 : ENABLE

# Uploaded Firmware File Cloud URL
- replace the Firmware File URL (Where the bin file is stored) in simcom.c file with your cloud URL
- Example : https://www.mosquitto.com/beta/iot/blink.bin


