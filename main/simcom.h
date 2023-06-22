#ifndef SIMCOM_H
#define SIMCOM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief simcom uart initialization
 * 
 */
void simcomm_uart_init(void); 

/**
 * @brief Function to Send AT commands to simcom via uart
 * 
 * @param logName Tag Name
 * @param data AT command
 */
void send_cmd_to_simcomm(const char *logName, const char *data);

/**
 * @brief simcom uart receive task
 * 
 */
void sim_rx_task();


/**
 * @brief Function to initialize stack & task Related to simcom
 * 
 */
void simcom_start();

/**
 * @brief Function to release stack & task related to simcom
 * 
 */
void simcom_stop();

/**
 * @brief Function to get Firmware file size
 * @return bin_size Firmware file size
*/
int get_binary_file_size();

#ifdef __cplusplus
}
#endif

#endif // SIMCOM