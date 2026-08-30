#if !defined(WIFI_STA_H)
#define WIFI_STA_H

/**
 * @brief Initializes Wi-Fi in STA mode and connects to the configured AP.
 *        This function blocks until initialization is complete, but the 
 *        actual connection handling runs asynchronously in system tasks.
 */
void wifi_manager_init(void);


#endif // WIFI_STA_H
