/*
 * Ethernet Driver for ESP32-P4-NANO
 * IP101GRI PHY via internal EMAC (RMII interface)
 * 
 * Hardware Configuration (ESP32-P4-NANO):
 *   MDC  = GPIO31
 *   MDIO = GPIO52
 *   PHY Reset = GPIO51
 *   PHY Address = 1
 *   RMII: TXD0=34, TXD1=35, TX_EN=49, RXD0=29, RXD1=30, CRS_DV=28, REF_CLK=50
 *   Clock: External 25MHz crystal on IP101 → 50MHz RMII clock
 */

#include <string.h>
#include "ethernet_init.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Ethernet";

// ESP32-P4-NANO Ethernet pin definitions
#define ETH_MDC_GPIO     31
#define ETH_MDIO_GPIO    52
#define ETH_PHY_RST_GPIO 51
#define ETH_PHY_ADDR      1

// State
static esp_eth_handle_t s_eth_handle = NULL;
static esp_netif_t *s_eth_netif = NULL;
static esp_eth_netif_glue_handle_t s_eth_glue = NULL;
static bool s_link_up = false;
static bool s_got_ip = false;
static esp_netif_ip_info_t s_eth_ip_info = {0};

/* ====== Event Handlers ====== */

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
    uint8_t mac_addr[6] = {0};

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        s_link_up = true;
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up - MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        break;

    case ETHERNET_EVENT_DISCONNECTED:
        s_link_up = false;
        s_got_ip = false;
        memset(&s_eth_ip_info, 0, sizeof(s_eth_ip_info));
        ESP_LOGW(TAG, "Ethernet Link Down - cable disconnected?");
        break;

    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;

    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        s_link_up = false;
        s_got_ip = false;
        break;

    default:
        break;
    }
}

static void eth_got_ip_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    s_eth_ip_info = event->ip_info;
    s_got_ip = true;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "  IP:      " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "  Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(TAG, "  Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
}

/* ====== Public API ====== */

esp_err_t ethernet_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing Ethernet (IP101 PHY, MDC=%d, MDIO=%d, RST=%d)...",
             ETH_MDC_GPIO, ETH_MDIO_GPIO, ETH_PHY_RST_GPIO);

    // MAC configuration
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    if (!mac) {
        ESP_LOGE(TAG, "Failed to create ESP32 Ethernet MAC");
        return ESP_FAIL;
    }

    // PHY configuration (IP101GRI)
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;

    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
    if (!phy) {
        ESP_LOGE(TAG, "Failed to create IP101 PHY");
        mac->del(mac);
        return ESP_FAIL;
    }

    // Install Ethernet driver
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ret = esp_eth_driver_install(&config, &s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet driver install failed: %s", esp_err_to_name(ret));
        phy->del(phy);
        mac->del(mac);
        return ret;
    }

    // Create default Ethernet netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    if (!s_eth_netif) {
        ESP_LOGE(TAG, "Failed to create Ethernet netif");
        esp_eth_driver_uninstall(s_eth_handle);
        s_eth_handle = NULL;
        return ESP_FAIL;
    }

    // Attach Ethernet driver to TCP/IP stack
    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    ret = esp_netif_attach(s_eth_netif, s_eth_glue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to attach Ethernet to netif: %s", esp_err_to_name(ret));
        esp_netif_destroy(s_eth_netif);
        esp_eth_driver_uninstall(s_eth_handle);
        s_eth_netif = NULL;
        s_eth_handle = NULL;
        return ret;
    }

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &eth_got_ip_handler, NULL));

    // Start Ethernet
    ret = esp_eth_start(s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Ethernet initialized - waiting for link...");
    return ESP_OK;
}

void ethernet_deinit(void)
{
    if (s_eth_handle) {
        esp_eth_stop(s_eth_handle);
        esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler);
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_got_ip_handler);
        
        if (s_eth_glue) {
            esp_eth_del_netif_glue(s_eth_glue);
            s_eth_glue = NULL;
        }
        if (s_eth_netif) {
            esp_netif_destroy(s_eth_netif);
            s_eth_netif = NULL;
        }
        esp_eth_driver_uninstall(s_eth_handle);
        s_eth_handle = NULL;
    }
    s_link_up = false;
    s_got_ip = false;
}

bool ethernet_is_link_up(void)
{
    return s_link_up;
}

bool ethernet_is_connected(void)
{
    return s_got_ip;
}

esp_err_t ethernet_get_ip(char *ip_str, size_t len)
{
    if (!ip_str || len < 16) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_got_ip) {
        return ESP_FAIL;
    }
    snprintf(ip_str, len, IPSTR, IP2STR(&s_eth_ip_info.ip));
    return ESP_OK;
}

esp_netif_t *ethernet_get_netif(void)
{
    return s_eth_netif;
}
