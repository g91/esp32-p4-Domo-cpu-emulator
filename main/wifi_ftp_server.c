/*
 * WiFi and FTP Server for ESP32-P4
 * Provides WiFi connectivity and FTP access to SD card
 */

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "wifi_ftp_server.h"
#include "ssh_debug_server.h"

static const char *TAG = "WiFi_FTP";

// Default WiFi credentials
#define DEFAULT_SSID      "doc007"
#define DEFAULT_PASSWORD  "16221699"

// FTP Configuration
#define FTP_PORT          21
#define FTP_DATA_PORT     20
#define FTP_ROOT_DIR      "/sdcard"

// WiFi event group
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static bool wifi_connected = false;
static TaskHandle_t ftp_task_handle = NULL;

// FTP server state
typedef struct {
    int control_socket;
    int data_socket;
    struct sockaddr_in data_addr;
    bool is_passive;
    char current_dir[256];
    bool running;
} ftp_session_t;

static ftp_session_t ftp_session = {0};

/* ====== WiFi Event Handler ====== */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to WiFi (%d/10)", s_retry_num);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to WiFi");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        wifi_connected = true;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Auto-start SSH debug server when WiFi connects
        ESP_LOGI(TAG, "Starting SSH debug server...");
        ssh_debug_server_init();
        ESP_LOGI(TAG, "Telnet debug console available at: telnet " IPSTR " 23", 
                 IP2STR(&event->ip_info.ip));
    }
}

/* ====== WiFi Functions ====== */

esp_err_t wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    
    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    
    // Configure and start WiFi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialized, connecting to %s...", DEFAULT_SSID);
    
    // Connect to default network
    return ftp_wifi_connect(DEFAULT_SSID, DEFAULT_PASSWORD);
}

esp_err_t ftp_wifi_connect(const char *ssid, const char *password)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }
    
    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    s_retry_num = 0;
    esp_err_t ret = esp_wifi_connect();
    
    if (ret == ESP_OK) {
        // Wait for connection
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdFALSE,
                pdFALSE,
                pdMS_TO_TICKS(10000));
        
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Connected to WiFi: %s", ssid);
            return ESP_OK;
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGE(TAG, "Failed to connect to WiFi: %s", ssid);
            return ESP_FAIL;
        } else {
            ESP_LOGW(TAG, "WiFi connection timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    
    return ret;
}

esp_err_t wifi_scan(void)
{
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    
    ESP_LOGI(TAG, "Starting WiFi scan...");
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi scan failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    if (ap_count == 0) {
        ESP_LOGI(TAG, "No WiFi networks found");
        return ESP_OK;
    }
    
    wifi_ap_record_t *ap_info = malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (!ap_info) {
        return ESP_ERR_NO_MEM;
    }
    
    esp_wifi_scan_get_ap_records(&ap_count, ap_info);
    
    printf("\n=== Available WiFi Networks ===\n");
    printf("%-32s %-17s %4s %4s\n", "SSID", "BSSID", "RSSI", "CH");
    printf("-------------------------------------------------------------------\n");
    
    for (int i = 0; i < ap_count; i++) {
        printf("%-32s %02x:%02x:%02x:%02x:%02x:%02x %4d %4d %s\n",
               (char *)ap_info[i].ssid,
               ap_info[i].bssid[0], ap_info[i].bssid[1], ap_info[i].bssid[2],
               ap_info[i].bssid[3], ap_info[i].bssid[4], ap_info[i].bssid[5],
               ap_info[i].rssi,
               ap_info[i].primary,
               (ap_info[i].authmode == WIFI_AUTH_OPEN) ? "Open" : "Secured");
    }
    printf("\n");
    
    free(ap_info);
    return ESP_OK;
}

bool wifi_is_connected(void)
{
    // Check actual WiFi connection status instead of internal flag
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        // Also verify we have an IP address
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) {
            esp_netif_ip_info_t ip_info;
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                return (ip_info.ip.addr != 0);  // Return true if we have a valid IP
            }
        }
    }
    return false;
}

esp_err_t wifi_get_ip(char *ip_str, size_t len)
{
    if (!ip_str || len < 16) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        return ESP_FAIL;
    }
    
    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret == ESP_OK) {
        snprintf(ip_str, len, IPSTR, IP2STR(&ip_info.ip));
    }
    
    return ret;
}

/* ====== FTP Server Functions ====== */

static void ftp_send_response(int ctrl_sock, int code, const char *message)
{
    char response[512];
    snprintf(response, sizeof(response), "%d %s\r\n", code, message);
    send(ctrl_sock, response, strlen(response), 0);
}

static void ftp_handle_user(int ctrl_sock, const char *arg)
{
    ftp_send_response(ctrl_sock, 230, "Login successful");
}

static void ftp_handle_syst(int ctrl_sock)
{
    ftp_send_response(ctrl_sock, 215, "UNIX Type: L8");
}

static void ftp_handle_pwd(int ctrl_sock)
{
    char response[512];
    snprintf(response, sizeof(response), "\"%s\" is current directory", ftp_session.current_dir);
    ftp_send_response(ctrl_sock, 257, response);
}

static void ftp_handle_type(int ctrl_sock, const char *arg)
{
    ftp_send_response(ctrl_sock, 200, "Type set");
}

static void ftp_handle_pasv(int ctrl_sock)
{
    // Create data socket for passive mode
    int data_sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (data_sock < 0) {
        ftp_send_response(ctrl_sock, 425, "Can't open data connection");
        return;
    }
    
    struct sockaddr_in data_addr;
    data_addr.sin_family = AF_INET;
    data_addr.sin_addr.s_addr = INADDR_ANY;
    data_addr.sin_port = 0;  // Let system assign port
    
    if (bind(data_sock, (struct sockaddr *)&data_addr, sizeof(data_addr)) < 0) {
        close(data_sock);
        ftp_send_response(ctrl_sock, 425, "Can't open data connection");
        return;
    }
    
    listen(data_sock, 1);
    
    // Get assigned port
    socklen_t addr_len = sizeof(data_addr);
    getsockname(data_sock, (struct sockaddr *)&data_addr, &addr_len);
    int port = ntohs(data_addr.sin_port);
    
    // Get server IP
    char ip_str[16];
    wifi_get_ip(ip_str, sizeof(ip_str));
    
    // Parse IP into octets
    unsigned int ip[4];
    sscanf(ip_str, "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]);
    
    char response[256];
    snprintf(response, sizeof(response), 
             "Entering Passive Mode (%u,%u,%u,%u,%d,%d)",
             ip[0], ip[1], ip[2], ip[3], port / 256, port % 256);
    
    ftp_session.data_socket = data_sock;
    ftp_session.is_passive = true;
    
    ftp_send_response(ctrl_sock, 227, response);
}

static void ftp_handle_list(int ctrl_sock)
{
    ftp_send_response(ctrl_sock, 150, "Opening data connection for directory listing");
    
    // Accept data connection
    int data_conn;
    if (ftp_session.is_passive) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        data_conn = accept(ftp_session.data_socket, (struct sockaddr *)&client_addr, &addr_len);
    } else {
        data_conn = ftp_session.data_socket;
    }
    
    if (data_conn < 0) {
        ftp_send_response(ctrl_sock, 425, "Can't open data connection");
        return;
    }
    
    // List directory
    DIR *dir = opendir(ftp_session.current_dir);
    if (dir) {
        struct dirent *entry;
        char line[512];
        
        while ((entry = readdir(dir)) != NULL) {
            // Simple Unix-style listing
            struct stat st;
            char full_path[512];
            snprintf(full_path, sizeof(full_path), "%s/%s", ftp_session.current_dir, entry->d_name);
            
            if (stat(full_path, &st) == 0) {
                char type = S_ISDIR(st.st_mode) ? 'd' : '-';
                snprintf(line, sizeof(line), "%crw-rw-rw-   1 user  user  %8ld Jan  1 00:00 %s\r\n",
                         type, (long)st.st_size, entry->d_name);
                send(data_conn, line, strlen(line), 0);
            }
        }
        closedir(dir);
    }
    
    close(data_conn);
    if (ftp_session.is_passive) {
        close(ftp_session.data_socket);
    }
    
    ftp_send_response(ctrl_sock, 226, "Directory send OK");
}

static void ftp_handle_cwd(int ctrl_sock, const char *arg)
{
    char new_path[512];
    
    if (arg[0] == '/') {
        snprintf(new_path, sizeof(new_path), "%s%s", FTP_ROOT_DIR, arg);
    } else {
        snprintf(new_path, sizeof(new_path), "%s/%s", ftp_session.current_dir, arg);
    }
    
    DIR *dir = opendir(new_path);
    if (dir) {
        closedir(dir);
        strlcpy(ftp_session.current_dir, new_path, sizeof(ftp_session.current_dir));
        ftp_send_response(ctrl_sock, 250, "Directory changed");
    } else {
        ftp_send_response(ctrl_sock, 550, "Failed to change directory");
    }
}

static void ftp_handle_retr(int ctrl_sock, const char *filename)
{
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "%s/%s", ftp_session.current_dir, filename);
    
    FILE *file = fopen(filepath, "rb");
    if (!file) {
        ftp_send_response(ctrl_sock, 550, "File not found");
        return;
    }
    
    ftp_send_response(ctrl_sock, 150, "Opening data connection for file transfer");
    
    // Accept data connection
    int data_conn;
    if (ftp_session.is_passive) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        data_conn = accept(ftp_session.data_socket, (struct sockaddr *)&client_addr, &addr_len);
    } else {
        data_conn = ftp_session.data_socket;
    }
    
    if (data_conn < 0) {
        fclose(file);
        ftp_send_response(ctrl_sock, 425, "Can't open data connection");
        return;
    }
    
    // Send file
    char buffer[1024];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        send(data_conn, buffer, bytes, 0);
    }
    
    fclose(file);
    close(data_conn);
    if (ftp_session.is_passive) {
        close(ftp_session.data_socket);
    }
    
    ftp_send_response(ctrl_sock, 226, "Transfer complete");
}

static void ftp_handle_dele(int ctrl_sock, const char *filename)
{
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "%s/%s", ftp_session.current_dir, filename);
    
    if (unlink(filepath) == 0) {
        ftp_send_response(ctrl_sock, 250, "File deleted");
    } else {
        ftp_send_response(ctrl_sock, 550, "Delete failed");
    }
}

static void ftp_handle_mkd(int ctrl_sock, const char *dirname)
{
    char dirpath[512];
    if (dirname[0] == '/') {
        snprintf(dirpath, sizeof(dirpath), "%s%s", FTP_ROOT_DIR, dirname);
    } else {
        snprintf(dirpath, sizeof(dirpath), "%s/%s", ftp_session.current_dir, dirname);
    }
    
    if (mkdir(dirpath, 0777) == 0) {
        char response[256];
        snprintf(response, sizeof(response), "\"%s\" created", dirname);
        ftp_send_response(ctrl_sock, 257, response);
    } else {
        ftp_send_response(ctrl_sock, 550, "Create directory failed");
    }
}

static void ftp_handle_rmd(int ctrl_sock, const char *dirname)
{
    char dirpath[512];
    if (dirname[0] == '/') {
        snprintf(dirpath, sizeof(dirpath), "%s%s", FTP_ROOT_DIR, dirname);
    } else {
        snprintf(dirpath, sizeof(dirpath), "%s/%s", ftp_session.current_dir, dirname);
    }
    
    if (rmdir(dirpath) == 0) {
        ftp_send_response(ctrl_sock, 250, "Directory removed");
    } else {
        ftp_send_response(ctrl_sock, 550, "Remove directory failed");
    }
}

static void ftp_handle_size(int ctrl_sock, const char *filename)
{
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "%s/%s", ftp_session.current_dir, filename);
    
    struct stat st;
    if (stat(filepath, &st) == 0) {
        char response[64];
        snprintf(response, sizeof(response), "%ld", (long)st.st_size);
        ftp_send_response(ctrl_sock, 213, response);
    } else {
        ftp_send_response(ctrl_sock, 550, "File not found");
    }
}

static void ftp_handle_stor(int ctrl_sock, const char *filename)
{
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "%s/%s", ftp_session.current_dir, filename);
    
    ESP_LOGI(TAG, "FTP STOR: Creating file %s", filepath);
    
    FILE *file = fopen(filepath, "wb");
    if (!file) {
        ESP_LOGE(TAG, "FTP STOR: Failed to create file %s", filepath);
        ftp_send_response(ctrl_sock, 550, "Can't create file");
        return;
    }
    
    ftp_send_response(ctrl_sock, 150, "Opening data connection for file upload");
    
    // Accept data connection
    int data_conn;
    if (ftp_session.is_passive) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        data_conn = accept(ftp_session.data_socket, (struct sockaddr *)&client_addr, &addr_len);
    } else {
        data_conn = ftp_session.data_socket;
    }
    
    if (data_conn < 0) {
        fclose(file);
        ftp_send_response(ctrl_sock, 425, "Can't open data connection");
        return;
    }
    
    // Receive file
    char buffer[1024];
    int bytes;
    size_t total_bytes = 0;
    while ((bytes = recv(data_conn, buffer, sizeof(buffer), 0)) > 0) {
        size_t written = fwrite(buffer, 1, bytes, file);
        total_bytes += written;
        if (written != bytes) {
            ESP_LOGE(TAG, "FTP STOR: Write error (wrote %d of %d bytes)", written, bytes);
            break;
        }
    }
    
    fflush(file);  // Ensure data is written to SD card
    fclose(file);
    close(data_conn);
    if (ftp_session.is_passive) {
        close(ftp_session.data_socket);
    }
    
    ESP_LOGI(TAG, "FTP STOR: Upload complete (%zu bytes)", total_bytes);
    ftp_send_response(ctrl_sock, 226, "Upload complete");
}

static void ftp_server_task(void *pvParameters)
{
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    // Create control socket
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create FTP socket");
        vTaskDelete(NULL);
        return;
    }
    
    int reuse = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(FTP_PORT);
    
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "FTP bind failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    
    listen(listen_sock, 1);
    
    char ip_str[16];
    wifi_get_ip(ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "FTP server started on %s:%d", ip_str, FTP_PORT);
    
    ftp_session.running = true;
    
    while (ftp_session.running) {
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            continue;
        }
        
        ESP_LOGI(TAG, "FTP client connected");
        
        // Initialize session
        strcpy(ftp_session.current_dir, FTP_ROOT_DIR);
        ftp_session.control_socket = client_sock;
        ftp_session.is_passive = false;
        
        // Send welcome message
        ftp_send_response(client_sock, 220, "ESP32-P4 FTP Server Ready");
        
        // Handle commands
        char buffer[512];
        while (ftp_session.running) {
            int len = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
            if (len <= 0) {
                break;
            }
            
            buffer[len] = '\0';
            
            // Remove \r\n
            char *p = strchr(buffer, '\r');
            if (p) *p = '\0';
            p = strchr(buffer, '\n');
            if (p) *p = '\0';
            
            // Parse command
            char *cmd = strtok(buffer, " ");
            char *arg = strtok(NULL, "");
            
            if (!cmd) continue;
            
            ESP_LOGI(TAG, "FTP Command: %s %s", cmd, arg ? arg : "");
            
            if (strcasecmp(cmd, "USER") == 0) {
                ftp_handle_user(client_sock, arg);
            } else if (strcasecmp(cmd, "PASS") == 0) {
                ftp_send_response(client_sock, 230, "Login successful");
            } else if (strcasecmp(cmd, "SYST") == 0) {
                ftp_handle_syst(client_sock);
            } else if (strcasecmp(cmd, "PWD") == 0) {
                ftp_handle_pwd(client_sock);
            } else if (strcasecmp(cmd, "TYPE") == 0) {
                ftp_handle_type(client_sock, arg);
            } else if (strcasecmp(cmd, "PASV") == 0) {
                ftp_handle_pasv(client_sock);
            } else if (strcasecmp(cmd, "LIST") == 0) {
                ftp_handle_list(client_sock);
            } else if (strcasecmp(cmd, "CWD") == 0) {
                ftp_handle_cwd(client_sock, arg);
            } else if (strcasecmp(cmd, "RETR") == 0) {
                ftp_handle_retr(client_sock, arg);
            } else if (strcasecmp(cmd, "STOR") == 0) {
                ftp_handle_stor(client_sock, arg);
            } else if (strcasecmp(cmd, "DELE") == 0) {
                ftp_handle_dele(client_sock, arg);
            } else if (strcasecmp(cmd, "MKD") == 0) {
                ftp_handle_mkd(client_sock, arg);
            } else if (strcasecmp(cmd, "RMD") == 0) {
                ftp_handle_rmd(client_sock, arg);
            } else if (strcasecmp(cmd, "SIZE") == 0) {
                ftp_handle_size(client_sock, arg);
            } else if (strcasecmp(cmd, "QUIT") == 0) {
                ftp_send_response(client_sock, 221, "Goodbye");
                break;
            } else if (strcasecmp(cmd, "NOOP") == 0) {
                ftp_send_response(client_sock, 200, "OK");
            } else {
                ftp_send_response(client_sock, 502, "Command not implemented");
            }
        }
        
        close(client_sock);
        ESP_LOGI(TAG, "FTP client disconnected");
    }
    
    close(listen_sock);
    ftp_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t ftp_server_init(void)
{
    if (ftp_task_handle != NULL) {
        ESP_LOGW(TAG, "FTP server already running");
        return ESP_OK;
    }
    
    if (!wifi_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected, cannot start FTP server");
        return ESP_FAIL;
    }
    
    // Check if SD card mount point exists
    DIR *dir = opendir(FTP_ROOT_DIR);
    if (!dir) {
        ESP_LOGE(TAG, "SD card not mounted at %s, cannot start FTP server", FTP_ROOT_DIR);
        ESP_LOGE(TAG, "Please mount SD card first using 'sd_mount' command");
        return ESP_FAIL;
    }
    closedir(dir);
    
    BaseType_t ret = xTaskCreate(ftp_server_task, "ftp_server", 8192, NULL, 5, &ftp_task_handle);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create FTP server task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

void ftp_server_stop(void)
{
    if (ftp_task_handle) {
        ftp_session.running = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool ftp_server_is_running(void)
{
    return (ftp_task_handle != NULL);
}
