#include "wifi_station.h"
#include <cstring>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs.h>
#include "nvs_flash.h"
#include <esp_netif.h>
#include <esp_system.h>
#include "ssid_manager.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/inet.h>

// 前向声明全局函数
static bool example_is_our_netif(const char *prefix, esp_netif_t *netif);

// 添加IPv6地址类型的字符串数组
static const char *example_ipv6_addr_types_to_str[] = {
    "ESP_IP6_ADDR_IS_UNKNOWN",
    "ESP_IP6_ADDR_IS_GLOBAL",
    "ESP_IP6_ADDR_IS_LINK_LOCAL",
    "ESP_IP6_ADDR_IS_SITE_LOCAL",
    "ESP_IP6_ADDR_IS_UNIQUE_LOCAL",
    "ESP_IP6_ADDR_IS_IPV4_MAPPED_IPV6"
};

#define TAG "wifi"
#define WIFI_EVENT_CONNECTED BIT0
#define MAX_RECONNECT_COUNT 5

static int s_retry_num = 0;
static volatile bool isconnected = false;
static esp_netif_t *wifi_sta_netif = NULL;
// 新增一个队列来处理扫描结果，避免在事件处理函数中直接处理
static QueueHandle_t s_scan_event_queue = NULL;
static SemaphoreHandle_t s_semph_get_ip_addrs = NULL;
#if CONFIG_LWIP_IPV6
static SemaphoreHandle_t s_semph_get_ip6_addrs = NULL;
#endif

WifiStation& WifiStation::GetInstance() {
    static WifiStation instance;
    return instance;
}

WifiStation::WifiStation() {
    // Create the event group
    event_group_ = xEventGroupCreate();
    
    // 创建扫描结果处理队列
    if (s_scan_event_queue == NULL) {
        s_scan_event_queue = xQueueCreate(2, sizeof(uint32_t)); // 队列长度2，只需要标记事件发生即可
    }
    
    // 创建处理任务
    xTaskCreate([](void* arg) {
        WifiStation* station = static_cast<WifiStation*>(arg);
        uint32_t event;
        while (true) {
            if (xQueueReceive(s_scan_event_queue, &event, portMAX_DELAY)) {
                if (event == WIFI_EVENT_SCAN_DONE) {
                    station->ProcessScanResult();
                }
            }
        }
    }, "wifi_process", 4096, this, 5, NULL); // 增加栈大小到4K
}

WifiStation::~WifiStation() {
    vEventGroupDelete(event_group_);
}

void WifiStation::AddAuth(const std::string &&ssid, const std::string &&password) {
    auto& ssid_manager = SsidManager::GetInstance();
    ssid_manager.AddSsid(ssid, password);
}

void WifiStation::Stop() {
    ESP_LOGI(TAG, "Stopping WiFi");
    
    // 1. 停止定时器
    if (timer_handle_ != nullptr) {
        esp_timer_stop(timer_handle_);
        esp_timer_delete(timer_handle_);
        timer_handle_ = nullptr;
    }

    // 2. 停止WiFi
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(err));
    }
    
    // 3. 注销事件处理程序
    UnregisterEventHandlers();
    
    // 4. 反初始化WiFi
    err = esp_wifi_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinit WiFi: %s", esp_err_to_name(err));
    }
    
    ESP_LOGI(TAG, "WiFi stopped");
}

void WifiStation::OnScanBegin(std::function<void()> on_scan_begin) {
    on_scan_begin_ = on_scan_begin;
}

void WifiStation::OnConnect(std::function<void(const std::string& ssid)> on_connect) {
    on_connect_ = on_connect;
}

void WifiStation::OnConnected(std::function<void(const std::string& ssid)> on_connected) {
    on_connected_ = on_connected;
}

void WifiStation::RegisterEventHandlers() {
    // 先注销之前可能存在的事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, &WifiEventHandler, this));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &WifiEventHandler, this));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect, this));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip, this));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect, wifi_sta_netif));
#if CONFIG_LWIP_IPV6
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &example_handler_on_sta_got_ipv6, this));
#endif
}

void WifiStation::UnregisterEventHandlers() {
    // 注销特定事件处理程序，以避免与通用处理程序冲突
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, &WifiEventHandler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &WifiEventHandler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect));
#if CONFIG_LWIP_IPV6
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_GOT_IP6, &example_handler_on_sta_got_ipv6));
#endif
}

// 启动WiFi，最重要的调用入口
void WifiStation::Start() {
    ESP_LOGI(TAG, "WiFi STA initialization starting");
#ifdef CONFIG_LWIP_IPV6
    ESP_LOGI(TAG, "IPv6支持已启用 (CONFIG_LWIP_IPV6)");
#else
    ESP_LOGW(TAG, "IPv6支持未启用 (无CONFIG_LWIP_IPV6)");
    return;
#endif
    // 初始化WiFi栈之前，先创建定时器，以便在出错时重试
    if (timer_handle_ == nullptr) {
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                WifiStation* station = static_cast<WifiStation*>(arg);
                ESP_LOGI(TAG, "Timer triggered - attempting WiFi initialization");
                station->Start();  // 重试初始化
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "WiFiRetryTimer",
            .skip_unhandled_events = true
        };
        esp_err_t err = esp_timer_create(&timer_args, &timer_handle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
            return;
        }
    }
    
    // 1. 确保esp_netif已初始化
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {  // 如果已初始化，会返回ESP_ERR_INVALID_STATE
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        // 5秒后重试
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000);
        return;
    }
    
    // 2. 创建默认事件循环
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {  // 如果已存在，会返回ESP_ERR_INVALID_STATE
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        // 5秒后重试
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000);
        return;
    }
    
    // 3. 创建默认WiFi站点接口
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
    wifi_sta_netif = esp_netif_new(&netif_config);
    if (wifi_sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi STA netif");
        // 5秒后重试
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000);
        return;
    }
    
    // 设置IPv6配置 - 启用IPv6
    // 在ESP-IDF中，只需创建链路本地地址，SLAAC将自动启用
    ESP_LOGI(TAG, "IPv6 will be enabled when WiFi is connected");
    
    // 将接口连接到WiFi驱动
    err = esp_netif_attach_wifi_station(wifi_sta_netif);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to attach WiFi STA netif: %s", esp_err_to_name(err));
        esp_netif_destroy(wifi_sta_netif);
        // 5秒后重试
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000);
        return;
    }
    
    // 创建WiFi驱动程序的默认事件处理程序
    err = esp_wifi_set_default_wifi_sta_handlers();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set default WiFi STA handlers: %s", esp_err_to_name(err));
        esp_netif_destroy(wifi_sta_netif);
        // 5秒后重试
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000);
        return;
    }
    
    // 4. 注册事件处理程序
    RegisterEventHandlers();
       
    // 5. 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    
    // 6. 设置WiFi模式
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    
    // 7. 启动WiFi
    err = esp_wifi_start();
    
    // 8. 设置扫描定时器
    if (timer_handle_ != nullptr) {
        esp_timer_create_args_t scan_timer_args = {
            .callback = [](void* arg) {
                WifiStation* station = static_cast<WifiStation*>(arg);
                ESP_LOGD(TAG, "Timer triggered WiFi scan");
                esp_err_t err = esp_wifi_scan_start(nullptr, false);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start WiFi scan: %s", esp_err_to_name(err));
                    if (station->timer_handle_ != nullptr) {
                        esp_timer_start_once(station->timer_handle_, 5 * 1000 * 1000);
                    }
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "WiFiScanTimer",
            .skip_unhandled_events = true
        };
        
        // 删除之前创建的定时器(如果有)并创建新的扫描定时器
        if (timer_handle_ != nullptr) {
            esp_timer_stop(timer_handle_);
            esp_timer_delete(timer_handle_);
        }
        
        err = esp_timer_create(&scan_timer_args, &timer_handle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create scan timer: %s", esp_err_to_name(err));
            // 即使没有定时器，WiFi可能仍能正常工作
        }
    }
    
    ESP_LOGI(TAG, "WiFi initialization completed successfully");
    esp_netif_action_connected((esp_netif_t*)wifi_sta_netif, NULL, 0, NULL);
    // 稍作等待
    vTaskDelay(pdMS_TO_TICKS(3000));
    return;
    // 在连接成功后调用，打印实际的接口名称
    // esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    // ESP_LOGI(TAG, "实际接口名称: %s", esp_netif_get_desc(netif));
}

bool WifiStation::WaitForConnected(int timeout_ms) {
    int elapsed = 0;
    int check_interval = 100; // 每100ms检查一次
    while (elapsed < timeout_ms) {
        if (isconnected) {
            // ESP_LOGI(TAG, "WiFi连接成功,返回主板逻辑");
            return true;
        }
        vTaskDelay(check_interval / portTICK_PERIOD_MS);
        elapsed += check_interval;
    }
    return false;
}

// 处理扫描结果
void WifiStation::HandleScanResult() {
    ESP_LOGI(TAG, "HandleScanResult");
    if (IsConnected()) {
        // 如果定时器仍在运行，停止它
        if (timer_handle_ != nullptr) {
            esp_timer_stop(timer_handle_);
        }
        return;
    }
    uint16_t ap_num = 0;
    esp_err_t err = esp_wifi_scan_get_ap_num(&ap_num);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP number: %s", esp_err_to_name(err));
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000); // 5秒后重试
        return;
    }
    
    if (ap_num == 0) {
        ESP_LOGI(TAG, "No AP found, wait for next scan");
        esp_timer_start_once(timer_handle_, 10 * 1000 * 1000);
        return;
    }
    
    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)malloc(ap_num * sizeof(wifi_ap_record_t));
    if (ap_records == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for AP records");
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000); // 5秒后重试
        return;
    }
    
    err = esp_wifi_scan_get_ap_records(&ap_num, ap_records);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP records: %s", esp_err_to_name(err));
        free(ap_records);
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000); // 5秒后重试
        return;
    }

    try {
        // sort by rssi descending
        std::sort(ap_records, ap_records + ap_num, [](const wifi_ap_record_t& a, const wifi_ap_record_t& b) {
            return a.rssi > b.rssi;
        });

        auto& ssid_manager = SsidManager::GetInstance();
        auto ssid_list = ssid_manager.GetSsidList();
        for (int i = 0; i < ap_num; i++) {
            auto ap_record = ap_records[i];
            auto it = std::find_if(ssid_list.begin(), ssid_list.end(), [ap_record](const SsidItem& item) {
                return strcmp((char *)ap_record.ssid, item.ssid.c_str()) == 0;
            });
            if (it != ssid_list.end()) {
                ESP_LOGI(TAG, "Found AP: %s, BSSID: %02x:%02x:%02x:%02x:%02x:%02x, RSSI: %d, Channel: %d, Authmode: %d",
                    (char *)ap_record.ssid, 
                    ap_record.bssid[0], ap_record.bssid[1], ap_record.bssid[2],
                    ap_record.bssid[3], ap_record.bssid[4], ap_record.bssid[5],
                    ap_record.rssi, ap_record.primary, ap_record.authmode);
                WifiApRecord record = {
                    .ssid = it->ssid,
                    .password = it->password,
                    .channel = ap_record.primary,
                    .authmode = ap_record.authmode
                };
                memcpy(record.bssid, ap_record.bssid, 6);
                connect_queue_.push_back(record);
            }
        }
    } catch (std::exception& e) {
        ESP_LOGE(TAG, "Exception during scan processing: %s", e.what());
        free(ap_records);
        esp_timer_start_once(timer_handle_, 5 * 1000 * 1000); // 5秒后重试
        return;
    }
    
    free(ap_records);

    if (connect_queue_.empty()) {
        ESP_LOGI(TAG, "No matching APs found, wait for next scan");
        esp_timer_start_once(timer_handle_, 10 * 1000 * 1000);
        return;
    }

    StartConnect();
}

// 开始连接热点
void WifiStation::StartConnect() {
    ESP_LOGI(TAG, "StartConnect");
    auto ap_record = connect_queue_.front();
    connect_queue_.erase(connect_queue_.begin());
    ssid_ = ap_record.ssid;
    password_ = ap_record.password;

    if (on_connect_) {
        on_connect_(ssid_);
    }
    ESP_LOGI(TAG, "Start connect to %s", ssid_.c_str());
    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config));
    strcpy((char *)wifi_config.sta.ssid, ap_record.ssid.c_str());
    if (ap_record.password.empty()) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }else{
        strcpy((char *)wifi_config.sta.password, ap_record.password.c_str());
    }
    wifi_config.sta.channel = ap_record.channel;
    memcpy(wifi_config.sta.bssid, ap_record.bssid, 6);
    wifi_config.sta.bssid_set = true;
    // example_wifi_sta_do_connect(wifi_config, true);
    s_semph_get_ip_addrs = xSemaphoreCreateBinary();
    if (s_semph_get_ip_addrs == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore for IP addresses");
        return;
    }
#if CONFIG_LWIP_IPV6
    s_semph_get_ip6_addrs = xSemaphoreCreateBinary();
    if (s_semph_get_ip6_addrs == NULL) {
        vSemaphoreDelete(s_semph_get_ip_addrs);
        ESP_LOGE(TAG, "Failed to create semaphore for IP6 addresses");
        return;
    }
#endif
    s_retry_num = 0;
    UnregisterEventHandlers();
    RegisterEventHandlers();
    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
        return;
    }
    
    // 死等IP
    ESP_LOGI(TAG, "Waiting for IP(s)");
#if CONFIG_LWIP_IPV4
    if (xSemaphoreTake(s_semph_get_ip_addrs, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGW(TAG, "IPv4地址等待超时，继续执行");
    }
#endif
#if CONFIG_LWIP_IPV6
    if (xSemaphoreTake(s_semph_get_ip6_addrs, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGW(TAG, "IPv6地址等待超时，继续执行");
    }
#endif 
    ESP_LOGI(TAG, "等待IP地址完成");
    isconnected = true;
    return;
}

// 获取RSSI
int8_t WifiStation::GetRssi() {
    // Get station info
    wifi_ap_record_t ap_info;
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
    return ap_info.rssi;
}

// 获取信道
uint8_t WifiStation::GetChannel() {
    // Get station info
    wifi_ap_record_t ap_info;
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
    return ap_info.primary;
}

// 检查WiFi连接状态
bool WifiStation::IsConnected() {
    // 检查WiFi连接状态和IP地址获取状态
    return isconnected;
}

// 设置WiFi省电模式
void WifiStation::SetPowerSaveMode(bool enabled) {
    ESP_ERROR_CHECK(esp_wifi_set_ps(enabled ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE));
}

// 非常有用，否则无法扫描热点
void WifiStation::WifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    auto* this_ = static_cast<WifiStation*>(arg);
    if (this_ == nullptr) {
        ESP_LOGE(TAG, "WifiEventHandler: Invalid argument");
        return;
    }

    if (event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START received");
        if (this_->on_scan_begin_) {
            this_->on_scan_begin_();
        }
        
        // 启动WiFi后等待一小段时间再扫描，确保驱动稳定
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 在WiFi启动完成事件中开始扫描
        esp_err_t err = esp_wifi_scan_start(nullptr, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WiFi scan: %s", esp_err_to_name(err));
            // 如果扫描失败，等待一段时间后重试
            if (this_->timer_handle_ != nullptr) {
                esp_timer_start_once(this_->timer_handle_, 5 * 1000 * 1000); // 5秒后重试
            }
        }
    } else if (event_id == WIFI_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "WIFI_EVENT_SCAN_DONE received");
        
        // 在事件处理函数中只发送通知，不做实际处理
        // 这样可以避免在系统事件任务中消耗太多栈空间
        if (s_scan_event_queue != NULL) {
            uint32_t event = WIFI_EVENT_SCAN_DONE;
            xQueueSend(s_scan_event_queue, &event, 0);
        }
    } else {
        // 忽略其他WiFi事件，由特定的处理程序处理
        ESP_LOGD(TAG, "Ignoring WiFi event: %d (handled by specific handlers)", (int)event_id);
    }
}

// 在函数外部定义一个全局的辅助函数
static esp_err_t example_wifi_sta_do_disconnect(void) {
    return ESP_OK;
}

void WifiStation::example_handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    WifiStation* this_ = static_cast<WifiStation*>(arg);
    isconnected = false;
    ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED received");
    if (this_->IsConnected()) {
        xEventGroupClearBits(this_->event_group_, WIFI_EVENT_CONNECTED);
    }
    
    s_retry_num++;
    if (s_retry_num > 3) {
        ESP_LOGI(TAG, "WiFi Connect failed %d times, stop reconnect.", s_retry_num);
        /* let example_wifi_sta_do_connect() return */
        if (s_semph_get_ip_addrs) {
            xSemaphoreGive(s_semph_get_ip_addrs);
            // ESP_LOGI(TAG, "Give semaphore for IPv4");
        }
        if (s_semph_get_ip6_addrs) {
            xSemaphoreGive(s_semph_get_ip6_addrs);
            // ESP_LOGI(TAG, "Give semaphore for IPv6");
        }
        ::example_wifi_sta_do_disconnect();  // 修改为调用全局函数
        return;
    }
    
    wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t*)event_data;  // 添加强制类型转换
    if (disconn->reason == WIFI_REASON_ROAMING) {
        ESP_LOGD(TAG, "station roaming, do nothing");
        return;
    }
    
    ESP_LOGI(TAG, "Wi-Fi disconnected %d, trying to reconnect...", disconn->reason);
    
    if (this_) {
        if (this_->reconnect_count_ < MAX_RECONNECT_COUNT) {
            this_->reconnect_count_++;
            ESP_LOGI(TAG, "Reconnecting to %s (attempt %d / %d)", 
                     this_->ssid_.c_str(), this_->reconnect_count_, MAX_RECONNECT_COUNT);
            
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reconnect: %s", esp_err_to_name(err));
                if (this_->timer_handle_ != nullptr) {
                    esp_timer_start_once(this_->timer_handle_, 3 * 1000 * 1000); // 3秒后重试
                }
            }
            return;
        }

        if (!this_->connect_queue_.empty()) {
            this_->StartConnect();
            return;
        }
        
        ESP_LOGI(TAG, "No more AP to connect, wait for next scan");
        if (this_->timer_handle_ != nullptr) {
            esp_timer_start_once(this_->timer_handle_, 10 * 1000 * 1000);
        }
        return;
    }
    
    // 如果this_为空，执行原始连接逻辑
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return;
    }
    ESP_ERROR_CHECK(err);
}

void WifiStation::example_handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED received");
    // 添加延迟，确保接口稳定
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_netif_t* sta_netif = (esp_netif_t*)esp_netif;
    // 1. 创建链路本地地址 - 这会自动启用IPv6
    esp_err_t ret = esp_netif_create_ip6_linklocal(sta_netif);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create IPv6 link-local address: %s", esp_err_to_name(ret));

        // 尝试另一种方式获取接口
        sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if(sta_netif) {
            ESP_LOGI(TAG, "尝试在标准接口上创建IPv6链路本地地址");
            ret = esp_netif_create_ip6_linklocal(sta_netif);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "二次尝试仍失败: %s", esp_err_to_name(ret));
            }
        }
    } 
    ESP_LOGI(TAG, "IPv6 link-local address created successfully");
    WifiStation* this_ = &WifiStation::GetInstance();
    this_->UnregisterEventHandlers();
    this_->RegisterEventHandlers();
    // 2. 通过尝试获取全局地址来触发Router Solicitation (RS) 消息
    // 这仅用于触发RS消息，不必担心返回错误
    esp_ip6_addr_t tmp_addr;
    ESP_LOGI(TAG, "Triggering IPv6 Router Solicitation...");
    esp_netif_get_ip6_global(sta_netif, &tmp_addr);
}

void WifiStation::example_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    s_retry_num = 0;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!example_is_our_netif("wifi-station", event->esp_netif)) {  // 去掉类名前缀
        return;
    }
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    if (s_semph_get_ip_addrs) {
        xSemaphoreGive(s_semph_get_ip_addrs);
    } else {
        ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));
    }
}

void WifiStation::example_handler_on_sta_got_ipv6(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    if (!example_is_our_netif("wifi-station", event->esp_netif)) {  // 去掉类名前缀
        return;
    }
    if (s_semph_get_ip6_addrs) {
        // ESP_LOGI(TAG, "Give semaphore for IPv6");
        xSemaphoreGive(s_semph_get_ip6_addrs);
        isconnected = true;
    }
    esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
    ESP_LOGI(TAG, "Got IPv6 event: Interface \"%s\" address: " IPV6STR ", type: %s", esp_netif_get_desc(event->esp_netif),
             IPV62STR(event->ip6_info.ip), example_ipv6_addr_types_to_str[ipv6_type]);
    // 让出CPU时间片
    taskYIELD();
}

// 修改为全局函数
static bool example_is_our_netif(const char *prefix, esp_netif_t *netif) {
    // 暂时返回true以确认是否是过滤问题
    return true;
}

// 声明一个额外的辅助函数 - 移到前面
static esp_err_t example_wifi_stop(void) {
    return ESP_OK;
}

void WifiStation::example_wifi_shutdown(void)
{
    ::example_wifi_sta_do_disconnect();  // 修改为调用全局函数
    example_wifi_stop();
}

// 修改函数实现，去掉static关键字
bool WifiStation::netif_desc_matches_with(esp_netif_t *netif, void *ctx)
{
    return strcmp((const char*)ctx, esp_netif_get_desc(netif)) == 0;
}

esp_netif_t *WifiStation::get_example_netif_from_desc(const char *desc)
{
    return esp_netif_find_if(netif_desc_matches_with, (void*)desc);
}

// 将原来的HandleScanResult改名为ProcessScanResult，并设为public
void WifiStation::ProcessScanResult() {
    HandleScanResult(); // 调用原函数，但在一个有足够栈空间的专用任务中
}

// 打印IPv6状态，用于调试
void WifiStation::PrintIPv6Status() {
    if (!IsConnected()) {
        ESP_LOGI(TAG, "WiFi not connected, cannot get IPv6 status");
        return;
    }
    
    // 获取当前连接的netif
    esp_netif_t *netif = get_example_netif_from_desc("wifi-station");
    if (netif == NULL) {
        ESP_LOGE(TAG, "Failed to get WiFi netif for IPv6 status");
        return;
    }
    
    // ESP-IDF中一般支持的IPv6地址数量上限
    #define MAX_IP6_ADDRS_PER_NETIF 6
    
    // 获取所有IPv6地址
    esp_ip6_addr_t addresses[MAX_IP6_ADDRS_PER_NETIF];
    int addr_count = 0;
    
    ESP_LOGI(TAG, "=== IPv6 Status ===");
    
    for (int i = 0; i < MAX_IP6_ADDRS_PER_NETIF; i++) {
        if (esp_netif_get_ip6_linklocal(netif, &addresses[i]) == ESP_OK) {
            // 链路本地地址
            addr_count++;
            char ip_str[48];
            snprintf(ip_str, sizeof(ip_str), IPV6STR, IPV62STR(addresses[i]));
            ESP_LOGI(TAG, "IPv6 link-local address (%d): %s", i, ip_str);
        }
    }
    
    // 尝试获取全局IPv6地址
    esp_ip6_addr_t global_addr;
    if (esp_netif_get_ip6_global(netif, &global_addr) == ESP_OK) {
        addr_count++;
        char ip_str[48];
        snprintf(ip_str, sizeof(ip_str), IPV6STR, IPV62STR(global_addr));
        ESP_LOGI(TAG, "IPv6 global address: %s", ip_str);
    }
    
    if (addr_count == 0) {
        ESP_LOGI(TAG, "No IPv6 addresses found");
    } else {
        ESP_LOGI(TAG, "Found %d IPv6 addresses", addr_count);
    }
    
    ESP_LOGI(TAG, "===================");
}