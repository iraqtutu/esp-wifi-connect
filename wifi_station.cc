#include "wifi_station.h"
#include <cstring>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs.h>
#include "nvs_flash.h"
#include <esp_netif.h>
#include <esp_system.h>
#include "ssid_manager.h"
#include <lwip/ip6_addr.h>
#include <lwip/sockets.h>    // 添加套接字API支持
#include <arpa/inet.h>       // 添加IP地址转换函数支持
#include <netdb.h>           // 添加网络数据库函数支持

#define TAG "wifi"
#define WIFI_EVENT_CONNECTED BIT0
#define MAX_RECONNECT_COUNT 5

WifiStation& WifiStation::GetInstance() {
    static WifiStation instance;
    return instance;
}

WifiStation::WifiStation() {
    // Create the event group
    event_group_ = xEventGroupCreate();
}

WifiStation::~WifiStation() {
    vEventGroupDelete(event_group_);
}

void WifiStation::AddAuth(const std::string &&ssid, const std::string &&password) {
    auto& ssid_manager = SsidManager::GetInstance();
    ssid_manager.AddSsid(ssid, password);
}

void WifiStation::Stop() {
    if (timer_handle_ != nullptr) {
        esp_timer_stop(timer_handle_);
        esp_timer_delete(timer_handle_);
        timer_handle_ = nullptr;
    }

    // Reset the WiFi stack
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    
    // 取消注册事件处理程序
    if (instance_any_id_ != nullptr) {
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id_));
        instance_any_id_ = nullptr;
    }
    if (instance_got_ip_ != nullptr) {
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip_));
        instance_got_ip_ = nullptr;
    }
    if (instance_got_ip6_ != nullptr) {
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_GOT_IP6, instance_got_ip6_));
        instance_got_ip6_ = nullptr;
    }
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

void WifiStation::Start() {
    // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &WifiStation::WifiEventHandler,
                                                        this,
                                                        &instance_any_id_));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &WifiStation::IpEventHandler,
                                                        this,
                                                        &instance_got_ip_));
    // ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
    //                                                     IP_EVENT_GOT_IP6,
    //                                                     &WifiStation::Ipv6EventHandler,
    //                                                     this,
    //                                                     &instance_got_ip6_));

    // Create the default event loop
    esp_netif_create_default_wifi_sta();

    // Initialize the WiFi stack in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Setup the timer to scan WiFi
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            esp_wifi_scan_start(nullptr, false);
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "WiFiScanTimer",
        .skip_unhandled_events = true
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle_));
}

bool WifiStation::WaitForConnected(int timeout_ms) {
    auto bits = xEventGroupWaitBits(event_group_, WIFI_EVENT_CONNECTED, pdFALSE, pdFALSE, timeout_ms / portTICK_PERIOD_MS);
    return (bits & WIFI_EVENT_CONNECTED) != 0;
}

void WifiStation::HandleScanResult() {
    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)malloc(ap_num * sizeof(wifi_ap_record_t));
    esp_wifi_scan_get_ap_records(&ap_num, ap_records);
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
    free(ap_records);

    if (connect_queue_.empty()) {
        ESP_LOGI(TAG, "Wait for next scan");
        esp_timer_start_once(timer_handle_, 10 * 1000);
        return;
    }

    StartConnect();
}

void WifiStation::StartConnect() {
    auto ap_record = connect_queue_.front();
    connect_queue_.erase(connect_queue_.begin());
    ssid_ = ap_record.ssid;
    password_ = ap_record.password;

    if (on_connect_) {
        on_connect_(ssid_);
    }

    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config));
    strcpy((char *)wifi_config.sta.ssid, ap_record.ssid.c_str());
    strcpy((char *)wifi_config.sta.password, ap_record.password.c_str());
    wifi_config.sta.channel = ap_record.channel;
    memcpy(wifi_config.sta.bssid, ap_record.bssid, 6);
    wifi_config.sta.bssid_set = true;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    reconnect_count_ = 0;
    ESP_ERROR_CHECK(esp_wifi_connect());
}

int8_t WifiStation::GetRssi() {
    // Get station info
    wifi_ap_record_t ap_info;
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
    return ap_info.rssi;
}

uint8_t WifiStation::GetChannel() {
    // Get station info
    wifi_ap_record_t ap_info;
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
    return ap_info.primary;
}

bool WifiStation::IsConnected() {
    return xEventGroupGetBits(event_group_) & WIFI_EVENT_CONNECTED;
}

void WifiStation::SetPowerSaveMode(bool enabled) {
    ESP_ERROR_CHECK(esp_wifi_set_ps(enabled ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE));
}

// Static event handler functions
void WifiStation::WifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    auto* this_ = static_cast<WifiStation*>(arg);
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_scan_start(nullptr, false);
        if (this_->on_scan_begin_) {
            this_->on_scan_begin_();
        }
    } else if (event_id == WIFI_EVENT_SCAN_DONE) {
        this_->HandleScanResult();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(this_->event_group_, WIFI_EVENT_CONNECTED);
        if (this_->reconnect_count_ < MAX_RECONNECT_COUNT) {
            ESP_ERROR_CHECK(esp_wifi_connect());
            this_->reconnect_count_++;
            ESP_LOGI(TAG, "Reconnecting %s (attempt %d / %d)", this_->ssid_.c_str(), this_->reconnect_count_, MAX_RECONNECT_COUNT);
            return;
        }

        if (!this_->connect_queue_.empty()) {
            this_->StartConnect();
            return;
        }
        
        ESP_LOGI(TAG, "No more AP to connect, wait for next scan");
        esp_timer_start_once(this_->timer_handle_, 10 * 1000);
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
    }
}

void WifiStation::IpEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    auto* this_ = static_cast<WifiStation*>(arg);
    auto* event = static_cast<ip_event_got_ip_t*>(event_data);

    char ip_address[16];
    esp_ip4addr_ntoa(&event->ip_info.ip, ip_address, sizeof(ip_address));
    this_->ip_address_ = ip_address;
    ESP_LOGI(TAG, "Got IP: %s", this_->ip_address_.c_str());
    
    xEventGroupSetBits(this_->event_group_, WIFI_EVENT_CONNECTED);
    if (this_->on_connected_) {
        this_->on_connected_(this_->ssid_);
    }
    this_->connect_queue_.clear();
    this_->reconnect_count_ = 0;
}

void WifiStation::Ipv6EventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    static int retry_count = 0; // 添加静态计数器跟踪尝试次数
    const int max_retries = 3;  // 最多尝试3次
    
    auto* this_ = static_cast<WifiStation*>(arg);
    auto* event = static_cast<ip_event_got_ip6_t*>(event_data);
    
    char ip6_address[48];
    ip6addr_ntoa_r((const ip6_addr_t*)&event->ip6_info.ip, ip6_address, sizeof(ip6_address));
    this_->ipv6_address_ = ip6_address;
    
    // 判断IPv6地址类型
    ip6_addr_t* addr = (ip6_addr_t*)&event->ip6_info.ip;
    bool is_link_local = ip6_addr_islinklocal(addr);
    bool is_global = ip6_addr_isglobal(addr);
    bool is_unique_local = ip6_addr_isuniquelocal(addr);
    
    // 打印IPv6地址的详细信息
    ESP_LOGI(TAG, "获得IPv6地址: %s", ip6_address);
    ESP_LOGI(TAG, "  - 网络接口: %s", esp_netif_get_desc(event->esp_netif));
    ESP_LOGI(TAG, "  - 地址类型: %s%s%s", 
             is_link_local ? "链路本地" : "",
             is_global ? "全局" : "",
             is_unique_local ? "唯一本地" : "");
    
    // 如果只获取到链路本地地址，尝试请求全局地址，但有尝试次数限制
    if (is_link_local && !is_global && !is_unique_local && retry_count < max_retries) {
        retry_count++;
        ESP_LOGI(TAG, "仅获取到链路本地地址，尝试获取全局IPv6地址...(尝试 %d/%d)", retry_count, max_retries);
        
        // 请求路由器公告(RA)和DHCPv6
        esp_netif_t* netif = event->esp_netif;
        
        // 确保创建链路本地地址
        if (esp_netif_create_ip6_linklocal(netif) != ESP_OK) {
            ESP_LOGW(TAG, "创建链路本地地址失败");
        }
        
        // 尝试启动DHCPv6客户端
        if (esp_netif_dhcpc_start(netif) != ESP_OK) {
            ESP_LOGW(TAG, "DHCPv6客户端启动失败");
        } else {
            ESP_LOGI(TAG, "DHCPv6客户端启动成功");
        }
        
        // 设置标志，指示我们仅有本地地址
        this_->has_global_ipv6_ = false;
        
        // 主动触发一些网络活动以促进RA
        // 这里我们使用IP6层创建套接字
        int sock = socket(AF_INET6, SOCK_DGRAM, 0);
        if (sock >= 0) {
            // 设置不绑定到特定接口
            int set = 0;
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &set, sizeof(set));
            
            // 尝试连接到常见的IPv6服务如Google DNS
            struct sockaddr_in6 addr;
            memset(&addr, 0, sizeof(addr));
            addr.sin6_family = AF_INET6;
            addr.sin6_port = htons(53); // DNS端口
            
            // 使用阿里的IPv6 DNS服务器
            inet_pton(AF_INET6, "2400:3200:baba::1", &addr.sin6_addr);
            
            ESP_LOGI(TAG, "尝试连接到IPv6 DNS服务器以触发RA...");
            connect(sock, (struct sockaddr*)&addr, sizeof(addr));
            
            // 发送一些数据
            uint8_t data[10] = {0};
            send(sock, data, 10, 0);
            
            // 关闭套接字
            close(sock);
            ESP_LOGI(TAG, "促进RA尝试完成");
        } else {
            ESP_LOGW(TAG, "无法创建IPv6套接字");
        }
    } else if (is_link_local && !is_global && !is_unique_local && retry_count >= max_retries) {
        // 达到最大尝试次数，只使用链路本地地址
        ESP_LOGW(TAG, "达到最大尝试次数(%d)，将只使用链路本地IPv6地址", max_retries);
        ESP_LOGW(TAG, "您的网络可能不支持全局IPv6地址");
        this_->has_global_ipv6_ = false;
    } else if (is_global || is_unique_local) {
        ESP_LOGI(TAG, "成功获取全局或唯一本地IPv6地址!");
        this_->has_global_ipv6_ = true;
        retry_count = 0; // 重置计数器，以便将来可以重新尝试
    }
    
    if (!(xEventGroupGetBits(this_->event_group_) & WIFI_EVENT_CONNECTED)) {
        xEventGroupSetBits(this_->event_group_, WIFI_EVENT_CONNECTED);
        if (this_->on_connected_) {
            this_->on_connected_(this_->ssid_);
        }
    }
}
