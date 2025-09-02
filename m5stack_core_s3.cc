#include "wifi_board.h"
#include "cores3_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "config.h"
#include "power_save_timer.h"
#include "i2c_device.h"
#include "axp2101.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <wifi_station.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9341.h>
#include <esp_timer.h>
#include "esp32_camera.h"

#include "otto_emoji_display.h"
#include "system_info.h"    //add this headfiles for Cocube
#include "mcp_server.h"     //add this headfiles for Cocube
#include "mqtt_client.h"    //add this headfiles for Cocube
#include "esp_wifi.h"       //add this headfiles for Cocube


#define TAG "M5StackCoreS3Board"

#define WIFI_CONNECTED_BIT BIT0     //add this headfiles for Cocube


LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);


// Position data structure
struct Position {       
    int x;
    int y;
    int yaw;
}position;

static EventGroupHandle_t wifi_event_group;

esp_mqtt_client_handle_t mqtt_client_ = nullptr;

std::string generate_mac_prefix() {        //add this function for Cocube
    std::string mac_address = SystemInfo::GetMacAddress();  
    // 从 "aa:bb:cc:dd:ee:ff" 格式中提取最后两个字节  
    size_t last_colon = mac_address.find_last_of(':');  
    size_t second_last_colon = mac_address.find_last_of(':', last_colon - 1);  
      
    std::string last_two_bytes = mac_address.substr(second_last_colon + 1);  
    // 移除冒号并转换为大写  
    last_two_bytes.erase(std::remove(last_two_bytes.begin(), last_two_bytes.end(), ':'), last_two_bytes.end());  
    std::transform(last_two_bytes.begin(), last_two_bytes.end(), last_two_bytes.begin(), ::toupper);  
      
    ESP_LOGI(TAG, "Generated MAC prefix: %s", last_two_bytes.c_str());  
    return last_two_bytes;  
}


class CoCube {      //add this class for Cocube
   public:
    // Helper function to format parameters
    std::string format_params(const std::vector<std::string>& params) {
        std::ostringstream oss;
        for (size_t i = 0; i < params.size(); ++i) {
            if (i > 0) oss << ",";
            oss << params[i];
        }
        return oss.str();
    }

    // Generate data_to_send for each command

    std::string move(const std::string& direction = "forward", int speed = 40) {
        std::string modified_direction = "cocube;" + direction;
        std::vector<std::string> params = {modified_direction, std::to_string(speed)};
        return "CoCube move," + format_params(params);
    }

    std::string rotate(const std::string& direction = "left", int speed = 30) {
        std::string modified_direction = "cocube;" + direction;
        std::vector<std::string> params = {modified_direction, std::to_string(speed)};
        return "CoCube rotate," + format_params(params);
    }

    std::string move_msecs(const std::string& direction = "forward", int speed = 40, int duration = 1000) {
        std::string modified_direction = "cocube;" + direction;
        std::vector<std::string> params = {modified_direction, std::to_string(speed), std::to_string(duration)};
        return "CoCube move for msecs," + format_params(params);
    }

    std::string move_to_landmark(int landmark = 1) {
        std::vector<std::string> params = {std::to_string(landmark)};
        return "move_to_landmark," + format_params(params);
    }

    std::string rotate_msecs(const std::string& direction = "left", int speed = 40, int duration = 1000) {
        std::string modified_direction = "cocube;" + direction;
        std::vector<std::string> params = {modified_direction, std::to_string(speed), std::to_string(duration)};
        return "CoCube rotate for msecs," + format_params(params);
    }

    std::string move_by_steps(const std::string& direction = "forward", int speed = 40, int step = 50) {
        std::string modified_direction = "cocube;" + direction;
        std::vector<std::string> params = {modified_direction, std::to_string(speed), std::to_string(step)};
        return "CoCube move by step," + format_params(params);
    }

    std::string rotate_by_degree(const std::string& direction = "left", int speed = 40, int degree = 90) {
        std::string modified_direction = "cocube;" + direction;
        std::vector<std::string> params = {modified_direction, std::to_string(speed), std::to_string(degree)};
        return "CoCube rotate by degree," + format_params(params);
    }

    std::string set_wheel_speed(int left_speed = 40, int right_speed = 40) {
        std::vector<std::string> params = {std::to_string(left_speed), std::to_string(right_speed)};
        return "CoCube set wheel," + format_params(params);
    }

    std::string wheels_stop() { return "CoCube wheels stop"; }

    std::string wheels_break() { return "CoCube wheels break"; }

    std::string rotate_to_angle(int angle = 0, int speed = 40) {
        std::vector<std::string> params = {std::to_string(angle), std::to_string(speed)};
        return "CoCube rotate to angle," + format_params(params);
    }

    std::string rotate_to_target(int target_x = 0, int target_y = 0, int speed = 30) {
        std::vector<std::string> params = {std::to_string(target_x), std::to_string(target_y), std::to_string(speed)};
        return "CoCube point towards," + format_params(params);
    }

    std::string move_to_target(int target_x = 0, int target_y = 0, int speed = 50) {
        std::vector<std::string> params = {std::to_string(target_x), std::to_string(target_y), std::to_string(speed)};
        return "CoCube move to," + format_params(params);
    }

    // External Module Functions
    std::string gripper_open() { return "ccmodule_gripper open"; }

    std::string gripper_close() { return "ccmodule_gripper close"; }

    std::string change_color(int color = 65280) {
        std::vector<std::string> params = {std::to_string(color)};
        return "set display color," + format_params(params);
    }

    std::string change_image(int img_num = 2) {
        std::vector<std::string> params = {std::to_string(img_num)};
        return "drawBMPfile,wanxiang" + format_params(params) + ".bmp,0,0";
    }

};

bool is_wifi_connected() {
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return true;  // Wi-Fi connected
    } else {
        return false;  // Wi-Fi not connected
    }
}

// Handle received MQTT messages
static esp_err_t mqtt_message_handler_cb(esp_mqtt_event_handle_t event) {
    std::string mac_prefix = generate_mac_prefix();
    std::string mqtt_sub_topic = "/" + mac_prefix + "/position";
    
    // Print received topic and data for debugging
    ESP_LOGI(TAG, "Received message on topic: %.*s", event->topic_len, event->topic);
    ESP_LOGI(TAG, "Message data: %.*s", event->data_len, event->data);
    
    // Check if topic matches correctly (using strncmp for partial matching)
    if (strncmp(event->topic, mqtt_sub_topic.c_str(), event->topic_len) == 0) {
        // Assume message content is "87,74,191" (format: "x,y,yaw")
        std::string message(event->data, event->data + event->data_len);
        
        size_t pos = 0;  
        while ((pos = message.find("，", pos)) != std::string::npos) {  
            message.replace(pos, 3, ",");  // 中文逗号占3个字节  
            pos += 1;  
        }   
        // Parse message (format: "x,y,yaw")
        Position new_position;
        int ret = std::sscanf(message.c_str(), "%d,%d,%d", &new_position.x, &new_position.y, &new_position.yaw);

        if (ret == 3) {
            // If parsing successful, save position
            position = new_position;  // Update current pose

            // Output updated pose data
            ESP_LOGI(TAG, "Updated position: x=%d, y=%d, yaw=%d", position.x, position.y, position.yaw);
        
        } else {
            ESP_LOGE(TAG, "Failed to parse position data: %s", message.c_str());
        }
    }
    else {
        // If received unexpected topic, print debug log
        ESP_LOGE(TAG, "Received message on unexpected topic: %.*s", event->topic_len, event->topic);
    }

    return ESP_OK;
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    std::string mac_prefix = generate_mac_prefix();
    std::string mqtt_sub_topic = "/" + mac_prefix + "/position";
    
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    // Handle MQTT events
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Subscribe to topic
        esp_mqtt_client_subscribe(mqtt_client_, mqtt_sub_topic.c_str(), 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_DATA:
        mqtt_message_handler_cb(event);
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        break;
    default:
        break;
    }
}

void InitializeMqtt() {
    // Get 4-digit alphanumeric prefix
    std::string mac_prefix = generate_mac_prefix();
    
    // Modify MQTT topics
    std::string mqtt_pub_topic = "/" + mac_prefix + "/control";
    std::string mqtt_sub_topic = "/" + mac_prefix + "/position";

    ESP_LOGI(TAG, "MQTT PUB Topic: %s", mqtt_pub_topic.c_str());
    ESP_LOGI(TAG, "MQTT SUB Topic: %s", mqtt_sub_topic.c_str());

    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    while (!is_wifi_connected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second
        ESP_LOGI(TAG, "Wi-Fi not connected yet...");
    }
    ESP_LOGI(TAG, "Wi-Fi connected!");
    
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi not connected. Cannot start MQTT.");
        return;
    }

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker =
            {
                .address = {.uri = "mqtt://broker.emqx.io", .port = 1883},
            },
        .credentials = {.username = "emqx", .authentication = {.password = "public"}}};

    mqtt_client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client_ == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    ESP_LOGI(TAG, "MQTT client initialized");

    esp_err_t err = esp_mqtt_client_register_event(mqtt_client_, MQTT_EVENT_ANY, mqtt_event_handler, mqtt_client_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "MQTT event handler registered");

    // Both subscription and publication topics use generated topics
    esp_mqtt_client_subscribe(mqtt_client_, mqtt_sub_topic.c_str(), 0);
    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), "message", 0, 0, 0);

    err = esp_mqtt_client_start(mqtt_client_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MQTT client started");
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);  // Set event
    }
}

void mqtt_task(void* pvParameters) {
    ESP_LOGI(TAG, "MQTT task started");

    // Wait for Wi-Fi connection event
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi connected!");

    // Initialize MQTT
    InitializeMqtt();

    vTaskDelete(NULL);  // Task completed, delete itself
}
   

class Pmic : public Axp2101 {
public:
    // Power Init
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        uint8_t data = ReadReg(0x90);
        data |= 0b10110100;
        WriteReg(0x90, data);
        WriteReg(0x99, (0b11110 - 5));
        WriteReg(0x97, (0b11110 - 2));
        WriteReg(0x69, 0b00110101);
        WriteReg(0x30, 0b111111);
        WriteReg(0x90, 0xBF);
        WriteReg(0x94, 33 - 5);
        WriteReg(0x95, 33 - 5);
    }

    void SetBrightness(uint8_t brightness) {
        brightness = ((brightness + 641) >> 5);
        WriteReg(0x99, brightness);
    }
};


class CustomBacklight : public Backlight {
public:
    CustomBacklight(Pmic *pmic) : pmic_(pmic) {}

    void SetBrightnessImpl(uint8_t brightness) override {
        pmic_->SetBrightness(target_brightness_);
        brightness_ = target_brightness_;
    }

private:
    Pmic *pmic_;
};


class Aw9523 : public I2cDevice {
public:
    // Exanpd IO Init
    Aw9523(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x02, 0b00000111);  // P0
        WriteReg(0x03, 0b10001111);  // P1
        WriteReg(0x04, 0b00011000);  // CONFIG_P0
        WriteReg(0x05, 0b00001100);  // CONFIG_P1
        WriteReg(0x11, 0b00010000);  // GCR P0 port is Push-Pull mode.
        WriteReg(0x12, 0b11111111);  // LEDMODE_P0
        WriteReg(0x13, 0b11111111);  // LEDMODE_P1
    }

    void ResetAw88298() {
        ESP_LOGI(TAG, "Reset AW88298");
        WriteReg(0x02, 0b00000011);
        vTaskDelay(pdMS_TO_TICKS(10));
        WriteReg(0x02, 0b00000111);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void ResetIli9342() {
        ESP_LOGI(TAG, "Reset IlI9342");
        WriteReg(0x03, 0b10000001);
        vTaskDelay(pdMS_TO_TICKS(20));
        WriteReg(0x03, 0b10000011);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

class Ft6336 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    
    Ft6336(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t chip_id = ReadReg(0xA3);
        ESP_LOGI(TAG, "Get chip ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Ft6336() {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    inline const TouchPoint_t& GetTouchPoint() {
        return tp_;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
};


class M5StackCoreS3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_;
    Aw9523* aw9523_;
    Ft6336* ft6336_;
    LcdDisplay* display_;
    Esp32Camera* camera_;
    esp_timer_handle_t touchpad_timer_;
    PowerSaveTimer* power_save_timer_;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(10);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            pmic_->PowerOff();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeAw9523() {
        ESP_LOGI(TAG, "Init AW9523");
        aw9523_ = new Aw9523(i2c_bus_, 0x58);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void PollTouchpad() {
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        const int64_t TOUCH_THRESHOLD_MS = 500;  // 触摸时长阈值，超过500ms视为长按
        
        ft6336_->UpdateTouchPoint();
        auto& touch_point = ft6336_->GetTouchPoint();
        
        // 检测触摸开始
        if (touch_point.num > 0 && !was_touched) {
            was_touched = true;
            touch_start_time = esp_timer_get_time() / 1000; // 转换为毫秒
        } 
        // 检测触摸释放
        else if (touch_point.num == 0 && was_touched) {
            was_touched = false;
            int64_t touch_duration = (esp_timer_get_time() / 1000) - touch_start_time;
            
            // 只有短触才触发
            if (touch_duration < TOUCH_THRESHOLD_MS) {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting && 
                    !WifiStation::GetInstance().IsConnected()) {
                    ResetWifiConfiguration();
                }
                app.ToggleChatState();
            }
        }
    }

    void InitializeFt6336TouchPad() {
        ESP_LOGI(TAG, "Init FT6336");
        ft6336_ = new Ft6336(i2c_bus_, 0x38);
        
        // 创建定时器，20ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                M5StackCoreS3Board* board = (M5StackCoreS3Board*)arg;
                board->PollTouchpad();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_37;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_36;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeCopilotTools() {                  //add this function for Cocube
        
        wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

        // Create MQTT task
        //xTaskCreate(mqtt_task, "MQTT_Task", 8192, NULL, 5, NULL);
        xTaskCreatePinnedToCore(mqtt_task, "MQTT_Task",8192,NULL,5,NULL,0);
        
        CoCube cocube;
        auto& mcp_server = McpServer::GetInstance();  

        //获取位置信息
        mcp_server.AddTool(
            "Cocube.get_position",  
            "Get robot current position and orientation",  
            PropertyList(),  
            [](const PropertyList& properties) -> ReturnValue {  
                char json_str[256];  
                snprintf(json_str, sizeof(json_str),   
                    "{\"x\": %d, \"y\": %d, \"yaw\": %d}",   
                    position.x, position.y, position.yaw);  
                return std::string(json_str);  
            });


        // mcp_server.AddTool("Cocube.get_position",  
        //     "Get robot's current position (x, y, yaw)",  
        //     PropertyList(), // 无参数，只读工具  
        //     [](const PropertyList& properties) -> ReturnValue {  
        //         return std::to_string(position.x) + "," +   
        //             std::to_string(position.y) + "," +   
        //             std::to_string(position.yaw);  
        // }); 

        //Register Move method
        mcp_server.AddTool(
            "Cocube.Move",
            "Control robot to move forward or backward for a period of time\n"
            "Parameters:\n"  
            "- direction: forward/backward, default is forward\n"  
            "- speed: rotation speed 0-50 (default: 40)\n"  
            "- duration: rotation time in milliseconds/毫秒 (default: 1000ms)\n"
            "If no duration is specified, robot will move for 1 second by default.", 
            PropertyList({Property("direction", kPropertyTypeString),
                           Property("speed", kPropertyTypeInteger, 40, 0, 50),
                           Property("duration", kPropertyTypeInteger, 1000)
                        }),
            [&cocube](const PropertyList& properties) -> ReturnValue {

                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic

                std::string direction = "forward";
                int speed = 40;
                int duration = 1000;

                if (properties["direction"].type() == kPropertyTypeString) {
                    direction = properties["direction"].value<std::string>();
                }
                if (properties["speed"].type() == kPropertyTypeInteger) {
                    speed = properties["speed"].value<int>();
                }
                if (properties["duration"].type() == kPropertyTypeInteger) {
                    duration = properties["duration"].value<int>();
                }

                std::string msg = cocube.move_msecs(direction, speed, duration);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "Move");
                }
                return true;
            });

        // Register Move to landmark method
        mcp_server.AddTool(
            "Cocube.Move to landmark",
             "Control robot to move to a landmark building.\n"
             "Parameters:\n"  
             "- landmark: Landmark building number (1-4), default is 1",
            PropertyList({ 
                Property("landmark", kPropertyTypeInteger, 1,1,4)
            }),
            [&cocube](const PropertyList& properties) -> ReturnValue {

                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic

                int landmark = 1;

                if (properties["landmark"].type() == kPropertyTypeInteger) {
                    landmark = properties["landmark"].value<int>();
                }

                std::string msg = cocube.move_to_landmark(landmark);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "Move to landmark");
                }
                return true;
            });

        // Register Rotate method
        mcp_server.AddTool(
            "Cocube.Rotate", 
            "Control robot to rotate left or right for a period of time.\n"  
            "Parameters:\n"  
            "- direction: left or right (default: left)\n"  
            "- speed: rotation speed 0-50 (default: 30)\n"  
            "- duration: rotation time in milliseconds/毫秒 (default: 1000ms)\n"
            "If no duration is specified, robot will rotate for 1 second by default.", 
            PropertyList({Property("direction", kPropertyTypeString),
                           Property("speed", kPropertyTypeInteger, 30, 0, 50),
                           Property("duration",  kPropertyTypeInteger, 1000)//The unit of duration is ms
                        }),
            [&cocube](const PropertyList& properties) -> ReturnValue {
                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                std::string direction = "left";
                int speed = 30;
                int duration = 1000;

                if (properties["direction"].type() == kPropertyTypeString) {
                    direction = properties["direction"].value<std::string>();
                }
                if (properties["speed"].type() == kPropertyTypeInteger) {
                    speed = properties["speed"].value<int>();
                }
                if (properties["duration"].type() == kPropertyTypeInteger) {
                    duration = properties["duration"].value<int>();
                }

                std::string msg = cocube.rotate_msecs(direction, speed, duration);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "Rotate");
                }
                return true;
            });

        // Register MoveBySteps method
        mcp_server.AddTool(
            "Cocube.MoveBySteps", 
            "Control robot to move specific number of steps\n"
            "Parameters:\n"  
            "- direction: Direction (forward/backward), default is forward\n"  
            "- speed: Speed (0-50), default is 40\n"  
            "- step: Steps, default is 50", 
            PropertyList({Property("direction", kPropertyTypeString ),
                           Property("speed", kPropertyTypeInteger, 40, 0, 50),
                           Property("step",  kPropertyTypeInteger, 50)}),
            [&cocube](const PropertyList& properties) -> ReturnValue {
                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                std::string direction = "forward";
                int speed = 40;
                int step = 50;

                if (properties["direction"].type() == kPropertyTypeString) {
                    direction = properties["direction"].value<std::string>();
                }
                if (properties["speed"].type() == kPropertyTypeInteger) {
                    speed = properties["speed"].value<int>();
                }
                if (properties["step"].type() == kPropertyTypeInteger) {
                    step = properties["step"].value<int>();
                }

                std::string msg = cocube.move_by_steps(direction, speed, step);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "MoveBySteps");
                }
                return true;
            });

        // Register RotateByDegree method
        mcp_server.AddTool(
            "Cocube.RotateByDegree", 
            "Control robot to rotate left or right by specific angle\n"
            "Parameters:\n"  
            "- direction: Direction (left/right), default is left\n"  
            "- speed: Speed (0-50), default is 40\n"  
            "- degree: Angle, default is 90", 
            PropertyList({Property("direction",kPropertyTypeString),
                           Property("speed", kPropertyTypeInteger, 40, 0, 50),
                           Property("degree",kPropertyTypeInteger, 90)}),
            [&cocube](const PropertyList& properties) -> ReturnValue {
                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                std::string direction = "left";
                int speed = 40;
                int degree = 90;

                if (properties["direction"].type() == kPropertyTypeString) {
                    direction = properties["direction"].value<std::string>();
                }
                if (properties["speed"].type() == kPropertyTypeInteger) {
                    speed = properties["speed"].value<int>();
                }
                if (properties["degree"].type() == kPropertyTypeInteger) {
                    degree = properties["degree"].value<int>();
                }

                std::string msg = cocube.rotate_by_degree(direction, speed, degree);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "RotateByDegree");
                }
                return true;
            });

        // Register MoveToTarget method
        mcp_server.AddTool(
            "Cocube.MoveToTarget",
            "Control robot to move to target point\n"
            "Parameters:\n"  
            "- target_x: Target X coordinate, default is 0\n"  
            "- target_y: Target Y coordinate, default is 0\n"  
            "- speed: Speed (0-50), default is 50", 
            PropertyList({Property("target_x", kPropertyTypeInteger, 0),
                          Property("target_y",kPropertyTypeInteger, 0),
                          Property("speed",kPropertyTypeInteger, 50, 0, 50)}),
                           [&cocube](const PropertyList& properties) -> ReturnValue {
                                // Get MAC address prefix and build dynamic MQTT topic
                                std::string mac_prefix = generate_mac_prefix();
                                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int target_x = 0;
                               int target_y = 0;
                               int speed = 50;

                               if (properties["target_x"].type() == kPropertyTypeInteger) {
                                   target_x = properties["target_x"].value<int>();
                               }
                               if (properties["target_y"].type() == kPropertyTypeInteger) {
                                   target_y = properties["target_y"].value<int>();
                               }
                               if (properties["speed"].type() == kPropertyTypeInteger) {
                                   speed = properties["speed"].value<int>();
                               }

                               std::string msg = cocube.move_to_target(target_x, target_y, speed);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "MoveToTarget");
                               }
                               return true;
                           });

        // Register rotate_to_angle method
        mcp_server.AddTool(
            "Cocube.RotateToAngle", 
            "Control robot to rotate to specified angle\n"
             "Parameters:\n"  
            "- angle: Target rotation angle, default is 0\n"  
            "- speed: Rotation speed (0-50), default is 40",  
                           PropertyList({Property("angle",kPropertyTypeInteger, 0),
                                          Property("speed", kPropertyTypeInteger, 40, 0, 50)}),
                           [&cocube](const PropertyList& properties) -> ReturnValue {
                            // Get MAC address prefix and build dynamic MQTT topic
                                std::string mac_prefix = generate_mac_prefix();
                                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int angle = 0;
                               int speed = 40;

                               if (properties["angle"].type() == kPropertyTypeInteger) {
                                   angle = properties["angle"].value<int>();
                               }
                               if (properties["speed"].type() == kPropertyTypeInteger) {
                                   speed = properties["speed"].value<int>();
                               }

                               std::string msg = cocube.rotate_to_angle(angle, speed);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "RotateToAngle");
                               }
                               return true;
                           });

        // Register rotate_to_target method
        mcp_server.AddTool(
            "Cocube.RotateToTarget", 
            "Control robot to rotate towards target point\n"
            "Parameters:\n"  
            "- target_x: Target X coordinate, default is 0\n"  
            "- target_y: Target Y coordinate, default is 0\n"  
            "- speed: Rotation speed (0-50), default is 30", 
            PropertyList({Property("target_x", kPropertyTypeInteger, 0),
                           Property("target_y", kPropertyTypeInteger, 0),
                           Property("speed",  kPropertyTypeInteger, 30, 0, 50)}),
            [&cocube](const PropertyList& properties) -> ReturnValue {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int target_x = 0;
                               int target_y = 0;
                               int speed = 30;

                               if (properties["target_x"].type() == kPropertyTypeInteger) {
                                   target_x = properties["target_x"].value<int>();
                               }
                               if (properties["target_y"].type() == kPropertyTypeInteger) {
                                   target_y = properties["target_y"].value<int>();
                               }
                               if (properties["speed"].type() == kPropertyTypeInteger) {
                                   speed = properties["speed"].value<int>();
                               }

                               std::string msg = cocube.rotate_to_target(target_x, target_y, speed);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "RotateToTarget");
                               }
                               return true;
                           });

        // Register gripper open method
        mcp_server.AddTool(
            "Cocube.GripperOpen", 
            "Control robot gripper to open", 
            PropertyList(),
            [&cocube](const PropertyList& properties) -> ReturnValue {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               std::string msg = cocube.gripper_open();
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "GripperOpen");
                               }
                               return true;
                           });

        // Register gripper close method
        mcp_server.AddTool(
            "Cocube.GripperClose", 
            "Control robot gripper to close, or shoot ball", 
            PropertyList(),
            [&cocube](const PropertyList& properties) -> ReturnValue {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               std::string msg = cocube.gripper_close();
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "GripperClose");
                               }
                               return true;
                           });

        // Register change color method
        mcp_server.AddTool(
            "Cocube.ChangeColor",
            "Control robot screen color\n"
            "Parameters:\n"  
            "- color_R: Red component, 0-255, default is 0\n"  
            "- color_G: Green component, 0-255, default is 235\n"  
            "- color_B: Blue component, 0-255, default is 0", 
                           PropertyList({
                               Property("color_R", kPropertyTypeInteger, 0, 0, 255),
                               Property("color_G",  kPropertyTypeInteger, 235, 0,255),
                               Property("color_B",  kPropertyTypeInteger, 0, 0,255)}),
                           [&cocube](const PropertyList& properties) -> ReturnValue {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int color_r = 0;
                               int color_g = 255;
                               int color_b = 0;

                               if (properties["color_R"].type() == kPropertyTypeInteger) {
                                   color_r = properties["color_R"].value<int>();
                               }

                               if (properties["color_G"].type() == kPropertyTypeInteger) {
                                   color_g = properties["color_G"].value<int>();
                               }

                               if (properties["color_B"].type() == kPropertyTypeInteger) {
                                   color_b = properties["color_B"].value<int>();
                               }

                               int color = (color_r << 16) | (color_g << 8) | color_b;  // RGB to int

                               std::string msg = cocube.change_color(color);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "ChangeColor");
                               }
                               return true;
                           });

        // Register change screen image method
        mcp_server.AddTool(
            "Cocube.ChangeImage",
            "Change robot screen display avatar pattern\n"
            "Parameters:\n"  
            "- img_num: Pattern number, 1-7, default is 2, 1 is lion, 2 is polar bear, 3 is panda, 4 is hedgehog, 5 is cat, 6 is sheep, 7 is tiger"  ,
            PropertyList({
                Property("img_num", kPropertyTypeInteger, 2, 1, 7),
            }),
            [&cocube](const PropertyList& properties) -> ReturnValue {
            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int img_num = 2;

                               if (properties["img_num"].type() == kPropertyTypeInteger) {
                                   img_num = properties["img_num"].value<int>();
                               }

                               std::string msg = cocube.change_image(img_num);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   
                                   ESP_LOGI(TAG, "ChangeImage");
                               }
                               return true;
                           });

        // Register WheelsStop method
       mcp_server.AddTool(
            "Cocube.WheelsStop",
            "Stop robot", 
            PropertyList(),
            [&cocube](const PropertyList& properties) -> ReturnValue {
            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
            std::string msg = cocube.wheels_stop();
            if (mqtt_client_ != nullptr) {
                esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                ESP_LOGI(TAG, "WheelsStop");
            }
            return true;
        });

        // Register shoot method
        mcp_server.AddTool(
            "Cocube.Shoot",
            "Soccer shoot", 
            PropertyList(),
            [&cocube](const PropertyList& properties) -> ReturnValue {
            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
            std::string msg = "CoCube shoot";
            if (mqtt_client_ != nullptr) {
                esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                ESP_LOGI(TAG, "CoCube shoot");
            }
            return true;
        });

    }

    void InitializeIli9342Display() {
        ESP_LOGI(TAG, "Init IlI9342");

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_3;
        io_config.dc_gpio_num = GPIO_NUM_35;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        aw9523_->ResetIli9342();

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new OttoEmojiDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_20_4,
                                        .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = font_emoji_64_init(),
#endif
                                    });
    }

     void InitializeCamera() {
        // Open camera power
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;  
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
    }

public:
    M5StackCoreS3Board() {
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeAxp2101();
        InitializeAw9523();
        I2cDetect();
        InitializeSpi();
        InitializeIli9342Display();
        InitializeCamera();
        InitializeFt6336TouchPad();
        // InitializeCocubeConnection();  //add this line for Cocube
        InitializeCopilotTools();      //add this line for Cocube
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CoreS3AudioCodec audio_codec(i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_AW88298_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveMode(bool enabled) override {
        if (!enabled) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveMode(enabled);
    }

    virtual Backlight *GetBacklight() override {
        static CustomBacklight backlight(pmic_);
        return &backlight;
    }
};

DECLARE_BOARD(M5StackCoreS3Board);
