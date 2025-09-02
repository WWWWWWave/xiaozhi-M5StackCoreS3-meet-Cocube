#include <esp_log.h>

#include <sstream>
#include <string>
#include <vector>

#include "audio_codec.h"
#include "board.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "iot/thing.h"
#include "mqtt_client.h"
#include "esp_system.h"
#include <esp_mac.h>

#define TAG "Copilot"

std::string generate_mac_prefix() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);  // Get MAC address consistent with SoftAP interface

    // Generate 4-digit hex prefix consistent with GetSsid() (mac[4] and mac[5])
    char prefix[5];  // 4 digits + 1 terminator
    snprintf(prefix, sizeof(prefix), "%02X%02X", mac[4], mac[5]);

    ESP_LOGI(TAG, "Generated MAC prefix: %s", prefix);
    return std::string(prefix);
}

// Position data structure
struct Position {
    int x;
    int y;
    int yaw;
}position;

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group;

esp_mqtt_client_handle_t mqtt_client_ = nullptr;

class CoCube {
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

namespace iot {

class Copilot : public Thing {
   public:
    Copilot() : Thing("Copilot", "Robot driving management") {
        wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

        // Create MQTT task
        xTaskCreate(mqtt_task, "MQTT_Task", 8192, NULL, 5, NULL);
        
        // Define property: x_position (robot's X coordinate)
        properties_.AddNumberProperty("x_position", "Robot's current X-axis coordinate", [this]() -> int {
            return position.x;  // Return robot's X coordinate
        });

        // Define property: y_position (robot's Y coordinate)
        properties_.AddNumberProperty("y_position", "Robot's current Y-axis coordinate", [this]() -> int {
            return position.y;  // Return robot's Y coordinate
        });

        // Define property: yaw (robot's orientation angle)
        properties_.AddNumberProperty("yaw", "Robot's current rotation angle", [this]() -> int {
            return position.yaw;  // Return robot's Y coordinate
        });

        CoCube cocube;

        // Register Move method
        methods_.AddMethod(
            "Move", "Control robot to move forward or backward for a period of time",
            ParameterList({Parameter("direction", "Direction (forward/backward), default is forward", kValueTypeString, false),
                           Parameter("speed", "Speed (0-50), default is 40", kValueTypeNumber, false),
                           Parameter("duration", "Duration (milliseconds), default is 1000", kValueTypeNumber, false)}),
            [&cocube](const ParameterList& parameters) {

                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic

                std::string direction = "forward";
                int speed = 40;
                int duration = 1000;

                if (parameters["direction"].type() == kValueTypeString) {
                    direction = parameters["direction"].string();
                }
                if (parameters["speed"].type() == kValueTypeNumber) {
                    speed = parameters["speed"].number();
                }
                if (parameters["duration"].type() == kValueTypeNumber) {
                    duration = parameters["duration"].number();
                }

                std::string msg = cocube.move_msecs(direction, speed, duration);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "Move");
                }
            });

        // Register Move to landmark method
        methods_.AddMethod(
            "Move to landmark", "Control robot to move to a landmark building",
            ParameterList({Parameter("landmark", "Landmark building number (1-4), default is 1", kValueTypeNumber, false)}),
            [&cocube](const ParameterList& parameters) {

                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic

                int landmark = 1;

                if (parameters["landmark"].type() == kValueTypeNumber) {
                    landmark = parameters["landmark"].number();
                }

                std::string msg = cocube.move_to_landmark(landmark);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "Move to landmark");
                }
            });

        // Register Rotate method
        methods_.AddMethod(
            "Rotate", "Control robot to rotate left or right for a period of time",
            ParameterList({Parameter("direction", "Direction (left/right), default is left", kValueTypeString, false),
                           Parameter("speed", "Speed (0-50), default is 30", kValueTypeNumber, false),
                           Parameter("duration", "Duration (milliseconds), default is 1000", kValueTypeNumber, false)}),
            [&cocube](const ParameterList& parameters) {
                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                std::string direction = "left";
                int speed = 30;
                int duration = 1000;

                if (parameters["direction"].type() == kValueTypeString) {
                    direction = parameters["direction"].string();
                }
                if (parameters["speed"].type() == kValueTypeNumber) {
                    speed = parameters["speed"].number();
                }
                if (parameters["duration"].type() == kValueTypeNumber) {
                    duration = parameters["duration"].number();
                }

                std::string msg = cocube.rotate_msecs(direction, speed, duration);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "Rotate");
                }
            });

        // Register MoveBySteps method
        methods_.AddMethod(
            "MoveBySteps", "Control robot to move specific number of steps",
            ParameterList({Parameter("direction", "Direction (forward/backward), default is forward", kValueTypeString, false),
                           Parameter("speed", "Speed (0-50), default is 40", kValueTypeNumber, false),
                           Parameter("step", "Steps, default is 50", kValueTypeNumber, false)}),
            [&cocube](const ParameterList& parameters) {
                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                std::string direction = "forward";
                int speed = 40;
                int step = 50;

                if (parameters["direction"].type() == kValueTypeString) {
                    direction = parameters["direction"].string();
                }
                if (parameters["speed"].type() == kValueTypeNumber) {
                    speed = parameters["speed"].number();
                }
                if (parameters["step"].type() == kValueTypeNumber) {
                    step = parameters["step"].number();
                }

                std::string msg = cocube.move_by_steps(direction, speed, step);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "MoveBySteps");
                }
            });

        // Register RotateByDegree method
        methods_.AddMethod(
            "RotateByDegree", "Control robot to rotate left or right by specific angle",
            ParameterList({Parameter("direction", "Direction (left/right), default is left", kValueTypeString, false),
                           Parameter("speed", "Speed (0-50), default is 40", kValueTypeNumber, false),
                           Parameter("degree", "Angle, default is 90", kValueTypeNumber, false)}),
            [&cocube](const ParameterList& parameters) {
                // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                std::string direction = "left";
                int speed = 40;
                int degree = 90;

                if (parameters["direction"].type() == kValueTypeString) {
                    direction = parameters["direction"].string();
                }
                if (parameters["speed"].type() == kValueTypeNumber) {
                    speed = parameters["speed"].number();
                }
                if (parameters["degree"].type() == kValueTypeNumber) {
                    degree = parameters["degree"].number();
                }

                std::string msg = cocube.rotate_by_degree(direction, speed, degree);
                if (mqtt_client_ != nullptr) {
                    esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                    ESP_LOGI(TAG, "RotateByDegree");
                }
            });

        // Register MoveToTarget method
        methods_.AddMethod("MoveToTarget", "Control robot to move to target point",
                           ParameterList({Parameter("target_x", "Target X coordinate, default is 0", kValueTypeNumber, false),
                                          Parameter("target_y", "Target Y coordinate, default is 0", kValueTypeNumber, false),
                                          Parameter("speed", "Speed (0-50), default is 50", kValueTypeNumber, false)}),
                           [&cocube](const ParameterList& parameters) {
                                // Get MAC address prefix and build dynamic MQTT topic
                                std::string mac_prefix = generate_mac_prefix();
                                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int target_x = 0;
                               int target_y = 0;
                               int speed = 50;

                               if (parameters["target_x"].type() == kValueTypeNumber) {
                                   target_x = parameters["target_x"].number();
                               }
                               if (parameters["target_y"].type() == kValueTypeNumber) {
                                   target_y = parameters["target_y"].number();
                               }
                               if (parameters["speed"].type() == kValueTypeNumber) {
                                   speed = parameters["speed"].number();
                               }

                               std::string msg = cocube.move_to_target(target_x, target_y, speed);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "MoveToTarget");
                               }
                           });

        // Register rotate_to_angle method
        methods_.AddMethod("RotateToAngle", "Control robot to rotate to specified angle",
                           ParameterList({Parameter("angle", "Target rotation angle, default is 0", kValueTypeNumber, false),
                                          Parameter("speed", "Rotation speed (0-50), default is 40", kValueTypeNumber, false)}),
                           [&cocube](const ParameterList& parameters) {
                            // Get MAC address prefix and build dynamic MQTT topic
                                std::string mac_prefix = generate_mac_prefix();
                                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int angle = 0;
                               int speed = 40;

                               if (parameters["angle"].type() == kValueTypeNumber) {
                                   angle = parameters["angle"].number();
                               }
                               if (parameters["speed"].type() == kValueTypeNumber) {
                                   speed = parameters["speed"].number();
                               }

                               std::string msg = cocube.rotate_to_angle(angle, speed);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "RotateToAngle");
                               }
                           });

        // Register rotate_to_target method
        methods_.AddMethod("RotateToTarget", "Control robot to rotate towards target point",
                           ParameterList({Parameter("target_x", "Target X coordinate, default is 0", kValueTypeNumber, false),
                                          Parameter("target_y", "Target Y coordinate, default is 0", kValueTypeNumber, false),
                                          Parameter("speed", "Rotation speed (0-50), default is 30", kValueTypeNumber, false)}),
                           [&cocube](const ParameterList& parameters) {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int target_x = 0;
                               int target_y = 0;
                               int speed = 30;

                               if (parameters["target_x"].type() == kValueTypeNumber) {
                                   target_x = parameters["target_x"].number();
                               }
                               if (parameters["target_y"].type() == kValueTypeNumber) {
                                   target_y = parameters["target_y"].number();
                               }
                               if (parameters["speed"].type() == kValueTypeNumber) {
                                   speed = parameters["speed"].number();
                               }

                               std::string msg = cocube.rotate_to_target(target_x, target_y, speed);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "RotateToTarget");
                               }
                           });

        // Register gripper open method
        methods_.AddMethod("GripperOpen", "Control robot gripper to open", ParameterList(),
                           [&cocube](const ParameterList& parameters) {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               std::string msg = cocube.gripper_open();
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "GripperOpen");
                               }
                           });

        // Register gripper close method
        methods_.AddMethod("GripperClose", "Control robot gripper to close, or shoot ball", ParameterList(),
                           [&cocube](const ParameterList& parameters) {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               std::string msg = cocube.gripper_close();
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "GripperClose");
                               }
                           });

        // Register change color method
        methods_.AddMethod("ChangeColor", "Control robot screen color",
                           ParameterList({
                               Parameter("color_R", "Red component, 0-255, default is 0", kValueTypeNumber, false),
                               Parameter("color_G", "Green component, 0-255, default is 255", kValueTypeNumber, false),
                               Parameter("color_B", "Blue component, 0-255, default is 0", kValueTypeNumber, false)}),
                           [&cocube](const ParameterList& parameters) {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int color_r = 0;
                               int color_g = 255;
                               int color_b = 0;

                               if (parameters["color_R"].type() == kValueTypeNumber) {
                                   color_r = parameters["color_R"].number();
                               }

                               if (parameters["color_G"].type() == kValueTypeNumber) {
                                   color_g = parameters["color_G"].number();
                               }

                               if (parameters["color_B"].type() == kValueTypeNumber) {
                                   color_b = parameters["color_B"].number();
                               }

                               int color = (color_r << 16) | (color_g << 8) | color_b;  // RGB to int

                               std::string msg = cocube.change_color(color);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "ChangeColor");
                               }
                           });

        // Register change screen image method
        methods_.AddMethod("ChangeImage", "Change robot screen display avatar pattern",
                           ParameterList({
                               Parameter("img_num", "Pattern number, 1-7, default is 2, 1 is lion, 2 is polar bear, 3 is panda, 4 is hedgehog, 5 is cat, 6 is sheep, 7 is tiger", kValueTypeNumber, false),
                           }),
                           [&cocube](const ParameterList& parameters) {
                            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
                               int img_num = 2;

                               if (parameters["img_num"].type() == kValueTypeNumber) {
                                   img_num = parameters["img_num"].number();
                               }

                               std::string msg = cocube.change_image(img_num);
                               if (mqtt_client_ != nullptr) {
                                   esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                                   ESP_LOGI(TAG, "ChangeImage");
                               }
                           });

        // Register WheelsStop method
        methods_.AddMethod("WheelsStop", "Stop robot", ParameterList(), [&cocube](const ParameterList& parameters) {
            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
            std::string msg = cocube.wheels_stop();
            if (mqtt_client_ != nullptr) {
                esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                ESP_LOGI(TAG, "WheelsStop");
            }
        });

        // Register shoot method
        methods_.AddMethod("Shoot", "Soccer shoot", ParameterList(), [&cocube](const ParameterList& parameters) {
            // Get MAC address prefix and build dynamic MQTT topic
                std::string mac_prefix = generate_mac_prefix();
                std::string mqtt_pub_topic = "/" + mac_prefix + "/control";  // Build publish topic
            std::string msg = "CoCube shoot";
            if (mqtt_client_ != nullptr) {
                esp_mqtt_client_publish(mqtt_client_, mqtt_pub_topic.c_str(), msg.c_str(), msg.length(), 0, 0);
                ESP_LOGI(TAG, "CoCube shoot");
            }
        });
    }
};

}  // namespace iot

DECLARE_THING(Copilot);