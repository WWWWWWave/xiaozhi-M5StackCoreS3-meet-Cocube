参考如下：https://www.hackster.io/cocube/cocube-meets-m5-cores3-an-ai-chat-robot-52be0f
由于示例中使用IOT功能，在目前的mcp版本中已经弃用，因此需要对代码进行修改，并将其直接加入到小智M5StackCoreS3的板级初始化文件中
源代码请见Copilot.cc

将代码18-287行均复制到m5stack_core_s3.cc中，放在文件起始位置即可
修改mqtt_message_handler_cb()函数，示例如下：
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
        
        以下五行为添加的代码，目的是使用 string::replace 替换中文逗号  
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




然后添加mcpaddtool函数如下：

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

即可实现功能