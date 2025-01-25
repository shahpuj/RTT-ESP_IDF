#include "driver/twai.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "BNO055ESP32.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "math.h"


static const char* TAG = "BNO055ESP32Example";



static const char *TAG_TWAI = "TWAI";
static const char *TAG_DATA = "DATA";

// Data to be transmitted
twai_message_t testMessage = {
    .identifier = (uint32_t)10, // IN DECIMAL!!!!! CHANGE TO HEX FOR MOTEC DONT BE RESTARTED!!!!!
    .data_length_code = 8,
    .data = {0, 100, 0, 200, 0, 150, 0, 250},
};
twai_message_t gyroMsg = {
    .identifier = (uint32_t)16,// IN DECIMAL!!!!! CHANGE TO HEX FOR MOTEC DONT BE RESTARTED!!!!!
    .data_length_code = 8,
    .data = {0, 100, 0, 200, 0, 150, 0, 250},
};
twai_message_t accelMsg = {
    .identifier = (uint32_t)17,
    .data_length_code = 8,
    .data = {0, 100, 0, 200, 0, 150, 0, 250},
};

int failCount = 0;
int passCount = 0;
int progCount = 0; 

gpio_num_t bnoRst = GPIO_NUM_32 ; 

void TWAIconfig() {
    // Filter configuration
    twai_filter_config_t filter_config = {
        .acceptance_code = (uint32_t)(0x640 << 21),
        .acceptance_mask = (uint32_t)(0x7FF << 21),
        .single_filter = true};

    // Timing configuration
    twai_timing_config_t timing_config = {
        .clk_src = TWAI_CLK_SRC_DEFAULT,
        .quanta_resolution_hz = 20000000,
        .brp = 0,
        .tseg_1 = 15,
        .tseg_2 = 4,
        .sjw = 3,
        .triple_sampling = false};

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    // General configuration
    twai_general_config_t general_config = {
        .controller_id = 0,
        .mode = TWAI_MODE_NO_ACK,
        .tx_io = GPIO_NUM_17,
        .rx_io = GPIO_NUM_16,
        .clkout_io = (GPIO_NUM_0),
        .bus_off_io = (GPIO_NUM_0),
        .tx_queue_len = (128),
        .rx_queue_len = (128),
        .alerts_enabled = (uint32_t)(TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF),
        .clkout_divider = 0,
        .intr_flags = (1 << 1)};

    // Install TWAI driver
    if (twai_driver_install(&general_config, &t_config, &filter_config) == ESP_OK) {
        ESP_LOGI(TAG_TWAI, "Driver installed");
    } else {
        ESP_LOGE(TAG_TWAI, "Failed to install driver");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG_TWAI, "Driver started");
    } else {
        ESP_LOGE(TAG_TWAI, "Failed to start driver");
        return;
    }
}

void bnoconfig(){
    
    vTaskDelay(100);
    BNO055 bno(UART_NUM_1, GPIO_NUM_33, GPIO_NUM_25);
    try {
        bno.begin();  // BNO055 is in CONFIG_MODE until it is changed
        bno.enableExternalCrystal();
        // bno.setSensorOffsets(storedOffsets);
        // bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
        /* you can specify a PoWeRMode using:
                - setPwrModeNormal(); (Default on startup)
                - setPwrModeLowPower();
                - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
        */
        bno.setOprModeNdof();
        ESP_LOGI(TAG, "Setup Done.");
    } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
        ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
        return;
    } catch (std::exception& ex) {
        ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
        return;
    }

    try {
        int8_t temperature = bno.getTemp();
        ESP_LOGI(TAG, "TEMP: %d°C", temperature);

        int16_t sw = bno.getSWRevision();
        uint8_t bl_rev = bno.getBootloaderRevision();
        ESP_LOGI(TAG, "SW rev: %d, bootloader rev: %u", sw, bl_rev);

        bno055_self_test_result_t res = bno.getSelfTestResult();
        ESP_LOGI(TAG, "Self-Test Results: MCU: %u, GYR:%u, MAG:%u, ACC: %u", res.mcuState, res.gyrState, res.magState,
                 res.accState);
    } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    } catch (std::exception& ex) {
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    }

}

void twai_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    while (true) {
        // Transmit gyroMsg
        if (twai_transmit(&gyroMsg, pdMS_TO_TICKS(1)) == ESP_OK) {
            passCount++;
        } else {
            failCount++;
        }

        vTaskDelay(pdMS_TO_TICKS(5));

        // Transmit accelMsg
        if (twai_transmit(&accelMsg, pdMS_TO_TICKS(1)) == ESP_OK) {
            passCount++;
        } else {
            failCount++;
        }

        // Print statistics every 1000 messages
        if (progCount % 100 == 0) {
            printf("QUEUED = %d, FAILED = %d\n", passCount, failCount);
            progCount = 0;
        }
        esp_task_wdt_reset();

        progCount++;
        vTaskDelay(pdMS_TO_TICKS(5)); // Ensure 10ms loop delay
    }
    // while (true) {
    //    progCount ++ ; 
    //     // Transmit message
    //     if (twai_transmit(&testMessage, pdMS_TO_TICKS(1)) == ESP_OK) {
    //         // ESP_LOGI(TAG_TWAI, "Message transmitted");
    //         passCount++;
    //     } else {
    //         failCount++;
    //     }

    //     // Print statistics
    //     if (progCount % 1000 == 0) {
    //         printf("QUEUED = %d, FAILED = %d\n", passCount, failCount);
    //         esp_task_wdt_reset();
    //         progCount = 0 ; 
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(1));
    // }
}

void fakeData(void *pvParameters) {
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();
    
        // Prepare data for transmission
        // for (int i = 0; i < 8; i++) {
        //     testMessage.data[i] = (testMessage.data[i] + 10) % 256; // Example modification
        // }

        // ESP_LOGI(TAG_DATA, "Data prepared: [%d, %d, %d, %d, %d, %d, %d, %d]",
        //          testMessage.data[0], testMessage.data[1], testMessage.data[2], testMessage.data[3],
        //          testMessage.data[4], testMessage.data[5], testMessage.data[6], testMessage.data[7]);

        // Simulate data preparation time
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void calibrationRead(void *pvParameters) {
    esp_task_wdt_add(NULL);
    BNO055 bno(UART_NUM_1, GPIO_NUM_33, GPIO_NUM_25);

    try {
        bno.begin();  // BNO055 is in CONFIG_MODE until it is changed
        bno.enableExternalCrystal();
        // bno.setSensorOffsets(storedOffsets);
        // bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
        /* you can specify a PoWeRMode using:
                - setPwrModeNormal(); (Default on startup)
                - setPwrModeLowPower();
                - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
        */
        bno.setOprModeNdof();
        ESP_LOGI(TAG, "Setup Done.");
    } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
        ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
        return;
    } catch (std::exception& ex) {
        ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
        return;
    }

    try {
        int8_t temperature = bno.getTemp();
        ESP_LOGI(TAG, "TEMP: %d°C", temperature);

        int16_t sw = bno.getSWRevision();
        uint8_t bl_rev = bno.getBootloaderRevision();
        ESP_LOGI(TAG, "SW rev: %d, bootloader rev: %u", sw, bl_rev);

        bno055_self_test_result_t res = bno.getSelfTestResult();
        ESP_LOGI(TAG, "Self-Test Results: MCU: %u, GYR:%u, MAG:%u, ACC: %u", res.mcuState, res.gyrState, res.magState,
                 res.accState);
    } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    } catch (std::exception& ex) {
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    }

    while (1) {
        try {
            // Calibration 3 = fully calibrated, 0 = not calibrated
            bno055_calibration_t cal = bno.getCalibration();
            bno055_vector_t v = bno.getVectorEuler();
            ESP_LOGI(TAG, "Euler: X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u", v.x, v.y, v.z, cal.sys,
                     cal.gyro, cal.accel, cal.mag);
            if (cal.gyro == 3 && cal.accel == 3 && cal.mag == 3) {
                ESP_LOGI(TAG, "Fully Calibrated.");
                bno.setOprModeConfig();                         // Change to OPR_MODE
                bno055_offsets_t txt = bno.getSensorOffsets();  // NOTE: this must be executed in CONFIG_MODE
                ESP_LOGI(TAG,
                         "\nOffsets:\nAccel: X:%d, Y:%d, Z:%d;\nMag: X:%d, Y:%d, Z:%d;\nGyro: X:%d, Y:%d, Z:%d;\nAccelRadius: "
                         "%d;\nMagRadius: %d;\n",
                         txt.accelOffsetX, txt.accelOffsetY, txt.accelOffsetZ, txt.magOffsetX, txt.magOffsetY, txt.magOffsetZ,
                         txt.gyroOffsetX, txt.gyroOffsetY, txt.gyroOffsetZ, txt.accelRadius, txt.magRadius);
                ESP_LOGI(TAG,
                         "Store this values, place them using setSensorOffsets() after every reset of the BNO055 to avoid the "
                         "calibration process, unluckily MAG requires to be calibrated after every reset, for more information "
                         "check datasheet.");
                break;
            }
            esp_task_wdt_reset();

        } catch (BNO055BaseException& ex) {
            ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
            return;
        } catch (std::exception& ex) {
            ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
    }
   
}

// void sensorRead(void *pvPavameters){

//     // CALCULATED MEMS OFFSETS 
//      bno055_offsets_t storedOffsets = {
//     .accelOffsetX = -38,
//     .accelOffsetY = -38,
//     .accelOffsetZ = -36,
//     .magOffsetX = -107,
//     .magOffsetY = -87,
//     .magOffsetZ = -357,
//     .gyroOffsetX = 1,
//     .gyroOffsetY = -3,
//     .gyroOffsetZ = -1,
//     .accelRadius = 1000,
//     .magRadius = 618
// };

//     esp_task_wdt_add(NULL);
//     BNO055 bno(UART_NUM_1, GPIO_NUM_33, GPIO_NUM_25);
//     try {
//         bno.begin();  // BNO055 is in CONFIG_MODE until it is changed
//         bno.enableExternalCrystal();
//         void setAccelConfig(bno055_accel_range_t range = BNO055_CONF_ACCEL_RANGE_4G,
//                         bno055_accel_bandwidth_t bandwidth = BNO055_CONF_ACCEL_BANDWIDTH_62_5HZ,
//                         bno055_accel_mode_t mode = BNO055_CONF_ACCEL_MODE_NORMAL);
//         void setGyroConfig(bno055_gyro_range_t range = BNO055_CONF_GYRO_RANGE_1000DPS,
//                        bno055_gyro_bandwidth_t bandwidth = BNO055_CONF_GYRO_BANDWIDTH_23HZ,
//                        bno055_gyro_mode_t mode = BNO055_CONF_GYRO_MODE_NORMAL);
    
//        bno.setSensorOffsets(storedOffsets);

//         // bno.setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
//         /* you can specify a PoWeRMode using:
//                 - setPwrModeNormal(); (Default on startup)
//                 - setPwrModeLowPower();
//                 - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
//         */
//         bno.setOprModeNdof();
//         ESP_LOGI(TAG, "Setup Done.");
//     } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
//         ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
//         return;
//     } catch (std::exception& ex) {
//         ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
//         return;
//     }
//     try {
//         int8_t temperature = bno.getTemp();
//         ESP_LOGI(TAG, "TEMP: %d°C", temperature);

//         int16_t sw = bno.getSWRevision();
//         uint8_t bl_rev = bno.getBootloaderRevision();
//         ESP_LOGI(TAG, "SW rev: %d, bootloader rev: %u", sw, bl_rev);

//         bno055_self_test_result_t res = bno.getSelfTestResult();
//         ESP_LOGI(TAG, "Self-Test Results: MCU: %u, GYR:%u, MAG:%u, ACC: %u", res.mcuState, res.gyrState, res.magState,
//                  res.accState);
//     } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
//         ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
//         return;
//     } catch (std::exception& ex) {
//         ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
//         return;
//     }

//     //DATAPULL STRUCTS LOOPED THROUGH AS TEMP MEM
//     bno055_vector_t  linearAccel ; 
//     bno055_vector_t  rotationalAccel ; 

//     while (true) {
//         rotationalAccel = bno.getVectorGyroscope();
//         linearAccel = bno.getVectorLinearAccel();

//     // printf("Linear Acceleration - X: %.2f, Y: %.2f, Z: %.2f\n", 
//     // linearAccel.x, linearAccel.y, linearAccel.z);
//     printf("Rotational Acceleration - X: %.2f, Y: %.2f, Z: %.2f\n", 
//     rotationalAccel.x, rotationalAccel.y, rotationalAccel.z);

//         // Convert accelerometer data to G (4 significant figures)
//         float linearAccelX_g = roundf((linearAccel.x / 9.80665) * 10000) / 10000;
//         float linearAccelY_g = roundf((linearAccel.y / 9.80665) * 10000) / 10000;
//         float linearAccelZ_g = roundf((linearAccel.z / 9.80665) * 10000) / 10000;

//         // Convert gyroscope data to signed 16-bit and populate CAN message
//         gyroMsg.data[0] = (uint8_t)(((int16_t)(rotationalAccel.x * 1000)) >> 8); // High byte
//         gyroMsg.data[1] = (uint8_t)(((int16_t)(rotationalAccel.x * 1000)) & 0xFF); // Low byte
//         gyroMsg.data[2] = (uint8_t)(((int16_t)(rotationalAccel.y * 1000)) >> 8);
//         gyroMsg.data[3] = (uint8_t)(((int16_t)(rotationalAccel.y * 1000)) & 0xFF);
//         gyroMsg.data[4] = (uint8_t)(((int16_t)(rotationalAccel.z * 1000)) >> 8);
//         gyroMsg.data[5] = (uint8_t)(((int16_t)(rotationalAccel.z * 1000)) & 0xFF);
//         gyroMsg.data[6] = 0x00; // Placeholder high byte
//         gyroMsg.data[7] = 0x00; // Placeholder low byte

//         // Convert linear acceleration in G to signed 16-bit and populate CAN message
//         accelMsg.data[0] = (uint8_t)(((int16_t)(linearAccelX_g * 1000)) >> 8); // High byte
//         accelMsg.data[1] = (uint8_t)(((int16_t)(linearAccelX_g * 1000)) & 0xFF); // Low byte
//         accelMsg.data[2] = (uint8_t)(((int16_t)(linearAccelY_g * 1000)) >> 8);
//         accelMsg.data[3] = (uint8_t)(((int16_t)(linearAccelY_g * 1000)) & 0xFF);
//         accelMsg.data[4] = (uint8_t)(((int16_t)(linearAccelZ_g * 1000)) >> 8);
//         accelMsg.data[5] = (uint8_t)(((int16_t)(linearAccelZ_g * 1000)) & 0xFF);
//         accelMsg.data[6] = 0x00; // Placeholder high byte
//         accelMsg.data[7] = 0x00; // Placeholder low byte
//         vTaskDelay(pdMS_TO_TICKS(5)); // 10ms delay
//         esp_task_wdt_reset();
//     }

// }

void sensorRead(void *pvParameters) {
// CALCULATED MEMS OFFSETS FOR SENSOR CALIBRATION
bno055_offsets_t storedOffsets = {
    .accelOffsetX = -38,
    .accelOffsetY = -38,
    .accelOffsetZ = -36,
    .magOffsetX = -107,
    .magOffsetY = -87,
    .magOffsetZ = -357,
    .gyroOffsetX = 1,
    .gyroOffsetY = -3,
    .gyroOffsetZ = -1,
    .accelRadius = 1000,
    .magRadius = 618
};

// INITIALIZE BNO055 SENSOR
esp_task_wdt_add(NULL); // Add watchdog task
BNO055 bno(UART_NUM_1, GPIO_NUM_33, GPIO_NUM_25); // Initialize BNO055 over UART

try {
    bno.begin();  // BNO055 starts in CONFIG_MODE
    bno.enableExternalCrystal(); // Use external crystal for better accuracy

    // SET SENSOR CONFIGURATIONS
    // Accelerometer: Set range to ±4G, bandwidth to 62.5Hz, mode to NORMAL
    bno.setAccelConfig(BNO055_CONF_ACCEL_RANGE_4G,
                       BNO055_CONF_ACCEL_BANDWIDTH_62_5HZ,
                       BNO055_CONF_ACCEL_MODE_NORMAL);

    // Gyroscope: Set range to ±1000°/s, bandwidth to 23Hz, mode to NORMAL
    bno.setGyroConfig(BNO055_CONF_GYRO_RANGE_1000DPS,
                      BNO055_CONF_GYRO_BANDWIDTH_23HZ,
                      BNO055_CONF_GYRO_MODE_NORMAL);

    // LOAD STORED CALIBRATION OFFSETS
    bno.setSensorOffsets(storedOffsets);

    // SET OPERATION MODE TO NDOF (9 Degrees of Freedom for Fused Data)
    bno.setOprModeNdof();
    ESP_LOGI(TAG, "Setup Done.");
} catch (BNO055BaseException &ex) {
    ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
    return;
} catch (std::exception &ex) {
    ESP_LOGE(TAG, "Setup Failed, Error: %s", ex.what());
    return;
}

// VERIFY SENSOR SETUP AND GET BASIC INFO
try {
    int8_t temperature = bno.getTemp();
    ESP_LOGI(TAG, "TEMP: %d°C", temperature);

    int16_t sw = bno.getSWRevision();
    uint8_t bl_rev = bno.getBootloaderRevision();
    ESP_LOGI(TAG, "SW rev: %d, bootloader rev: %u", sw, bl_rev);

    bno055_self_test_result_t res = bno.getSelfTestResult();
    ESP_LOGI(TAG, "Self-Test Results: MCU: %u, GYR: %u, MAG: %u, ACC: %u",
             res.mcuState, res.gyrState, res.magState, res.accState);
} catch (BNO055BaseException &ex) {
    ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
    return;
} catch (std::exception &ex) {
    ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
    return;
}

// TEMPORARY DATA STRUCTURES FOR SENSOR READINGS
bno055_vector_t linearAccel; // Linear acceleration data
bno055_vector_t rotationalAccel; // Gyroscope data

// SENSOR DATA PROCESSING LOOP
while (true) {
    // READ SENSOR DATA
    rotationalAccel = bno.getVectorGyroscope();    // Gyroscope readings in °/s
    linearAccel = bno.getVectorLinearAccel();      // Linear acceleration in m/s^2

    // PRINT RAW SENSOR DATA (FOR DEBUGGING)
    printf("Linear Acceleration - X: %.2f, Y: %.2f, Z: %.2f\n",
           linearAccel.x, linearAccel.y, linearAccel.z);
    printf("Rotational Acceleration - X: %.2f, Y: %.2f, Z: %.2f\n",
           rotationalAccel.x, rotationalAccel.y, rotationalAccel.z);

    // CONVERT LINEAR ACCELERATION TO 'g'
    float linearAccelX_g = roundf((linearAccel.x / 9.80665) * 10000) / 10000;
    float linearAccelY_g = roundf((linearAccel.y / 9.80665) * 10000) / 10000;
    float linearAccelZ_g = roundf((linearAccel.z / 9.80665) * 10000) / 10000;

    // Scale down gyroscope values to prevent overflow
    float scaledGyroX = rotationalAccel.x / 1000.0;
    float scaledGyroY = rotationalAccel.y / 1000.0;
    float scaledGyroZ = rotationalAccel.z / 1000.0;


    // POPULATE GYRO CAN MESSAGE
    gyroMsg.data[0] = (uint8_t)(((int16_t)(rotationalAccel.x * 1000)) >> 8); // High byte of X
    gyroMsg.data[1] = (uint8_t)(((int16_t)(rotationalAccel.x * 1000)) & 0xFF); // Low byte of X
    gyroMsg.data[2] = (uint8_t)(((int16_t)(rotationalAccel.y * 1000)) >> 8); // High byte of Y
    gyroMsg.data[3] = (uint8_t)(((int16_t)(rotationalAccel.y * 1000)) & 0xFF); // Low byte of Y
    gyroMsg.data[4] = (uint8_t)(((int16_t)(rotationalAccel.z * 1000)) >> 8); // High byte of Z
    gyroMsg.data[5] = (uint8_t)(((int16_t)(rotationalAccel.z * 1000)) & 0xFF); // Low byte of Z
    gyroMsg.data[6] = 0x00; // Placeholder
    gyroMsg.data[7] = 0x00; // Placeholder

    // POPULATE ACCELERATION CAN MESSAGE
    accelMsg.data[0] = (uint8_t)(((int16_t)(linearAccelX_g * 1000)) >> 8); // High byte of X
    accelMsg.data[1] = (uint8_t)(((int16_t)(linearAccelX_g * 1000)) & 0xFF); // Low byte of X
    accelMsg.data[2] = (uint8_t)(((int16_t)(linearAccelY_g * 1000)) >> 8); // High byte of Y
    accelMsg.data[3] = (uint8_t)(((int16_t)(linearAccelY_g * 1000)) & 0xFF); // Low byte of Y
    accelMsg.data[4] = (uint8_t)(((int16_t)(linearAccelZ_g * 1000)) >> 8); // High byte of Z
    accelMsg.data[5] = (uint8_t)(((int16_t)(linearAccelZ_g * 1000)) & 0xFF); // Low byte of Z
    accelMsg.data[6] = 0x00; // Placeholder
    accelMsg.data[7] = 0x00; // Placeholder

    // ADD A SMALL DELAY FOR STABLE SAMPLING
    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay
    esp_task_wdt_reset(); // Reset watchdog timer to prevent timeout
}

}






extern "C" void app_main(void) {
    TWAIconfig();
    bnoconfig(); 
    // Create tasks pinned to specific cores
    xTaskCreatePinnedToCore(twai_task, "TWAI Task", 4096, NULL, 16, NULL, 1);            // Pin TWAI task to Core 0
    xTaskCreatePinnedToCore(sensorRead, "Sensor Read", 4096, NULL, 5, NULL, 0); // Pin Sensor Read task to Core 1
}
