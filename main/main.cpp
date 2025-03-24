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

int failCount = 0;
int passCount = 0;
int progCount = 0; 

gpio_num_t bnoRst = GPIO_NUM_32 ; 

uint32_t alerts_to_enable = 
    TWAI_ALERT_TX_IDLE |
    TWAI_ALERT_TX_SUCCESS |
    TWAI_ALERT_RX_DATA |
    TWAI_ALERT_BELOW_ERR_WARN |
    TWAI_ALERT_ERR_ACTIVE |
    TWAI_ALERT_RECOVERY_IN_PROGRESS |
    TWAI_ALERT_BUS_RECOVERED |
    TWAI_ALERT_ARB_LOST |
    TWAI_ALERT_ABOVE_ERR_WARN |
    TWAI_ALERT_BUS_ERROR |
    TWAI_ALERT_TX_FAILED |
    TWAI_ALERT_RX_QUEUE_FULL |
    TWAI_ALERT_ERR_PASS |
    TWAI_ALERT_BUS_OFF;

    uint32_t* alerts = new uint32_t; 
    void twaiErrPrint() {
        if (*alerts & TWAI_ALERT_TX_IDLE) {
            ESP_LOGI(TAG_TWAI, "TX idle: No more messages queued for transmission");
        }
        if (*alerts & TWAI_ALERT_TX_SUCCESS) {
            ESP_LOGI(TAG_TWAI, "TX success: Previous transmission successful");
        }
        if (*alerts & TWAI_ALERT_RX_DATA) {
            ESP_LOGI(TAG_TWAI, "RX data: Frame received and added to RX queue");
        }
        if (*alerts & TWAI_ALERT_BELOW_ERR_WARN) {
            ESP_LOGI(TAG_TWAI, "Below error warning: Error counters dropped below warning limit");
        }
        if (*alerts & TWAI_ALERT_ERR_ACTIVE) {
            ESP_LOGI(TAG_TWAI, "Error active: TWAI controller is error active");
        }
        if (*alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
            ESP_LOGI(TAG_TWAI, "Bus recovery in progress");
        }
        if (*alerts & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG_TWAI, "Bus recovered: Recovery complete");
        }
        if (*alerts & TWAI_ALERT_ARB_LOST) {
            ESP_LOGW(TAG_TWAI, "Arbitration lost during previous transmission");
        }
        if (*alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
            ESP_LOGW(TAG_TWAI, "Above error warning: Error counter exceeded warning level");
        }
        if (*alerts & TWAI_ALERT_BUS_ERROR) {
            ESP_LOGE(TAG_TWAI, "Bus error: Bit/Stuff/CRC/Form/ACK error on bus");
        }
        if (*alerts & TWAI_ALERT_TX_FAILED) {
            ESP_LOGE(TAG_TWAI, "TX failed: Previous transmission failed");
        }
        if (*alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            ESP_LOGW(TAG_TWAI, "RX queue full: Frame lost");
        }
        if (*alerts & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG_TWAI, "Error passive: Controller is in error passive state");
        }
        if (*alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG_TWAI, "BUS OFF: Triggering recovery.");
            twai_initiate_recovery();
        
            // Optional: Wait for recovery to complete
            while (true) {
                uint32_t recoveryAlerts;
                if (twai_read_alerts(&recoveryAlerts, pdMS_TO_TICKS(1000)) == ESP_OK) {
                    if (recoveryAlerts & TWAI_ALERT_BUS_RECOVERED) {
                        ESP_LOGI(TAG_TWAI, "Recovery complete.");
                        break;
                    }
                }
            }
        }
        
    }
    
void TWAIconfig() {
    
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
    ESP_ERROR_CHECK(twai_reconfigure_alerts(alerts_to_enable, NULL));
    ESP_LOGI(TAG, "TWAI alerts configured");


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

}
esp_err_t res; 
esp_err_t errStatus; 

void twai_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    while (true) {
        // Transmit gyroMsg
        errStatus = twai_transmit(&gyroMsg, pdMS_TO_TICKS(1));
        

        vTaskDelay(pdMS_TO_TICKS(5));

        // Transmit accelMsg
        errStatus = twai_transmit(&accelMsg, pdMS_TO_TICKS(1));

        if (errStatus == ESP_OK) {
            passCount++;
        } else {
            ESP_LOGE(TAG_TWAI,"Gyro Tx Failed: %s",esp_err_to_name(errStatus));
            failCount++;
        }

        // Print statistics every 100 messages
        if (progCount % 100 == 0) {
            printf("QUEUED = %d, FAILED = %d\n", passCount, failCount);
            progCount = 0;
            res = twai_read_alerts(alerts,pdMS_TO_TICKS(0));
            if(res == ESP_ERR_TIMEOUT){
                ESP_LOGI(TAG_TWAI,"BUS OK"); 
            }
            else if(res == ESP_OK){
                twaiErrPrint(); 
            }
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
    bno.setOprModeIMU();
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
bool gyroInvalid; 
bool accelInvalid; 
// SENSOR DATA PROCESSING LOOP
while (true) {
    // READ SENSOR DATA
    rotationalAccel = bno.getVectorGyroscope();    // Gyroscope readings in °/s
    linearAccel = bno.getVectorLinearAccel();      // Linear acceleration in m/s^2
    bool gyroInvalid = isnan(rotationalAccel.x) || isnan(rotationalAccel.y) || isnan(rotationalAccel.z);
    bool accelInvalid = isnan(linearAccel.x) || isnan(linearAccel.y) || isnan(linearAccel.z);
    if (gyroInvalid) {
        ESP_LOGW(TAG, "Invalid gyro data detected. Forcing -1 values.");
        for (int i = 0; i < 6; i++) gyroMsg.data[i] = 0xFF;  // -1 in int16_t is 0xFFFF
    }
    if (accelInvalid) {
        ESP_LOGW(TAG, "Invalid accel data detected. Forcing -1 values.");
        for (int i = 0; i < 6; i++) accelMsg.data[i] = 0xFF;
    }

    
    // PRINT RAW SENSOR DATA (FOR DEBUGGING)
    printf("Linear Acceleration - X: %.2f, Y: %.2f, Z: %.2f\n",
           linearAccel.x, linearAccel.y, linearAccel.z);
    printf("Rotational Acceleration - X: %.2f, Y: %.2f, Z: %.2f\n",
           rotationalAccel.x, rotationalAccel.y, rotationalAccel.z);

           if (!gyroInvalid) {
            float gx = fminf(fmaxf(rotationalAccel.x, -327.67f), 327.67f);
            float gy = fminf(fmaxf(rotationalAccel.y, -327.67f), 327.67f);
            float gz = fminf(fmaxf(rotationalAccel.z, -327.67f), 327.67f);
            
            int16_t gx_i = (int16_t)(gx * 100);
            int16_t gy_i = (int16_t)(gy * 100);
            int16_t gz_i = (int16_t)(gz * 100);
        
            gyroMsg.data[0] = gx_i >> 8;
            gyroMsg.data[1] = gx_i & 0xFF;
            gyroMsg.data[2] = gy_i >> 8;
            gyroMsg.data[3] = gy_i & 0xFF;
            gyroMsg.data[4] = gz_i >> 8;
            gyroMsg.data[5] = gz_i & 0xFF;
    }
    // CONVERT LINEAR ACCELERATION TO 'g'
    if (!accelInvalid) {
        float ax = roundf((linearAccel.x / 9.80665f) * 1000.0f);
        float ay = roundf((linearAccel.y / 9.80665f) * 1000.0f);
        float az = roundf((linearAccel.z / 9.80665f) * 1000.0f);
    
        ax = fminf(fmaxf(ax, -32767.0f), 32767.0f);
        ay = fminf(fmaxf(ay, -32767.0f), 32767.0f);
        az = fminf(fmaxf(az, -32767.0f), 32767.0f);
    
        int16_t ax_i = (int16_t)ax;
        int16_t ay_i = (int16_t)ay;
        int16_t az_i = (int16_t)az;
    
        accelMsg.data[0] = ax_i >> 8;
        accelMsg.data[1] = ax_i & 0xFF;
        accelMsg.data[2] = ay_i >> 8;
        accelMsg.data[3] = ay_i & 0xFF;
        accelMsg.data[4] = az_i >> 8;
        accelMsg.data[5] = az_i & 0xFF;
    }
    
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
