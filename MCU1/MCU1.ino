/*
 * @file MCU1.ino
 * @brief 智慧农业终端主控制器 - MCU1
 * @description 负责传感器数据采集、GDD计算、系统健康监控和设备控制
 * 
 * 主要功能：
 * 1. DHT11温湿度传感器数据采集
 * 2. 基于GDD(Growing Degree Days)模型的农作物成熟度预测
 * 3. 超声波传感器距离检测与继电器控制
 * 4. 用户按钮交互（OLED控制、系统重启、GDD重置）
 * 5. 与MCU2的串口通信和数据同步
 * 6. EEPROM数据持久化存储
 * 7. 时间加速模式支持
 * 8. 系统健康状态监控
 */

#include <DHT.h>
#include <Wire.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>

// ====== 硬件引脚定义 ======
#define DHT_PIN            2       // 温湿度传感器数据引脚
#define DHT_TYPE           DHT11   // 传感器型号：DHT11
#define ULTRASONIC_TRIG    3       // 超声波传感器触发引脚
#define ULTRASONIC_ECHO    4       // 超声波传感器回声引脚
#define RELAY_CONTROL      5       // 继电器控制引脚
#define BUTTON_INPUT       6       // 用户按钮输入引脚
#define COMM_RX_PIN        10      // 与MCU2通信接收引脚
#define COMM_TX_PIN        11      // 与MCU2通信发送引脚
#define SERIAL_BAUD_RATE   9600    // 串口通信波特率

// ====== 系统时间间隔定义 ======
#define SENSOR_SAMPLE_INTERVAL    5000UL   // 传感器采样周期（5秒）
#define STATUS_REPORT_INTERVAL    10000UL   // 系统状态报告周期
#define TIME_SYNC_INTERVAL        60000UL  // 时间同步周期（正常模式，与MCU2保持一致）
#define ULTRASONIC_CHECK_INTERVAL 500UL    // 超声波检测间隔（500毫秒）

// ====== 农作物生长参数 ======
#define GDD_BASE_TEMPERATURE      10.0     // GDD计算基准温度（摄氏度）
#define CROP_TOTAL_GDD_REQUIRED   500.0    // 作物成熟所需总GDD值
#define CROP_MATURITY_THRESHOLD   0.8      // 作物成熟度阈值（80%）

// ====== 按钮交互定义 ======
#define BUTTON_SHORT_PRESS_MAX    1000     // 短按最大时长（毫秒）
#define BUTTON_LONG_PRESS_MIN     1000     // 长按最小时长（毫秒）
#define BUTTON_RESET_PRESS_MIN    3000     // 重置功能最小按压时长（毫秒）

// ====== 时间加速控制 ======
bool isTimeAccelerationEnabled = false;    // 时间加速模式开关
float timeAccelerationFactor = 100.0;      // 时间加速倍数（默认100倍）
unsigned long lastTimeAcceleration = 0;    // 上次时间加速更新时间

// ====== 硬件对象初始化 ======
DHT dhtSensor(DHT_PIN, DHT_TYPE);                    // 温湿度传感器对象
SoftwareSerial mcuComm(COMM_RX_PIN, COMM_TX_PIN);   // MCU间通信串口对象

// ====== GDD计算相关变量 ======
// GDD(Growing Degree Days)生长度日模型：用于预测农作物成熟时间的核心算法
// 计算公式：日GDD = (日最高温 + 日最低温)/2 - 基准温度(10°C)
float gddAccumulated = 0.0;     // 累计生长度日值，跟踪作物总生长进度
float dailyTempMax = -100.0;    // 当日最高温度记录（初始值为极小值便于比较更新）
float dailyTempMin = 100.0;     // 当日最低温度记录（初始值为极大值便于比较更新）
float previousDayTempMax = -100.0; // 前一天最高温度，用于跨日数据估算和备份
float previousDayTempMin = 100.0;  // 前一天最低温度，用于跨日数据估算和备份
int lastRecordedDay = -1;       // 上次记录GDD的日期，防止重复计算同一天的GDD

// ====== 用户交互状态 ======
bool isOledDisplayEnabled = true;       // OLED显示屏开关状态
bool isButtonPressed = false;           // 按钮当前按下状态
unsigned long buttonPressStartTime = 0; // 按钮按下开始时间

// ====== 系统定时器 ======
unsigned long lastSensorSample = 0;    // 上次传感器采样时间
unsigned long lastTimeSync = 0;        // 上次时间同步时间
unsigned long lastStatusReport = 0;    // 上次状态报告时间

// ====== 系统健康监控 ======
// 系统状态枚举：用于监控整体系统健康状况
typedef enum {
    SYSTEM_HEALTHY = 1,     // 系统健康：所有传感器正常工作
    SYSTEM_WARNING = 2,     // 系统警告：部分传感器出现故障但不影响核心功能
    SYSTEM_CRITICAL = 3     // 系统严重故障：关键传感器失效，影响系统正常运行
} SystemStatus_t;

SystemStatus_t currentSystemStatus = SYSTEM_HEALTHY;  // 当前系统状态
uint8_t dhtSensorFailCount = 0;         // DHT传感器连续失败次数（超过5次为严重故障）
uint8_t ultrasonicFailCount = 0;        // 超声波传感器连续失败次数（超过5次为严重故障）
uint8_t sensorFailureCount = 0;         // 传感器总失败次数统计

// ====== 超声波距离检测 ======
#define DISTANCE_BUFFER_SIZE 5              // 距离数据滑动窗口大小（用于中值滤波）
#define PROXIMITY_THRESHOLD 50.0            // 接近检测阈值（厘米），小于此值触发继电器

// ====== 农作物生长天数记录 ======
int growthDays = 0;                         // 累计生长天数计数器
// 距离数据滑动窗口：用于超声波传感器数据的中值滤波，减少噪声干扰
float distanceBuffer[DISTANCE_BUFFER_SIZE] = {1000, 1000, 1000, 1000, 1000};
uint8_t distanceBufferIndex = 0;        // 距离缓冲区当前索引（循环使用）
bool isRelayActive = false;             // 继电器当前激活状态（true=激活，false=关闭）

/**
 * @brief 计算基于时间加速因子的动态间隔
 * @param baseInterval 基础时间间隔（毫秒）
 * @return 调整后的时间间隔（毫秒）
 */
unsigned long getAcceleratedInterval(unsigned long baseInterval) {
    if (isTimeAccelerationEnabled) {
        // 时间加速时，间隔应该缩短相应倍数，但设置最小间隔防止MCU2处理不过来
        unsigned long acceleratedInterval = (unsigned long)(baseInterval / timeAccelerationFactor);
        const unsigned long MIN_INTERVAL = 50; // 最小间隔50ms，防止数据发送过快
        return (acceleratedInterval < MIN_INTERVAL) ? MIN_INTERVAL : acceleratedInterval;
    }
    return baseInterval;
}

/**
 * @brief 系统初始化函数
 * @description 初始化所有硬件组件、通信接口和系统参数
 */
void setup() {
    Serial.begin(9600);
    mcuComm.begin(9600);
    dhtSensor.begin();
    
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(RELAY_CONTROL, OUTPUT);
    pinMode(BUTTON_INPUT, INPUT_PULLUP);
    
    digitalWrite(RELAY_CONTROL, LOW);
    
    // 设置系统初始时间（2025年5月24日 15:00:00）
    setTime(15, 0, 0, 24, 5, 2025);
    
    // 初始化超声波滑动窗口
    for (int i = 0; i < DISTANCE_BUFFER_SIZE; i++) {
        distanceBuffer[i] = 100.0; // 默认距离值
    }
    
    // 从EEPROM加载保存的农业数据
    restoreDataFromEEPROM();
    
    Serial.println(F("[SYSTEM] MCU1 Smart Agriculture Terminal Initialized"));
    Serial.println(F("[SYSTEM] All subsystems ready for operation"));
    Serial.print(F("[SYSTEM] Loaded GDD: "));
    Serial.print(gddAccumulated, 2);
    Serial.print(F(", Maturity: "));
    Serial.print(getCropMaturityPercentage(), 1);
    Serial.println(F("%"));
    
    // 输出当前系统时间以确认时间设置正确
    Serial.print(F("[SYSTEM] Current time: "));
    printFormattedDateTime();
    Serial.println();
    
    // 立即发送初始数据到MCU2，避免显示器长时间等待
    delay(2000); // 等待MCU2初始化完成
    
    // 多次发送初始数据确保MCU2能够接收到
    for (int i = 0; i < 3; i++) {
        transmitDataToMCU2();
        transmitStatusReport();
        delay(500); // 每次发送间隔500ms
        Serial.print(F("[INIT] Initial data sent to MCU2 - attempt "));
        Serial.println(i + 1);
    }
    
    // 输出串口命令帮助信息
    Serial.println(F("\n[SERIAL_CMD] 可用的串口监视器命令:"));
    Serial.println(F("  ACCEL_ON        - 启用时间加速"));
    Serial.println(F("  ACCEL_OFF       - 禁用时间加速"));
    Serial.println(F("  ACCEL_FACTOR=X  - 设置加速因子 (1-86400)"));
    Serial.println(F("  STATUS          - 显示当前系统状态"));
    Serial.println(F("  CROP_REPORT     - 显示每日农作物生长报告"));
    Serial.println(F("  HELP            - 显示帮助信息"));
    Serial.println(F("请输入任意命令到串口监视器以控制 MCU1\n"));    
}

/**
 * @brief 初始化GPIO引脚配置
 */
void initializeGPIOPins() {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(RELAY_CONTROL, OUTPUT);
    pinMode(BUTTON_INPUT, INPUT_PULLUP);
    
    // 确保继电器初始状态为关闭
    digitalWrite(RELAY_CONTROL, LOW);
}

/**
 * @brief 从EEPROM恢复持久化数据
 */
void restoreDataFromEEPROM() {
    // 恢复GDD累计值
    EEPROM.get(0, gddAccumulated);
    if (isnan(gddAccumulated) || gddAccumulated < 0) {
        gddAccumulated = 0.0;
        Serial.println(F("[EEPROM] GDD value invalid, reset to 0"));
    }
    
    // 恢复当日最高温度，添加合理范围检查
    EEPROM.get(sizeof(gddAccumulated), dailyTempMax);
    if (isnan(dailyTempMax) || dailyTempMax < -50.0 || dailyTempMax > 80.0) {
        dailyTempMax = -100.0;  // 重置为合理的初始值
        Serial.println(F("[EEPROM] Daily max temperature invalid, reset to -100°C"));
    }
    
    // 恢复当日最低温度，添加合理范围检查
    EEPROM.get(sizeof(gddAccumulated) + sizeof(dailyTempMax), dailyTempMin);
    if (isnan(dailyTempMin) || dailyTempMin < -50.0 || dailyTempMin > 80.0) {
        dailyTempMin = 100.0;
        Serial.println(F("[EEPROM] Daily min temperature invalid, reset to 100°C"));
    }
    
    // 读取前一天最高温度
    EEPROM.get(sizeof(gddAccumulated) + sizeof(dailyTempMax) + sizeof(dailyTempMin) + sizeof(int), previousDayTempMax);
    if (isnan(previousDayTempMax) || previousDayTempMax < -50.0 || previousDayTempMax > 80.0) {
        previousDayTempMax = -100.0;
        Serial.println(F("[EEPROM] Previous day max temperature invalid, reset to -100°C"));
    }
    
    // 读取前一天最低温度
    EEPROM.get(sizeof(gddAccumulated) + sizeof(dailyTempMax) + sizeof(dailyTempMin) + sizeof(int) + sizeof(float), previousDayTempMin);
    if (isnan(previousDayTempMin) || previousDayTempMin < -50.0 || previousDayTempMin > 80.0) {
        previousDayTempMin = 100.0;
        Serial.println(F("[EEPROM] Previous day min temperature invalid, reset to 100°C"));
    }
    
    // 输出恢复的数据用于调试
    Serial.print(F("[EEPROM] Restored - GDD: "));
    Serial.print(gddAccumulated, 2);
    Serial.print(F(", Max Temp: "));
    Serial.print(dailyTempMax, 1);
    Serial.print(F(", Min Temp: "));
    Serial.println(dailyTempMin, 1);
}

/**
 * @brief 主循环函数
 * @description 系统主要控制逻辑，包括时间管理、数据采集、通信处理等
 */
void loop() {
    unsigned long currentTime = millis();
    
    // 统一时间管理：支持动态切换加速模式
    handleUnifiedTimeManagement(currentTime);
    
    // 处理来自串口监视器的命令（用于调试和控制）
    if (Serial.available()) {
        String serialCommand = Serial.readStringUntil('\n');
        serialCommand.trim();
        if (serialCommand.length() > 0) {
            Serial.print(F("[SERIAL_CMD] Received: "));
            Serial.println(serialCommand);
            
            // 处理串口命令
            if (serialCommand.startsWith("CMD|")) {
                processRemoteCommand(serialCommand);
            } else {
                // 简化命令格式，支持直接输入命令
                if (serialCommand.equalsIgnoreCase("ACCEL_ON")) {
                    processRemoteCommand("CMD|TIME_ACCEL|ON");
                } else if (serialCommand.equalsIgnoreCase("ACCEL_OFF")) {
                    processRemoteCommand("CMD|TIME_ACCEL|OFF");
                } else if (serialCommand.startsWith("ACCEL_FACTOR=")) {
                    String factor = serialCommand.substring(13);
                    processRemoteCommand("CMD|TIME_ACCEL_FACTOR|" + factor);
                } else if (serialCommand.equalsIgnoreCase("STATUS")) {
                    // 显示当前状态
                    Serial.print(F("[STATUS] Time Acceleration: "));
                    Serial.println(isTimeAccelerationEnabled ? F("ON") : F("OFF"));
                    Serial.print(F("[STATUS] Acceleration Factor: "));
                    Serial.println(timeAccelerationFactor, 0);
                    Serial.print(F("[STATUS] Current Time: "));
                    printFormattedDateTime();
                    Serial.println();
                    Serial.print(F("[STATUS] GDD Accumulated: "));
                    Serial.println(gddAccumulated, 2);
                } else if (serialCommand.equalsIgnoreCase("CROP_REPORT")) {
                    // 手动触发每日农作物生长报告
                    outputDailyCropInfo();
                } else if (serialCommand.equalsIgnoreCase("HELP")) {
                    // 显示帮助信息
                    Serial.println(F("[HELP] Available commands:"));
                    Serial.println(F("  ACCEL_ON        - Enable time acceleration"));
                    Serial.println(F("  ACCEL_OFF       - Disable time acceleration"));
                    Serial.println(F("  ACCEL_FACTOR=X  - Set acceleration factor (1-86400)"));
                    Serial.println(F("  STATUS          - Show current system status"));
                    Serial.println(F("  CROP_REPORT     - Show daily crop growth report"));
                    Serial.println(F("  HELP            - Show this help message"));
                    Serial.println(F("  CMD|...         - Send raw command to MCU1"));
                } else {
                    Serial.println(F("[SERIAL_CMD] Unknown command. Type HELP for available commands."));
                }
            }
        }
    }
    
    // 处理来自MCU2的指令
    if (mcuComm.available()) {
        String receivedMessage = mcuComm.readStringUntil('\n');
        receivedMessage.trim();
        if (receivedMessage.length() > 0) {
            // MCU2通信时间记录
            static unsigned long lastMCU2ContactTime = millis();
            lastMCU2ContactTime = millis();
            
            if (receivedMessage.startsWith("TIME|")) {
                // MCU2发送时间给MCU1进行同步（根据项目要求第9条）
                synchronizeTime(receivedMessage.substring(5));
                // 注意：不再回复时间给MCU2，因为MCU2是时间权威
            } else if (receivedMessage.startsWith("CMD|")) {
                processRemoteCommand(receivedMessage);
            } else if (receivedMessage.startsWith("HEARTBEAT|")) {
                // 处理MCU2的心跳信号
                Serial.println(F("[MCU1] Received heartbeat from MCU2"));
            }
        }
    }
    
    // 定时采样传感器并计算GDD - 支持时间加速
    unsigned long acceleratedSensorInterval = getAcceleratedInterval(SENSOR_SAMPLE_INTERVAL);
    if (millis() - lastSensorSample >= acceleratedSensorInterval) {
        lastSensorSample = millis();
        performSensorSampling();
        saveDataToEEPROM();
        
        // 时间加速模式下输出调试信息
        if (isTimeAccelerationEnabled) {
            Serial.print(F("[SENSOR_ACCEL] Sampling interval: "));
            Serial.print(acceleratedSensorInterval);
            Serial.println(F("ms"));
        }
    }
    
    // 定时发送系统状态 - 支持时间加速
    unsigned long acceleratedReportInterval = getAcceleratedInterval(STATUS_REPORT_INTERVAL);
    if (millis() - lastStatusReport >= acceleratedReportInterval) {
        lastStatusReport = millis();
        transmitDataToMCU2();
        transmitStatusReport();
        
        // 时间加速模式下输出调试信息
        if (isTimeAccelerationEnabled) {
            Serial.print(F("[REPORT_ACCEL] Report interval: "));
            Serial.print(acceleratedReportInterval);
            Serial.println(F("ms"));
        }
    }
    
    // 每天输出当天日期和农作物后期需要的生长天数
    static int lastOutputDay = -1;
    int currentDay = day();
    if (currentDay != lastOutputDay) {
        lastOutputDay = currentDay;
        outputDailyCropInfo();
    }
    
    // 超声波距离检测和继电器控制 - 支持时间加速
    static unsigned long lastUltrasonicCheck = 0;
    unsigned long acceleratedUltrasonicInterval = getAcceleratedInterval(ULTRASONIC_CHECK_INTERVAL);
    if (millis() - lastUltrasonicCheck >= acceleratedUltrasonicInterval) {
        lastUltrasonicCheck = millis();
        
        // 时间加速模式下输出调试信息
        if (isTimeAccelerationEnabled) {
            Serial.print(F("[ULTRASONIC_ACCEL] Check interval: "));
            Serial.print(acceleratedUltrasonicInterval);
            Serial.println(F("ms"));
        }
        monitorProximityAndControlRelay();
    }
    
    // 处理按钮事件
    handleButtonInteraction();
}

/**
 * @brief 统一时间管理函数
 * @details 支持动态切换正常模式和加速模式，确保两个MCU时间同步
 */
void handleUnifiedTimeManagement(unsigned long currentTime) {
    if (isTimeAccelerationEnabled) {
        // 优化的时间加速模式：每秒按加速倍数增加时间
        if (currentTime - lastTimeAcceleration >= 1000) {
            lastTimeAcceleration = currentTime;
            
            // 修复：正确的时间加速计算 - 每秒增加timeAccelerationFactor秒
            static int cachedTimeIncrement = 0;
            static float lastCachedFactor = 0;
            
            if (timeAccelerationFactor != lastCachedFactor) {
                // 每秒增加timeAccelerationFactor秒，减去正常流逝的1秒
                cachedTimeIncrement = (int)(timeAccelerationFactor - 1);
                lastCachedFactor = timeAccelerationFactor;
            }
            
            // 记录加速前的日期
            int previousDay = day();
            
            adjustTime(cachedTimeIncrement);
            
            // 检查是否跨日，如果跨日则触发GDD计算
            int currentDay = day();
            if (currentDay != previousDay) {
                Serial.println(F("[TIME_ACCEL] Day changed, triggering GDD calculation"));
                calculateAndAccumulateGDD();
                prepareForNewDay(currentDay);
                saveDataToEEPROM();
            }
            
            // 优化：减少串口输出频率，每10秒输出一次状态
            static unsigned long lastStatusOutput = 0;
            if (currentTime - lastStatusOutput >= 10000) {
                lastStatusOutput = currentTime;
                Serial.print(F("[TIME_ACCEL] "));
                printFormattedDateTime();
                Serial.print(F(" (x"));
                Serial.print(timeAccelerationFactor, 0);
                Serial.println(F(")"));
            }
        }
    } else {
        // 正常模式：优化的时间同步管理
        // MCU2每60秒主动发送时间，MCU1被动接收并同步
        // 这里只需要维护本地时间的正常流逝
        static unsigned long lastNormalTimeUpdate = 0;
        if (currentTime - lastNormalTimeUpdate >= 1000) {
            lastNormalTimeUpdate = currentTime;
            // 正常模式下时间由系统时钟自动维护，无需手动调整
            
            // 优化：减少不必要的同步检查，MCU2会主动发送
            static unsigned long lastSyncCheck = 0;
            if (currentTime - lastSyncCheck >= TIME_SYNC_INTERVAL) {
                lastSyncCheck = currentTime;
                Serial.println(F("[TIME_SYNC] Waiting for MCU2 time synchronization"));
            }
        }
    }
}

/**
 * @brief 处理MCU间通信
 */
void handleCommunication() {
    if (mcuComm.available()) {
        String receivedMessage = mcuComm.readStringUntil('\n');
        receivedMessage.trim();
        
        if (receivedMessage.startsWith("TIME|")) {
            synchronizeTime(receivedMessage);
        } else if (receivedMessage.startsWith("CMD|")) {
            processRemoteCommand(receivedMessage);
        }
    }
}

/**
 * @brief 优化的时间同步接收函数
 * @param timeString 时间字符串 (格式: TIME|YYYY-MM-DD HH:MM:SS)
 * @details 根据项目要求第9条，MCU2每60秒告诉MCU1现在的日期和时间
 * @note MCU2是时间权威，MCU1接收并同步时间，优化了解析效率和错误处理
 */
void synchronizeTime(const String &timeString) {
    // 优化：提取实际时间部分，去除"TIME|"前缀
    String actualTimeString = timeString.substring(5); // 跳过"TIME|"
    
    Serial.print(F("[TIME_SYNC] Received time from MCU2: "));
    Serial.println(actualTimeString);
    
    // 修复：支持多种时间字符串格式
    // 标准格式: "YYYY-MM-DD HH:MM:SS" (19字符)
    // 简化格式: "MM-DD HH:MM:SS" (14字符)
    bool isStandardFormat = (actualTimeString.length() == 19 && 
                            actualTimeString.charAt(4) == '-' && actualTimeString.charAt(7) == '-' &&
                            actualTimeString.charAt(10) == ' ' && actualTimeString.charAt(13) == ':' &&
                            actualTimeString.charAt(16) == ':');
    
    bool isSimplifiedFormat = (actualTimeString.length() == 14 && 
                              actualTimeString.charAt(2) == '-' && actualTimeString.charAt(5) == ' ' &&
                              actualTimeString.charAt(8) == ':' && actualTimeString.charAt(11) == ':');
    
    if (!isStandardFormat && !isSimplifiedFormat) {
        Serial.print(F("[TIME_SYNC] Invalid time string format: "));
        Serial.println(actualTimeString);
        return;
    }
    
    // 优化：减少调试输出，仅在需要时显示
    #ifdef DEBUG_TIME_SYNC
    Serial.print(F("[TIME_SYNC] MCU1 time before sync: "));
    printFormattedDateTime();
    Serial.println();
    #endif
    
    // 修复：根据格式类型解析时间
    int y, mo, d, h, mi, s;
    
    if (isStandardFormat) {
        // 标准格式: "YYYY-MM-DD HH:MM:SS"
        y = actualTimeString.substring(0, 4).toInt();
        mo = actualTimeString.substring(5, 7).toInt();
        d = actualTimeString.substring(8, 10).toInt();
        h = actualTimeString.substring(11, 13).toInt();
        mi = actualTimeString.substring(14, 16).toInt();
        s = actualTimeString.substring(17, 19).toInt();
    } else {
        // 简化格式: "MM-DD HH:MM:SS" - 使用当前年份
        y = year(); // 保持当前年份
        mo = actualTimeString.substring(0, 2).toInt();
        d = actualTimeString.substring(3, 5).toInt();
        h = actualTimeString.substring(6, 8).toInt();
        mi = actualTimeString.substring(9, 11).toInt();
        s = actualTimeString.substring(12, 14).toInt();
    }
    
    // 优化：更高效的范围验证
    if (y < 2020 || y > 2030 || mo < 1 || mo > 12 || 
        d < 1 || d > 31 || h > 23 || mi > 59 || s > 59) {
        Serial.println(F("[TIME_SYNC] Invalid time values parsed"));
        return;
    }
    
    setTime(h, mi, s, d, mo, y);
    
    Serial.print(F("[TIME_SYNC] MCU1 synchronized to: "));
    printFormattedDateTime();
    Serial.println();
    Serial.println(F("[TIME_SYNC] MCU1 successfully synchronized with MCU2 time"));
}

/**
 * @brief 处理按钮交互
 */
void handleButtonInteraction() {
    bool currentButtonState = !digitalRead(BUTTON_INPUT); // 按钮按下为LOW
    
    if (currentButtonState && !isButtonPressed) {
        // 按钮刚被按下
        isButtonPressed = true;
        buttonPressStartTime = millis();
    } else if (!currentButtonState && isButtonPressed) {
        // 按钮刚被释放
        isButtonPressed = false;
        unsigned long pressDuration = millis() - buttonPressStartTime;
        
        if (pressDuration < BUTTON_SHORT_PRESS_MAX) {
            // 短按：切换OLED状态
            isOledDisplayEnabled = !isOledDisplayEnabled;
            Serial.print(F("[BUTTON] OLED "));
            Serial.println(isOledDisplayEnabled ? F("ENABLED") : F("DISABLED"));
            
            // 向MCU2发送OLED控制命令
            mcuComm.print("CMD|OLED|");
            mcuComm.println(isOledDisplayEnabled ? "ON" : "OFF");
            Serial.print(F("[COMMAND_TX] OLED command sent to MCU2: "));
            Serial.println(isOledDisplayEnabled ? "ON" : "OFF");
        } else if (pressDuration >= BUTTON_LONG_PRESS_MIN && pressDuration < BUTTON_RESET_PRESS_MIN) {
            // 长按：系统重启
            Serial.println(F("[BUTTON] System reboot initiated"));
            delay(100);
            performSystemReboot();
        } else if (pressDuration >= BUTTON_RESET_PRESS_MIN) {
            // 超长按：重置GDD
            Serial.println(F("[BUTTON] GDD reset initiated"));
            executeGDDReset();
        }
    }
}

/**
 * @brief 处理来自MCU2的远程指令
 * @param command 接收到的指令字符串
 */
void processRemoteCommand(const String &command) {
    Serial.print(F("[COMMAND] Received: "));
    Serial.println(command);
    
    if (command.startsWith("CMD|RESET_GDD")) {
        executeGDDReset();
        mcuComm.println("RESP|GDD_RESET|SUCCESS");
        Serial.println(F("[COMMAND] GDD reset executed"));
    }
    else if (command.startsWith("CMD|OLED|ON")) {
        isOledDisplayEnabled = true;
        Serial.println(F("[COMMAND] OLED enabled remotely"));
        mcuComm.println("RESP|OLED|ON|SUCCESS");
    }
    else if (command.startsWith("CMD|OLED|OFF")) {
        isOledDisplayEnabled = false;
        Serial.println(F("[COMMAND] OLED disabled remotely"));
        mcuComm.println("RESP|OLED|OFF|SUCCESS");
    }
    else if (command.startsWith("CMD|SYSTEM_REBOOT")) {
        Serial.println(F("[COMMAND] System reboot initiated remotely"));
        delay(100);
        performSystemReboot();
    }
    else if (command.startsWith("CMD|TIME_ACCEL|ON")) {
        isTimeAccelerationEnabled = true;
        lastTimeAcceleration = millis();
        Serial.println(F("[COMMAND] Time acceleration enabled remotely"));
        mcuComm.println("RESP|TIME_ACCEL|ON|SUCCESS");
        
        Serial.println(F("[SYNC] Notifying MCU2 to enable time acceleration"));
        mcuComm.println("CMD|TIME_ACCEL|ON");
    }
    else if (command.startsWith("CMD|TIME_ACCEL|OFF")) {
        isTimeAccelerationEnabled = false;
        Serial.println(F("[COMMAND] Time acceleration disabled remotely"));
        mcuComm.println("RESP|TIME_ACCEL|OFF|SUCCESS");
        
        Serial.println(F("[SYNC] Notifying MCU2 to disable time acceleration"));
        mcuComm.println("CMD|TIME_ACCEL|OFF");
    }
    else if (command.startsWith("CMD|TIME_ACCEL_FACTOR|")) {
        float newFactor = command.substring(22).toFloat();
        if (newFactor >= 1.0 && newFactor <= 86400.0) {
            timeAccelerationFactor = newFactor;
            Serial.print(F("[COMMAND] Time acceleration factor set to: "));
            Serial.println(timeAccelerationFactor, 0);
            mcuComm.println("RESP|TIME_ACCEL_FACTOR|SUCCESS");
            
            Serial.println(F("[SYNC] Notifying MCU2 of new acceleration factor"));
            mcuComm.print("CMD|TIME_ACCEL_FACTOR|");
            mcuComm.println(newFactor, 0);
        } else {
            Serial.println(F("[COMMAND] Invalid acceleration factor (must be 1-86400)"));
            mcuComm.println("RESP|TIME_ACCEL_FACTOR|ERROR");
        }
    }
    else {
        Serial.println(F("[COMMAND] Unknown command received"));
        mcuComm.println("RESP|ERROR|UNKNOWN_COMMAND");
    }
}

/**
 * @brief 执行系统软件重启
 */
void performSystemReboot() {
    Serial.println(F("[SYSTEM] Performing software reset..."));
    asm volatile ("jmp 0");  // 跳转到程序起始地址实现软重启
}

/**
 * @brief 执行GDD数据重置操作
 * @description 清除所有累计的GDD数据，重置温度记录
 */
void executeGDDReset() {
    Serial.println(F("[GDD_RESET] Initiating GDD data reset..."));
    
    // 重置所有GDD相关变量
    gddAccumulated = 0.0;
    dailyTempMax = -100.0;
    dailyTempMin = 100.0;
    lastRecordedDay = -1;
    
    // 立即保存重置状态到EEPROM
    saveDataToEEPROM();
    
    Serial.println(F("[GDD_RESET] All GDD data cleared successfully"));
    Serial.print(F("[GDD_RESET] Reset GDD Sum: "));
    Serial.println(gddAccumulated, 2);
    Serial.print(F("[GDD_RESET] Current Date: "));
    printFormattedDate();
    Serial.println();
    
    // 向MCU2发送重置后的数据
    transmitDataToMCU2();
}

/**
 * @brief 执行传感器数据采样和GDD计算
 * @description 读取温湿度传感器数据，更新日温度范围，计算并累积GDD值
 */
void performSensorSampling() {
    float humidity = NAN, temperature = NAN;
    
    // 多次尝试读取传感器数据以提高可靠性
    bool sensorReadSuccess = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        humidity = dhtSensor.readHumidity();
        temperature = dhtSensor.readTemperature();
        
        if (!isnan(humidity) && !isnan(temperature)) {
            sensorReadSuccess = true;
            break;
        }
        delay(100);  // 短暂延迟后重试
    }
    
    if (!sensorReadSuccess) {
        Serial.println(F("[SENSOR_ERROR] DHT sensor reading failed"));
        dhtSensorFailCount++;
        return;
    } else {
        dhtSensorFailCount = 0;
    }
    
    // 验证传感器数据的合理性
    if (temperature < -50.0 || temperature > 80.0) {
        Serial.print(F("[SENSOR_ERROR] Temperature out of valid range: "));
        Serial.print(temperature, 1);
        Serial.println(F("°C"));
        dhtSensorFailCount++;
        return;
    }
    
    if (humidity < 0.0 || humidity > 100.0) {
        Serial.print(F("[SENSOR_ERROR] Humidity out of valid range: "));
        Serial.print(humidity, 1);
        Serial.println(F("%"));
        dhtSensorFailCount++;
        return;
    }
    
    // 更新当日温度极值
    if (dailyTempMax == -100.0) {
        dailyTempMax = temperature;
    } else {
        dailyTempMax = max(dailyTempMax, temperature);
    }
    
    if (dailyTempMin == 100.0) {
        dailyTempMin = temperature;
    } else {
        dailyTempMin = min(dailyTempMin, temperature);
    }
    
    // 检查是否需要计算新一天的GDD
    int currentDay = day();
    if (currentDay != lastRecordedDay) {
        calculateAndAccumulateGDD();
        prepareForNewDay(currentDay);
    }
    
    // 输出当前传感器读数和温度范围
    Serial.print(F("[SENSOR] Temp: "));
    Serial.print(temperature, 1);
    Serial.print(F("°C, Humidity: "));
    Serial.print(humidity, 1);
    Serial.print(F("%, Daily Range: "));
    Serial.print(dailyTempMin, 1);
    Serial.print(F("~"));
    Serial.print(dailyTempMax, 1);
    Serial.println(F("°C"));
}

/**
 * @brief 计算并累积当日GDD值
 */
void calculateAndAccumulateGDD() {
    // 验证温度数据的有效性，防止异常值影响GDD计算
    if (dailyTempMax == -100.0 || dailyTempMin == 100.0) {
        Serial.println(F("[GDD_CALC] No valid temperature data for GDD calculation"));
        return;
    }
    
    // 确保温度数据的逻辑性（最高温度应该大于等于最低温度）
    if (dailyTempMax < dailyTempMin) {
        Serial.println(F("[GDD_CALC] Invalid temperature range, skipping GDD calculation"));
        return;
    }
    
    float dailyAverageTemp = (dailyTempMax + dailyTempMin) / 2.0;
    float dailyGDD = dailyAverageTemp - GDD_BASE_TEMPERATURE;
    
    Serial.print(F("[GDD_CALC] Daily Max: "));
    Serial.print(dailyTempMax, 1);
    Serial.print(F("°C, Min: "));
    Serial.print(dailyTempMin, 1);
    Serial.print(F("°C, Avg: "));
    Serial.print(dailyAverageTemp, 1);
    Serial.print(F("°C, GDD: "));
    Serial.println(dailyGDD, 2);
    
    // 只有正值GDD才累积，负值表示温度过低不利于作物生长
    if (dailyGDD > 0) {
        gddAccumulated += dailyGDD;
        Serial.print(F("[GDD_CALC] Added "));
        Serial.print(dailyGDD, 2);
        Serial.println(F(" to accumulated GDD"));
    } else {
        Serial.println(F("[GDD_CALC] Negative GDD, not added to accumulation"));
    }
    
    Serial.print(F("[GDD_TOTAL] Date: "));
    printFormattedDate();
    Serial.print(F(", Accumulated GDD: "));
    Serial.println(gddAccumulated, 2);
}

/**
 * @brief 为新的一天准备数据
 * @param newDay 新的日期
 */
void prepareForNewDay(int newDay) {
    Serial.print(F("[NEW_DAY] Preparing for new day: "));
    Serial.println(newDay);
    
    // 保存当天温度数据到前一天变量中
    if (dailyTempMax != -100.0 && dailyTempMin != 100.0) {
        previousDayTempMax = dailyTempMax;
        previousDayTempMin = dailyTempMin;
        Serial.print(F("[NEW_DAY] Saved previous day temp range: "));
        Serial.print(previousDayTempMin, 1);
        Serial.print(F("~"));
        Serial.print(previousDayTempMax, 1);
        Serial.println(F("°C"));
    }
    
    lastRecordedDay = newDay;
    // 重置当日温度极值
    dailyTempMax = -100.0;
    dailyTempMin = 100.0;
    
    growthDays++;
    
    Serial.print(F("[GDD] New day started: Day "));
    Serial.print(growthDays);
    Serial.print(F(", Date: "));
    Serial.print(year());
    Serial.print(F("-"));
    Serial.print(month());
    Serial.print(F("-"));
    Serial.println(newDay);
    
    Serial.println(F("[NEW_DAY] Daily temperature range reset"));
}

/**
 * @brief 计算作物剩余生长天数
 * @return 预计剩余天数，0表示已成熟，-1表示当前条件下无法生长
 */
int calculateRemainingGrowthDays() {
    if (gddAccumulated >= CROP_TOTAL_GDD_REQUIRED) {
        return 0;  // 作物已达到成熟要求
    }
    
    float remainingGDDRequired = CROP_TOTAL_GDD_REQUIRED - gddAccumulated;
    
    // 检查温度数据是否有效（避免使用重置后的初始值）
    if (dailyTempMax == -100.0 || dailyTempMin == 100.0) {
        // 当天温度数据未初始化或刚被重置，尝试使用前一天的真实数据
        if (previousDayTempMax != -100.0 && previousDayTempMin != 100.0) {
            float previousDayAvgTemp = (previousDayTempMax + previousDayTempMin) / 2.0;
            float previousDayGDD = max(0.0, previousDayAvgTemp - GDD_BASE_TEMPERATURE);
            if (previousDayGDD <= 0) {
                return -1;  // 前一天温度条件下作物无法生长
            }
            Serial.print(F("[CALC] Using previous day temp for estimation: "));
            Serial.print(previousDayAvgTemp, 1);
            Serial.println(F("°C"));
            return (int)ceil(remainingGDDRequired / previousDayGDD);
        } else {
            // 如果前一天数据也无效，返回-1表示无法估算
            return -1;
        }
    }
    
    // 基于最近的日平均温度估算每日GDD增长量
    float recentDailyAvgTemp = (dailyTempMax + dailyTempMin) / 2.0;
    float estimatedDailyGDD = max(0.0, recentDailyAvgTemp - GDD_BASE_TEMPERATURE);
    
    if (estimatedDailyGDD <= 0) {
        return -1;  // 当前温度条件下作物无法继续生长
    }
    
    return (int)ceil(remainingGDDRequired / estimatedDailyGDD);
}

/**
 * @brief 获取作物当前成熟度百分比
 * @return 成熟度百分比（0-100%）
 */
float getCropMaturityPercentage() {
    return min(100.0, (gddAccumulated / CROP_TOTAL_GDD_REQUIRED) * 100.0);
}

/**
 * @brief 获取作物当前生长阶段描述
 * @return 生长阶段字符串
 */
String getCropGrowthStage() {
    float maturityPercent = getCropMaturityPercentage();
    
    if (maturityPercent < 25.0) {
        return "Seedling";
    } else if (maturityPercent < 50.0) {
        return "Vegetative";
    } else if (maturityPercent < 80.0) {
        return "Flowering";
    } else if (maturityPercent < 100.0) {
        return "Ripening";
    } else {
        return "Mature";
    }
}

/**
 * @brief 向MCU2传输关键农业数据
 * @description 发送GDD累积值、日最高/最低温度、剩余天数、成熟度百分比和时间加速状态
 * @format DATA|gddSum|dayMax|dayMin|remainingDays|maturityPercent|ACCEL=0/1
 */
void transmitDataToMCU2() {
    int remainingGrowthDays = calculateRemainingGrowthDays();
    float maturityPercentage = getCropMaturityPercentage();
    
    // 构建并发送数据包到MCU2 - 修正格式以匹配MCU2解析逻辑
    mcuComm.print("DATA|");
    mcuComm.print(gddAccumulated, 2);           // GDD累积值
    mcuComm.print("|");
    mcuComm.print(dailyTempMax, 1);             // 日最高温度
    mcuComm.print("|");
    mcuComm.print(dailyTempMin, 1);             // 日最低温度
    mcuComm.print("|");
    mcuComm.print(remainingGrowthDays);         // 剩余天数
    mcuComm.print("|");
    mcuComm.print(maturityPercentage, 1);       // 成熟度百分比
    mcuComm.print("|");
    mcuComm.print(isTimeAccelerationEnabled ? "ACCEL=1" : "ACCEL=0");  // 时间加速状态
    if (isTimeAccelerationEnabled) {
        mcuComm.print("|FACTOR=");
        mcuComm.print(timeAccelerationFactor, 0);   // 时间加速因子
    }
    mcuComm.println();
    
    // 输出传输日志和完整数据字符串
    Serial.print(F("[DATA_TX] Transmitted to MCU2: GDD="));
    Serial.print(gddAccumulated, 2);
    Serial.print(F(", TempMax="));
    Serial.print(dailyTempMax, 1);
    Serial.print(F("°C, TempMin="));
    Serial.print(dailyTempMin, 1);
    Serial.print(F("°C, Remaining Days="));
    Serial.print(remainingGrowthDays);
    Serial.print(F(", Maturity="));
    Serial.print(maturityPercentage, 1);
    Serial.print(F("%, TimeAccel="));
    Serial.println(isTimeAccelerationEnabled ? "ON" : "OFF");
    
    // 输出完整的发送数据字符串用于调试
    Serial.print(F("[DATA_TX] Full data string: DATA|"));
    Serial.print(gddAccumulated, 2);
    Serial.print(F("|"));
    Serial.print(dailyTempMax, 1);
    Serial.print(F("|"));
    Serial.print(dailyTempMin, 1);
    Serial.print(F("|"));
    Serial.print(remainingGrowthDays);
    Serial.print(F("|"));
    Serial.print(maturityPercentage, 1);
    Serial.print(F("|"));
    Serial.print(isTimeAccelerationEnabled ? "ACCEL=1" : "ACCEL=0");
    if (isTimeAccelerationEnabled) {
        Serial.print(F("|FACTOR="));
        Serial.print(timeAccelerationFactor, 0);
    }
    Serial.println();
}

/**
 * @brief 评估系统健康状态
 * @description 根据传感器失败次数评估系统整体健康状况
 */
void evaluateSystemHealth() {
    // 评估系统健康状态
    if (dhtSensorFailCount >= 5 || ultrasonicFailCount >= 5) {
        currentSystemStatus = SYSTEM_CRITICAL; // 严重故障
    } else if (dhtSensorFailCount >= 2 || ultrasonicFailCount >= 2) {
        currentSystemStatus = SYSTEM_WARNING; // 警告状态
    } else {
        currentSystemStatus = SYSTEM_HEALTHY; // 健康状态
    }
    
    // 如果系统状态为严重故障，输出警告信息
    if (currentSystemStatus == SYSTEM_CRITICAL) {
        Serial.println(F("[CRITICAL] System health critical - sensor failures detected!"));
        Serial.print(F("[CRITICAL] DHT failures: "));
        Serial.print(dhtSensorFailCount);
        Serial.print(F(", Ultrasonic failures: "));
        Serial.println(ultrasonicFailCount);
    }
}

/**
 * @brief 发送系统状态报告到MCU2
 * @description 传输系统健康状态、传感器故障计数和继电器状态
 * @format STATUS|systemStatus|dhtFailCount|ultraFailCount|relayStatus
 */
void transmitStatusReport() {
    // 输出系统状态到串口
    Serial.print(F("[STATUS] System Status: "));
    switch(currentSystemStatus) {
        case SYSTEM_HEALTHY: Serial.println(F("Healthy")); break;
        case SYSTEM_WARNING: Serial.println(F("Warning")); break;
        case SYSTEM_CRITICAL: Serial.println(F("Critical")); break;
    }
    
    Serial.print(F("[STATUS] DHT Failures: "));
    Serial.print(dhtSensorFailCount);
    Serial.print(F(", Ultrasonic Failures: "));
    Serial.println(ultrasonicFailCount);
    
    // 发送状态报告到MCU2 - 格式匹配MCU2解析逻辑
    mcuComm.print("STATUS|");
    mcuComm.print(currentSystemStatus);         // 系统状态枚举值
    mcuComm.print("|");
    mcuComm.print(dhtSensorFailCount);          // DHT传感器失败次数
    mcuComm.print("|");
    mcuComm.print(ultrasonicFailCount);         // 超声波传感器失败次数
    mcuComm.print("|");
    mcuComm.print(isRelayActive ? "ACTIVE" : "INACTIVE");  // 继电器状态
    mcuComm.println();
    
    Serial.print(F("[STATUS_TX] Status sent to MCU2: Status="));
    Serial.print(currentSystemStatus);
    Serial.print(F(", DHT_Fail="));
    Serial.print(dhtSensorFailCount);
    Serial.print(F(", Ultra_Fail="));
    Serial.print(ultrasonicFailCount);
    Serial.print(F(", Relay="));
    Serial.println(isRelayActive ? "ACTIVE" : "INACTIVE");
    
    // 如果是严重故障状态，考虑系统保护措施
    if (currentSystemStatus == SYSTEM_CRITICAL) {
        Serial.println(F("[WARNING] System in critical state - monitoring closely"));
        // 可以在这里添加额外的保护措施，如降低采样频率等
    }
}

/**
 * @brief 将关键农业数据保存到EEPROM
 * @description 持久化存储GDD累积值、最后记录日期和温度极值
 */
void saveDataToEEPROM() {
    int eepromAddress = 0;
    
    // 保存GDD累积值
    if (!isnan(gddAccumulated)) {
        EEPROM.put(eepromAddress, gddAccumulated);
    }
    eepromAddress += sizeof(float);
    
    // 保存最后记录的日期
    EEPROM.put(eepromAddress, lastRecordedDay);
    eepromAddress += sizeof(int);
    
    // 保存日最高温度
    if (!isnan(dailyTempMax)) {
        EEPROM.put(eepromAddress, dailyTempMax);
    }
    eepromAddress += sizeof(float);
    
    // 保存日最低温度
    if (!isnan(dailyTempMin)) {
        EEPROM.put(eepromAddress, dailyTempMin);
    }
    eepromAddress += sizeof(float);
    
    // 保存前一天最高温度
    if (!isnan(previousDayTempMax)) {
        EEPROM.put(eepromAddress, previousDayTempMax);
    }
    eepromAddress += sizeof(float);
    
    // 保存前一天最低温度
    if (!isnan(previousDayTempMin)) {
        EEPROM.put(eepromAddress, previousDayTempMin);
    }
    
    Serial.println(F("[EEPROM] Agricultural data saved successfully"));
}

/**
 * @brief 监控接近度并控制继电器状态
 * @description 持续监控超声波传感器数据，根据检测结果控制继电器开关
 */
void monitorProximityAndControlRelay() {
    // 发送超声波脉冲信号
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    
    // 读取回声信号持续时间
    long echoDuration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
    float rawDistance = (echoDuration > 0) ? (echoDuration * 0.034f / 2) : -1;
    
    // 检测超声波传感器故障
    if (rawDistance < 0 || rawDistance > 400) {
        ultrasonicFailCount++;
    } else {
        ultrasonicFailCount = 0; // 成功读取，重置失败计数
    }
    
    // 应用滑动窗口中值滤波
    if (rawDistance >= 2.0 && rawDistance <= 400.0) {
        distanceBuffer[distanceBufferIndex] = rawDistance;
    }
    distanceBufferIndex = (distanceBufferIndex + 1) % DISTANCE_BUFFER_SIZE;
    
    // 计算中值距离
    float sortedDistances[DISTANCE_BUFFER_SIZE];
    memcpy(sortedDistances, distanceBuffer, sizeof(distanceBuffer));
    for (uint8_t i = 0; i < DISTANCE_BUFFER_SIZE - 1; i++) {
        for (uint8_t j = i + 1; j < DISTANCE_BUFFER_SIZE; j++) {
            if (sortedDistances[j] < sortedDistances[i]) {
                float temp = sortedDistances[i];
                sortedDistances[i] = sortedDistances[j];
                sortedDistances[j] = temp;
            }
        }
    }
    
    float medianDistance = sortedDistances[DISTANCE_BUFFER_SIZE / 2];
    bool objectDetected = (medianDistance < PROXIMITY_THRESHOLD);
    
    // 根据检测结果控制继电器
    if (objectDetected != isRelayActive) {
        isRelayActive = objectDetected;
        digitalWrite(RELAY_CONTROL, isRelayActive ? HIGH : LOW);
        Serial.print(F("[RELAY] "));
        Serial.println(isRelayActive ? F("ACTIVATED") : F("DEACTIVATED"));
    }
}

/**
 * @brief 处理用户按钮交互事件
 * @description 检测按钮按压时长并执行相应功能：短按切换OLED，长按重启，超长按重置GDD
 */
void handleUserButtonInteraction() {
    bool currentButtonState = (digitalRead(BUTTON_INPUT) == LOW);
    unsigned long currentTime = millis();
    
    if (currentButtonState && !isButtonPressed) {
        isButtonPressed = true;
        buttonPressStartTime = currentTime;
    }
    
    if (!currentButtonState && isButtonPressed) {
        isButtonPressed = false;
        unsigned long pressDuration = currentTime - buttonPressStartTime;
        
        Serial.print(F("[BUTTON] Press duration: "));
        Serial.print(pressDuration);
        Serial.println(F("ms"));
        
        if (pressDuration < BUTTON_SHORT_PRESS_MAX) {
            // 短按：切换OLED显示状态
            isOledDisplayEnabled = !isOledDisplayEnabled;
            Serial.print(F("[OLED] "));
            Serial.println(isOledDisplayEnabled ? F("ENABLED") : F("DISABLED"));
            mcuComm.println(isOledDisplayEnabled ? "CMD|OLED|ON" : "CMD|OLED|OFF");
        } else if (pressDuration >= BUTTON_LONG_PRESS_MIN && pressDuration < BUTTON_RESET_PRESS_MIN) {
            // 长按（1-3秒）：系统重启
            Serial.println(F("[REBOOT] System reboot initiated by button"));
            delay(100);
            performSystemReboot();
        } else if (pressDuration >= BUTTON_RESET_PRESS_MIN) {
            // 超长按（3秒以上）：重置GDD数据
            Serial.println(F("[RESET] GDD reset initiated by button"));
            executeGDDReset();
        }
    }
}

/**
 * @brief 格式化输出当前系统日期
 * @description 以"YYYY/MM/DD"格式输出当前日期
 */
void printFormattedDate() {
    Serial.print(year());
    Serial.write('/');
    if (month() < 10) Serial.write('0');
    Serial.print(month());
    Serial.write('/');
    if (day() < 10) Serial.write('0');
    Serial.print(day());
}

/**
 * @brief 每日输出农作物生长信息
 * @description 输出当天日期和农作物后期需要的生长天数
 */
void outputDailyCropInfo() {
    int remainingDays = calculateRemainingGrowthDays();
    float maturityPercentage = getCropMaturityPercentage();
    
    Serial.println(F("\n========== 每日农作物生长报告 =========="));
    Serial.print(F("[日期] 当天日期: "));
    printFormattedDate();
    Serial.println();
    
    Serial.print(F("[生长状态] 当前成熟度: "));
    Serial.print(maturityPercentage, 1);
    Serial.println(F("%"));
    
    Serial.print(F("[生长状态] 累计GDD值: "));
    Serial.print(gddAccumulated, 2);
    Serial.print(F(" / "));
    Serial.print(CROP_TOTAL_GDD_REQUIRED, 0);
    Serial.println();
    
    if (remainingDays == 0) {
        Serial.println(F("[预测] 农作物已达到成熟标准！"));
    } else if (remainingDays == -1) {
        Serial.println(F("[预测] 当前温度条件下农作物无法继续生长或数据不足"));
        if (dailyTempMax != -100.0 && dailyTempMin != 100.0) {
            Serial.print(F("[建议] 当前日平均温度: "));
            Serial.print((dailyTempMax + dailyTempMin) / 2.0, 1);
            Serial.print(F("°C，需要高于基准温度 "));
            Serial.print(GDD_BASE_TEMPERATURE, 0);
            Serial.println(F("°C"));
        } else if (previousDayTempMax != -100.0 && previousDayTempMin != 100.0) {
            Serial.print(F("[建议] 基于前一天温度数据: "));
            Serial.print((previousDayTempMax + previousDayTempMin) / 2.0, 1);
            Serial.println(F("°C"));
        } else {
            Serial.println(F("[建议] 温度数据不足，无法进行准确估算"));
        }
    } else {
        Serial.print(F("[预测] 农作物后期还需要生长天数: "));
        Serial.print(remainingDays);
        Serial.println(F(" 天"));
        
        // 显示估算所基于的温度数据来源
        if (dailyTempMax != -100.0 && dailyTempMin != 100.0) {
            Serial.println(F("  (基于当日温度数据估算)"));
        } else if (previousDayTempMax != -100.0 && previousDayTempMin != 100.0) {
            Serial.println(F("  (基于前一天温度数据估算)"));
        }
        
        if (dailyTempMax != -100.0 && dailyTempMin != 100.0) {
            Serial.print(F("[预测] 基于当前日平均温度 "));
            Serial.print((dailyTempMax + dailyTempMin) / 2.0, 1);
            Serial.println(F("°C 的估算"));
        } else {
            Serial.println(F("[预测] 基于默认适宜温度20°C的估算"));
        }
    }
    
    Serial.print(F("[生长阶段] 当前阶段: "));
    Serial.println(getCropGrowthStage());
    
    Serial.println(F("======================================\n"));
}

/**
 * @brief 格式化输出当前系统日期和时间
 * @description 以"YYYY/MM/DD HH:MM:SS"格式输出完整的日期时间
 */
void printFormattedDateTime() {
    printFormattedDate();
    Serial.print(F(" "));
    if (hour() < 10) Serial.print("0");
    Serial.print(hour());
    Serial.print(":");
    if (minute() < 10) Serial.print("0");
    Serial.print(minute());
    Serial.print(":");
    if (second() < 10) Serial.print("0");
    Serial.print(second());
}

/**
 * @brief 格式化时间戳为字符串
 * @return 格式化的时间字符串
 */
String formatTimestamp() {
    char timeBuffer[20];
    sprintf(timeBuffer, "%04d-%02d-%02d %02d:%02d:%02d", 
            year(), month(), day(), hour(), minute(), second());
    return String(timeBuffer);
}
