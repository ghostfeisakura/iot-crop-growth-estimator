/*
 * @file MCU2.ino
 * @brief 智能农业终端系统 - MCU2显示控制单元
 * @description 负责OLED显示管理、时间同步、数据可视化和用户界面
 * 
 * 主要功能：
 * 1. OLED显示屏控制和多页面UI管理
 * 2. 接收并解析MCU1发送的农业数据
 * 3. 实时显示GDD值、温度、成熟度等关键信息
 * 4. 系统状态监控和健康状态显示
 * 5. 时间同步和加速模式支持
 * 6. 数据超时检测和异常处理
 * 7. 用户界面的自动页面切换
 * 8. 与MCU1的双向通信协议处理
 */

// ====== 系统库文件引用 ======
#include <U8g2lib.h>        // OLED显示库
#include <Wire.h>           // I2C通信库
#include <TimeLib.h>        // 时间管理库
#include <SoftwareSerial.h> // 软件串口库

// ====== 硬件对象初始化 ======
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oledDisplay(U8G2_R0, U8X8_PIN_NONE);

// ====== 通信引脚定义 ======
#define MCU_COMM_RX_PIN        11
#define MCU_COMM_TX_PIN        10
#define COMMUNICATION_BAUD_RATE 9600

SoftwareSerial mcuCommunication(MCU_COMM_RX_PIN, MCU_COMM_TX_PIN);

// ====== 农业参数常量定义 ======
const float CROP_TOTAL_GDD_REQUIRED = 500.0;

// ====== 显示更新时间间隔 ======
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000;
const unsigned long PAGE_AUTO_SWITCH_INTERVAL = 5000;
const unsigned long TIME_SYNC_INTERVAL = 60000;
const unsigned long DATA_TIMEOUT_INTERVAL = 11000;

// ====== 农业数据变量 ======
// 从MCU1接收的核心农业监测数据
float accumulatedGDD = -1.0;            // 累计生长度日值（-1表示未接收到有效数据）
float dailyMaxTemperature = 0.0;        // 当日最高温度（摄氏度）
float dailyMinTemperature = 0.0;        // 当日最低温度（摄氏度）
int estimatedRemainingDays = -1;        // 预估剩余生长天数（-1表示无法估算）
float cropMaturityPercentage = 0.0;     // 作物成熟度百分比（0-100%）

// ====== 时间加速功能变量 ======
// 时间加速模式：用于快速模拟农作物生长过程
bool isTimeAccelerationEnabled = false; // 时间加速模式开关状态
float timeAccelerationFactor = 1.0;     // 时间加速倍数（1.0为正常速度）
unsigned long lastTimeAcceleration = 0; // 上次时间加速更新的时间戳
bool isTimeAccelerated = false;         // 当前是否处于加速状态标志

// ====== 系统状态监控变量 ======
// 系统健康状态和传感器故障监控
uint8_t systemHealthStatus = 1;         // 系统健康状态（1=健康，2=警告，3=严重）
uint8_t dhtSensorFailureCount = 0;      // DHT传感器失败次数统计
uint8_t ultrasonicFailureCount = 0;     // 超声波传感器失败次数统计
bool isRelayActive = false;             // 继电器激活状态
unsigned long lastDataReceiveTime = 0;  // 上次接收到MCU1数据的时间戳

// ====== 显示控制变量 ======
// OLED显示屏状态和数据有效性控制
bool isOledDisplayEnabled = true;        // OLED显示屏开关状态（可通过MCU1按钮控制）
bool hasReceivedValidData = false;       // 是否已接收到MCU1的有效数据标志

// ====== 显示页面枚举定义 ======
// 多页面UI系统：支持自动切换显示不同类型的信息
enum DisplayPageType {
    DISPLAY_PAGE_MAIN = 0,      // 主页面：显示时间和基本GDD信息
    DISPLAY_PAGE_CROP_STATUS,   // 作物状态页面：显示详细的生长信息
    DISPLAY_PAGE_SYSTEM_INFO,   // 系统信息页面：显示健康状态和传感器信息
    DISPLAY_PAGE_TOTAL_COUNT    // 页面总数（用于循环切换）
};

DisplayPageType currentDisplayPage = DISPLAY_PAGE_MAIN;  // 当前显示的页面
unsigned long lastPageSwitchTime = 0;    // 上次页面切换的时间戳

// ====== 时间管理变量 ======
// 各种定时任务的时间戳记录
unsigned long lastDisplayUpdateMillis = 0;  // 上次显示更新时间
unsigned long lastTimeSyncMillis = 0;       // 上次时间同步时间
unsigned long lastTimeUpdateMillis = 0;     // 上次时间更新时间

void drawMainPage() {
  oledDisplay.setFont(u8g2_font_6x10_tf);
  
  oledDisplay.setCursor(0, 10);
  oledDisplay.print(F("Time:"));
  if (isTimeAccelerated) {
    oledDisplay.print(F(" [ACCEL]"));
  }
  
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d", year(), month(), day());
  oledDisplay.setCursor(0, 22);
  oledDisplay.print(buf);
  
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hour(), minute(), second());
  oledDisplay.setCursor(0, 34);
  oledDisplay.print(buf);
  
  if (accumulatedGDD >= 0) {
    oledDisplay.setCursor(0, 46);
    oledDisplay.print(F("GDD: "));
    oledDisplay.print(accumulatedGDD, 1);
    oledDisplay.print(F("/"));
    oledDisplay.print(CROP_TOTAL_GDD_REQUIRED, 0);
    
    oledDisplay.setCursor(0, 58);
    oledDisplay.print(F("T:"));
    oledDisplay.print(dailyMinTemperature, 1);
    oledDisplay.print(F("~"));
    oledDisplay.print(dailyMaxTemperature, 1);
    oledDisplay.print(F("C"));
  } else {
    oledDisplay.setCursor(0, 46);
    oledDisplay.print(F("Waiting..."));
  }
}

void drawCropPage() {
  oledDisplay.setFont(u8g2_font_6x10_tf);
  
  oledDisplay.setCursor(0, 10);
  oledDisplay.print(F("Crop Status"));
  
  if (accumulatedGDD >= 0) {
    oledDisplay.setCursor(0, 24);
    oledDisplay.print(F("Maturity: "));
    oledDisplay.print(cropMaturityPercentage, 1);
    oledDisplay.print(F("%"));
    
    int barWidth = 100;
    int barHeight = 8;
    int barX = 14;
    int barY = 30;
    
    oledDisplay.drawFrame(barX, barY, barWidth, barHeight);
    int fillWidth = (int)((cropMaturityPercentage / 100.0) * (barWidth - 2));
    if (fillWidth > 0) {
      oledDisplay.drawBox(barX + 1, barY + 1, fillWidth, barHeight - 2);
    }
    
    oledDisplay.setCursor(0, 50);
    oledDisplay.print(F("Days: "));
    if (estimatedRemainingDays >= 0) {
      oledDisplay.print(estimatedRemainingDays);
    } else {
      oledDisplay.print(F("--"));
    }
    
    oledDisplay.setCursor(0, 62);
    if (cropMaturityPercentage < 25) {
      oledDisplay.print(F("Seedling"));
    } else if (cropMaturityPercentage < 50) {
      oledDisplay.print(F("Vegetative"));
    } else if (cropMaturityPercentage < 80) {
      oledDisplay.print(F("Flowering"));
    } else if (cropMaturityPercentage < 100) {
      oledDisplay.print(F("Ripening"));
    } else {
      oledDisplay.print(F("Mature"));
    }
  } else {
    oledDisplay.setCursor(0, 30);
    oledDisplay.print(F("No data"));
  }
}

void drawSystemPage() {
  oledDisplay.setFont(u8g2_font_6x10_tf);
  
  oledDisplay.setCursor(0, 10);
  oledDisplay.print(F("System"));
  
  oledDisplay.setCursor(0, 24);
  oledDisplay.print(F("Status: "));
  switch(systemHealthStatus) {
    case 1: oledDisplay.print(F("OK")); break;
    case 2: oledDisplay.print(F("WARN")); break;
    case 3: oledDisplay.print(F("ERR")); break;
    default: oledDisplay.print(F("UNK")); break;
  }
  
  oledDisplay.setCursor(0, 36);
  oledDisplay.print(F("DHT: "));
  oledDisplay.print(dhtSensorFailureCount);
  
  oledDisplay.setCursor(0, 48);
  oledDisplay.print(F("Ultra: "));
  oledDisplay.print(ultrasonicFailureCount);
  
  oledDisplay.setCursor(0, 60);
  if (hasReceivedValidData) {
    oledDisplay.print(F("MCU1: OK"));
  } else {
    oledDisplay.print(F("MCU1: NO"));
  }
}

/**
 * @brief 绘制当前选中的显示页面
 * @description 根据currentDisplayPage变量绘制对应的页面内容，并在右下角显示页面指示器
 */
void drawCurrentPage() {
  oledDisplay.clearBuffer();
  
  switch (currentDisplayPage) {
    case DISPLAY_PAGE_MAIN:
      drawMainPage();
      break;
    case DISPLAY_PAGE_CROP_STATUS:
      drawCropPage();
      break;
    case DISPLAY_PAGE_SYSTEM_INFO:
      drawSystemPage();
      break;
    default:
      drawMainPage();
      break;
  }
  
  // 在右下角显示页面指示器（如"1/3"）
  oledDisplay.setFont(u8g2_font_4x6_tf);
  char pageInfo[10];
  snprintf(pageInfo, sizeof(pageInfo), "%d/%d", (int)currentDisplayPage + 1, (int)DISPLAY_PAGE_TOTAL_COUNT);
  oledDisplay.setCursor(oledDisplay.getDisplayWidth() - 20, oledDisplay.getDisplayHeight() - 1);
  oledDisplay.print(pageInfo);
}

/**
 * @brief 自动更新显示页面
 * @description 每隔PAGE_AUTO_SWITCH_INTERVAL时间自动切换到下一个页面
 */
void updateDisplayPage() {
  unsigned long now = millis();
  if (now - lastPageSwitchTime >= PAGE_AUTO_SWITCH_INTERVAL) {
    lastPageSwitchTime = now;
    currentDisplayPage = (DisplayPageType)((currentDisplayPage + 1) % DISPLAY_PAGE_TOTAL_COUNT);
  }
}

/**
 * @brief MCU2系统初始化函数
 * @description 初始化串口通信、OLED显示屏、I2C总线和系统时间
 */
void setup() {
  // 初始化串口通信
  Serial.begin(COMMUNICATION_BAUD_RATE);
  mcuCommunication.begin(COMMUNICATION_BAUD_RATE);
  
  // 初始化I2C总线和OLED显示屏
  Wire.begin();
  oledDisplay.begin();
  
  // 设置系统初始时间（与MCU1保持一致）
  setTime(15, 0, 0, 24, 5, 2025);
  
  lastPageSwitchTime = millis();
  
  // 如果OLED显示启用，初始化显示内容
  if (isOledDisplayEnabled) {
    oledDisplay.setPowerSave(0);
    oledDisplay.firstPage();
    do {
      drawCurrentPage();
    } while (oledDisplay.nextPage());
  }
  
  Serial.println(F("[MCU2] Ready"));
}

/**
 * @brief MCU2主循环函数
 * @description 处理时间管理、数据接收、显示更新和系统监控
 */
void loop() {
  unsigned long now = millis();
  
  // 处理时间加速模式
  handleUnifiedTimeManagement(now);
  
  // 定期与MCU1进行时间同步
  if (now - lastTimeSyncMillis >= TIME_SYNC_INTERVAL) {
    lastTimeSyncMillis = now;
    synchronizeTimeWithMCU1();
  }

  // 检测数据接收超时
  if (hasReceivedValidData && (now - lastDataReceiveTime >= DATA_TIMEOUT_INTERVAL)) {
    Serial.println(F("[MCU2] No data timeout"));
  }
 
  // 处理来自MCU1的串口数据
  while (mcuCommunication.available()) {
    String receivedLine = mcuCommunication.readStringUntil('\n');
    receivedLine.trim();
    
    if (receivedLine.length() > 0) {
      if (receivedLine.startsWith("DATA|")) {
        processReceivedData(receivedLine);
      } else if (receivedLine.startsWith("STATUS|")) {
        processReceivedData(receivedLine);
      } else if (receivedLine.startsWith("CMD|")) {
         processRemoteCommand(receivedLine);
      }
    }
  }

  // 定期更新OLED显示内容
  if (now - lastDisplayUpdateMillis >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateMillis = now;
    updateDisplayPage();
    
    if (isOledDisplayEnabled) {
      oledDisplay.firstPage();
      do {
        drawCurrentPage();
      } while (oledDisplay.nextPage());
    }
  }
}

/**
 * @brief 处理从MCU1接收到的数据
 * @param receivedData 接收到的数据字符串
 * @description 解析DATA|和STATUS|格式的数据，更新农业参数和系统状态
 */
void processReceivedData(const String &receivedData) {
  lastDataReceiveTime = millis();
  
  if (receivedData.startsWith("DATA|")) {
    // 解析农业数据：DATA|GDD|MaxTemp|MinTemp|RemainingDays|Maturity|ACCEL=x|FACTOR=y
    String data = receivedData.substring(5);
    
    int pos1 = data.indexOf('|');
    int pos2 = data.indexOf('|', pos1 + 1);
    int pos3 = data.indexOf('|', pos2 + 1);
    int pos4 = data.indexOf('|', pos3 + 1);
    int pos5 = data.indexOf('|', pos4 + 1);
    
    if (pos1 > 0 && pos2 > pos1 && pos3 > pos2 && pos4 > pos3) {
      float newGDD = data.substring(0, pos1).toFloat();
      float newMaxTemp = data.substring(pos1 + 1, pos2).toFloat();
      float newMinTemp = data.substring(pos2 + 1, pos3).toFloat();
      int newRemainingDays = data.substring(pos3 + 1, pos4).toInt();
      
      // 验证并更新农业数据
      if (newGDD >= 0) accumulatedGDD = newGDD;
      if (newMaxTemp >= -50.0 && newMaxTemp <= 80.0) dailyMaxTemperature = newMaxTemp;
      if (newMinTemp >= -50.0 && newMinTemp <= 80.0) dailyMinTemperature = newMinTemp;
      if (newRemainingDays >= 0) estimatedRemainingDays = newRemainingDays;
      
      if (pos5 > pos4) {
        float newMaturity = data.substring(pos4 + 1, pos5).toFloat();
        if (newMaturity >= 0.0 && newMaturity <= 100.0) {
          cropMaturityPercentage = newMaturity;
        }
        
        String accelPart = data.substring(pos5 + 1);
        isTimeAccelerated = accelPart.indexOf("ACCEL=1") >= 0;
        
        int factorPos = accelPart.indexOf("FACTOR=");
        if (factorPos >= 0) {
          String factorStr = accelPart.substring(factorPos + 7);
          int endPos = factorStr.indexOf("|");
          if (endPos > 0) {
            factorStr = factorStr.substring(0, endPos);
          }
          float newFactor = factorStr.toFloat();
          if (newFactor > 0 && newFactor <= 86400) {
            timeAccelerationFactor = newFactor;
          }
        }
      }
      
      hasReceivedValidData = true;
    }
    
  } else if (receivedData.startsWith("STATUS|")) {
    int p1 = receivedData.indexOf('|', 7);
    int p2 = receivedData.indexOf('|', p1 + 1);
    int p3 = receivedData.indexOf('|', p2 + 1);
    
    if (p1 > 6 && p2 > p1 && p3 > p2) {
      int newSystemStatus = receivedData.substring(7, p1).toInt();
      int newDhtFailCount = receivedData.substring(p1 + 1, p2).toInt();
      int newUltraFailCount = receivedData.substring(p2 + 1, p3).toInt();
      
      if (newSystemStatus >= 0 && newSystemStatus <= 2) {
        systemHealthStatus = newSystemStatus;
      }
      
      if (newDhtFailCount >= 0) {
        dhtSensorFailureCount = newDhtFailCount;
      }
      
      if (newUltraFailCount >= 0) {
        ultrasonicFailureCount = newUltraFailCount;
      }
      
      hasReceivedValidData = true;
    }
  }
}

void synchronizeTimeWithMCU1() {
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           year(), month(), day(), hour(), minute(), second());
  
  String timeMessage = "TIME|" + String(buf);
  mcuCommunication.println(timeMessage);
}

void handleUnifiedTimeManagement(unsigned long currentTime) {
  if (isTimeAccelerationEnabled) {
    unsigned long timeDelta = currentTime - lastTimeAcceleration;
    
    if (timeDelta >= 1000) {
      unsigned long acceleratedSeconds = (timeDelta / 1000) * (timeAccelerationFactor - 1);
      
      if (acceleratedSeconds > 0) {
        adjustTime(acceleratedSeconds);
        lastTimeAcceleration = currentTime;
        isTimeAccelerated = true;
      }
    }
  } else {
    isTimeAccelerated = false;
  }
}

void processRemoteCommand(String command) {
  if (command.startsWith("CMD|TIME_ACCEL|ON")) {
    isTimeAccelerationEnabled = true;
    lastTimeAcceleration = millis();
    mcuCommunication.println("RESP|TIME_ACCEL|ON|SUCCESS");
  } else if (command.startsWith("CMD|TIME_ACCEL|OFF")) {
    isTimeAccelerationEnabled = false;
    mcuCommunication.println("RESP|TIME_ACCEL|OFF|SUCCESS");
  } else if (command.startsWith("CMD|TIME_ACCEL_FACTOR|")) {
    float newFactor = command.substring(22).toFloat();
    if (newFactor >= 1.0 && newFactor <= 86400.0) {
      timeAccelerationFactor = newFactor;
      mcuCommunication.println("RESP|TIME_ACCEL_FACTOR|SUCCESS");
    } else {
      mcuCommunication.println("RESP|TIME_ACCEL_FACTOR|ERROR");
    }
  } else if (command.startsWith("CMD|OLED|")) {
    if (command.endsWith("ON")) {
      isOledDisplayEnabled = true;
      oledDisplay.setPowerSave(0);
      mcuCommunication.println("RESP|OLED|ON|OK");
    } else if (command.endsWith("OFF")) {
      isOledDisplayEnabled = false;
      oledDisplay.setPowerSave(1);
      mcuCommunication.println("RESP|OLED|OFF|OK");
    }
  }
}