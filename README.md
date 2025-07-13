# IoT Crop Growth Estimator

> 基于物联网 (IoT) 的农作物生长时间预测终端

---

## 项目简介
本项目通过双 MCU 架构 (MCU1 + MCU2) 与多种环境传感器，实现对农作物生长环境的实时监测，并基于 **GDD (Growing Degree Days)** 模型预测作物成熟时间及剩余生长天数。系统通过 OLED 屏幕、本地按钮和串口通信提供人机交互，可切换 *时间加速模式* 进行快速验证与展示。

## 主要功能
1. **环境数据采集**：DHT11 温湿度、超声波距离等传感器。
2. **GDD 计算与成熟度预测**：按照日均温度累积 GDD 并计算成熟度百分比。
3. **OLED 多页面显示**：主页面、作物状态、系统信息三大页面自动轮播。
4. **系统健康监控**：传感器故障计数、自检与状态上报。
5. **双 MCU 串口通信**：MCU1 负责采集与计算；MCU2 负责显示与时间同步。
6. **EEPROM 数据持久化**：断电后保存 GDD、温度极值等关键数据。
7. **时间加速模式**：最高支持 86400× 加速，用于实验与演示。

## 硬件架构
| 元件 | 说明 |
| ---- | ---- |
| MCU1 | Arduino UNO R3 (主控制器) |
| MCU2 | Arduino UNO R3 (显示控制器) |
| DHT11 | 温湿度传感器 |
| HC-SR04 | 超声波距离传感器 |
| SSD1306 0.96" OLED | I2C 接口显示屏 |
| 继电器 | 控制外部设备 (LED灯) |

连接示意图：

![MCU1&MCU2 连接示意](MCU1&MCU2.png)

> 详细引脚定义请查看 `MCU1/MCU1.ino` 与 `MCU2/MCU2.ino` 文件顶部的宏定义部分。

## 软件依赖
* Arduino IDE ≥ 2.0
* Arduino 库
  * `DHT sensor library`
  * `TimeLib`
  * `EEPROM`
  * `U8g2`
  * `SoftwareSerial`

> 可在 Arduino Library Manager 中直接搜索并安装以上库。

## 目录结构
```
.
├── MCU1/                 # 主控制器固件
│   └── MCU1.ino
├── MCU2/                 # 显示控制器固件
│   └── MCU2.ino
```


### 串口命令 (MCU1)
| 命令 | 功能 |
| ---- | ---- |
| `ACCEL_ON` | 启用时间加速模式 |
| `ACCEL_OFF` | 关闭时间加速 |
| `ACCEL_FACTOR=X` | 设置加速倍数 (1–86400) |
| `STATUS` | 输出系统健康状态 |
| `CROP_REPORT` | 输出当日作物生长报告 |
| `HELP` | 查看帮助信息 |

## License
本项目采用 **MIT License**，详情见 [LICENSE](LICENSE) 文件。 