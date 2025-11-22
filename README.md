# Energy_Meter_Linux

Dự án đo lường và giám sát năng lượng (Điện áp, Dòng điện, Công suất) sử dụng vi điều khiển ESP32 kết hợp với cảm biến INA226. Dữ liệu được hiển thị trực tiếp trên màn hình OLED và gửi qua cổng Serial (USB) để lưu trữ vào máy tính Linux (Fedora) dưới dạng file CSV.

## 🚀 Tính năng
- **Đo lường:** Điện áp Bus (V), Dòng điện (A), Công suất (W) theo thời gian thực.
- **Hiển thị:** Màn hình OLED 0.96" giao tiếp SPI.
- **Logging:** Tool Python tự động thu thập dữ liệu từ ESP32 và lưu vào file `.csv` trên máy tính.
- **Giao thức:**
  - INA226: I2C
  - OLED (SSD1306): SPI
  - Data Logging: UART/USB

## 🛠️ Phần cứng yêu cầu
1. **ESP32 Development Board** (ESP32-WROOM-32).
2. **Module INA226** (Cảm biến dòng/áp).
3. **Màn hình OLED 0.96"** (Driver SSD1306, giao tiếp SPI).
4. **Điện trở Shunt** (R_shunt = `0.1 Ohm`).

## 🔌 Sơ đồ đấu nối (Pinout)

### 1. INA226 (I2C)
| Chân INA226 | Chân ESP32 | Ghi chú |
|-------------|------------|---------|
| VCC         | 3.3V       |         |
| GND         | GND        |         |
| SDA         | GPIO 21    | I2C Data |
| SCL         | GPIO 22    | I2C Clock |

### 2. OLED SSD1306 (SPI)
| Chân OLED   | Chân ESP32 | Ghi chú |
|-------------|------------|---------|
| VCC         | 3.3V       |         |
| GND         | GND        |         |
| CS          | GPIO 5     | Chip Select |
| RES (RST)   | GPIO 16    | Reset |
| DC          | GPIO 17    | Data/Command |
| D1 (MOSI)   | GPIO 23    | Data Input |
| D0 (SCLK)   | GPIO 18    | Clock |

## 📂 Cấu trúc dự án
```text
Energy_Meter_Linux/
├── CMakeLists.txt          # File cấu hình build gốc
├── communicate_init/       # Thư viện cấu hình SPI bus
│   ├── spi_init.c
│   └── spi_init.h
├── components/             # Các component driver
│   ├── display/            # Driver OLED & SSD1306
│   └── esp_ina226/         # Driver INA226
├── main/                   # Source code chính
│   ├── Energy_Meter.c      # App main loop
│   └── CMakeLists.txt
├── logger.py               # Script Python để log dữ liệu trên PC
└── data_logs/              # Thư mục chứa file kết quả (.csv)
