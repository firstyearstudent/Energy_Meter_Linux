import serial
import time
import csv
import os

# --- CẤU HÌNH ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# Tên thư mục và tên file
FOLDER_NAME = "data_logs"
FILE_NAME = "ina_test.csv"

def main():
    # 1. Xử lý đường dẫn file
    # Lấy đường dẫn thư mục hiện tại nơi file python đang chạy
    current_dir = os.getcwd()
    # Tạo đường dẫn đầy đủ: current_dir/data_logs/ina226_ketqua.csv
    full_path = os.path.join(current_dir, FOLDER_NAME, FILE_NAME)

    # Kiểm tra xem thư mục data_logs có tồn tại không
    if not os.path.exists(os.path.join(current_dir, FOLDER_NAME)):
        print(f"Lỗi: Không tìm thấy thư mục '{FOLDER_NAME}' tại {current_dir}")
        print("Vui lòng tạo thư mục này trước hoặc chỉnh sửa lại đường dẫn.")
        return

    print(f"Dữ liệu sẽ được lưu vào: {full_path}")

    # 2. Kết nối Serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Đang kết nối tới {SERIAL_PORT}...")
        time.sleep(2) # Đợi ESP32 ổn định

        # Xóa bộ đệm để tránh nhận dữ liệu rác ban đầu
        ser.reset_input_buffer()

    except serial.SerialException:
        print(f"Lỗi: Không thể mở cổng {SERIAL_PORT}. Hãy kiểm tra dây cáp hoặc 'sudo chmod 666 {SERIAL_PORT}'.")
        return

    print("Đang ghi log... Nhấn Ctrl+C để dừng.")

    # 3. Mở file và ghi dữ liệu
    # mode='a': append (ghi nối tiếp), nếu muốn ghi đè mỗi lần chạy lại thì dùng mode='w'
    with open(full_path, mode='a', newline='') as csv_file:
        writer = csv.writer(csv_file)

        # Nếu file mới tinh (kích thước = 0), ghi thêm header
        if os.stat(full_path).st_size == 0:
            writer.writerow(["Timestamp_ms", "Voltage_V", "Current_A", "Power_W"])

        try:
            while True:
                if ser.in_waiting > 0:
                    # Đọc dữ liệu từ ESP32
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                    except UnicodeDecodeError:
                        continue

                    # Lọc dữ liệu: chỉ xử lý dòng có dấu phẩy (định dạng CSV)
                    if ',' in line:
                        data = line.split(',')
                        # Đảm bảo đủ 4 cột dữ liệu
                        if len(data) == 4:
                            writer.writerow(data)
                            print(f"Đã lưu: {line}")
                            csv_file.flush() # Lưu ngay lập tức xuống ổ cứng
                    else:
                        # In ra console để debug nhưng không lưu vào file
                        print(f"[ESP32 Info]: {line}")

        except KeyboardInterrupt:
            print("\nĐã dừng chương trình.")
            ser.close()

if __name__ == "__main__":
    main()
