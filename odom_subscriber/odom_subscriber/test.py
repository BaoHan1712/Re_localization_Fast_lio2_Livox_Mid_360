import serial
import serial.tools.list_ports
import os

TARGET_PORT = "/dev/ttyUSB0"

def inspect_target_port(port_name: str, baudrate: int = 115200):
    # 1. Liệt kê toàn bộ các cổng serial đang active trong kernel
    ports = serial.tools.list_ports.comports()
    matched = next((p for p in ports if p.device == port_name), None)

    if not matched:
        print(f"[x] Không tìm thấy {port_name} trong sysfs.")
        print("Danh sách port hiện hữu:")
        for p in ports:
            print(f" - {p.device} (VID:PID = {hex(p.vid) if p.vid else 'None'}:{hex(p.pid) if p.pid else 'None'}, Description: {p.description})")
        return

    # 2. In metadata chi tiết của thiết bị
    print(f"[+] Thiết bị được nhận diện: {matched.device}")
    print(f"    - Mô tả      : {matched.description}")
    print(f"    - Hardware ID: {matched.hwid}")
    print(f"    - VID:PID    : {hex(matched.vid) if matched.vid else 'N/A'}:{hex(matched.pid) if matched.pid else 'N/A'}")
    print(f"    - Serial No  : {matched.serial_number}")

    # 3. Kiểm tra quyền truy cập file descriptor cấp OS
    if not os.access(port_name, os.R_OK | os.W_OK):
        print(f"[!] Cảnh báo: User hiện tại không có quyền Read/Write trên {port_name}.")
        print("    Khắc phục: Chạy lệnh `sudo usermod -aG dialout $USER` rồi logout/login lại.")
        return

    # 4. Thử bắt tay mở cổng I/O
    try:
        with serial.Serial(port=port_name, baudrate=baudrate, timeout=1) as ser:
            if ser.is_open:
                print(f"[+] Kết nối thành công tới {port_name} @ {baudrate} bps.")
    except serial.SerialException as e:
        print(f"[x] Lỗi mở cổng (có thể bị tiến trình khác chiếm dụng): {e}")

if __name__ == "__main__":
    inspect_target_port(TARGET_PORT)