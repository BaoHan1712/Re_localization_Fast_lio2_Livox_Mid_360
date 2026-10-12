import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

# ==============================================================================
# 1. BỘ LỌC THÔNG THẤP (LOW-PASS FILTER / EMA) DÙNG CHUNG
# ==============================================================================
class LowPassFilter:
    """
    Bộ lọc số Exponential Moving Average (EMA).
    Công thức: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
    - alpha: Hệ số lọc từ (0.0 đến 1.0)
      + alpha càng NHỎ (ví dụ 0.05 - 0.1): Lọc nhiễu rất mạnh, số đứng yên rất tĩnh nhưng bị trễ (lag) khi di chuyển nhanh.
      + alpha càng LỚN (ví dụ 0.3 - 0.7): Phản hồi tức thì theo chuyển động thực tế nhưng số dễ rung nhẹ.
      + alpha = 1.0: Không lọc (dữ liệu thô).
    - deadband: Vùng chết (ngưỡng nhiễu). Nếu độ thay đổi nhỏ hơn ngưỡng này thì bỏ qua, giữ nguyên giá trị cũ để triệt tiêu rung khi đứng yên.
    """
    def __init__(self, alpha=0.2, deadband=0.0):
        self.alpha = alpha
        self.deadband = deadband
        self.filtered_val = None

    def update(self, current_val):
        if self.filtered_val is None:
            self.filtered_val = current_val
            return self.filtered_val

        # Nếu độ lệch nhỏ hơn ngưỡng deadband thì giữ nguyên (loại bỏ rung lắc vi mô)
        if abs(current_val - self.filtered_val) < self.deadband:
            return self.filtered_val

        # Cập nhật giá trị lọc
        self.filtered_val = self.alpha * current_val + (1.0 - self.alpha) * self.filtered_val
        return self.filtered_val

    def update_angle(self, current_deg):
        """Hàm lọc riêng cho góc (xử lý biên quay tránh giật khi chuyển từ +180 sang -180 độ)"""
        if self.filtered_val is None:
            self.filtered_val = current_deg
            return self.filtered_val

        # Tính khoảng chênh lệch góc ngắn nhất
        diff = current_deg - self.filtered_val
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0

        if abs(diff) < self.deadband:
            return self.filtered_val

        self.filtered_val = self.filtered_val + self.alpha * diff
        return self.filtered_val


# ==============================================================================
# 2. HÀM CHUYỂN QUATERNION SANG EULER (ROLL, PITCH, YAW)
# ==============================================================================
def quaternion_to_euler(x, y, z, w):
    # Roll (xoay quanh trục X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (xoay quanh trục Y)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (xoay quanh trục Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# ==============================================================================
# 3. NODE ROS 2 LẮNG NGHE VÀ LỌC DỮ LIỆU
# ==============================================================================
class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener_node')

        # 1. Hệ số mượt alpha (từ 0.01 đến 1.0):
        #    Khuyên dùng: 0.2 cho vị trí (cm), 0.15 cho góc quay (deg)
        ALPHA_POSITION = 0.2  # Cho X, Y, Z
        ALPHA_ANGLE    = 0.3   # Cho Roll, Pitch, Yaw

        # 2. Vùng chết Deadband (ngưỡng bỏ qua rung lắc khi đứng yên):
        #    Nếu robot đứng yên mà số vẫn nhảy ±0.3 cm hoặc ±0.2 độ -> tăng nhẹ lên.
        DEADBAND_POSITION = 0.50  # Đơn vị: cm (thay đổi dưới 0.3cm coi như đứng yên)
        DEADBAND_ANGLE    = 0.20  # Đơn vị: độ (thay đổi dưới 0.2° coi như không quay)
        # ----------------------------------------------------------------------

        # Khởi tạo các bộ lọc riêng biệt cho từng trục
        self.filter_x     = LowPassFilter(alpha=ALPHA_POSITION, deadband=DEADBAND_POSITION)
        self.filter_y     = LowPassFilter(alpha=ALPHA_POSITION, deadband=DEADBAND_POSITION)
        self.filter_z     = LowPassFilter(alpha=ALPHA_POSITION, deadband=DEADBAND_POSITION)
        self.filter_roll  = LowPassFilter(alpha=ALPHA_ANGLE, deadband=DEADBAND_ANGLE)
        self.filter_pitch = LowPassFilter(alpha=ALPHA_ANGLE, deadband=DEADBAND_ANGLE)
        self.filter_yaw   = LowPassFilter(alpha=ALPHA_ANGLE, deadband=DEADBAND_ANGLE)

        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10)
        self.get_logger().info("Odom Listener Node da khoi dong. Dang loc du lieu /Odometry...")

    def odom_callback(self, msg):
        # 1. Lấy dữ liệu thô (Raw data)
        raw_x_cm = msg.pose.pose.position.x * 100.0
        raw_y_cm = msg.pose.pose.position.y * 100.0
        raw_z_cm = msg.pose.pose.position.z * 100.0

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        raw_roll, raw_pitch, raw_yaw = quaternion_to_euler(qx, qy, qz, qw)

        # 2. Đưa qua các hàm lọc nhiễu
        f_x = self.filter_x.update(raw_x_cm)
        f_y = self.filter_y.update(raw_y_cm)
        f_z = self.filter_z.update(raw_z_cm)

        f_roll  = self.filter_roll.update_angle(raw_roll)
        f_pitch = self.filter_pitch.update_angle(raw_pitch)
        f_yaw   = self.filter_yaw.update_angle(raw_yaw)

        # 3. Làm tròn và ép kiểu số nguyên để hiển thị/gửi đi
        x_cm = int(round(f_x))
        y_cm = int(round(f_y))
        z_cm = int(round(f_z))
        roll = int(round(f_roll))
        pitch = int(round(f_pitch))
        yaw = int(round(f_yaw))

        # 4. In ra terminal
        output = (
            f"\r[TỌA ĐỘ cm] X: {x_cm:5d} | Y: {y_cm:5d} | Z: {z_cm:5d}  "
            f"[GÓC deg] Roll: {roll:4d}° | Pitch: {pitch:4d}° | Yaw: {yaw:4d}°    "
        )
        print(output, end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()