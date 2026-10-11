import numpy as np
from scipy.interpolate import PchipInterpolator, CubicSpline
import math

def normalize_angle(angle):
    """Chuẩn hóa góc về dải [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def generate_trajectory(x_anchors, y_anchors, ds=0.1, v_max=2.0, a_lat_max=1.5):
    """
    Sinh quỹ đạo mịn từ các điểm neo (Anchor Points) bằng PCHIP Spline:
    - x_anchors, y_anchors: Tọa độ điểm neo
    - ds: Khoảng cách giữa các Waypoint (mặt định 0.1m)
    - v_max: Vận tốc tối đa trên đường thẳng (m/s)
    - a_lat_max: Gia tốc ngang tối đa chịu đựng khi vao cua (m/s^2)
    """
    # 1. Tính quãng đường tích lũy (s) qua các điểm neo và lọc các điểm trùng lặp
    s_anchors = [0.0]
    filtered_x = [x_anchors[0]]
    filtered_y = [y_anchors[0]]
    
    for i in range(1, len(x_anchors)):
        dx = x_anchors[i] - filtered_x[-1]
        dy = y_anchors[i] - filtered_y[-1]
        dist = math.hypot(dx, dy)
        
        # Bỏ qua nếu điểm quá gần (ngăn lỗi PchipInterpolator crash do s_anchors bị trùng)
        if dist > 0.001:
            s_anchors.append(s_anchors[-1] + dist)
            filtered_x.append(x_anchors[i])
            filtered_y.append(y_anchors[i])
            
    s_anchors = np.array(s_anchors)
    x_anchors = filtered_x
    y_anchors = filtered_y
    
    # Kiểm tra xem đây có phải là đường vòng khép kín không (Closed Loop)
    is_closed_loop = math.hypot(x_anchors[-1] - x_anchors[0], y_anchors[-1] - y_anchors[0]) == 0
    
    if is_closed_loop:
        # THỦ THUẬT PADDING CHO PCHIP:
        # Để đạo hàm tại điểm nối (điểm đầu/cuối) khớp nhau hoàn hảo mà không dùng CubicSpline
        # Ta copy điểm áp chót lên trước điểm đầu, và copy điểm thứ hai ra sau điểm cuối.
        padded_x = [x_anchors[-2]] + x_anchors + [x_anchors[1]]
        padded_y = [y_anchors[-2]] + y_anchors + [y_anchors[1]]
        
        s_padded = [0.0]
        for i in range(1, len(padded_x)):
            dx = padded_x[i] - padded_x[i-1]
            dy = padded_y[i] - padded_y[i-1]
            s_padded.append(s_padded[-1] + math.hypot(dx, dy))
            
        cs_x = PchipInterpolator(s_padded, padded_x)
        cs_y = PchipInterpolator(s_padded, padded_y)
        
        # Chỉ sinh quỹ đạo trong khoảng của các điểm gốc
        s_start = s_padded[1]
        s_end = s_padded[-2]
        s_dense = np.arange(s_start, s_end, ds)
        if len(s_dense) == 0 or (s_end - s_dense[-1]) > 1e-5:
            s_dense = np.append(s_dense, s_end)
    else:
        cs_x = PchipInterpolator(s_anchors, x_anchors)
        cs_y = PchipInterpolator(s_anchors, y_anchors)
        
        s_dense = np.arange(0, s_anchors[-1], ds)
        if len(s_dense) == 0 or (s_anchors[-1] - s_dense[-1]) > 1e-5:
            s_dense = np.append(s_dense, s_anchors[-1])
            
    x_ref = cs_x(s_dense)
    y_ref = cs_y(s_dense)
    
    # 4. Tính góc Theta (Đạo hàm bậc 1)
    dx = cs_x.derivative(1)(s_dense)
    dy = cs_y.derivative(1)(s_dense)
    
    theta_ref = [normalize_angle(math.atan2(dy[i], dx[i])) for i in range(len(s_dense))]
    
    # 5. Tính độ cong Kappa (Đạo hàm bậc 2) để giới hạn Vận tốc
    ddx = cs_x.derivative(2)(s_dense)
    ddy = cs_y.derivative(2)(s_dense)
    
    v_ref = np.zeros_like(s_dense)
    for i in range(len(s_dense)):
        kappa = (dx[i] * ddy[i] - dy[i] * ddx[i]) / ((dx[i]**2 + dy[i]**2)**1.5 + 1e-8)
        
        if abs(kappa) > 0.01: # Nếu là đường cong
            v_turn = math.sqrt(a_lat_max / abs(kappa))
            v_ref[i] = min(v_max, v_turn)
        else: # Nếu là đường thẳng
            v_ref[i] = v_max
            
    # Đóng gói thành list các mảng [x, y, theta, v]
    waypoints = np.vstack((x_ref, y_ref, theta_ref, v_ref)).T
    return waypoints.tolist()


# import math
# import numpy as np
# from scipy.interpolate import PchipInterpolator

# def normalize_angle(angle):
#     """Chuẩn hóa góc về dải [-pi, pi]"""
#     return math.atan2(math.sin(angle), math.cos(angle))

# def generate_trajectory(x_anchors, y_anchors, ds=0.1, v_max=2.0, a_lat_max=1.5):
#     """
#     Sinh quỹ đạo mịn từ các điểm neo (Anchor Points) bằng PCHIP Spline:
#     - x_anchors, y_anchors: Tọa độ điểm neo
#     - ds: Khoảng cách giữa các Waypoint (mặt định 0.1m)
#     - v_max: Vận tốc tối đa trên đường thẳng (m/s)
#     - a_lat_max: Gia tốc ngang tối đa chịu đựng khi vao cua (m/s^2)
#     """
#     # 1. Tính quãng đường tích lũy (s)
#     s_anchors = [0.0]
#     for i in range(1, len(x_anchors)):
#         dx = x_anchors[i] - x_anchors[i-1]
#         dy = y_anchors[i] - y_anchors[i-1]
#         dist = math.hypot(dx, dy)
#         s_anchors.append(s_anchors[-1] + dist)
        
#     s_anchors = np.array(s_anchors)
    
#     # 2. Nội suy PchipInterpolator theo s
#     cs_x = PchipInterpolator(s_anchors, x_anchors)
#     cs_y = PchipInterpolator(s_anchors, y_anchors)
    
#     # 3. Tạo dãy s dày đặc (mỗi ds mét có 1 điểm)
#     s_dense = np.arange(0, s_anchors[-1], ds)
#     if len(s_dense) == 0 or (s_anchors[-1] - s_dense[-1]) > 1e-5:
#         s_dense = np.append(s_dense, s_anchors[-1])
    
#     x_ref = cs_x(s_dense)
#     y_ref = cs_y(s_dense)
    
#     # 4. Tính góc Theta_ref (Đạo hàm bậc 1)
#     dx = cs_x.derivative(1)(s_dense)
#     dy = cs_y.derivative(1)(s_dense)
#     theta_ref = [normalize_angle(math.atan2(dy[i], dx[i])) for i in range(len(s_dense))]
    
#     # 5. Tính độ cong Kappa (Đạo hàm bậc 2) để tự động điều chỉnh tốc độ v_ref
#     ddx = cs_x.derivative(2)(s_dense)
#     ddy = cs_y.derivative(2)(s_dense)
    
#     v_ref = np.zeros_like(s_dense)
#     for i in range(len(s_dense)):
#         kappa = (dx[i] * ddy[i] - dy[i] * ddx[i]) / ((dx[i]**2 + dy[i]**2)**1.5 + 1e-8)
        
#         if abs(kappa) > 0.01:  # Nếu là đường cong
#             v_turn = math.sqrt(a_lat_max / abs(kappa))
#             v_ref[i] = min(v_max, v_turn)
#         else:                  # Nếu là đường thẳng
#             v_ref[i] = v_max
            
#     waypoints = np.vstack((x_ref, y_ref, theta_ref, v_ref)).T
#     return waypoints.tolist()