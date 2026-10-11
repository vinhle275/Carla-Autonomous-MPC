import numpy as np
from scipy.interpolate import PchipInterpolator
import math

def normalize_angle(angle):
    """Chuẩn hóa góc về dải [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def generate_trajectory(x_anchors, y_anchors, ds=0.1, v_max=10.0, a_lat_max=3.0):
    """
    Sinh quỹ đạo mịn từ các điểm neo (Anchor Points) bằng PCHIP Spline:
    - x_anchors, y_anchors: Tọa độ điểm neo đã được snap vào tâm đường (ô xám)
    - ds: Khoảng cách giữa các Waypoint (mặc định 0.1m)
    - v_max: Vận tốc tối đa trên đường thẳng (m/s)
    - a_lat_max: Gia tốc ngang tối đa khi vào cua (m/s^2)
    """
    # 1. Tính quãng đường tích lũy (s) qua các điểm neo và LỌC CÁC ĐIỂM TRÙNG LẶP
    # (Tránh lỗi crash PchipInterpolator khi người dùng click snap vào cùng 1 ô xám)
    s_anchors = [0.0]
    filtered_x = [x_anchors[0]]
    filtered_y = [y_anchors[0]]
    
    for i in range(1, len(x_anchors)):
        dx = x_anchors[i] - filtered_x[-1]
        dy = y_anchors[i] - filtered_y[-1]
        dist = math.hypot(dx, dy)
        
        # Bỏ qua nếu điểm quá gần / trùng nhau do snap vào cùng 1 ô xám
        if dist > 0.001:
            s_anchors.append(s_anchors[-1] + dist)
            filtered_x.append(x_anchors[i])
            filtered_y.append(y_anchors[i])
            
    s_anchors = np.array(s_anchors)
    x_anchors = filtered_x
    y_anchors = filtered_y
    
    # 2. Kiểm tra xem đây có phải là đường vòng khép kín không (Closed Loop)
    is_closed_loop = math.hypot(x_anchors[-1] - x_anchors[0], y_anchors[-1] - y_anchors[0]) < 1.0
    
    if is_closed_loop and len(x_anchors) >= 4:
        # THỦ THUẬT PADDING CHO PCHIP:
        # Để đạo hàm tại điểm nối (điểm đầu/cuối) khớp nhau hoàn hảo,
        # copy điểm áp chót lên trước điểm đầu, và copy điểm thứ hai ra sau điểm cuối.
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
    
    # 3. Tính góc Theta (Đạo hàm bậc 1) và làm mịn góc liên tục (np.unwrap)
    dx = cs_x.derivative(1)(s_dense)
    dy = cs_y.derivative(1)(s_dense)
    
    raw_theta = [normalize_angle(math.atan2(dy[i], dx[i])) for i in range(len(s_dense))]
    theta_ref = np.unwrap(raw_theta)  # Đảm bảo góc không bị giật +-pi
    
    # 4. Tính độ cong Kappa (Đạo hàm bậc 2) để giới hạn Vận tốc khi vào cua
    ddx = cs_x.derivative(2)(s_dense)
    ddy = cs_y.derivative(2)(s_dense)
    
    v_ref = np.zeros_like(s_dense)
    for i in range(len(s_dense)):
        kappa = (dx[i] * ddy[i] - dy[i] * ddx[i]) / ((dx[i]**2 + dy[i]**2)**1.5 + 1e-8)
        
        if abs(kappa) > 0.01:  # Nếu là đường cong
            v_turn = math.sqrt(a_lat_max / abs(kappa))
            v_ref[i] = min(v_max, v_turn)
        else:                  # Nếu là đường thẳng
            v_ref[i] = v_max
            
    # Đóng gói thành list các mảng [x, y, theta, v]
    waypoints = np.vstack((x_ref, y_ref, theta_ref, v_ref)).T
    return waypoints.tolist()
