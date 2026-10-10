import math
import numpy as np
from scipy.interpolate import PchipInterpolator

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
    # 1. Tính quãng đường tích lũy (s)
    s_anchors = [0.0]
    for i in range(1, len(x_anchors)):
        dx = x_anchors[i] - x_anchors[i-1]
        dy = y_anchors[i] - y_anchors[i-1]
        dist = math.hypot(dx, dy)
        s_anchors.append(s_anchors[-1] + dist)
        
    s_anchors = np.array(s_anchors)
    
    # 2. Nội suy PchipInterpolator theo s
    cs_x = PchipInterpolator(s_anchors, x_anchors)
    cs_y = PchipInterpolator(s_anchors, y_anchors)
    
    # 3. Tạo dãy s dày đặc (mỗi ds mét có 1 điểm)
    s_dense = np.arange(0, s_anchors[-1], ds)
    if len(s_dense) == 0 or (s_anchors[-1] - s_dense[-1]) > 1e-5:
        s_dense = np.append(s_dense, s_anchors[-1])
    
    x_ref = cs_x(s_dense)
    y_ref = cs_y(s_dense)
    
    # 4. Tính góc Theta_ref (Đạo hàm bậc 1)
    dx = cs_x.derivative(1)(s_dense)
    dy = cs_y.derivative(1)(s_dense)
    theta_ref = [normalize_angle(math.atan2(dy[i], dx[i])) for i in range(len(s_dense))]
    
    # 5. Tính độ cong Kappa (Đạo hàm bậc 2) để tự động điều chỉnh tốc độ v_ref
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
            
    waypoints = np.vstack((x_ref, y_ref, theta_ref, v_ref)).T
    return waypoints.tolist()
