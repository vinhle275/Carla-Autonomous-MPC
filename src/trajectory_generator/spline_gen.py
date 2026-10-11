import math
import numpy as np
from scipy.interpolate import PchipInterpolator

def normalize_angle(angle):
    """Chuẩn hóa góc về dải [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def generate_trajectory(x_anchors, y_anchors, ds=0.1, v_max=10.0, a_lat_max=3.0):
    """
    Sinh quỹ đạo mịn từ các điểm neo (Anchor Points) bằng PCHIP Spline:
    - x_anchors, y_anchors: Tọa độ điểm neo
    - ds: Khoảng cách giữa các Waypoint (mặt định 0.1m)
    - v_max: Vận tốc tối đa trên đường thẳng (m/s) -> mặc định 10.0 m/s (36 km/h)
    - a_lat_max: Gia tốc ngang tối đa khi vào cua (m/s^2) -> mặc định 3.0 m/s^2
    """
    s_anchors = [0.0]
    for i in range(1, len(x_anchors)):
        dx = x_anchors[i] - x_anchors[i-1]
        dy = y_anchors[i] - y_anchors[i-1]
        dist = math.hypot(dx, dy)
        s_anchors.append(s_anchors[-1] + dist)
        
    s_anchors = np.array(s_anchors)
    
    cs_x = PchipInterpolator(s_anchors, x_anchors)
    cs_y = PchipInterpolator(s_anchors, y_anchors)
    
    s_dense = np.arange(0, s_anchors[-1], ds)
    if len(s_dense) == 0 or (s_anchors[-1] - s_dense[-1]) > 1e-5:
        s_dense = np.append(s_dense, s_anchors[-1])
    
    x_ref = cs_x(s_dense)
    y_ref = cs_y(s_dense)
    
    dx = cs_x.derivative(1)(s_dense)
    dy = cs_y.derivative(1)(s_dense)
    theta_ref = [normalize_angle(math.atan2(dy[i], dx[i])) for i in range(len(s_dense))]
    
    ddx = cs_x.derivative(2)(s_dense)
    ddy = cs_y.derivative(2)(s_dense)
    
    v_ref = np.zeros_like(s_dense)
    for i in range(len(s_dense)):
        kappa = (dx[i] * ddy[i] - dy[i] * ddx[i]) / ((dx[i]**2 + dy[i]**2)**1.5 + 1e-8)
        
        if abs(kappa) > 0.01:
            v_turn = math.sqrt(a_lat_max / abs(kappa))
            v_ref[i] = min(v_max, v_turn)
        else:
            v_ref[i] = v_max
            
    waypoints = np.vstack((x_ref, y_ref, theta_ref, v_ref)).T
    return waypoints.tolist()
