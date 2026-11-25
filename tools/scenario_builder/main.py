#!/usr/bin/env python3
"""
==============================================================================
PROCESS 0: scenario_builder (Offline Scenario & Trajectory Setup GUI)
- Click waypoints to generate Cubic Spline trajectory
- Click to place static obstacles (obstacles.csv)
- Draw moving obstacle paths (moving_obstacles.csv)
==============================================================================
"""

import os
import sys
import csv

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
TRAJECTORY_CSV_PATH = os.path.join(BASE_DIR, "configs/trajectories/reference_trajectory.csv")
OBSTACLES_CSV_PATH = os.path.join(BASE_DIR, "configs/trajectories/obstacles.csv")
MOVING_OBSTACLES_CSV_PATH = os.path.join(BASE_DIR, "configs/trajectories/moving_obstacles.csv")


def save_waypoints_to_csv(waypoints, filename=TRAJECTORY_CSV_PATH):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'theta_rad', 'v_m_s'])
        for wp in waypoints:
            writer.writerow([round(wp[0], 4), round(wp[1], 4),
                             round(wp[2], 4), round(wp[3], 4)])
    print(f"\n[THÀNH CÔNG] Đã lưu {len(waypoints)} waypoints vào: {filename}")


def save_obstacles_to_csv(obstacles, filename=OBSTACLES_CSV_PATH):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'size'])
        for obs in obstacles:
            writer.writerow([round(obs['x'], 4), round(obs['y'], 4), 0.5])
    print(f"[THÀNH CÔNG] Đã lưu {len(obstacles)} vật cản tĩnh vào: {filename}")


def save_moving_obstacles_to_csv(moving_obstacles, filename=MOVING_OBSTACLES_CSV_PATH):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['obstacle_id', 'wp_index', 'x', 'y', 'speed_ms'])
        for obs in moving_obstacles:
            obs_id = obs['id']
            speed = obs['speed_ms']
            for wp_idx, wp in enumerate(obs['waypoints']):
                writer.writerow([obs_id, wp_idx,
                                 round(wp['x'], 4), round(wp['y'], 4),
                                 round(speed, 4)])
    print(f"[THÀNH CÔNG] Đã lưu {len(moving_obstacles)} vật cản di động vào: {filename}")


def main():
    print("==========================================================")
    print("   PROCESS 0: scenario_builder (GUI Tool Tiền kỳ)        ")
    print("==========================================================")
    print(f"Trajectoy file: {TRAJECTORY_CSV_PATH}")
    print(f"Obstacles file: {OBSTACLES_CSV_PATH}")
    print(f"Moving obs file: {MOVING_OBSTACLES_CSV_PATH}")
    print("Nếu các file CSV trên đã tồn tại, bạn có thể chạy mô phỏng trực tiếp!")


if __name__ == '__main__':
    main()
