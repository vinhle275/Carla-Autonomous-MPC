import math
import numpy as np
import carla
import pygame

class CameraViewer:
    """
    Cửa sổ hiển thị góc nhìn bám theo xe (Dedicated Chase / Follow Camera Window) bằng Pygame:
    - Mở một cửa sổ riêng biệt độc lập, tải hình ảnh trực tiếp từ sensor camera gắn trên xe.
    - Cực kỳ mượt mà, giảm thiểu tối đa hiện tượng giật lag so với việc xoay camera thủ công trên CARLA.
    - Hỗ trợ đổi góc nhìn nhanh bằng phím Tab hoặc V (Chase Cam sau xe, Bird-Eye trên cao, Hood Cam mui xe).
    - Hiển thị HUD thông số thời gian thực (vận tốc, góc lái, trạng thái né vật cản, số lượng vật cản).
    """

    VIEW_PRESETS = [
        {
            'name': 'CHASE CAM (Góc nhìn phía sau xe)',
            'transform': carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-12.0, yaw=0.0))
        },
        {
            'name': 'BIRD-EYE CAM (Góc nhìn từ trên cao bám xe)',
            'transform': carla.Transform(carla.Location(x=-2.0, z=18.0), carla.Rotation(pitch=-75.0, yaw=0.0))
        },
        {
            'name': 'HOOD CAM (Góc nhìn trước mui xe)',
            'transform': carla.Transform(carla.Location(x=2.0, z=1.2), carla.Rotation(pitch=-5.0, yaw=0.0))
        }
    ]

    def __init__(self, world, ego_vehicle, width=800, height=600):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.width = width
        self.height = height
        self.current_preset_idx = 0

        # Khởi tạo Pygame
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(
            (self.width, self.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        pygame.display.set_caption("Carla MPC - Camera Theo Dõi Xe (Nhấn 'V' đổi góc nhìn, 'ESC' đóng)")
        self.font = pygame.font.SysFont('DejaVu Sans', 16, bold=True)
        self.font_small = pygame.font.SysFont('DejaVu Sans', 13)

        self.surface = None
        self.sensor = None
        self._is_active = True

        # Gắn RGB Camera vào xe
        self._spawn_camera()

    def _spawn_camera(self):
        """Khởi tạo và gắn cảm biến Camera RGB lên xe Ego"""
        bp_lib = self.world.get_blueprint_library()
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', '90')

        initial_transform = self.VIEW_PRESETS[self.current_preset_idx]['transform']
        self.sensor = self.world.spawn_actor(
            camera_bp,
            initial_transform,
            attach_to=self.ego_vehicle
        )

        # Lắng nghe dữ liệu ảnh từ CARLA
        self.sensor.listen(self._on_camera_image)
        print(f"[CAMERA VIEWER] Đã kích hoạt cửa sổ camera theo dõi xe ({self.width}x{self.height}) - Góc nhìn: {self.VIEW_PRESETS[self.current_preset_idx]['name']}")

    def _on_camera_image(self, image):
        """Xử lý buffer hình ảnh từ cảm biến CARLA thành Surface của Pygame"""
        if not self._is_active:
            return
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        # Bỏ kênh alpha (BGRA -> BGR) và đổi sang RGB
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def switch_view(self):
        """Chuyển đổi luân phiên giữa các góc nhìn camera"""
        if not self.sensor or not self.sensor.is_alive:
            return
        self.current_preset_idx = (self.current_preset_idx + 1) % len(self.VIEW_PRESETS)
        preset = self.VIEW_PRESETS[self.current_preset_idx]
        self.sensor.set_transform(preset['transform'])
        print(f"[CAMERA VIEWER] Đã chuyển sang: {preset['name']}")

    def render(self, hud_info=None):
        """
        Cập nhật khung hình lên cửa sổ Pygame và vẽ lớp thông tin HUD:
        - hud_info: dict chứa {speed_kmh, target_kmh, steer_deg, bypass_text, obs_count, waypoint_idx, total_waypoints}
        """
        if not self._is_active:
            return True

        # Xử lý các sự kiện bàn phím trên cửa sổ Pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._is_active = False
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self._is_active = False
                    return False
                elif event.key in (pygame.K_v, pygame.K_TAB, pygame.K_c):
                    self.switch_view()

        # Vẽ ảnh camera
        if self.surface is not None:
            self.display.blit(self.surface, (0, 0))
        else:
            self.display.fill((20, 20, 20))
            loading_txt = self.font.render("Đang kết nối camera stream từ xe...", True, (200, 200, 200))
            self.display.blit(loading_txt, (self.width // 2 - 140, self.height // 2 - 10))

        # Vẽ lớp HUD thông số điều khiển (Overlay Dashboard)
        if hud_info is not None:
            self._draw_hud(hud_info)

        pygame.display.flip()
        return True

    def _draw_hud(self, hud_info):
        """Vẽ thanh thông số trong suốt ở góc màn hình"""
        hud_surface = pygame.Surface((self.width, 70), pygame.SRCALPHA)
        hud_surface.fill((10, 15, 25, 185))  # Nền bán trong suốt
        self.display.blit(hud_surface, (0, 0))

        speed_kmh = hud_info.get('speed_kmh', 0.0)
        target_kmh = hud_info.get('target_kmh', 0.0)
        steer_deg = hud_info.get('steer_deg', 0.0)
        bypass_text = hud_info.get('bypass_text', 'BÁM QUỸ ĐẠO')
        obs_count = hud_info.get('obs_count', 0)
        current_idx = hud_info.get('current_idx', 0)
        total_idx = hud_info.get('total_idx', 0)

        # Dòng 1: Vận tốc, góc lái và trạng thái né
        speed_color = (0, 255, 180) if speed_kmh >= 10.0 else (255, 200, 50)
        t1 = self.font.render(f"Tốc độ: {speed_kmh:4.1f} km/h (Mục tiêu: {target_kmh:4.1f})", True, speed_color)
        t2 = self.font.render(f"Góc lái: {steer_deg:+5.1f}°", True, (255, 255, 255))

        if "PHANH GẤP" in bypass_text:
            bypass_color = (255, 40, 40)
        elif "NÉ" in bypass_text or "DỪNG" in bypass_text:
            bypass_color = (255, 120, 60)
        else:
            bypass_color = (80, 220, 255)
        t3 = self.font.render(f"Trạng thái: {bypass_text}", True, bypass_color)

        self.display.blit(t1, (15, 10))
        self.display.blit(t2, (340, 10))
        self.display.blit(t3, (520, 10))

        if hud_info.get('is_emergency', False):
            pygame.draw.rect(self.display, (255, 30, 30), (0, 0, self.width, 4))

        # Dòng 2: Hướng dẫn phím và thông tin camera
        preset_name = self.VIEW_PRESETS[self.current_preset_idx]['name']
        t4 = self.font_small.render(f"[{preset_name}] | Nhấn 'V' hoặc 'Tab' để đổi góc nhìn | Vật cản nhìn thấy: {obs_count}", True, (200, 200, 200))
        t5 = self.font_small.render(f"Tiến độ: {current_idx}/{total_idx}", True, (160, 160, 160))

        self.display.blit(t4, (15, 40))
        self.display.blit(t5, (self.width - 150, 40))

    def destroy(self):
        """Dọn dẹp tài nguyên sensor và đóng cửa sổ Pygame"""
        self._is_active = False
        if self.sensor is not None and self.sensor.is_alive:
            try:
                self.sensor.stop()
                self.sensor.destroy()
                print("[CAMERA VIEWER] Đã giải phóng sensor camera an toàn.")
            except Exception as e:
                print(f"[CAMERA VIEWER] Cảnh báo khi dọn dẹp camera: {e}")
        try:
            pygame.quit()
        except Exception:
            pass
