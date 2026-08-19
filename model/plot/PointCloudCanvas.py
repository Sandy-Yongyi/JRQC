import os
import time
import math
import wx
import wx.glcanvas as glcanvas
import numpy as np
from OpenGL.GL import (
    glClearColor, glEnable, glPointSize, glClear, glViewport, glMatrixMode,
    glLoadIdentity, glScalef, glRotatef, glTranslatef, glHint,
    glBegin, glEnd, glVertex3f, glColor3f,
    glDrawArrays, glBindBuffer, glBufferData, glVertexPointer, glColorPointer,
    glEnableClientState, glDisableClientState,
    glGenBuffers, glDeleteBuffers,
    GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST, GL_POINT_SMOOTH, GL_POINT_SMOOTH_HINT,
    GL_NICEST, GL_COLOR_BUFFER_BIT, GL_PROJECTION, GL_MODELVIEW, GL_POINTS,
    GL_LINES, GL_ARRAY_BUFFER, GL_STATIC_DRAW, GL_VERTEX_ARRAY, GL_COLOR_ARRAY, GL_FLOAT
)
from OpenGL.GLU import gluPerspective
from model.utils.TomlLoader import TomlLoader


class PointCloudCanvas(glcanvas.GLCanvas):
    """
    在保持原有显示/交互/配色/边界不变的前提下，优化点云刷新：
    1) 使用 VBO/顶点数组 一次性提交点位与颜色（替代 Python 逐点 glVertex3f）
    2) 颜色与边界逻辑完全保持你原有代码
    3) 加入刷新合并（30 FPS），避免频繁 Refresh 导致的阻塞
    4) 方框线框仍然用原本的立即模式绘制，以确保外观/顺序一致
    """
    def __init__(self, parent):
        attribs = [
            glcanvas.WX_GL_RGBA,
            glcanvas.WX_GL_DOUBLEBUFFER,
            glcanvas.WX_GL_DEPTH_SIZE, 24,
        ]
        super().__init__(parent, -1, attribList=attribs)

        # --- 原有状态 ---
        self.context = glcanvas.GLContext(self)
        self.init = False
        self.points = np.empty((0, 3), dtype=np.float32)
        self.colors = None  # (N,3) or None

        self.box_lines = []   # 每个方框的线段点 (M_i,3)
        self.box_colors = []  # 每个方框的颜色 (3,)

        self.rotation_x = 0
        self.rotation_y = 70
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.last_mouse_pos = None
        self.distance = 10.0
        self.fov_y = 45.0
        self.pick_radius_px = 14.0
        self.scene_center = np.zeros(3, dtype=np.float32)
        self.orbit_center = None

        config_dir = os.getcwd() + "\\model\\tomls"
        self.read_data_config = TomlLoader.load(f"{config_dir}\\ReadDataConfig.toml")

        # --- 事件绑定（保持原样） ---
        self.Bind(wx.EVT_LEFT_DOWN, self.on_mouse_down)
        self.Bind(wx.EVT_LEFT_DCLICK, self.on_mouse_double_click)
        self.Bind(wx.EVT_LEFT_UP, self.on_mouse_up)
        self.Bind(wx.EVT_MOTION, self.on_mouse_move)
        self.Bind(wx.EVT_MOUSEWHEEL, self.on_mouse_wheel)
        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.Bind(wx.EVT_ERASE_BACKGROUND, lambda e: None)

        # --- OpenGL 初始化 ---
        self._vbo_enabled = True     # 支持标记（失败会自动降级）
        self._vbo_id_pos = None      # 点位置 VBO
        self._vbo_id_col = None      # 点颜色 VBO
        self._vbo_needs_upload = False

        # 刷新合并（节流）：最多 30 FPS
        self._last_paint_ts = 0.0
        self._min_paint_interval = 1.0 / 30.0
        self._repaint_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_repaint_timer, self._repaint_timer)
        self._repaint_pending = False

        self.init_gl()

    def init_gl(self):
        """初始化 OpenGL（保持你的原有设置）"""
        self.SetCurrent(self.context)
        try:
            glClearColor(1.0, 1.0, 1.0, 1.0)  # 背景白色（不变）
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_POINT_SMOOTH)
            glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)
            glPointSize(2)  # 点大小不变
            self.init = True
        except Exception as e:
            print(f"OpenGL初始化失败: {str(e)}")
            raise

    def _ensure_vbos(self):
        """创建 VBO（若失败自动降级）"""
        if not self._vbo_enabled:
            return
        try:
            if self._vbo_id_pos is None:
                self._vbo_id_pos = glGenBuffers(1)
            if self._vbo_id_col is None:
                self._vbo_id_col = glGenBuffers(1)
        except Exception as e:
            print(f"创建 VBO 失败，自动降级到客户端数组: {e}")
            self._vbo_enabled = False
            self._dispose_vbos()

    def _dispose_vbos(self):
        """释放 VBO 资源"""
        try:
            if self._vbo_id_pos:
                glDeleteBuffers(1, [self._vbo_id_pos])
            if self._vbo_id_col:
                glDeleteBuffers(1, [self._vbo_id_col])
        except Exception:
            pass
        self._vbo_id_pos = None
        self._vbo_id_col = None

    def _reset_view_state(self):
        """重置视角状态，确保新点云可重新完整显示。"""
        self.rotation_x = 0
        self.rotation_y = 70
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.distance = 10.0
        self.scene_center = np.zeros(3, dtype=np.float32)
        self.orbit_center = None
        self.last_mouse_pos = None

    def set_points(self, points):
        """
        更新点云数据（保持原始坐标轴与配色逻辑）：
        - 若包含颜色列（>=6），使用传入颜色
        - 否则仍采用“X轴蓝→绿→红渐变”逻辑（向量化）
        """
        if points is None or points.size == 0:
            self.points = np.empty((0, 3), dtype=np.float32)
            self.colors = None
            self._reset_view_state()
            self._vbo_needs_upload = True
            self._request_refresh()
            return

        # 坐标
        pts = np.asarray(points, dtype=np.float32)
        self.points = pts[:, :3].astype(np.float32, copy=False)
        self._reset_view_state()

        # 颜色
        if pts.shape[1] >= 6:
            self.colors = pts[:, 3:6].astype(np.float32, copy=False)
            # 保证颜色范围 0..1
            if self.colors.max() > 1.0:
                self.colors = np.clip(self.colors / 255.0, 0.0, 1.0)
        else:
            # 沿用原来的 X 轴渐变（蓝→绿→红），向量化实现
            x = self.points[:, 0]
            x_min = np.min(x)
            x_max = np.max(x)
            if x_max != x_min:
                xn = (x - x_min) / (x_max - x_min)
            else:
                xn = np.full_like(x, 0.5)

            # 前半段：蓝(0,0,1) -> 绿(0,1,0)
            # 后半段：绿(0,1,0) -> 红(1,0,0)
            r = np.zeros_like(xn, dtype=np.float32)
            g = np.zeros_like(xn, dtype=np.float32)
            b = np.zeros_like(xn, dtype=np.float32)
            mask = xn < 0.5
            g[mask] = xn[mask] * 2.0
            b[mask] = 1.0 - g[mask]
            r[~mask] = (xn[~mask] - 0.5) * 2.0
            g[~mask] = 1.0 - r[~mask]
            self.colors = np.stack([r, g, b], axis=1)

        # 标记需要把数据上传到显卡
        self._vbo_needs_upload = True
        self._request_refresh()

        print(f"接收到点数量: {len(self.points)}")

    def set_boxes(self, boxes_data):
        """设置方框线框数据，支持 BlockData 对象或字典格式"""
        self.box_lines = []
        self.box_colors = []
        if boxes_data is None:
            self._request_refresh()
            return

        # 检查是否是 BlockData 对象
        is_block_data_obj = hasattr(boxes_data, 'jig_data') and hasattr(boxes_data, 'outside_data') and hasattr(boxes_data, 'inside_data')

        if is_block_data_obj:
            outside_list = getattr(boxes_data, 'outside_data', []) or []
            out_x_min = None
            out_x_max = None
            if outside_list:
                out = outside_list[0]
                out_x_min = getattr(out, 'outside_x_min', None)
                out_x_max = getattr(out, 'outside_x_max', None)

                jig_list = getattr(boxes_data, 'jig_data', []) or []
                if out_x_min is not None and out_x_max is not None:
                    for jig in jig_list:
                        if getattr(jig, 'jig_y_min', None) is None or getattr(jig, 'jig_y_max', None) is None:
                            continue
                        if getattr(jig, 'jig_z_min', None) is None or getattr(jig, 'jig_z_max', None) is None:
                            continue

                        lines = self._create_box_lines(
                            out_x_min, out_x_max,
                            getattr(jig, 'jig_y_min', 0), getattr(jig, 'jig_y_max', 0),
                            getattr(jig, 'jig_z_min', 0), getattr(jig, 'jig_z_max', 0)
                        )
                        self.box_lines.append(lines)
                        self.box_colors.append([1.0, 0.0, 0.0])

                if getattr(out, 'outside_x_max', None) is not None:
                    lines = self._create_box_lines(
                        getattr(out, 'outside_x_min', 0), getattr(out, 'outside_x_max', 0),
                        getattr(out, 'outside_y_min', 0), getattr(out, 'outside_y_max', 0),
                        getattr(out, 'outside_z_min', 0), getattr(out, 'outside_z_max', 0)
                    )
                    self.box_lines.append(lines)
                    self.box_colors.append([0.0, 0.0, 1.0])

            inside_data = getattr(boxes_data, 'inside_data', []) or []
            for inside_obj in inside_data:
                subinside_list = getattr(inside_obj, 'subinside_datalist', []) or []
                for subinside in subinside_list:
                    lines = self._create_box_lines(
                        getattr(subinside, 'subinside_x_min', 0), getattr(subinside, 'subinside_x_max', 0),
                        getattr(subinside, 'subinside_y_min', 0), getattr(subinside, 'subinside_y_max', 0),
                        getattr(subinside, 'subinside_z_min', 0), getattr(subinside, 'subinside_z_max', 0)
                    )
                    self.box_lines.append(lines)
                    self.box_colors.append([0.0, 0.0, 0.0])

            self._request_refresh()
            return

        # jig_data（红）
        for jig in boxes_data.get("jig_data", []):
            lines = self._create_box_lines(
                jig["x_min"], jig["x_max"], jig["y_min"], jig["y_max"],
                jig["z_start"], jig["z_end"]
            )
            self.box_lines.append(lines)
            self.box_colors.append([1.0, 0.0, 0.0])

        # outside_data（蓝）
        out = boxes_data.get("outside_data", {})
        if out.get("x_max", -np.inf) > -np.inf:
            lines = self._create_box_lines(
                out["x_min"], out["x_max"], out["y_min"], out["y_max"],
                out["z_min"], out["z_max"]
            )
            self.box_lines.append(lines)
            self.box_colors.append([0.0, 0.0, 1.0])

        # inside_data（绿）
        inside_data = boxes_data.inside_data if is_block_data_obj else boxes_data.get("inside_data", [])
        if inside_data:
            for inside_obj in inside_data:
                subinside_list = getattr(inside_obj, 'subinside_datalist', [])
                if subinside_list:
                    for subinside in subinside_list:
                        lines = self._create_box_lines(
                            getattr(subinside, 'subinside_x_min', 0), getattr(subinside, 'subinside_x_max', 0),
                            getattr(subinside, 'subinside_y_min', 0), getattr(subinside, 'subinside_y_max', 0),
                            getattr(subinside, 'subinside_z_min', 0), getattr(subinside, 'subinside_z_max', 0)
                        )
                        self.box_lines.append(lines)
                        self.box_colors.append([0.0, 0.0, 0.0])

        self._request_refresh()

    def _create_box_lines(self, x_min, x_max, y_min, y_max, z_min, z_max):
        """生成方框的 12 条边（保持你的实现不变）"""
        corners = np.array([
            [x_min, y_min, z_min], [x_min, y_min, z_max],
            [x_min, y_max, z_min], [x_min, y_max, z_max],
            [x_max, y_min, z_min], [x_max, y_min, z_max],
            [x_max, y_max, z_min], [x_max, y_max, z_max]
        ], dtype=np.float32)

        edges = [
            (0, 1), (0, 2), (0, 4),
            (1, 3), (1, 5),
            (2, 3), (2, 6),
            (3, 7),
            (4, 5), (4, 6),
            (5, 7), (6, 7)
        ]

        lines = np.empty((len(edges) * 2, 3), dtype=np.float32)
        k = 0
        for i, j in edges:
            lines[k] = corners[i]
            k += 1
            lines[k] = corners[j]
            k += 1
        return lines

    def _request_refresh(self):
        """合并短时间内多次刷新请求，最大 30 FPS"""
        now = time.time()
        if now - self._last_paint_ts >= self._min_paint_interval:
            self.Refresh(False)
            self._last_paint_ts = now
            self._repaint_pending = False
        else:
            if not self._repaint_pending:
                delay_ms = max(1, int((self._min_paint_interval - (now - self._last_paint_ts)) * 1000))
                self._repaint_timer.StartOnce(delay_ms)
                self._repaint_pending = True

    def _on_repaint_timer(self, _evt):
        self.Refresh(False)
        self._last_paint_ts = time.time()
        self._repaint_pending = False

    def _get_view_plane_size(self):
        """返回当前观察距离下，视图中心平面的宽高（世界坐标）"""
        size = self.GetClientSize()
        width = max(1, size.width)
        height = max(1, size.height)
        view_height = 2.0 * self.distance * np.tan(np.deg2rad(self.fov_y * 0.5))
        view_width = view_height * (width / height)
        return view_width, view_height, width, height

    def _get_scene_metrics(self):
        """返回当前场景中心与视距参数。"""
        if self.points.size > 0:
            x_min = self.read_data_config["left_x_min"]
            x_max = self.read_data_config["left_x_max"]
            y_min = self.read_data_config["left_y_min"]
            y_max = self.read_data_config["left_y_max"]
            z_min, z_max = 0.0, float(self.read_data_config["max_scan_length"])

            width = x_max - x_min
            height = y_max - y_min
            depth = z_max - z_min
            max_dim = max(width, height, depth)
            center = np.array([
                (x_min + x_max) / 2.0,
                (y_min + y_max) / 2.0,
                (z_min + z_max) / 2.0,
            ], dtype=np.float32)
            distance = max_dim * 2.0 if max_dim > 0 else self.distance
            near = max(0.1, distance * 0.1)
            far = max(1000.0, distance * 10.0)
            return center, distance, near, far

        center = np.zeros(3, dtype=np.float32)
        return center, self.distance, 0.1, 100.0

    def _get_orbit_center(self):
        """返回当前旋转中心；未指定时回退到场景中心。"""
        if self.orbit_center is None:
            return self.scene_center
        return np.asarray(self.orbit_center, dtype=np.float32)

    def _get_rotation_matrix(self):
        """与 OpenGL 当前旋转顺序一致：先 Y 后 X。"""
        rx = math.radians(self.rotation_x)
        ry = math.radians(self.rotation_y)

        cos_x, sin_x = math.cos(rx), math.sin(rx)
        cos_y, sin_y = math.cos(ry), math.sin(ry)

        rot_x = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cos_x, -sin_x],
            [0.0, sin_x, cos_x],
        ], dtype=np.float32)

        rot_y = np.array([
            [cos_y, 0.0, sin_y],
            [0.0, 1.0, 0.0],
            [-sin_y, 0.0, cos_y],
        ], dtype=np.float32)

        return rot_x @ rot_y

    def _project_world_points(self, world_points, zoom=None, pan_x=None, pan_y=None):
        """将世界坐标投影到屏幕像素坐标，并返回相机坐标。"""
        if world_points is None or len(world_points) == 0:
            return np.empty((0, 2), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

        center, distance, near, _far = self._get_scene_metrics()
        self.scene_center = center
        orbit_center = self._get_orbit_center()

        zoom = self.zoom if zoom is None else zoom
        pan_x = self.pan_x if pan_x is None else pan_x
        pan_y = self.pan_y if pan_y is None else pan_y

        size = self.GetClientSize()
        width = max(1, size.width)
        height = max(1, size.height)
        aspect = width / height
        tan_half_fov = math.tan(math.radians(self.fov_y * 0.5))
        rotation = self._get_rotation_matrix()

        local_points = (np.asarray(world_points, dtype=np.float32) - orbit_center) * zoom
        camera_points = local_points @ rotation.T
        camera_points[:, 0] += pan_x
        camera_points[:, 1] += pan_y
        camera_points[:, 2] -= distance

        z = camera_points[:, 2]
        valid = z < -near
        screen = np.full((len(camera_points), 2), np.nan, dtype=np.float32)
        if not np.any(valid):
            return screen, camera_points

        inv_neg_z = 1.0 / (-z[valid])
        ndc_x = camera_points[valid, 0] * inv_neg_z / (tan_half_fov * aspect)
        ndc_y = camera_points[valid, 1] * inv_neg_z / tan_half_fov

        screen[valid, 0] = (ndc_x * 0.5 + 0.5) * width
        screen[valid, 1] = (1.0 - (ndc_y * 0.5 + 0.5)) * height
        return screen, camera_points

    def _pick_point(self, mouse_pos):
        """按当前屏幕投影拾取最接近鼠标的点云点。"""
        if self.points.size == 0:
            return None

        screen_points, camera_points = self._project_world_points(self.points[:, :3])
        if len(screen_points) == 0:
            return None

        dx = screen_points[:, 0] - mouse_pos.x
        dy = screen_points[:, 1] - mouse_pos.y
        valid = np.isfinite(dx) & np.isfinite(dy)
        if not np.any(valid):
            return None

        dist_sq = dx[valid] * dx[valid] + dy[valid] * dy[valid]
        within = dist_sq <= (self.pick_radius_px * self.pick_radius_px)
        if not np.any(within):
            return None

        valid_indices = np.flatnonzero(valid)
        candidate_indices = valid_indices[within]
        candidate_dist = dist_sq[within]
        candidate_depth = camera_points[candidate_indices, 2]

        best_order = np.lexsort((candidate_depth, candidate_dist))
        best_index = candidate_indices[best_order[0]]
        return self.points[best_index, :3].astype(np.float32, copy=False)

    def _zoom_to_picked_point(self, mouse_pos, new_zoom):
        """以拾取到的点为中心缩放；若未拾取到则退化为近似鼠标缩放。"""
        picked_point = self._pick_point(mouse_pos)
        if picked_point is None:
            view_width, view_height, width_px, height_px = self._get_view_plane_size()
            dx_px = mouse_pos.x - (width_px * 0.5)
            dy_px = (height_px * 0.5) - mouse_pos.y

            old_world_x = (dx_px / width_px) * view_width / self.zoom
            old_world_y = (dy_px / height_px) * view_height / self.zoom
            new_world_x = (dx_px / width_px) * view_width / new_zoom
            new_world_y = (dy_px / height_px) * view_height / new_zoom

            self.pan_x += new_world_x - old_world_x
            self.pan_y += new_world_y - old_world_y
            self.zoom = new_zoom
            return

        center, distance, near, _far = self._get_scene_metrics()
        self.scene_center = center
        orbit_center = self._get_orbit_center()
        rotation = self._get_rotation_matrix()
        local_point = (picked_point - orbit_center) * new_zoom
        camera_point = rotation @ local_point
        camera_z = camera_point[2] - distance

        if camera_z >= -near:
            self.zoom = new_zoom
            return

        size = self.GetClientSize()
        width = max(1, size.width)
        height = max(1, size.height)
        aspect = width / height
        tan_half_fov = math.tan(math.radians(self.fov_y * 0.5))

        ndc_x = (mouse_pos.x / width) * 2.0 - 1.0
        ndc_y = 1.0 - (mouse_pos.y / height) * 2.0

        desired_cam_x = ndc_x * (-camera_z) * tan_half_fov * aspect
        desired_cam_y = ndc_y * (-camera_z) * tan_half_fov

        self.pan_x = desired_cam_x - camera_point[0]
        self.pan_y = desired_cam_y - camera_point[1]
        self.zoom = new_zoom

    def on_paint(self, event):
        if not self.init:
            return

        wx.PaintDC(self)
        self.SetCurrent(self.context)

        # 视口 & 投影
        size = self.GetClientSize()
        glViewport(0, 0, size.width, size.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        center, self.distance, near, far = self._get_scene_metrics()
        self.scene_center = center
        orbit_center = self._get_orbit_center()
        orbit_x, orbit_y, orbit_z = orbit_center.tolist()
        gluPerspective(self.fov_y, size.width / max(1, size.height), near, far)

        # 清屏（背景白）
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # type: ignore

        # 模型视图矩阵
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -self.distance)
        glTranslatef(self.pan_x, self.pan_y, 0.0)
        glRotatef(self.rotation_x, 1.0, 0.0, 0.0)
        glRotatef(self.rotation_y, 0.0, 1.0, 0.0)

        glScalef(self.zoom, self.zoom, self.zoom)
        glTranslatef(-orbit_x, -orbit_y, -orbit_z)

        # --- 绘制点云（优化：VBO/顶点数组） ---
        if self.points.size > 0:
            self._draw_points_fast()

        # --- 绘制方框线框 ---
        for i, lines in enumerate(self.box_lines):
            if len(lines) == 0:
                continue
            c = self.box_colors[i]
            glColor3f(c[0], c[1], c[2])
            glBegin(GL_LINES)
            for p in lines:
                glVertex3f(p[0], p[1], p[2])
            glEnd()

        self.SwapBuffers()

    def _draw_points_fast(self):
        """使用 VBO / 顶点数组绘制点云，外观保持不变"""
        n = self.points.shape[0]
        if n == 0:
            return

        # 确保 VBO 可用（失败自动降级）
        self._ensure_vbos()

        # 需要上传/更新显存
        if self._vbo_enabled and self._vbo_needs_upload:
            try:
                glBindBuffer(GL_ARRAY_BUFFER, self._vbo_id_pos)
                glBufferData(GL_ARRAY_BUFFER, self.points.astype(np.float32, copy=False), GL_STATIC_DRAW)
                glBindBuffer(GL_ARRAY_BUFFER, self._vbo_id_col)
                glBufferData(GL_ARRAY_BUFFER, self.colors.astype(np.float32, copy=False), GL_STATIC_DRAW)  # type: ignore
                glBindBuffer(GL_ARRAY_BUFFER, 0)
                self._vbo_needs_upload = False
            except Exception as e:
                print(f"VBO 上传失败，降级到客户端数组: {e}")
                self._vbo_enabled = False
                self._dispose_vbos()

        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)

        if self._vbo_enabled:
            # 使用 VBO
            glBindBuffer(GL_ARRAY_BUFFER, self._vbo_id_pos)
            glVertexPointer(3, GL_FLOAT, 0, None)
            glBindBuffer(GL_ARRAY_BUFFER, self._vbo_id_col)
            glColorPointer(3, GL_FLOAT, 0, None)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
        else:
            # 客户端数组
            glVertexPointer(3, GL_FLOAT, 0, self.points)
            glColorPointer(3, GL_FLOAT, 0, self.colors)

        glDrawArrays(GL_POINTS, 0, n)

        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)

    def on_mouse_down(self, event):
        self.last_mouse_pos = event.GetPosition()
        event.Skip()

    def on_mouse_double_click(self, event):
        picked_point = self._pick_point(event.GetPosition())
        if picked_point is not None:
            self.orbit_center = np.array(picked_point, dtype=np.float32)
            print(
                f"旋转中心已设置为: X={picked_point[0]:.2f}, "
                f"Y={picked_point[1]:.2f}, Z={picked_point[2]:.2f}"
            )
            self._request_refresh()
        event.Skip()

    def on_mouse_up(self, event):
        self.last_mouse_pos = None
        event.Skip()

    def on_mouse_move(self, event):
        if not self.last_mouse_pos:
            return

        current_pos = event.GetPosition()
        dx = current_pos.x - self.last_mouse_pos.x
        dy = current_pos.y - self.last_mouse_pos.y

        if event.Dragging() and event.LeftIsDown():
            # 左键拖拽 = 旋转
            self.rotation_y += dx * 0.5
            self.rotation_x += dy * 0.5
            self.rotation_x = max(-90, min(90, self.rotation_x))

        elif event.Dragging() and event.RightIsDown():
            # 右键拖拽 = 平移
            view_width, view_height, width_px, height_px = self._get_view_plane_size()
            self.pan_x -= (dx / width_px) * view_width
            self.pan_y += (dy / height_px) * view_height

        self.last_mouse_pos = current_pos
        self._request_refresh()
        event.Skip()

    def on_mouse_wheel(self, event):
        wheel_rotation = event.GetWheelRotation()
        if wheel_rotation == 0:
            event.Skip()
            return

        old_zoom = self.zoom
        zoom_factor = 1.0 + wheel_rotation * 0.001
        new_zoom = max(0.1, min(10.0, old_zoom * zoom_factor))

        if new_zoom != old_zoom:
            self._zoom_to_picked_point(event.GetPosition(), new_zoom)

        self._request_refresh()
        event.Skip()

    def on_size(self, event):
        if not self.init:
            return
        size = self.GetClientSize()
        self.SetCurrent(self.context)
        glViewport(0, 0, size.width, size.height)
        self._request_refresh()
        event.Skip()

    def Destroy(self):
        try:
            self._dispose_vbos()
        except Exception:
            pass
        super().Destroy()
