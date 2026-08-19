import wx
from control.MachineConfigFrameControl import load_machine_config_for_ui, save_machine_config_from_ui


class MachineConfigFrame(wx.Dialog):
    PARAM_LABELS = {
        "tracking": "是否跟踪(0否/1是)",
        "y_move_min": "Y轴往复最小位置(mm)",
        "y_move_max": "Y轴往复最大位置(mm)",
        "out_front_x_offset": "外侧前X轴偏移(mm)",
        "out_after_x_offset": "外侧后X轴偏移(mm)",
        "in_front_x_offset": "内侧前X轴偏移(mm)",
        "in_after_x_offset": "内侧后X轴偏移(mm)",
        "x_pos_speed": "X轴定位速度(mm/s)",
        "x_recip_speed": "X轴往复速度(mm/s)",
        "x_status_offset": "喷枪出粉偏移(mm)",
        "origin_pos": "Y轴原点位置(mm)",
        "out_up_y_offset": "外侧上顶Y轴偏移(mm)",
        "out_down_y_offset": "外侧下底Y轴偏移(mm)",
        "in_up_y_offset": "内侧上顶Y轴偏移(mm)",
        "in_down_y_offset": "内侧下底Y轴偏移(mm)",
        "y_pos_speed": "Y轴定位速度(mm/s)",
        "y_recip_speed": "Y轴往复速度(mm/s)",
        "out_z_front_offset": "外侧Z轴前定位偏移(mm)",
        "out_z_after_offset": "外侧Z轴后定位偏移(mm)",
        "in_z_front_offset": "内侧Z轴前定位偏移(mm)",
        "in_z_after_offset": "内侧Z轴后定位偏移(mm)",
        "z_back_speed": "Z轴反喷速度(mm/s)",
        "z_zeroing_speed": "Z轴归零速度(mm/s)",
        "outside_total_cycles": "外侧往复喷涂次数",
        "inside_total_cycles": "内侧往复喷涂次数",
        "recip_reduce_distance": "往复减少距离(mm)",
    }

    FRAME_BY_FRAME_PARAM_KEYS = [
        "tracking",
        "y_move_min",
        "y_move_max",
        "out_front_x_offset",
        "out_after_x_offset",
        "x_pos_speed",
        "x_recip_speed",
        "out_up_y_offset",
        "out_down_y_offset",
        "y_pos_speed",
        "y_recip_speed",
        "out_z_front_offset",
        "out_z_after_offset",
        "z_back_speed",
        "z_zeroing_speed",
        "x_status_offset",
        "outside_total_cycles",
    ]

    DEVICE_PARAM_KEYS = {
        0: [
            "out_front_x_offset", "x_pos_speed", "x_recip_speed", "x_status_offset", "out_up_y_offset", "out_down_y_offset", "y_pos_speed", "y_recip_speed",
            "out_z_front_offset", "out_z_after_offset", "z_back_speed", "z_zeroing_speed", "outside_total_cycles", "recip_reduce_distance",
        ],
        1: [
            "out_front_x_offset", "out_after_x_offset", "in_front_x_offset", "in_after_x_offset", "x_pos_speed", "x_recip_speed",
            "x_status_offset",
            "out_up_y_offset", "out_down_y_offset", "in_up_y_offset", "in_down_y_offset", "y_pos_speed", "y_recip_speed",
            "out_z_front_offset", "out_z_after_offset", "in_z_front_offset", "in_z_after_offset", "z_back_speed", "z_zeroing_speed",
            "outside_total_cycles", "inside_total_cycles", "recip_reduce_distance",
        ],
        2: [
            "out_front_x_offset", "out_after_x_offset", "in_front_x_offset", "in_after_x_offset", "x_pos_speed", "x_recip_speed",
            "x_status_offset",
            "out_up_y_offset", "out_down_y_offset", "in_up_y_offset", "in_down_y_offset", "y_pos_speed", "y_recip_speed",
            "out_z_front_offset", "out_z_after_offset", "in_z_front_offset", "in_z_after_offset", "z_back_speed", "z_zeroing_speed",
            "outside_total_cycles", "inside_total_cycles", "recip_reduce_distance",
        ],
    }

    def __init__(self, parent, sn: int, control_queue=None,
                 strategy_name="frame_by_frame", title_prefix="设备参数设置"):
        super().__init__(parent, title=f"{title_prefix} - SN[{sn}]",
                         style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.sn = sn
        self.control_queue = control_queue
        self.strategy_name = strategy_name
        self.ctrls = {}
        self.flat_ctrls = {}
        self.machine_cfg = load_machine_config_for_ui(self.sn, self.strategy_name)
        self.PARAM_ORDER = self._get_param_order_for_config(sn, self.machine_cfg)
        self._build_ui()
        self._load_values()
        if self.control_queue is None:
            wx.MessageBox("警告：控制队列无效，修改的参数可能无法实时生效", "警告", wx.OK | wx.ICON_WARNING)

    def _get_param_order_for_config(self, sn: int, machine_cfg: dict):
        """根据SN号返回需要显示的参数列表"""
        if self.strategy_name == "frame_by_frame":
            keys = self.FRAME_BY_FRAME_PARAM_KEYS
        else:
            keys = self.DEVICE_PARAM_KEYS.get(sn, [])
        return [(key, self.PARAM_LABELS[key]) for key in keys if key in self.PARAM_LABELS]

    def _uses_dual_config_pages(self):
        machine_type = self.machine_cfg.get("type")
        return self.strategy_name == "complete_workpiece" and machine_type == "xn_side"

    def _build_param_grid(self, parent, ctrls: dict):
        grid = wx.FlexGridSizer(rows=0, cols=2, hgap=8, vgap=8)
        grid.AddGrowableCol(1, 1)

        for key, label in self.PARAM_ORDER:
            grid.Add(wx.StaticText(parent, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.TextCtrl(parent)
            ctrls[key] = ctrl
            grid.Add(ctrl, 1, wx.EXPAND)
        return grid

    def _build_param_scroller(self, parent, ctrls: dict):
        scroller = wx.ScrolledWindow(parent, style=wx.VSCROLL | wx.HSCROLL)
        scroller.SetScrollRate(12, 12)

        scroller_sizer = wx.BoxSizer(wx.VERTICAL)
        scroller_sizer.Add(self._build_param_grid(scroller, ctrls), 0, wx.ALL | wx.EXPAND, 15)
        scroller.SetSizer(scroller_sizer)
        scroller.FitInside()

        content_size = scroller_sizer.CalcMin()
        display_w, display_h = wx.GetDisplaySize()
        max_width = max(520, int(display_w * 0.8))
        max_height = max(320, int(display_h * 0.65))
        width = min(content_size.GetWidth() + wx.SystemSettings.GetMetric(wx.SYS_VSCROLL_X), max_width)
        height = min(content_size.GetHeight() + wx.SystemSettings.GetMetric(wx.SYS_HSCROLL_Y), max_height)
        scroller.SetMinSize(wx.Size(width, height))
        return scroller

    def _build_ui(self):
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        if self._uses_dual_config_pages():
            notebook = wx.Notebook(self)

            cabinet_panel = wx.Panel(notebook)
            cabinet_sizer = wx.BoxSizer(wx.VERTICAL)
            cabinet_sizer.Add(self._build_param_scroller(cabinet_panel, self.ctrls), 1, wx.EXPAND)
            cabinet_panel.SetSizer(cabinet_sizer)
            notebook.AddPage(cabinet_panel, "柜体配置")

            flat_panel = wx.Panel(notebook)
            flat_sizer = wx.BoxSizer(wx.VERTICAL)
            flat_sizer.Add(self._build_param_scroller(flat_panel, self.flat_ctrls), 1, wx.EXPAND)
            flat_panel.SetSizer(flat_sizer)
            notebook.AddPage(flat_panel, "平板配置")

            main_sizer.Add(notebook, 1, wx.ALL | wx.EXPAND, 10)
        else:
            main_sizer.Add(self._build_param_scroller(self, self.ctrls), 1, wx.ALL | wx.EXPAND, 10)

        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        save_btn = wx.Button(self, label="保存参数修改")
        cancel_btn = wx.Button(self, label="取消")

        save_btn.Bind(wx.EVT_BUTTON, self.on_save)
        cancel_btn.Bind(wx.EVT_BUTTON, lambda e: self.EndModal(wx.ID_CANCEL))

        btn_sizer.AddStretchSpacer()
        btn_sizer.Add(save_btn, 0, wx.RIGHT, 10)
        btn_sizer.Add(cancel_btn, 0)

        main_sizer.Add(btn_sizer, 0, wx.ALL | wx.EXPAND, 10)

        self.SetSizer(main_sizer)
        self.Layout()
        main_sizer.Fit(self)
        self.SetMinSize(self.GetSize())
        self.Centre()

    def _load_ctrl_values(self, ctrls: dict, values: dict, fallback_values: dict | None = None):
        fallback_values = fallback_values or {}
        for key, ctrl in ctrls.items():
            if key in values:
                ctrl.SetValue(str(values[key]))
            elif key in fallback_values:
                ctrl.SetValue(str(fallback_values[key]))

    def _load_values(self):
        cfg = self.machine_cfg or load_machine_config_for_ui(self.sn, self.strategy_name)
        self._load_ctrl_values(self.ctrls, cfg)

        if self._uses_dual_config_pages():
            flat_cfg = cfg.get("flat")
            if not isinstance(flat_cfg, dict):
                flat_cfg = {}
            self._load_ctrl_values(self.flat_ctrls, flat_cfg, cfg)

    def _collect_values(self, ctrls: dict):
        values = {}
        for key, ctrl in ctrls.items():
            values[key] = int(ctrl.GetValue())
        return values

    def on_save(self, event):
        try:
            values = self._collect_values(self.ctrls)
            if self._uses_dual_config_pages():
                values["flat"] = self._collect_values(self.flat_ctrls)

            save_machine_config_from_ui(
                self.sn,
                values,
                self.control_queue,
                strategy_name=self.strategy_name,
            )
            wx.MessageBox("参数已保存并生效", "成功", wx.OK | wx.ICON_INFORMATION)
            self.EndModal(wx.ID_OK)
        except Exception as e:
            wx.MessageBox(str(e), "参数错误", wx.OK | wx.ICON_ERROR)
