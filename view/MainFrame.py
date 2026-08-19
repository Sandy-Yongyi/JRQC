import wx
import os
# import sys
# # 将项目的根目录添加到 sys.path 中
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from model.plot.PointCloudCanvas import PointCloudCanvas


BG_COLOR_LIGHT_GREEN = wx.Colour(240, 250, 250)
BG_COLOR_WHITE = wx.WHITE
BG_COLOR_GREEN = wx.Colour(107, 175, 107)
BG_COLOR_RED = wx.Colour(244, 100, 80)
BG_COLOR_BLUE = wx.Colour(150, 230, 244)


class BorderedPanel(wx.Panel):
    """带黑色边框的面板。"""

    def __init__(self, parent, label="", bg_color=None):
        super().__init__(parent)
        self.label = label
        self.bg_color = bg_color or wx.Colour(220, 247, 221)
        self.SetBackgroundColour(self.bg_color)
        self.Bind(wx.EVT_PAINT, self._on_paint)

    def _on_paint(self, event):
        dc = wx.PaintDC(self)
        rect = self.GetClientRect()

        dc.SetBrush(wx.Brush(self.bg_color))
        dc.DrawRectangle(rect)

        dc.SetPen(wx.Pen(wx.BLACK, 3))
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.DrawRectangle(0, 0, rect.width, rect.height)

        if self.label:
            font = wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
            dc.SetFont(font)
            dc.SetTextForeground(wx.BLACK)
            text_width, text_height = dc.GetTextExtent(self.label)
            label_x = 15
            label_y = 15
            bg_margin = 3

            dc.SetBrush(wx.Brush(self.bg_color))
            dc.SetPen(wx.TRANSPARENT_PEN)
            dc.DrawRectangle(
                label_x - bg_margin,
                label_y - bg_margin,
                text_width + 2 * bg_margin,
                text_height + 2 * bg_margin,
            )
            dc.DrawText(self.label, label_x, label_y)


class ModernButton(wx.Button):
    """项目原有的彩色按钮。"""

    def __init__(self, parent, label, size=wx.Size(150, 50), bg_color=None, text_color=None):
        super().__init__(parent, label=label, size=size)
        self.bg_color = bg_color or BG_COLOR_BLUE
        self.text_color = text_color or wx.BLACK
        self.disabled_color = wx.Colour(180, 180, 180)
        self.visual_active = True
        self.current_text_color = self.text_color

        font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.SetFont(font)
        self.Bind(wx.EVT_PAINT, self._on_paint)
        self._update_button_appearance()

    def _update_button_appearance(self):
        if self.IsEnabled() and self.visual_active:
            self.current_bg = wx.Colour(
                max(0, self.bg_color.red - 20),
                max(0, self.bg_color.green - 20),
                max(0, self.bg_color.blue - 20),
            )
            self.current_text_color = self.text_color
        else:
            self.current_bg = self.disabled_color
            self.current_text_color = wx.BLACK

        self.SetBackgroundColour(self.current_bg)
        self.SetForegroundColour(self.current_text_color)

    def SetVisualActive(self, active=True):
        self.visual_active = active
        self._update_button_appearance()
        self.Refresh()

    def Enable(self, enable=True):
        super().Enable(enable)
        self._update_button_appearance()
        self.Refresh()

    def _on_paint(self, event):
        dc = wx.PaintDC(self)
        rect = self.GetClientRect()

        dc.SetBrush(wx.Brush(self.current_bg))
        dc.SetPen(wx.Pen(self.current_bg))
        dc.DrawRectangle(rect)

        border_color = wx.Colour(
            max(0, self.current_bg.red - 30),
            max(0, self.current_bg.green - 30),
            max(0, self.current_bg.blue - 30),
        )
        dc.SetPen(wx.Pen(border_color, 2))
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.DrawRectangle(0, 0, rect.width - 1, rect.height - 1)

        label = self.GetLabel()
        font = self.GetFont()
        dc.SetFont(font)
        text_width, text_height = dc.GetTextExtent(label)
        x = (rect.width - text_width) // 2
        y = (rect.height - text_height) // 2
        dc.SetTextForeground(self.current_text_color)
        dc.DrawText(label, x, y)


class MainFrame(wx.Frame):
    def __init__(self, parent):
        super().__init__(parent, title="Joihey Software")
        self.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)

        icon_path = os.path.join(os.path.dirname(__file__), "..", "data", "picture", "junhe.ico")
        if os.path.exists(icon_path):
            icon = wx.Icon(icon_path, wx.BITMAP_TYPE_ICO)
            self.SetIcon(icon)

        self.InitUI()
        self.Maximize(True)
        self._set_control_buttons_active(False)
        self._set_param_buttons_enabled(False)

    def InitUI(self):
        scrolled_window = wx.ScrolledWindow(self)
        scrolled_window.SetScrollRate(20, 20)
        scrolled_window.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)

        panel = wx.Panel(scrolled_window)
        panel.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)

        left_scrolled = wx.ScrolledWindow(panel, size=wx.Size(430, -1))
        left_scrolled.SetScrollRate(10, 10)
        left_scrolled.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)
        left_scrolled.SetMinSize(wx.Size(430, -1))
        left_vbox = wx.BoxSizer(wx.VERTICAL)

        control_card = BorderedPanel(left_scrolled, label="操作控制", bg_color=BG_COLOR_LIGHT_GREEN)
        control_card_sizer = wx.BoxSizer(wx.VERTICAL)
        btn_sizer1 = wx.BoxSizer(wx.HORIZONTAL)
        self.start_btn = ModernButton(
            control_card,
            label="程序开始运行",
            size=wx.Size(180, 55),
            bg_color=BG_COLOR_GREEN,
            text_color=wx.WHITE,
        )
        self.stop_btn = ModernButton(
            control_card,
            label="程序结束运行",
            size=wx.Size(180, 55),
            bg_color=BG_COLOR_RED,
            text_color=wx.WHITE,
        )
        btn_sizer1.Add(self.start_btn, 0, wx.ALL, 8)
        btn_sizer1.Add(self.stop_btn, 0, wx.ALL, 8)
        control_card_sizer.AddSpacer(30)
        control_card_sizer.Add(btn_sizer1, 0, wx.EXPAND | wx.ALL, 5)
        control_card.SetSizer(control_card_sizer)

        param_card = BorderedPanel(left_scrolled, label="参数设置", bg_color=BG_COLOR_LIGHT_GREEN)
        param_card_sizer = wx.BoxSizer(wx.VERTICAL)
        status_label = wx.StaticText(param_card, label="         左侧 →                → 右侧")
        status_label.SetFont(wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
        status_label.SetForegroundColour(wx.BLACK)

        btn_sizer2 = wx.BoxSizer(wx.HORIZONTAL)
        self.left_out_fx_btn = ModernButton(param_card, label="仿形升降机(1号)", size=wx.Size(180, 50))
        self.right_xn_side_btn = ModernButton(param_card, label="右侧侧面云雀(3号)", size=wx.Size(180, 50))
        btn_sizer2.Add(self.left_out_fx_btn, 0, wx.ALL, 8)
        btn_sizer2.Add(self.right_xn_side_btn, 0, wx.ALL, 8)

        btn_sizer3 = wx.BoxSizer(wx.HORIZONTAL)
        self.left_xn_side_btn = ModernButton(param_card, label="左侧侧面云雀(2号)", size=wx.Size(180, 50))
        btn_sizer3.Add(self.left_xn_side_btn, 0, wx.ALL, 8)

        param_card_sizer.AddSpacer(50)
        param_card_sizer.Add(status_label, 0, wx.EXPAND | wx.ALL, 5)
        param_card_sizer.AddSpacer(5)
        param_card_sizer.Add(btn_sizer2, 0, wx.EXPAND | wx.ALL, 5)
        param_card_sizer.Add(btn_sizer3, 0, wx.EXPAND | wx.ALL, 5)
        param_card.SetSizer(param_card_sizer)

        left_vbox.Add(control_card, 0, wx.EXPAND | wx.ALL, 10)
        left_vbox.Add(param_card, 0, wx.EXPAND | wx.ALL, 10)
        left_vbox.AddStretchSpacer()
        left_scrolled.SetSizer(left_vbox)

        right_panel = wx.Panel(panel)
        right_panel.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)
        right_sizer = wx.BoxSizer(wx.HORIZONTAL)

        display_panel = BorderedPanel(right_panel, label="3D点云画图显示", bg_color=BG_COLOR_LIGHT_GREEN)
        display_panel.SetMinSize(wx.Size(300, 200))
        display_sizer = wx.BoxSizer(wx.VERTICAL)
        self.gl_canvas = PointCloudCanvas(display_panel)
        display_sizer.AddSpacer(30)
        display_sizer.Add(self.gl_canvas, 1, wx.EXPAND | wx.ALL, 8)
        display_panel.SetSizer(display_sizer)

        status_panel = BorderedPanel(right_panel, label="状态输出", bg_color=BG_COLOR_LIGHT_GREEN)
        status_panel.SetMinSize(wx.Size(300, -1))
        status_sizer = wx.BoxSizer(wx.VERTICAL)
        self.status_text = wx.TextCtrl(status_panel, style=wx.TE_MULTILINE | wx.TE_READONLY)
        status_font = wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.status_text.SetFont(status_font)
        self.status_text.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)
        status_sizer.AddSpacer(30)
        status_sizer.Add(self.status_text, 1, wx.EXPAND | wx.ALL, 8)
        status_panel.SetSizer(status_sizer)

        right_sizer.Add(display_panel, 1, wx.EXPAND | wx.ALL, 10)
        right_sizer.Add(status_panel, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT, 10)
        right_panel.SetSizer(right_sizer)

        main_sizer.Add(left_scrolled, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.LEFT, 10)
        main_sizer.Add(right_panel, 1, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT, 10)
        panel.SetSizer(main_sizer)

        scrolled_sizer = wx.BoxSizer(wx.VERTICAL)
        scrolled_sizer.Add(panel, 1, wx.EXPAND)
        scrolled_window.SetSizer(scrolled_sizer)

        default_font = wx.SystemSettings.GetFont(wx.SYS_DEFAULT_GUI_FONT)
        self.SetFont(default_font)
        left_scrolled.SetVirtualSize(left_vbox.GetMinSize())

    def _set_param_buttons_enabled(self, enabled: bool):
        self.left_out_fx_btn.Enable(enabled)
        self.left_xn_side_btn.Enable(enabled)
        self.right_xn_side_btn.Enable(enabled)

    def _set_control_buttons_active(self, active: bool):
        self.start_btn.SetVisualActive(active)
        self.stop_btn.SetVisualActive(active)
        self.start_btn.Enable(True)
        self.stop_btn.Enable(active)

    def on_program_started(self):
        self._set_control_buttons_active(True)
        self._set_param_buttons_enabled(True)

    def on_program_stopped(self):
        self._set_control_buttons_active(False)
        self._set_param_buttons_enabled(False)


if __name__ == "__main__":
    app = wx.App()
    frame = MainFrame(None)
    frame.Show()
    app.MainLoop()
