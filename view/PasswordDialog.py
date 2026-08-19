import wx


class PasswordDialog(wx.Dialog):
    def __init__(self, parent, account_name="河村电器", title="身份验证"):
        super().__init__(parent, title=title, style=wx.DEFAULT_DIALOG_STYLE)
        self.account_name = account_name
        self.password_ctrl = None
        self._build_ui()

    def _build_ui(self):
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        form_sizer = wx.FlexGridSizer(rows=0, cols=2, hgap=10, vgap=12)
        form_sizer.AddGrowableCol(1, 1)

        form_sizer.Add(wx.StaticText(self, label="账户名："), 0, wx.ALIGN_CENTER_VERTICAL)
        account_ctrl = wx.TextCtrl(self, value=self.account_name, style=wx.TE_READONLY)
        form_sizer.Add(account_ctrl, 1, wx.EXPAND)

        form_sizer.Add(wx.StaticText(self, label="密码："), 0, wx.ALIGN_CENTER_VERTICAL)
        self.password_ctrl = wx.TextCtrl(self, style=wx.TE_PASSWORD | wx.TE_PROCESS_ENTER)
        form_sizer.Add(self.password_ctrl, 1, wx.EXPAND)

        main_sizer.Add(form_sizer, 0, wx.ALL | wx.EXPAND, 15)

        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        confirm_btn = wx.Button(self, wx.ID_OK, "确认")
        cancel_btn = wx.Button(self, wx.ID_CANCEL, "取消")

        btn_sizer.AddStretchSpacer()
        btn_sizer.Add(confirm_btn, 0, wx.RIGHT, 10)
        btn_sizer.Add(cancel_btn, 0)

        main_sizer.Add(btn_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 15)

        self.SetSizer(main_sizer)
        self.Layout()
        main_sizer.Fit(self)
        self.SetMinSize(self.GetSize())
        self.Centre()

        if self.password_ctrl is not None:
            self.password_ctrl.SetFocus()
            self.password_ctrl.Bind(wx.EVT_TEXT_ENTER, lambda e: self.EndModal(wx.ID_OK))

    def get_password(self):
        return "" if self.password_ctrl is None else self.password_ctrl.GetValue()
