object MlCapEditDialog: TMlCapEditDialog
  Left = 301
  Top = 164
  Width = 436
  Height = 270
  ActiveControl = Memo
  Caption = 'Multiline caption/text editor'
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -14
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  PixelsPerInch = 120
  TextHeight = 16
  object Panel1: TPanel
    Left = 0
    Top = 209
    Width = 428
    Height = 33
    Align = alBottom
    BevelOuter = bvNone
    TabOrder = 1
    object Load: TSpeedButton
      Left = 10
      Top = 1
      Width = 28
      Height = 27
      Hint = 'Load from file'
      Glyph.Data = {
        F6000000424DF600000000000000760000002800000010000000100000000100
        0400000000008000000000000000000000001000000010000000000000000000
        8000008000000080800080000000800080008080000080808000C0C0C0000000
        FF0000FF000000FFFF00FF000000FF00FF00FFFF0000FFFFFF00888888888888
        88888888888888888888000000000008888800333333333088880B0333333333
        08880FB03333333330880BFB0333333333080FBFB000000000000BFBFBFBFB08
        88880FBFBFBFBF0888880BFB0000000888888000888888880008888888888888
        8008888888880888080888888888800088888888888888888888}
      ParentShowHint = False
      ShowHint = True
      OnClick = LoadClick
    end
    object Save: TSpeedButton
      Left = 39
      Top = 1
      Width = 29
      Height = 27
      Hint = 'Save to file'
      Glyph.Data = {
        F6000000424DF600000000000000760000002800000010000000100000000100
        0400000000008000000000000000000000001000000010000000000000000000
        8000008000000080800080000000800080008080000080808000C0C0C0000000
        FF0000FF000000FFFF00FF000000FF00FF00FFFF0000FFFFFF00888888888888
        8888880000000000000880330000008803088033000000880308803300000088
        0308803300000000030880333333333333088033000000003308803088888888
        0308803088888888030880308888888803088030888888880308803088888888
        0008803088888888080880000000000000088888888888888888}
      ParentShowHint = False
      ShowHint = True
      OnClick = SaveClick
    end
    object OkButton: TButton
      Left = 119
      Top = 0
      Width = 91
      Height = 31
      Caption = 'Ok'
      ModalResult = 1
      TabOrder = 0
    end
    object CancelButton: TButton
      Left = 217
      Top = 0
      Width = 93
      Height = 31
      Caption = 'Cancel'
      ModalResult = 2
      TabOrder = 1
    end
  end
  object Panel2: TPanel
    Left = 0
    Top = 0
    Width = 428
    Height = 209
    Align = alClient
    BevelOuter = bvNone
    BorderWidth = 5
    TabOrder = 0
    object LineCounter: TLabel
      Left = 5
      Top = 5
      Width = 418
      Height = 16
      Align = alTop
      Caption = '0 Lines'
    end
    object Memo: TElEdit
      Left = 5
      Top = 21
      Width = 418
      Height = 183
      Cursor = crIBeam
      VertScrollBarStyles.ShowTrackHint = False
      VertScrollBarStyles.Width = 13
      VertScrollBarStyles.ButtonSize = 13
      HorzScrollBarStyles.ShowTrackHint = False
      HorzScrollBarStyles.Width = 13
      HorzScrollBarStyles.ButtonSize = 13
      UseCustomScrollBars = True
      AutoSize = False
      Alignment = taLeftJustify
      BorderSides = [ebsLeft, ebsRight, ebsTop, ebsBottom]
      RTLContent = False
      Transparent = False
      BorderStyle = bsSingle
      Multiline = True
      Flat = True
      LineBorderActiveColor = clBlack
      LineBorderInactiveColor = clBlack
      ScrollBars = ssBoth
      OnChange = MemoChange
      Align = alClient
      Ctl3D = True
      ParentColor = False
      ParentCtl3D = False
      TabOrder = 0
    end
  end
  object OpenDialog: TOpenDialog
    Filter = '*.*'
    FilterIndex = -1
    Left = 352
    Top = 216
  end
  object SaveDialog: TSaveDialog
    Filter = '*.*'
    Left = 384
    Top = 216
  end
end