object ElSectionsPropDlg: TElSectionsPropDlg
  Left = 488
  Top = 233
  BorderStyle = bsDialog
  Caption = 'ElHeader Sections editor'
  ClientHeight = 239
  ClientWidth = 327
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  KeyPreview = True
  Position = poScreenCenter
  OnCreate = FormCreate
  PixelsPerInch = 96
  TextHeight = 13
  object Panel3: TPanel
    Left = 0
    Top = 202
    Width = 327
    Height = 37
    Align = alBottom
    BevelOuter = bvNone
    TabOrder = 0
    object Button1: TButton
      Left = 73
      Top = 8
      Width = 75
      Height = 25
      Caption = 'OK'
      Default = True
      ModalResult = 1
      TabOrder = 0
    end
    object Button2: TButton
      Left = 153
      Top = 8
      Width = 75
      Height = 25
      Cancel = True
      Caption = 'Cancel'
      ModalResult = 2
      TabOrder = 1
    end
    object Button3: TButton
      Left = 233
      Top = 8
      Width = 75
      Height = 25
      Caption = 'Apply'
      TabOrder = 2
      OnClick = Button3Click
    end
  end
  object GroupBox1: TGroupBox
    Left = 0
    Top = 0
    Width = 327
    Height = 202
    Align = alClient
    Caption = 'Sections'
    TabOrder = 1
    object Panel1: TPanel
      Left = 2
      Top = 32
      Width = 164
      Height = 168
      Align = alClient
      BevelOuter = bvNone
      BorderWidth = 6
      Caption = 'Panel1'
      TabOrder = 0
      object SecList: TListBox
        Left = 6
        Top = 6
        Width = 152
        Height = 156
        Align = alClient
        ItemHeight = 13
        TabOrder = 0
        OnClick = SecListClick
        OnDblClick = SecListDblClick
        OnKeyPress = SecListKeyPress
      end
    end
    object Panel2: TPanel
      Left = 166
      Top = 32
      Width = 159
      Height = 168
      Align = alRight
      BevelOuter = bvNone
      BorderWidth = 6
      TabOrder = 1
      object AddBtn: TButton
        Left = 0
        Top = 6
        Width = 75
        Height = 25
        Caption = '&Add'
        TabOrder = 0
        OnClick = AddBtnClick
      end
      object DeleteBtn: TButton
        Left = 0
        Top = 72
        Width = 75
        Height = 25
        Caption = '&Delete'
        Enabled = False
        TabOrder = 2
        OnClick = DeleteBtnClick
      end
      object EditBtn: TButton
        Left = 0
        Top = 101
        Width = 75
        Height = 25
        Caption = '&Edit...'
        Enabled = False
        TabOrder = 1
        OnClick = EditBtnClick
      end
      object UpBtn: TButton
        Left = 80
        Top = 6
        Width = 75
        Height = 25
        Caption = 'Move &Up'
        Enabled = False
        TabOrder = 3
        OnClick = UpBtnClick
      end
      object DownBtn: TButton
        Left = 80
        Top = 35
        Width = 75
        Height = 25
        Caption = 'Move Dow&n'
        Enabled = False
        TabOrder = 4
        OnClick = DownBtnClick
      end
      object LoadBtn: TButton
        Left = 80
        Top = 72
        Width = 75
        Height = 25
        Caption = '&Load'
        TabOrder = 5
        OnClick = LoadBtnClick
      end
      object SaveBtn: TButton
        Left = 80
        Top = 101
        Width = 75
        Height = 25
        Caption = '&Save'
        TabOrder = 6
        OnClick = SaveBtnClick
      end
      object DuplicateBtn: TButton
        Left = 0
        Top = 35
        Width = 73
        Height = 25
        Caption = 'Du&plicate'
        Enabled = False
        TabOrder = 7
        OnClick = DuplicateBtnClick
      end
      object ReindexBtn: TButton
        Left = 0
        Top = 136
        Width = 75
        Height = 25
        Caption = 'Reindex'
        TabOrder = 8
        OnClick = ReindexBtnClick
      end
    end
    object TestHeader: TElHeader
      Left = 2
      Top = 15
      Width = 323
      Height = 17
      ActiveFilterColor = clBlack
      AllowDrag = False
      Align = alTop
      Color = clBtnFace
      Flat = True
      MoveOnDrag = False
      FilterColor = clBtnText
      LockHeight = False
      ResizeOnDrag = False
      RightAlignedText = False
      RightAlignedOrder = False
      Sections.Data = {F4FFFFFF00000000}
      StickySections = False
      Tracking = True
      WrapCaptions = False
    end
  end
  object OpenDlg: TOpenDialog
    DefaultExt = '.Elh'
    Filter = 'ElHeader Sections (*.Elh)|*.Elh|All files (*.*)|*.*'
    Options = [ofHideReadOnly, ofPathMustExist, ofFileMustExist]
    Left = 8
    Top = 168
  end
  object SaveDlg: TSaveDialog
    DefaultExt = '*.Elh'
    Filter = 'ElHeader files (*.Elh)|*.Elh|All files (*.*)|*.*'
    Options = [ofHideReadOnly, ofPathMustExist]
    Left = 40
    Top = 168
  end
end