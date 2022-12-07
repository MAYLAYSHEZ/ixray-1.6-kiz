object ColorMapItemsForm: TColorMapItemsForm
  Left = 325
  Top = 233
  BorderIcons = [biSystemMenu, biMinimize]
  BorderStyle = bsDialog
  Caption = 'Color Map'
  ClientHeight = 283
  ClientWidth = 284
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  Position = poScreenCenter
  OnCreate = FormCreate
  OnDestroy = FormDestroy
  OnShow = FormShow
  PixelsPerInch = 96
  TextHeight = 13
  object Label1: TLabel
    Left = 96
    Top = 0
    Width = 53
    Height = 13
    Caption = 'Color entry:'
  end
  object Label2: TLabel
    Left = 0
    Top = 0
    Width = 57
    Height = 13
    Caption = 'Color group:'
  end
  object Label3: TLabel
    Left = 200
    Top = 96
    Width = 57
    Height = 13
    Caption = 'Foreground:'
  end
  object Label4: TLabel
    Left = 200
    Top = 136
    Width = 61
    Height = 13
    Caption = 'Background:'
  end
  object Label5: TLabel
    Left = 200
    Top = 184
    Width = 41
    Height = 13
    Caption = 'Entry ID:'
  end
  object IDLbl: TLabel
    Left = 200
    Top = 200
    Width = 3
    Height = 13
  end
  object FgColor: TElColorCombo
    Left = 200
    Top = 112
    Width = 81
    Height = 21
    Style = csOwnerDrawFixed
    TabOrder = 10
    Text = 'clAqua'
    OnChange = FgColorChange
    Flat = True
    HorizontalScroll = False
    Transparent = False
    Options = [ccoNoColor, cco4BitColors, ccoSystemColors, ccoCustomChoice, ccoShowNames]
    SelectedColor = clAqua
  end
  object BkColor: TElColorCombo
    Left = 200
    Top = 152
    Width = 81
    Height = 21
    Style = csOwnerDrawFixed
    TabOrder = 11
    Text = 'clBlue'
    OnChange = BkColorChange
    Flat = True
    HorizontalScroll = False
    Transparent = False
    Options = [ccoNoColor, cco4BitColors, ccoSystemColors, ccoCustomChoice, ccoShowNames]
    SelectedColor = clBlue
  end
  object OkBtn: TElPopupButton
    Left = 200
    Top = 16
    Width = 81
    Height = 25
    ImageIndex = 0
    DrawDefaultFrame = False
    ModalResult = 1
    ShadowBtnHighlight = 14606295
    ShadowBtnShadow = 7764576
    ShadowBtnDkShadow = 5856328
    ShowFocus = False
    TextDrawType = tdtNormal
    Transparent = False
    Caption = 'Ok'
    TabOrder = 1
    Color = clBtnFace
    ParentColor = False
  end
  object CancelBtn: TElPopupButton
    Left = 200
    Top = 56
    Width = 81
    Height = 25
    ImageIndex = 0
    DrawDefaultFrame = False
    Cancel = True
    ModalResult = 2
    ShadowBtnHighlight = 14606295
    ShadowBtnShadow = 7764576
    ShadowBtnDkShadow = 5856328
    ShowFocus = False
    TextDrawType = tdtNormal
    Transparent = False
    Caption = 'Cancel'
    TabOrder = 2
    Color = clBtnFace
    ParentColor = False
  end
  object AddBtn: TElPopupButton
    Left = 96
    Top = 240
    Width = 73
    Height = 17
    ImageIndex = 0
    DrawDefaultFrame = False
    ShadowBtnHighlight = 14606295
    ShadowBtnShadow = 7764576
    ShadowBtnDkShadow = 5856328
    ShowFocus = False
    TextDrawType = tdtNormal
    Transparent = False
    Caption = 'Add Entry'
    Enabled = False
    TabOrder = 3
    Color = clBtnFace
    ParentColor = False
    OnClick = AddBtnClick
  end
  object DelBtn: TElPopupButton
    Left = 96
    Top = 264
    Width = 73
    Height = 17
    ImageIndex = 0
    DrawDefaultFrame = False
    ShadowBtnHighlight = 14606295
    ShadowBtnShadow = 7764576
    ShadowBtnDkShadow = 5856328
    ShowFocus = False
    TextDrawType = tdtNormal
    Transparent = False
    Caption = 'Delete Entry'
    Enabled = False
    TabOrder = 4
    Color = clBtnFace
    ParentColor = False
    OnClick = DelBtnClick
  end
  object AddGroupBtn: TElPopupButton
    Left = 2
    Top = 240
    Width = 73
    Height = 17
    ImageIndex = 0
    DrawDefaultFrame = False
    ShadowBtnHighlight = 14606295
    ShadowBtnShadow = 7764576
    ShadowBtnDkShadow = 5856328
    ShowFocus = False
    TextDrawType = tdtNormal
    Transparent = False
    Caption = 'Add Group'
    TabOrder = 8
    Color = clBtnFace
    ParentColor = False
    OnClick = AddGroupBtnClick
  end
  object DelGroupBtn: TElPopupButton
    Left = 2
    Top = 264
    Width = 73
    Height = 17
    ImageIndex = 0
    DrawDefaultFrame = False
    ShadowBtnHighlight = 14606295
    ShadowBtnShadow = 7764576
    ShadowBtnDkShadow = 5856328
    ShowFocus = False
    TextDrawType = tdtNormal
    Transparent = False
    Caption = 'Delete Group'
    Enabled = False
    TabOrder = 9
    Color = clBtnFace
    ParentColor = False
    OnClick = DelGroupBtnClick
  end
  object EntryLB: TElAdvancedListBox
    Left = 98
    Top = 16
    Width = 97
    Height = 217
    ItemHeight = 13
    TabOrder = 0
    OnClick = EntryLBClick
    Flat = True
    SelectedFont.Charset = DEFAULT_CHARSET
    SelectedFont.Color = clHighlightText
    SelectedFont.Height = -11
    SelectedFont.Name = 'MS Sans Serif'
    SelectedFont.Style = []
    BorderSides = [ebsLeft, ebsRight, ebsTop, ebsBottom]
    HorizontalScroll = False
  end
  object GroupLB: TElAdvancedListBox
    Left = 0
    Top = 16
    Width = 97
    Height = 217
    ItemHeight = 13
    TabOrder = 7
    OnClick = GroupLBClick
    Flat = True
    SelectedFont.Charset = DEFAULT_CHARSET
    SelectedFont.Color = clHighlightText
    SelectedFont.Height = -11
    SelectedFont.Name = 'MS Sans Serif'
    SelectedFont.Style = []
    BorderSides = [ebsLeft, ebsRight, ebsTop, ebsBottom]
    HorizontalScroll = False
  end
  object UseBkCB: TElCheckBox
    Left = 200
    Top = 256
    Width = 81
    Height = 17
    State = cbUnchecked
    AllowGrayed = False
    Alignment = taRightJustify
    Checked = False
    TextDrawType = tdtNormal
    Transparent = False
    Flat = True
    IsHTML = False
    Caption = 'Use BkColor'
    TabOrder = 5
    Color = clBtnFace
    ParentColor = False
    OnClick = UseBkCBClick
  end
  object UseFgCB: TElCheckBox
    Left = 200
    Top = 240
    Width = 80
    Height = 17
    State = cbUnchecked
    AllowGrayed = False
    Alignment = taRightJustify
    Checked = False
    TextDrawType = tdtNormal
    Transparent = False
    Flat = True
    IsHTML = False
    Caption = 'Use FgColor'
    TabOrder = 6
    Color = clBtnFace
    ParentColor = False
    OnClick = UseFgCBClick
  end
end