object ElDailyTipForm: TElDailyTipForm
  Left = 289
  Top = 263
  ActiveControl = OkBtn
  BorderStyle = bsDialog
  Caption = 'Tip of the Day'
  ClientHeight = 207
  ClientWidth = 398
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -14
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  Icon.Data = {
    0000010001002020100000000000E80200001600000028000000200000004000
    0000010004000000000080020000000000000000000000000000000000000000
    000000008000008000000080800080000000800080008080000080808000C0C0
    C0000000FF0000FF000000FFFF00FF000000FF00FF00FFFF0000FFFFFF000000
    0000000000000777770000000000000000000000000C40777770000000000000
    000000000CCC440777770000000000000000000CCCCC44407777700000000000
    00000CCCCCCE44440777770000000000000CCCCCCEECC4444077777000000000
    0CCCCCCCC4CCCC44407777770000000CCCCCCCCCC44CCCC4407777777000000C
    CCCCCCCCC444CCCC407777700000000CCE0CCCCCC4444CC0077770000000000E
    EC0CCCCCC44440077770000000000000CC0CCCCCC44440777000000000000000
    0C0CCCCCC44440700000000000000000000CCCCCC44440000000000000000000
    000CCCCCC44440000000000000000000000CCCCCC44440000000000000000000
    000CCCCCC44440000000000000000000000CCCCEEC4440000000000000000000
    000CCEECCCC4400000000000000000000CCEEC4444CC40000000000000000000
    0EECC0000444C00000000000000000000CC004444004C0000000000000000000
    0004444444400000000000000000000000044444444000000000000000000000
    0044CCC4C444000000000000000000000044CCCCC44400000000000000000000
    004CCCCCC44400000000000000000000004CCFCCC44400000000000000000000
    000CCCCCC4400000000000000000000000044CC4444000000000000000000000
    000004444000000000000000000000000000000000000000000000000000FFFE
    03FFFFF801FFFFE000FFFF80007FFE00003FF800001FE000000FC0000007C000
    001FC000007FC00001FFE00007FFF0001FFFF8003FFFFC003FFFFC003FFFFC00
    3FFFFC003FFFF8003FFFF0003FFFF0003FFFF0003FFFF8007FFFFC00FFFFF800
    7FFFF8007FFFF8007FFFF8007FFFFC00FFFFFC00FFFFFE01FFFFFF87FFFF}
  Position = poScreenCenter
  PixelsPerInch = 120
  TextHeight = 16
  object OkBtn: TElPopupButton
    Left = 293
    Top = 170
    Width = 92
    Height = 31
    ImageIndex = 0
    DrawDefaultFrame = False
    PopupPlace = ppRight
    ModalResult = 1
    NumGlyphs = 1
    Caption = '&Close'
    TabOrder = 0
    Color = clBtnFace
    ParentColor = False
  end
  object NextTimeCB: TCheckBox
    Left = 10
    Top = 175
    Width = 149
    Height = 21
    Caption = 'Show tips at startup'
    TabOrder = 1
  end
  object NextBtn: TElPopupButton
    Left = 197
    Top = 170
    Width = 92
    Height = 31
    ImageIndex = 0
    DrawDefaultFrame = False
    PopupPlace = ppRight
    NumGlyphs = 1
    Caption = '&Next Tip'
    TabOrder = 2
    Color = clBtnFace
    ParentColor = False
    OnClick = NextBtnClick
  end
  object Panel1: TPanel
    Left = 10
    Top = 10
    Width = 375
    Height = 149
    BevelOuter = bvNone
    TabOrder = 3
    object Panel2: TPanel
      Left = 0
      Top = 0
      Width = 59
      Height = 149
      Align = alLeft
      BevelOuter = bvNone
      Color = clBtnShadow
      TabOrder = 0
      object Image1: TImage
        Left = 11
        Top = 11
        Width = 39
        Height = 39
        AutoSize = True
        Picture.Data = {
          07544269746D617076020000424D760200000000000076000000280000002000
          0000200000000100040000000000000200000000000000000000100000001000
          000000000000000080000080000000808000800000008000800080800000C0C0
          C000808080000000FF0000FF000000FFFF00FF000000FF00FF00FFFF0000FFFF
          FF00333333333333333833333333333333333333333333333388833333333333
          3333333333333333388888333333333333333333333333338880888333333333
          3333333333333338880E088833333333333333333333338880EFE08883333333
          33333333333338880EFEFE08883333333333333333338880EFEFEFE088833333
          333333333338880EFE000EFE0888333333333333338880EFE00000EFE0888333
          3333333338880EFE88F7700EFE088833333333338880EFEF8F00070FEFE08883
          33333333380EFEF880F77000FEFE08333333333330EFEFE8FF000770EFEFE033
          333333330EFEFEF800333000FEFEFE033333333338EFEFE033BBB330EFEFE033
          33333333338EFE03BBBBBBB30EFE0333333333333338EF03BBFBFBF30FE03333
          333333333333803BBB808BBB3003333333333333333303BBFB000BFBF3033333
          3333333333330BBBBF808FBBB3033333333333333333BBFBFBFBFBFBFB303333
          333333333333BBBFBFB0BFBFBB303333333333333333FBFFFFF0FBFBFB303333
          333333333333BFFFFF808FBFBB303333333333333333BFFFFF000BFBFB303333
          3333333333333FFFFF000FBFBB0333333333333333333BFFFF000BFBFB033333
          33333333333333BFFF808FBBB0333333333333333333333BFBFBFBFB33333333
          33333333333333333BBBBB333333333333333333333333333333333333333333
          3333}
        Transparent = True
      end
    end
    object Panel3: TPanel
      Left = 59
      Top = 0
      Width = 316
      Height = 149
      Align = alClient
      BevelOuter = bvNone
      TabOrder = 1
      object Panel4: TPanel
        Left = 0
        Top = 0
        Width = 316
        Height = 31
        Align = alTop
        BevelOuter = bvNone
        Color = clWindow
        TabOrder = 0
        object Label1: TLabel
          Left = 0
          Top = 0
          Width = 228
          Height = 31
          Align = alLeft
          AutoSize = False
          Caption = '  Did you know...'
          Color = clBtnFace
          Font.Charset = DEFAULT_CHARSET
          Font.Color = clWindowText
          Font.Height = -23
          Font.Name = 'Times New Roman'
          Font.Style = [fsBold]
          ParentColor = False
          ParentFont = False
          Transparent = True
          Layout = tlCenter
        end
        object TipNumLabel: TLabel
          Left = 228
          Top = 0
          Width = 88
          Height = 31
          Align = alClient
          Alignment = taRightJustify
          Caption = 'Tip #8'
          Font.Charset = DEFAULT_CHARSET
          Font.Color = clWindowText
          Font.Height = -17
          Font.Name = 'Times New Roman'
          Font.Style = [fsBold]
          ParentFont = False
          Layout = tlCenter
        end
      end
      object Panel5: TPanel
        Left = 0
        Top = 32
        Width = 316
        Height = 117
        Align = alBottom
        BevelOuter = bvNone
        Color = clWindow
        TabOrder = 1
        object TipText: TElHTMLLabel
          Left = 10
          Top = 10
          Width = 296
          Height = 100
          Cursor = crDefault
          IsHTML = False
          WordWrap = True
          LinkColor = clBlue
          LinkStyle = [fsUnderline]
          AutoSize = False
        end
      end
    end
  end
end