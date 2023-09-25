# STM32 USB-PD Power
STM32G071KBT6N を使ったシンプルな USB Type-C/USB-PD シンクデバイス

![Circuit Board](photo3.jpg "Circuit Board")
- スイッチ(上記写真で左から) SW5(Lock), SW4(PDO/Pout) SW3(-) SW2(+) SW1(Power)
- SW1: 長押しで出力 ON/OFF のトグル
- SW2,3: 出力電圧調整
- SW4: 出力電力表示切り替え(PPSモードのみ)
- SW5: 出力中の電圧変更許可 (PPSモードのみ)

## Features
* PD シンクデバイス
* 調整可能な高電圧、電流出力
* 出力電圧、電流をグラフィカルLCD に表示
* 1x type-C コネクタ
* USB-PD v3 PPS(Programmable Power Source)をサポート
* 出力ON/OFF 切り替え
* 20mV step で電圧調整 (PPS mode)
* PPS 電圧を EEPROM に保存、読み込み

## Requirement
プロジェクトは STM32CubeIDE に対応します。
STM32CubeMX を使って生成されています。

## Schematics
回路図は PDF ファイルを参照してください。

## Article
実装の詳細は「トランジスタ技術」2023/9 に記事を掲載していますので、ご覧ください。

## Author
* Y. Tayama (PN)
* Contact: upd780c@gmail.com
