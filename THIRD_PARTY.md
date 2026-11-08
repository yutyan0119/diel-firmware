# 依存コードのライセンス

- [ZMK](https://github.com/zmkfirmware/zmk/blob/4dd95d7840eb3d5a3f59bb29fbf1ab054715ef85/LICENSE) はMITです。ZMKのボード記述を参照した設定には、同梱の [ZMKライセンス](licenses/ZMK.txt) にある上流の表示が適用されます。
- [Zephyr](https://github.com/zmkfirmware/zephyr/blob/10ba6d0cb38bc3d258775d27982f707599320085/LICENSE) の主ライセンスはApache-2.0です。Nordic HAL、CMSIS、Mbed TLSなどのモジュールにも、それぞれのライセンスが適用されます。
- [Zephyr SDK 0.16.9](https://github.com/zephyrproject-rtos/sdk-ng/tree/v0.16.9) のランタイムのうち、[Picolibc](https://github.com/zephyrproject-rtos/picolibc/tree/764ef4e401a8f4c6a86ab723533841f072885a5b) と [GCC](https://github.com/zephyrproject-rtos/gcc/tree/428d8d7b0542338244ca41ac06a5f3fa4f29bb6d) のライセンス文書は [licenses/](licenses/) にあります。
- ボードは既存の [Adafruit nRF52 UF2ブートローダー](https://github.com/adafruit/Adafruit_nRF52_Bootloader) を使います。このリポジトリのUF2はアプリ領域を対象にしており、ブートローダーやSoftDeviceのイメージは含みません。

Actions成果物の `SOURCE.txt` にはDIELのソースの版、`third-party/west-projects.json` には取得した各プロジェクトの版とURLが入ります。`third-party/` 以下に上流のライセンス文書を同梱しています。
