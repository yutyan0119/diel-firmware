# DIEL

DIEL（BC-0302）のZMKファームウェアと組み立てガイドです。

- [組み立て・書き込み・初回確認](docs/assembly.md)
- [GitHub Actionsのビルド](https://github.com/yutyan0119/diel-firmware/actions/workflows/build.yml)

GitHubにログインし、成功した実行の **Artifacts → diel-uf2** をダウンロードしてください。DIEL用は有線のPREC・PREC wirelessと書き込み先が異なります。

| UF2 | 用途 |
| --- | --- |
| `diel-diagnostic.uf2` | 初回の回路・キーの確認。USBシリアルで計測し、最初はキー送信を止めています |
| `diel-normal.uf2` | 通常版。USB／BLEとZMK Studioを使えます |
| `diel-power-save.uf2` | 省電力の評価版。電池駆動で5分間無操作のあと、Spaceを約0.3秒押して復帰します |
| `diel-power-save-diagnostic.uf2` | 省電力動作の診断版。USBシリアルから待機への遷移などを調べられます |

Actionsではビルド・EC判定の単体テスト・UF2の書き込み領域を確認します。公開版の取得遅延や押下・解放しきい値は初期値です。実機では診断版で全キーの差分とノイズを確認してから通常版を使ってください。

## ソースからビルドする

[ZMKのローカルツールチェーン](https://zmk.dev/docs/development/local-toolchain/setup)を用意して、リポジトリのルートで `bash tools/build.sh` を実行します。通常版だけ作る場合は `bash tools/build.sh normal` です。依存先のコミットは [config/west.yml](config/west.yml) で固定し、UF2は `dist/` に出力します。Actionsと同じビルドイメージは [ワークフロー](.github/workflows/build.yml) に記載しています。

| ファイル | 編集するときの目印 |
| --- | --- |
| [diel.keymap](boards/prec/diel/diel.keymap) | キー・レイヤー・コンボの割り当て |
| [diel.conf](config/diel.conf) | 取得遅延、しきい値、スキャン周期 |
| [diel-hardware.dtsi](boards/prec/diel/diel-hardware.dtsi) | 接続とFlash領域の定義 |
| [kscan_prec_ec.c](src/kscan_prec_ec.c) | 静電容量の測定、校正、電池の監視 |
| [power-save.conf](config/power-save.conf) | 評価用の待機・復帰の設定 |

通常版でZMK Studioを使うにはUSBで接続します。既定のキーマップでは、**SpaceまたはBackspaceを保持 → ↑＋↓を同時に押して保持 → O** でStudioのロックを解除します。

## ライセンス

DIELの独自ソースとガイドは [MIT](LICENSE) です。上流コードやビルド結果に関する各依存先の条件は [THIRD_PARTY.md](THIRD_PARTY.md) を参照してください。
