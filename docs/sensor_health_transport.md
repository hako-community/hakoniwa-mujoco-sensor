# SENS-006 health transport v1

2026-09-25、A案。外部リポジトリへのメッセージ追加を避けるため、標準`std_msgs/UInt8MultiArray`のdataにアプリケーション定義のバイト列を格納する。layoutは空、data_offsetは0。既存healthチャンネルIDと1024バイトのPDU領域を維持する。

C++実装は`include/sensors/common/sensor_health_wire.hpp`、Python実装は`python/hakoniwa_sensor_health`。C++の送信アダプターは`include/hakoniwa/pdu/adapter/std_msgs/sensor_health.hpp`（名前空間`hako::robots::pdu::adapter::std_msgs`）。送信PDUが`std_msgs`なので、他のアダプターと同じくPDUパッケージ名の下に置く。旧`adapter/hako_msgs/sensor_health.hpp`は廃止し、custom SensorHealth生成ヘッダーには依存しない。

## バイト形式

数値はlittle endian、固定ヘッダー66バイト。

|Offset|型|内容|
|---:|---|---|
|0|4 bytes|magic `HSH1`|
|4|uint16|version = 1|
|6|uint16|reserved = 0|
|8|uint32|全体バイト長|
|12|uint64|sequence|
|20|int64|source_time_ns|
|28|int64|scheduled_time_ns|
|36|int64|publish_time_ns|
|44|uint32|status|
|48|uint64|dropped_count|
|56|uint32|queue_depth|
|60,62,64|uint16各1|sensor_id / profile_id / calibration_idのバイト長|
|66以降|UTF-8|上記3文字列を順に連結|

文字列はそれぞれ最大127バイト、NUL禁止、sensor_idは必須。最大データ長447バイト。時刻は非負、scheduledとpublishはsource以上。statusは既存のbit 0〜6だけを許可する。magic/version/reserved/長さ/UTF-8/範囲の不正は拒否する。チェックサムは持たない。

旧`hako_msgs/SensorHealth`のPDUと互換性はない。送受信コードと所有リポジトリのPDU型設定を同時に移行する。healthの内部フィールド、欠測・stale・resetの意味は維持する。

## 検証とセットアップ

`tests/sensor_health_codec`でC++/Python相互変換、標準PDUラップ相互変換、独立した既知バイト列、不正データ、最大値、custom生成モジュール不在を検証する。

Windowsの準備・実行手順は兄弟runtimeリポジトリの[手順書](../../hakoniwa-mujoco-runtime/docs/windows_sensor_sim_setup.md)を参照。CMakeの`MJSENSOR_DIR`はstandalone sensorを指定する。third_partyのコピーを更新して解決しない。
