# S3 前のマルチ OS ビルド監査

調査日: 2026-09-19。対象は各リポジトリが管理する CMake source root のみであり、
vendor 配下の `third_party` と `external_library` のプロジェクトは変更対象外とする。

| リポジトリ | CMake source root | 標準 binary directory | 現在の移植性に関する所見 |
| --- | --- | --- | --- |
| hakoniwa-mujoco-sensor | リポジトリ root | `hakoniwa-mujoco-sensor/build/` | C++ core は移植可能。ただし依存探索には Linux パスが残り、camera には OS ごとの OpenGL/GLFW package が必要。 |
| hakoniwa-mujoco-runtime | リポジトリ root | `hakoniwa-mujoco-runtime/build/` | C++ core は移植可能。MuJoCo 探索が Linux の `.so` ファイル名とパスを固定している。 |
| hakoniwa-armpi | `mujoco_plant/` | `hakoniwa-armpi/build/` | Linux 専用の MuJoCo release URL、`/usr/local/hakoniwa`、Linux rpath、Bash のみの build entrypoint がある。 |
| hakoniwa-humanoid | `mujoco_plant/` | `hakoniwa-humanoid/build/` | ArmPi と同じく Linux 専用の MuJoCo URL/prefix/rpath がある。camera には OS ごとの GLFW/OpenGL 設定も必要。 |

実際の CMake source root にはすべて `CMakePresets.json` を追加した。`windows-msvc`、
`linux-release`、`macos-release` の preset は、それぞれの host OS 上で同じ
リポジトリ内 binary directory を使用する。異なる OS や generator 間で build
directory を共有することはできないため、checkout を別の platform へ移す場合は
build directory を削除してから再構成すること。

preset は起動方法と出力先を統一するものであり、Linux 専用の依存取得が Windows や
macOS でも動くようになるわけではない。S3 の前に、platform 固定の MuJoCo
download/link 処理を共通の `MUJOCO_ROOT` / `MUJOCO_INCLUDE_DIR` /
`MUJOCO_LIBRARY` 契約へ置き換える。さらに Hakoniwa prefix を `/usr/local` 固定に
せず設定可能にし、`CMAKE_INSTALL_RPATH` は ELF platform に限って使用する。
PowerShell と macOS 用の setup entrypoint を用意する、または package manager の
事前条件を文書化することも必要である。
