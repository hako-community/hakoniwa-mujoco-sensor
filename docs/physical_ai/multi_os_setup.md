# マルチ OS の build 環境

CMake が全リポジトリで使う MuJoCo の契約は次である。

| 変数 | 用途 |
| --- | --- |
| `MUJOCO_ROOT` | `include/mujoco/mujoco.h` と `lib/` または `bin/` を含む MuJoCo package root。 |
| `MUJOCO_INCLUDE_DIR` | `MUJOCO_ROOT` を使わない場合の header directory。 |
| `MUJOCO_LIBRARY` | `MUJOCO_ROOT` を使わない場合の import/shared library。 |

Hakoniwa のCMake buildは、OSを問わず **CMake packageをexportしたprefix** を
明示的に使う。Windowsの旧SDKが提供する `HAKO_CORE_INC_PATH`、`HAKO_CORE_LIB_PATH`、
`HAKO_DRONE_INC_PATH`、`HAKO_DRONE_LIB_PATH` は、既存実行環境との互換用であり、
新規CMake configureのprefixとしては使用しない。これらは通常
`hakoniwa-coreConfig.cmake`／`hakoniwa_pdu_endpointConfig.cmake`を含まないためである。

workspace-local SDKの標準配置は次とする。

```text
<workspace>/devai_build/sdk/<platform-toolchain>/hakoniwa-core/
<workspace>/devai_build/sdk/<platform-toolchain>/hakoniwa-pdu-endpoint/
```

各consumerには、同じSDK revisionを明示的に渡す。

```text
HAKONIWA_INSTALL_PREFIX=<...>/hakoniwa-core
HAKONIWA_PDU_ENDPOINT_PREFIX=<...>/hakoniwa-pdu-endpoint
CMAKE_PREFIX_PATH=<HAKONIWA_INSTALL_PREFIX>;<HAKONIWA_PDU_ENDPOINT_PREFIX>
```

Core prefixには
`lib/cmake/hakoniwa-core/hakoniwa-coreConfig.cmake`、PDU endpoint prefixには
`lib/cmake/hakoniwa_pdu_endpoint/hakoniwa_pdu_endpointConfig.cmake` が存在することを
configure前に確認する。Linuxの `/usr/local/hakoniwa` は明示的にinstallした場合のみ
使用し、暗黙のfallbackへ依存しない。

WindowsではVisual Studio x64 Developer Command Promptから、workspace-local SDKを
指定してCMake presetを使う。

```powershell
$env:HAKONIWA_INSTALL_PREFIX = "$PWD\devai_build\sdk\windows-msvc-x64\hakoniwa-core"
$env:HAKONIWA_PDU_ENDPOINT_PREFIX = "$PWD\devai_build\sdk\windows-msvc-x64\hakoniwa-pdu-endpoint"
$env:CMAKE_PREFIX_PATH = "$env:HAKONIWA_INSTALL_PREFIX;$env:HAKONIWA_PDU_ENDPOINT_PREFIX"
cmake --preset windows-msvc `
  -DHAKONIWA_INSTALL_PREFIX="$env:HAKONIWA_INSTALL_PREFIX" `
  -DHAKONIWA_PDU_ENDPOINT_PREFIX="$env:HAKONIWA_PDU_ENDPOINT_PREFIX"
```

`validate_hako_mujoco_env.ps1`は旧SDKの実行環境チェックとして利用できるが、上記の
CMake package検証の代替ではない。workspace SDKの構築手順と受入条件は
`devai_build/hakoniwa_workspace_cmake_sdk_plan_20260920.md` を正本とする。

Linux/macOSでも同じ二つのprefixをexportしてから実行する。

```sh
export HAKONIWA_INSTALL_PREFIX="$PWD/devai_build/sdk/<platform-toolchain>/hakoniwa-core"
export HAKONIWA_PDU_ENDPOINT_PREFIX="$PWD/devai_build/sdk/<platform-toolchain>/hakoniwa-pdu-endpoint"
export CMAKE_PREFIX_PATH="$HAKONIWA_INSTALL_PREFIX:$HAKONIWA_PDU_ENDPOINT_PREFIX"
cmake --preset linux-release # macOS は macos-release
```

macOS 用 MuJoCo package は利用者が取得して `MUJOCO_ROOT` に指定する。macOS
実機がないため、`macos-release` は CMake 構成の入口を揃えるものであり、実機上の
GLFW/OpenGL、Hakoniwa、MuJoCo の統合確認は未実施である。
