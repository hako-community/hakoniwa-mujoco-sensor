# external_library

★★★★ **2026-09-02 に新設。** それまで `hakoniwa-mujoco-sensor` は外部依存を**自分では持たず**、
configure のときに**リポジトリの外を探しに行っていた**。

| 依存 | 以前に拾っていた場所 | 実体 |
|---|---|---|
| MuJoCo | `../../.cache/deps/mujoco_bin-src` | **ドローンの木の外**（別プロジェクト＝ヒューマノイドのキャッシュ） |
| nlohmann/json | `../nlohmann-json/single_include` | **`hakoniwa-drone-pro/thirdparty/nolman`**（PRIVATE。しかも「使わない」と決めたリポジトリ） |

★★★ どちらも消えた瞬間にこのリポジトリはビルド不能になる。
**単体でビルドでき、単体で動かなければ意味がない。**

## いまの形

| 取り込む | 形 | 版 |
|---|---|---|
| `nolman`（nlohmann/json） | **git submodule** | `663058e7d`（★ 4 リポジトリで同一に揃えてある） |
| `mujoco` | **`mujoco.lock`（版 ＋ SHA256）で取得** | 3.9.0 |

## ★★★★ なぜ MuJoCo だけ submodule にしないのか

submodule で取れるのは**ソース**で、`libmujoco.so` は付いてこない。
自前ビルドに戻すと「ヘッダとライブラリを別々に用意する」形になり、
★★★ **版が食い違うとビルドは通って実行時に SEGV する**という地雷の入り口へ戻る
（このリポジトリで **3.10.0-dev ヘッダ ＋ 3.9.0 ライブラリ**の事故が実際に起きている）。

★★★★ **その 3.10.0-dev は `/data/buildman/drone/sensor/mujoco` に今も在り、
2026-09-02 まで `hakoniwa-mujoco-sensor` の探索候補に残っていた。**候補から外した。

## 用意のしかた

```bash
external_library/setup.sh
```

submodule を同期し、MuJoCo を lock の版で取得して SHA256 を検証する。
★ 最後に **symlink が他リポジトリを指していないこと**を断言して終わる（指していたら異常終了する）。

## ★★★ 版を上げるときは 4 リポジトリ同時に

`hakoniwa-mujoco-drone` / `hakoniwa-drone-companion` / `hakoniwa-mujoco-sensor` /
`hakoniwa-mujoco-runtime` は**同じ MuJoCo に対して組んだ成果物を混ぜる**。
片方だけ上げると**ビルドは通って実行時に SEGV する**。
