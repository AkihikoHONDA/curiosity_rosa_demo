# T01: リポジトリ雛形（overlay_ws / パッケージ骨格 / 基本設定ファイル群）

## 背景
本デモは Space ROS demos（Curiosity）公式Dockerイメージ上で動かしつつ、実装物はホスト側リポジトリを bind mount してコンテナに取り込む方針です（design.md「2.3 コンテナとワークスペースの境界」「3. ディレクトリ構成」）。  
また、requirements.md「4.3 ディレクトリ構造案」「6. 成果物インベントリ」により、最低限の起動導線（README、config、launch、overlay_ws/scripts/run）が先に揃っていることが、後続タスクの前提になります。

このタスクでは、後続の Simulator / Adapter / Agent / Visualizer を「追加していける状態」にするため、リポジトリの骨格と、colcon で通る最小パッケージを確立します。

## 目的 / 完了条件 (DoD)
- [ ] design.md「3.1 ツリー（提案：リポジトリ内構造）」に沿って、リポジトリ直下に `overlay_ws/` を持つ構造を作成する（requirements.mdの例と差異があっても、設計優先で統一する）。
- [ ] `overlay_ws/src/curiosity_rosa_demo/` が ROS2 Python package として成立しており、`colcon build` が通る（ノードの中身は空でよいが import error を出さない）。
- [ ] 以後のタスクが参照する「配置先ファイル」を先に作っておく（空ファイル／最小実装で可）:
  - `overlay_ws/src/curiosity_rosa_demo/launch/demo.launch.py`
  - `overlay_ws/src/curiosity_rosa_demo/config/topics.yaml`
  - `overlay_ws/src/curiosity_rosa_demo/config/thresholds.yaml`
  - `overlay_ws/src/curiosity_rosa_demo/config/tool_costs.yaml`
  - `overlay_ws/src/curiosity_rosa_demo/config/prompts.yaml`
  - `overlay_ws/src/curiosity_rosa_demo/config/rviz.yaml`
  - `overlay_ws/src/curiosity_rosa_demo/srv/CaptureAndScore.srv`（中身は後続で定義してよいが、置き場所は確定させる）
- [ ] 最小スモークテストを用意し、ローカルで実行できる（pytest でも単純な import test でも可）:
  - `pytest -q` が 0 exit で完了する
  - または `python -c "import curiosity_rosa_demo"` が成功する（こちらは補助で、正式DoDは pytest 推奨）
- [ ] `.gitignore` を整備し、ビルド生成物（build/, install/, log/ 等）や秘密情報（llm.yaml実体など）がコミット対象に入らない。

## 実装詳細
- 使用するクラス/関数:
  - このタスク時点では「骨格のみ」。後続タスクで `sim/`, `adapter/`, `agent/`, `viz/`, `tools/` を実装するため、`curiosity_rosa_demo` パッケージを import 可能な状態にする。
- 参照すべき設計:
  - design.md Section 2.3（underlay/overlay 方針）
  - design.md Section 3.1（提案ツリーの確定）
  - requirements.md Section 4.3（ディレクトリ案）および Section 6（成果物インベントリ）
- 具体作業（要点）:
  1. design.md のツリーを正とし、`curiosity_rosa_demo/overlay_ws/src/curiosity_rosa_demo/` を作成する。
  2. `package.xml`, `setup.py`, `setup.cfg`, `resource/curiosity_rosa_demo` を揃え、`ament_python` 前提で colcon に載せる。
  3. 後続で増えるサブパッケージ（例: `sim/`, `viz/`, `tools/` 等）を置けるよう、`curiosity_rosa_demo/` 配下のディレクトリだけは先に作ってよい（空 `__init__.py` を置き、import error を防ぐ）。
  4. テストは「雛形のimportが通る」ことをまず固定し、後続タスクで機能テストを増やす。

## 前提条件
- 完了しておくべきタスク: なし（最初のタスク）
