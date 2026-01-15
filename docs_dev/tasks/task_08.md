# T08: Adapter Node（Curiosity既存Serviceラップ＋排他制御、公開I/Fは /adapter/* Trigger）【更新版】

## 背景
Curiosityデモの既存I/F（移動・マスト操作）は `std_srvs/srv/Empty` のサービスとして提供される。
これをROSA Toolから直接叩くと、排他制御（マスト展開中は移動不可）や失敗理由の統一が崩れるため、Adapter Node を挟み、状態管理とエラー文言を集約する。

本設計の正は以下である。
- Adapterは Curiosity既存Service（Empty）を内部で呼ぶ。
- Adapterがエージェント向けに公開するServiceは `/adapter/*` 名前空間で提供し、型は `std_srvs/srv/Trigger` とする。
- 排他違反時の理由文字列は必ず `Need to close mast` に統一する。

## 目的 / 完了条件 (DoD)
- [ ] Adapter Node を実装し、以下の公開サービス（Trigger）を提供すること（名前は topics.yaml に従う）。
  - マスト操作: `mast_open`, `mast_close`, `mast_rotate`
  - 移動: `move_forward`, `turn_left`, `turn_right`, `move_stop`
  - 状態照会: `get_status`（Trigger.message に JSON文字列を返す）

- [ ] Adapter は Curiosity側の実体Service（Empty）を topics.yaml から参照し、内部で呼び出すこと。
  - Curiosity実体: `/move_forward`, `/turn_left`, `/turn_right`, `/move_stop`
  - Curiosity実体: `/mast_open`, `/mast_close`, `/mast_rotate`

- [ ] 排他制御を実装すること。
  - mast が open 状態のとき、移動系（move_forward/turn_left/turn_right/move_stop 以外の「移動」に該当するもの）を拒否する。
  - 拒否時は、Trigger.success=false かつ message を必ず `Need to close mast` にする。
  - ToolResult.error_reason も同じ文字列で一致させる（Tool側で改変しない）。

- [ ] Adapter は内部状態として最低限 `mast_is_open: bool` を持ち、成功時のみ更新すること。
  - mast_open 成功 → `mast_is_open=True`
  - mast_close 成功 → `mast_is_open=False`
  - mast_rotate 成功 → `mast_is_open=True` を維持（展開状態のまま）

- [ ] get_status は最低限以下を返すこと（JSON文字列として）。
  - `mast_is_open`
  - `move_allowed`（mast_is_open の否定）
  - `last_error_reason`（保持する場合）

  可能なら `rover_x` などを追加してもよいが、最初は最小でよい。

- [ ] テスト併走。
  - [ ] rclpy統合テスト（推奨）: ダミーのEmpty service群を立て、Adapter経由で呼べる
  - [ ] 排他テスト: mast_open後に move_forward → 拒否（messageが定型文一致）、mast_close後に move_forward → 許可

- [ ] 作成・修正ファイル。
  - `curiosity_rosa_demo/adapter/adapter_node.py`（新規）
  - `curiosity_rosa_demo/tests/test_adapter_exclusivity.py`（新規）
  - `config/topics.yaml`（adapter と curiosity の両方のserviceマッピングを整形）

## 実装詳細
- 実装メモ
  Adapterの公開I/Fを `/adapter/*` に分離する理由は、Curiosity既存Serviceと同名で衝突しないようにするためである。
  Trigger.message は、失敗理由の伝達に使う。排他拒否は速く返し、Curiosity実体serviceを呼びに行かない。

## 前提条件
- 完了しておくべきタスク:
  - T02
  - T03
  - T04
