# PROJECT_CONTRACT.md — プロジェクト契約（変更禁止リスト）: curiosity_rosa_demo

この文書は、本リポジトリにおける **「勝手に変えてはいけない約束事（契約）」** を一箇所に集約するためのものです。  
Codex（エージェント）は、実装中に“良かれ改善”でここに書かれた項目を変更してはなりません。

---

## 0. 目的

- **デモの成立条件（I/F・固定文言・設定スキーマ）を壊さない**
- 変更が必要になった場合も、必ず **提案 → 人間承認 → 反映** の手順で統制する
- `design.md` / `task_*.md` に分散する「絶対条件」を、この1枚で参照できるようにする

---

## 1. 本契約の適用範囲

- 本契約は **全タスク** に適用する。
- タスク定義（`task_{n}.md`）が本契約と矛盾する場合は、**作業を止め**、矛盾点を報告し、変更提案として扱う（勝手に解釈で突き進まない）。

---

## 2. ドキュメントの所在（既定）

開発用ドキュメントは原則 `docs_dev/` 配下にある前提で運用する（別パスの場合は repo 直下 `AGENTS.md`/`README.md` を正とする）。

- `docs_dev/design.md`
- `docs_dev/requirements.md`
- `docs_dev/tasks_overview.md`
- `docs_dev/tasks/task_*.md`
- `docs_dev/user_input.md`（存在すれば）
- `docs_dev/PROJECT_CONTRACT.md`（本書）

---

## 3. 変更禁止（Hard Locks）

以下は **一字一句・型名・名前空間を含めて変更禁止**。  
変更が必要なら「変更提案」に昇格し、承認が下りるまで一切変更しない。

### 3.1 Curiosity 既存I/F（実体 service）
- サービス型：`std_srvs/srv/Empty`
- サービス名（代表）：
  - `/move_forward`
  - `/turn_left`
  - `/turn_right`
  - `/move_stop`
  - `/mast_open`
  - `/mast_close`
  - `/mast_rotate`

> 注意：Tool から Curiosity 既存（Empty）を直接叩く構成にはしない（必ず Adapter 経由）。

### 3.2 Adapter 公開I/F（エージェント向け service）
- サービス型：`std_srvs/srv/Trigger`
- 公開名前空間：`/adapter/*`（Curiosity既存serviceと衝突しないため）
- 公開サービス（代表）：
  - `/adapter/move_forward`
  - `/adapter/turn_left`
  - `/adapter/turn_right`
  - `/adapter/move_stop`
  - `/adapter/mast_open`
  - `/adapter/mast_close`
  - `/adapter/mast_rotate`

### 3.3 固定文言（完全一致）
排他違反時の理由文字列は必ず以下に統一する（完全一致）。

- `Need to close mast`

この文字列は少なくとも以下で一致していること。
- Adapter の `TriggerResponse.message`
- Tool 層の `ToolResult.error_reason`
- 可視化やログ表示（Trace/Visualizer）での理由表示

### 3.4 観測I/F（CaptureAndScore）
- サービス名：`/capture_and_score`
- サービス型：`curiosity_rosa_demo/srv/CaptureAndScore`
- 期待する意味：
  - 観測不能（画像未受信・TF未解決等）は **疑似値で誤魔化さず** `ok=false` と `error_reason` で返す

### 3.5 画像トピック（加工済み）
- トピック名：`/capture/image_raw/compressed`
- メッセージ型：`sensor_msgs/msg/CompressedImage`

### 3.6 Trace 出力
- トピック名：`/trace/events`
- 内容：JSON 文字列（イベント種別・tool名・ok/error_reason・score 等を含める設計方針）
- 要求：Visualizer が落ちないよう decode は堅牢にする（壊れたJSONでもフォールバックする）

---

## 4. 変更制御（Change Control）

契約項目を変更したい場合、Codex は次を必ず行う。

1) **変更提案（Proposal）** を先に出す（コードは触らない）
   - 変更理由
   - 影響範囲（どのノード/ツール/テスト/設定/READMEが影響するか）
   - 後方互換策（移行期間・デフォルト・旧キー/旧I/Fサポートの要否）
   - テスト計画（何が通れば安全か）

2) ユーザー承認後にのみ反映する

3) 反映時は、少なくとも以下を同一PR/同一作業単位で揃える
   - 実装（Node/Tool/Infra）
   - 設定（YAML）
   - テスト（pytest/colcon）
   - README/起動手順（必要なら）

---

## 5. 設定（Config）に関する契約（Soft Locks）

以下は “原則固定” とする。変更は可能だが、**必ず提案＋承認が必要**。

### 5.1 設定ファイル群（ファイル名）
- `config/topics.yaml`
- `config/thresholds.yaml`
- `config/tool_costs.yaml`
- `config/prompts.yaml`
- `config/rviz.yaml`

### 5.2 設定探索の規約
- デフォルトは ament の `share/<pkg>/config` を探索
- 上書きは ROS2 parameter `config_dir` で可能にする

### 5.3 I/Fの記述方針（topics.yaml）
- Curiosity 実体I/F（Empty）と Adapter 公開I/F（Trigger）を **分離して記述できる** ことを維持する

---

## 6. Codex に渡すタスクプロンプトでの扱い（推奨）

各タスクで Codex に渡す指示には、最低限以下を含めることを推奨する。

- `docs_dev/PROJECT_CONTRACT.md を契約として扱い、勝手に変更しない`
- 変更が必要なら Gate B（計画提示）で “変更提案” として止める

---

## 7. 自己チェック（実装後に確認すること）

- `/adapter/*` は `Trigger` で提供されているか
- 排他違反の message / error_reason が `Need to close mast` に完全一致しているか
- Tool が Curiosity 既存（Empty）を直接叩いていないか（Adapter経由か）
- `/capture_and_score` が観測不能を `ok=false` + `error_reason` で返しているか
- `/trace/events` の decode が壊れた入力でも落ちないか
