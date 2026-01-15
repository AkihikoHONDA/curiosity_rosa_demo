# T02: 設定ローダ（YAML読込、デフォルト値、パス解決、最低限のバリデーション）【更新版】

## 背景
本デモは、トピック名や閾値、Toolコスト、プロンプトテンプレ等を設定ファイル（YAML）として外出しし、ノード実装をハードコードから切り離す方針である。
またコンテナ運用（bind mount）では設定編集の頻度が高く、設定の探索パス・必須キー・エラーメッセージの質がデモ安定性に直結する。

設計上の“正”として、topics.yaml と prompts.yaml は以下を満たす。
- topics.yaml は「Curiosity実体I/F（Empty）」と「Adapter公開I/F（Trigger）」を分離して記述できる。
- prompts.yaml は ROSA の RobotSystemPrompts（4カテゴリ）＋ bootstrap/memory/templates を同居させる。

## 目的 / 完了条件 (DoD)
- [ ] `overlay_ws/src/curiosity_rosa_demo/curiosity_rosa_demo/infra/config_loader.py` を実装し、以下のYAMLを読み込めること。
  - `config/topics.yaml`
  - `config/thresholds.yaml`
  - `config/tool_costs.yaml`
  - `config/prompts.yaml`
  - `config/rviz.yaml`（中身は後続で増えてよいが、読込対象として確定）

- [ ] 設定ディレクトリの解決規則を確定し、実装とREADME（またはdocstring）に明記すること。
  デフォルトは ament index から `curiosity_rosa_demo` の share directory を解決し、その配下 `config/` を参照する。
  上書きは ROS2 parameter `config_dir`（文字列）で指定できる（指定時はそのディレクトリを最優先）。

- [ ] 最低限のバリデーションを実装すること（不足キーは例外で落とす）。

  topics.yaml（必須）
  - `images.input_compressed`
  - `images.output_capture_compressed`
  - `trace.events`
  - `tf.world_frame`, `tf.base_frame`
  - `services.capture_and_score.name`（型も持つなら `type`）

  topics.yaml（必須：Adapter公開I/F）
  - `adapter.move_forward`, `adapter.turn_left`, `adapter.turn_right`, `adapter.move_stop`
  - `adapter.mast_open`, `adapter.mast_close`, `adapter.mast_rotate`
  - `adapter.get_status`
  - これらの型は原則 `std_srvs/srv/Trigger`（設計の正）

  topics.yaml（必須：Curiosity実体I/F）
  - `curiosity.move_forward`, `curiosity.turn_left`, `curiosity.turn_right`, `curiosity.move_stop`
  - `curiosity.mast_open`, `curiosity.mast_close`, `curiosity.mast_rotate`
  - これらの型は `std_srvs/srv/Empty`（外部仕様の正）

  thresholds.yaml（必須）
  - `light_model.x_min`, `light_model.x_good`
  - `quality.score_threshold`（0.0〜1.0）

  prompts.yaml（必須）
  - `robot_system_prompts.embodiment_and_persona`
  - `robot_system_prompts.critical_instructions`
  - `robot_system_prompts.relevant_context`
  - `robot_system_prompts.nuance_and_assumptions`
  - `bootstrap.enabled`, `bootstrap.text`
  - `memory.enabled`, `memory.max_events`
  - `templates`（空でも良いがキーは存在させる）

  tool_costs.yaml（必須）
  - `tools`（辞書）かつ各値は 0 以上の整数

- [ ] データ構造をコード上で扱いやすい形（dataclass）にすること。
  - `TopicsConfig / ThresholdsConfig / ToolCostsConfig / PromptsConfig / RvizConfig`
  - まとめて返す `ConfigBundle`

- [ ] テスト併走（最低限）。
  - [ ] `pytest -q` が通る
  - [ ] 正常系: 最小YAMLを読み込み、期待値が dataclass に入る
  - [ ] 異常系: 必須キー欠落・範囲外値で例外になる（例外メッセージにファイル名とキーを含める）

## 実装詳細
- 実装する関数（例）
  - `resolve_config_dir(node: Node | None, override: str | None = None) -> Path`
  - `load_all_configs(config_dir: Path | None = None, node: Node | None = None) -> ConfigBundle`
  - `load_topics_config(path: Path) -> TopicsConfig` 等

- 実装メモ
  設定値は辞書のまま下流へ渡さず、参照点を固定する。
  例として、下流は `cfg.topics.images.output_capture_compressed` のように参照できることを正とする。

## 前提条件
- 完了しておくべきタスク:
  - T01（リポジトリ雛形）
