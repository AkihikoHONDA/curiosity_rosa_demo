# T05: Simulator: 明るさモデル（tf→X推定→score算出、閾値判定、内部状態保持）

## 背景
本デモの「フレーム問題」要素の核は、“どの向き・どの場所で撮るべきか” をエージェントが試行錯誤し、暗い→改善策（mast回転 or 移動）を選ぶ点にあります。  
そのため、Simulator側で「環境は暗い／明るい」を決める観測モデル（light model）を、既存Curiosityデモの枠を崩さずに追加する必要があります（design.md「5.2 Simulator Node」「4.1 LightScore」「7.1 thresholds.yaml」）。

明るさモデルは、TFから rover の位置（world上の X）を推定し、その値から `score∈[0,1]` を算出し、閾値で `is_good` を決めます。ここを固定すると、後続の画像暗化（T06）や capture/service（T07）が迷わず実装できます。

## 目的 / 完了条件 (DoD)
- [ ] `curiosity_rosa_demo/sim/light_model.py`（または同等）を実装し、以下を満たす:
  - [ ] 入力: `rover_x`（float）
  - [ ] 出力: `LightScore(score: float, is_good: bool)`（T03のモデルを使用）
  - [ ] `score` は 0.0〜1.0 にクリップされる
  - [ ] `is_good` は thresholds.yaml の `quality.score_threshold`（例）により判定される
- [ ] Simulator Node が内部状態として「直近のscore」を保持できるようにする（T07のCaptureAndScoreで参照される前提）:
  - [ ] `last_score: LightScore | None`
  - [ ] `last_pose_x: float | None`
  - [ ] 更新周期はノードのtimer（例: 5〜10Hz程度。最終値を設計に合わせる）で良い（映像側は別タスク）
- [ ] TFから X を取得して light_model に流す処理を実装する（T04のTFユーティリティを使う）:
  - [ ] `lookup_x_in_world()` の `ToolResult` が `ok=False` の場合、Simulatorは `last_score` を更新しない（ログは残す）
- [ ] テスト併走:
  - [ ] ユニットテスト: `light_model.score()` の特性（境界・クリップ・単調性の最低保証）を検証する
  - [ ] 小粒度統合: static TF を流して `rover_x` が取れ、`last_score` が更新される（rclpyテスト or 可能なら関数分離で単体テストでも可）
- [ ] 作成・修正ファイル:
  - `curiosity_rosa_demo/sim/light_model.py`（新規）
  - `curiosity_rosa_demo/sim/simulator_node.py`（新規：骨格だけでも可。画像処理はT06で入れる）
  - `curiosity_rosa_demo/tests/test_light_model.py`（新規）
  - `config/thresholds.yaml`（キーの追加が必要な場合のみ、design.mdの例に合わせて整形）

## 実装詳細
- 使用するクラス/関数（例）
  - `class LightModel:`
    - `__init__(self, *, score_floor, score_ceil, x0, k, threshold, ...)`
    - `compute(self, rover_x: float) -> LightScore`
  - `SimulatorNode`（rclpy Node）
    - `self.timer = create_timer(period, self._tick)`
    - `_tick()` 内で:
      1) TFから `rover_x` を取得（T04ユーティリティ）
      2) `LightModel.compute(rover_x)` で `LightScore` を得る
      3) `self.last_score` 更新
- スコア関数の形（設計に沿って最小で良い）
  - design.md の「7.1 thresholds.yaml」および「5.2」の記述に従う。
  - 典型案:
    - “明るい領域” をある区間（例: `x ∈ [bright_x_min, bright_x_max]`）として線形 or シグモイドで 0→1 へ上げる
    - まずは単純な piecewise-linear（線形区間＋クリップ）で固定し、必要なら後で調整
- 参照すべき設計
  - design.md Section 5.2（Simulatorの責務）
  - design.md Section 4.1（LightScore）
  - design.md Section 7.1（thresholds.yaml）
  - design.md Section 6（tf/world_frame/base_frame）

## 前提条件
- 完了しておくべきタスク:
  - T02（設定ローダ：thresholds.yaml を読む）
  - T03（LightScoreモデル）
  - T04（TFユーティリティ）
