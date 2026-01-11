# T06: Simulator: 画像処理パイプライン（/image_raw/compressed購読、暗化＋スコア文字入れ、/capture/image_raw/compressedへ再Publish）【更新版】

## 背景
本デモでは「暗い／明るい」をエージェントが視覚的にも理解できるよう、Curiosityデモのカメラ画像を Simulator 側で加工し、暗いときは暗化しつつスコアを重畳表示して加工済み画像として再配信する。
加工済み画像の出力先は `/capture/image_raw/compressed` を正とする。

この処理が安定すると、後続の `CaptureAndScore`（T07）が「画像とスコアが整合した観測結果」を返せるようになり、Agent側のデバッグも容易になる。

## 目的 / 完了条件 (DoD)
- [ ] Simulator Node に画像購読・加工・再Publish を実装すること。

  - [ ] 入力購読: `topics.images.input_compressed`（例: `/image_raw/compressed`）

  - [ ] 出力Publish: `topics.images.output_capture_compressed`（例: `/capture/image_raw/compressed`）

- [ ] 画像加工仕様（最小でよいが、デモとして意味が伝わること）。

  - [ ] `last_score`（T05）に基づき暗化係数を適用できる（暗いほど暗くする）

  - [ ] 画像上に `score=0.xx` と `GOOD/BAD`（または同等）を重畳表示できる

  - [ ] `last_score` 未確定時の挙動を仕様化し、実装とテストで固定する

    「未確定表示」または「素通し」のどちらかに統一する。

- [ ] 画像は `sensor_msgs/msg/CompressedImage` を扱い、encode/decode は OpenCV（cv2）＋ numpy を用いること。

- [ ] CPU負荷を上げ過ぎない工夫を含めること。

  - [ ] 例: 受信コールバックでは最新画像を保持するだけにして、timerで一定周期（例: 5Hz）で加工・publishする

- [ ] テスト併走。

  - [ ] ユニットテスト: 「暗化＋文字入れ」関数を純関数として切り出し、出力サイズ維持、暗化強化で平均輝度が下がることを確認する

  - [ ] 小粒度統合（任意）: CompressedImage を投入すると `/capture/image_raw/compressed` に CompressedImage が出る

  rclpy統合が難しければ、関数テスト＋手動スモーク手順をDoDに含める。

- [ ] 作成・修正ファイル。

  - `curiosity_rosa_demo/sim/image_pipeline.py`（新規）

  - `curiosity_rosa_demo/sim/simulator_node.py`（T05で作成した骨格に追加）

  - `curiosity_rosa_demo/tests/test_image_pipeline.py`（新規）

  - `package.xml / setup.py`（opencv等の依存が必要なら追記）

## 実装詳細
- ノード実装上の要点
  画像callbackは「最新フレームを保存するだけ」にし、加工は timer 側で行う。
  `last_score` は更新タイミングとズレうるため、「直近値を表示する」仕様として割り切る。

## 前提条件
- 完了しておくべきタスク:
  - T02（設定ローダ）

  - T05（last_score の生成・保持）

  - T04（subscriber/publisher生成の補助）
