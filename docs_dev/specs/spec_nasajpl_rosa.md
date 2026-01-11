
# nasa-jpl/rosa パッケージ仕様記述書

## 1. パッケージ概要

`ROSA` (Robot Operating System Agent) は、LangChain フレームワークをベースに構築された、ROS 1 および ROS 2 システムと自然言語で対話するための AI エージェントパッケージです。
開発者は、LLM に対してロボットの「身体性（Embodiment）」、環境、能力、制約をプロンプトとして与えるとともに、ROS のトピック通信やサービス実行を行う「ツール」を定義することで、自律的な推論に基づくロボット制御を実現できます。

### 1.1 主要コンポーネント

* **ROSA クラス**: エージェントのメインエントリポイントです。ROS バージョンの指定、LLM インスタンスの管理、ツールやプロンプトの統合、対話履歴の保持を担当します。
* **RobotSystemPrompts**: ロボット固有のメタ情報を保持するクラスです。「身体性とペルソナ」「クリティカルな指示」「行動制約とガードレールの設定」など、複数のカテゴリに分割してプロンプトを管理できます。
* **ROSATools**: ROS 標準のプリミティブ（ノード、トピック、サービス、パラメータ、ログ）にアクセスするための組み込みツール群を提供します。これらは ROS 1 または ROS 2 の API をラップしており、正規表現によるフィルタリングやリスト取得が可能です。

---

## 2. コア仕様の詳細

### 2.1 エージェントの動作制御

エージェントは LangChain の `AgentExecutor` を内部で利用しており、デフォルトで最大 100 回のイテレーション（思考・行動のループ）が可能です。
また、ロボット制御における安全性を確保するため、以下の重要な動作要件がシステムプロンプトによって規定されています。

* **逐次的実行（Sequential Execution）**: 複数のツールを同時に呼び出すことは禁止されており、一つのアクションの結果を待ってから次のアクションを選択するよう強制されます。
* **リアルタイム情報の優先**: assumptions（推論上の仮定）ではなく、必ず現在の ROS グラフ（ノードやトピックの状態）をツールで確認してから回答することが要求されます。

### 2.2 拡張ツール（Custom Tools）の定義

開発者は、特定のロボット固有の機能を `@tool` デコレータを用いてエージェントに追加できます。追加されたツールは LLM によってその説明文（Docstring）に基づき選択されます。

* **入力**: 型ヒントを用いた引数を定義します。LLM はこの型に従って引数を生成します。
* **出力**: 実行結果や、エラーが発生した際の理由を文字列（または辞書）として返却します。エージェントはこの返却値を見て、次の行動を推論します。

---

## 3. カスタムエージェント作成の一般仕様

`ROSA` を用いて特定のロボット用エージェントを構築する際の、パッケージとしての拡張・構成仕様を以下に示します。

### 3.1 ツールによる能力の拡張

カスタムエージェントの能力は、Python 関数として定義された「ツール」のリストを `ROSA` インスタンスへ渡すことで決定されます。

* **Tool Discovery**: エージェントは LangChain のツールインターフェースに準拠した関数を、利用可能な「スキル」として認識します。
* **Feedback Loop**: ツールが例外をスローしたり、特定のエラーメッセージを返したりした場合、`ROSA` はその出力を観測（Observation）として思考プロセスに取り込み、再試行や代替手段の検討を自律的に行います。これは、環境の制約や動的な障害をエージェントが「発見」するための核となる仕様です。

### 3.2 構造化されたシステムプロンプトの構成

`RobotSystemPrompts` クラスを用いることで、エージェントの動作を決定づけるプロンプトを以下の属性ごとに定義・注入できます。

* **embodiment_and_persona**: ロボットの物理的特性や性格、役割（例：探査ローバー、配送ロボット等）を定義します。
* **critical_instructions**: 安全確保や運用上の最優先ルール（例：バッテリー残量への注意、衝突回避の優先）を記述します。
* **relevant_context**: 周辺環境やミッションの背景情報を与えます。
* **nuance_and_assumptions**: 開発者が LLM に持たせたい特定の知識や、暗黙の了解（例：特定の地形での挙動の傾向）を指定します。

### 3.3 ROS との統合インターフェース

パッケージは、ROS 1 および ROS 2 の両方に対応した抽象化レイヤーを提供します。

* **ROS バージョン指定**: `ROSA` クラスの初期化時に `ros_version` 引数（1 または 2）を指定することで、適切な内部ツールセットが読み込まれます。
* **通信モデルのラップ**: ROS サービス、アクション、トピックへのパブリッシュ/サブスクライブを Python 関数としてラップし、それをツールとしてエージェントに公開することが標準的な構築手法となります。

### 3.4 LLM インフラストラクチャの柔軟性

`ROSA` は LangChain の `BaseChatModel` を継承する任意の LLM クラスを受け入れます。

* **プロバイダ非依存**: OpenAI, Anthropic などの商用モデルに加え、Ollama 等を介したローカル LLM の利用も仕様上可能です。これにより、計算リソースや通信環境に応じたエージェント構成が可能となっています。

---

## 4. 導入とセットアップ仕様

`ROSA` は標準的な Python パッケージとして構成されており、ライブラリとしてのインポートおよび ROS ノードへの組み込みが可能です。

### 4.1 インストール方法

リポジトリの構成と GitHub Actions の定義に基づき、以下の方法で導入できます。

* **PyPI からのインストール**:
`jpl-rosa` という名称で PyPI に登録されており、標準的なパッケージ管理ツールでインストール可能です。

```bash
pip install jpl-rosa
```


* **ソースからの開発インストール**:
リポジトリをクローンし、編集可能モードでインストールすることで、カスタムツールやプロンプトの開発が容易になります。

```bash
git clone https://github.com/nasa-jpl/rosa.git
cd rosa
pip install -e .
```



### 4.2 依存関係と環境要件

* **Python ランタイム**: `python 3.9` 以上、`3.11` 未満が推奨されています。
* **ROS 環境**: `ROS_VERSION` 環境変数を参照して動作を切り替えるため、実行環境に ROS 1 (Noetic) または ROS 2 (Humble/Jazzy等) がインストールされ、`setup.bash` がソースされている必要があります。
* **LLM 連携**: `langchain` および各モデルプロバイダの SDK（`openai`, `anthropic` 等）に依存します。

### 4.3 ライブラリとしての取り込み仕様

開発者は自身のプロジェクトにおいて、以下のような構成で `ROSA` をインポートし、自律エージェントを定義します。

1. **エージェントの初期化**: `rosa.ROSA` クラスをインスタンス化します。この際、LLM インスタンスと ROS バージョンを渡します。
2. **プロンプトの設定**: `rosa.RobotSystemPrompts` を用いて、ロボット固有の定義（身体性や制約）を注入します。
3. **カスタムツールの登録**: `langchain.tools.tool` デコレータで作成した関数をリスト形式でエージェントに渡します。

### 4.4 プロジェクトへの統合パターン

* **スタンドアロン・スクリプト**: Python スクリプト内から直接 `ROSA` を呼び出し、コマンドラインや GUI と対話させる形態。
* **ROS ノードへの組み込み**: `rosa` をラップした専用の ROS ノードを作成し、特定の Topic や Service 経由で自然言語入力を受け取り、ロボットのアクション（Tool 実行）へ変換する形態（`turtle_agent` の実装例がこれに該当します）。

--- 

## 5. コードサンプル

Here is a quick and easy example showing how to add new tools and prompts to ROSA:

```python
from langchain.agents import tool
from rosa import ROSA, RobotSystemPrompts

@tool
def move_forward(distance: float) -> str:
    """
    Move the robot forward by the specified distance.
    
    :param distance: The distance to move the robot forward.
    """
    # Your code here ...
    return f"Moving forward by {distance} units."

prompts = RobotSystemPrompts(
    embodiment_and_persona="You are a cool robot that can move forward."
)

llm = get_your_llm_here()
rosa = ROSA(ros_version=1, llm=llm, tools=[move_forward], prompts=prompts)
rosa.invoke("Move forward by 2 units.")
```

- ツールの追加

```python
@tool
def descriptive_tool_name(param1: type1, param2: type2) -> str:
    """
    Description of the tool.
    
    :param param1: Description of param1 and how it is used.
    :param param2: Description of param2 and how it is used.
    """
    # Your code here ...
    return f"Action taken: {ACTION}, retrieved data: {DATA}."
```

- プロンプト

```python
prompts = RobotSystemPrompts(
    embodiment_and_persona="You are a cool robot that does cool stuff.",
    critical_instructions="You must confirm all actions with the operator before proceeding. Failure to do so might result in damage to the robot or its environment.",
)
```