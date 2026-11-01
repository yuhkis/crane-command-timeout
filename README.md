# CRANE+ V2　操作指令が途切れたら腕を止める

肩に6秒の軌道を一度だけ送り、操作側から0.10秒ごとに動作の継続を知らせる。操作側が送信をやめたあと、途絶を監視する場合と監視しない場合で、腕がどれだけ進むかをGazeboで比べる。

## 停止処理を読む

[trial.py](trial.py) の冒頭にある `watch_motion()` が停止を判断する。実行中の軌道があれば、明示STOPを受けたとき、または最後の有効な指令から設定時間が経ったときに `arm.cancel()` を1回呼ぶ。監視なしの条件では軌道を最後まで動かす。

同じファイルの `Receiver.on_target()` が指令の受信時刻を記録し、`allow_motion()` が有効時刻を更新する。肩を動かし始めるのは最初の1通だけなので、取消し後に指令が届いても同じ軌道を再開しない。初期姿勢に戻す操作と関節角の取得は [gazebo_arm.py](gazebo_arm.py)、引数・送信側の起動・結果保存は [trial_setup.py](trial_setup.py) に分けた。

## 動画と同じ2条件を実行する

必要な環境はROS 2 Humble、Gazebo Fortress、CRANE+ V2のJointTrajectoryController、Python 3。講座の環境でシミュレータと腕のコントローラを起動し、キーボード操作ノードは終了する。同じROSドメインのBashでこのディレクトリから実行する。[講座公開の起動例](https://github.com/matsuolab/physicalai_teleop)

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 trial.py baseline results/baseline-1
python3 trial.py guard results/guard-1
python3 analyze.py results
```

`baseline` と `guard` は、肩を直立に戻し、同じ−0.90 radの目標へ動かし、操作側が20通で送信を終える。監視なしでは腕が目標まで進み、監視ありでは途中で止まる。監視初期値は0.30秒で、例えば `--timeout 0.50` で変更できる。`python3 trial.py continuous results/continuous-1` では85通送り、途絶しないときの目標到達を確認できる。各試行に新しい出力先を使う。

動画では0分14秒から監視なし、0分25.5秒から監視あり、0分37秒から両条件を同時に表示する。提出用ZIPの `filmed/` は撮影時に実行した送信側・受信側の原版。撮影時の版を動かす場合は `cd filmed` のあとで上の2つの `trial.py` コマンドを使う。

## 追加実験

| 調べること | 実行方法 |
|---|---|
| 0.20秒と0.30秒の停止判定 | `python3 run_timing.py timing-results`、続いて `python3 analyze_timing.py timing-results`。通常周期・0.25秒の間隔・0.35秒の間隔・完全途絶を各設定で5回ずつ、計40試行する。反復数は `--repeats` で変更できる。 |
| 一度だけ指令が遅れたとき | `python3 trial.py timing one-gap --timeout 0.20 --resume-gap 0.25`。0.25秒は20通目から21通目までの合計で、その後は通常周期に戻る。`--resume-gap` を省けば20通で途絶する。 |
| 操作者がSTOPを出したとき | `python3 trial.py guard stop-test/sent --send-stop` と `python3 trial.py guard stop-test/silent` を実行し、`python3 analyze.py stop-test` で比較する。前者は20通目と同時にSTOPを送る。 |
| 欠落、再送、滞留 | 次のコマンドで到着基準と生成基準を各18試行、順序確認を別の18試行で比べる。 |

```bash
python3 run_faults.py fault-main
python3 run_faults.py fault-ordered --ordered-only
python3 analyze_faults.py fault-main
python3 analyze_faults.py fault-ordered
```

1条件だけなら、例えば `python3 trial.py fault one-replay --pattern replay --policy ordered`。送信側は `fault_stream.py`、受信側で指令を採用するかを決めるのは `Receiver.on_permit()`。

| 指令列 | 送信側が行うこと |
|---|---|
| `normal` | 10 Hzで70通を送り続ける |
| `scattered` / `burst` | 14・24・34・44通目を落とす / 20〜23通目を連続で落とす |
| `lag_loss` | すべて0.18秒遅らせ、21通目を落とす |
| `replay` | 20通で新規生成を終了し、19・20・19・20通目を0.18秒間隔で再送する |
| `queue_tail` | 20通で生成を終え、末尾5通を順番に0.045〜0.225秒遅らせる |

`arrival` はすべての受信、`ordered` は新しい生成時刻の受信だけで期限を更新する。`issued` は順序も確認し、生成時刻から0.30秒で失効させる。実験では送受信とも同じシミュレーション時計を使う。生成時刻が受信時刻より10 msを超えて先なら除外し、10 ms以内の先行は受信時刻に丸める。実機の時計ずれや制動性能はこの実験の対象外。

## 測定結果を読む

各試行の `events.json` が受信と取消しの時刻、`states.csv` が肩関節角。通信パターン試験は `sender.csv` に指令の生成・欠落・実送信も記録する。反復試験の `plan.csv` は試行順。集計スクリプトはROSを使わずにこれらを読み、`measurements.csv` と `summary.json` を作る。

- 追加移動は、基準時刻の関節角からその後5.1秒間に最も離れた角度。初回比較と明示STOPでは最後の受信、通信パターンの生成終了条件では最後の新規生成を基準とする。
- 静止時刻は、直前0.10秒の角度差から計算した速さが0.01 rad/s以下で0.30秒続いた区間の開始。軌道の取消しを要求した時刻と別に記録する。
- 提出用ZIPの `measurements/` に初回比較10試行（initial）、停止設定40試行（timeout）、明示STOP比較10試行（explicit-stop）、通信パターン54試行（fault-patterns）の公表用CSVを収録した。正常対照は初回比較とは別に実行した。

テーマは受講者が選び、生成AIをコードの実装・整理、計測・集計、動画・レポート作成に使用した。
