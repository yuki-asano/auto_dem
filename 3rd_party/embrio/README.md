# embrio 連携用 HTTP サーバ

auto_dem の計測シーケンス（ポリマー膜のピックアップ → 10 / 28 / 40 GHz の誘電率計測）を
**別 PC から起動・監視する**ための HTTP サーバと、それを euslisp の実装へつなぐ roseus ノードです。
合同会社 embrio 側のラボオーケストレータから auto_dem をリモート実行するために追加しました。

同じサーバが **FR3 の基本動作**（姿勢の読み出し・関節移動・直線移動・グリッパ）と、
操作 PCに接続した **USB カメラ**の中継も行います。embrio 側は AMR で運んだトレイを FR3 で受け取り、
スライダの上で傾けて膜を落とし、トレイを AMR へ戻すところまでを自分で振り付け、
その後の計測シーケンスを `/start` で起動します。`--devices` を指定すると、扱う装置を絞れます。

| ファイル | 内容 |
| --- | --- |
| `euslisp/auto_dem_embrio_node.l` | `dielectrometer-manip.l` を読み込み、計測一括関数と FR3 の基本動作を 1 本の ROS サービス（`roseus/StringString`）で公開する roseus ノード |
| `scripts/auto_dem_http_server.py` | 上記サービスを JSON/HTTP で包む rospy ノード。カメラの中継も同じポートで提供する |
| `scripts/embrio_camera.py` | USB カメラの中継処理（上記サーバから import される） |
| `launch/auto_dem_http_server.launch` | 上記 2 ノードと rosbridge を起動する launch ファイル（`auto_dem.launch` の代わりに使う） |

依存は次のとおりです。

- 計測・FR3（`dem` / `fr3`）: Python 3.8 の標準ライブラリ、rospy、`roseus`（StringString サービス型）、auto_dem とその依存（panda_eus など）。`fr3` はさらに actionlib と franka_msgs（franka_ros の一部）
- カメラ（`camera`）: ROS Noetic に含まれる `python3-opencv`（cv2 / numpy）

カメラを使う場合、起動ユーザが `video` グループに入っている必要があります（`/dev/video*` を開くため）。
roseus ノードは `dielectrometer-manip.l` と同じく irtviewer のウィンドウを開くので、**DISPLAY のある端末**
（またはログイン済みのデスクトップセッション）で起動してください。

## なぜ必要か

auto_dem の計測は `dielectrometer-manip.l` の euslisp 関数で、ROS トピック
`/start_dielectrometer_manip` で起動するだけです。完了の通知も、途中で止める口も、別 PC から
FR3 を動かす口もありませんでした。このサーバはその入口を足すだけのものです。

## 起動方法

操作 PC では `roslaunch auto_dem auto_dem.launch` の**代わりに**
`roslaunch auto_dem auto_dem_http_server.launch` を起動します。この launch は `auto_dem.launch` と同じ
rosbridge（9090 番）を含み、`dielectrometer_manip` ノードだけを embrio 用のノードに置き換えたものです。
embrio 用のノードも `/start_dielectrometer_manip` を受け取り、`dielectrometer-manip-node.l` の `cb` と
同じ処理（`measure-40G :do-measure t :insert-until-contact nil`）を実行するので、上流 README の
`rostopic pub -1 /start_dielectrometer_manip std_msgs/Bool "data: true"` はそのまま使えます。
それ以外の PC の手順は上流の README のとおりで、**起動する順番だけ守ってください。**

1. ロボット PC: `roscore`、`roslaunch auto_dem fr3_dielec.launch`
2. 操作 PC: `roslaunch auto_dem auto_dem_http_server.launch`（下記）
3. 誘電率計 PC（WSL）: ShockLine、`roscore`、`rosrun aet_instrument DielectricCalculator_server.py`（管理者権限）

**2 は 3 より先に起動します。** 誘電率計 PC の `DielectricCalculator_server.py` は、起動時に roslibpy で
操作 PC の rosbridge（`192.168.0.44:9090`）へつなぎに行き、そこから `/dielectrometer_cmd` を受け取ります。
rosbridge がまだ上がっていなければ接続に失敗して終了し、接続が切れたときも終了します。同じ理由で、
**操作 PC 側を再起動したら誘電率計 PC の `DielectricCalculator_server.py` も起動し直してください。**

操作 PC では catkin workspace を source した端末（`auto_dem.launch` を起動していたのと同じ端末）から:

```bash
# auto_dem が見つかるか確認（パスが出れば source 済み）
rospack find auto_dem
# 見つからなければ source する（~/.bashrc に書いてあれば不要）
source ~/catkin_ws/devel/setup.bash

# launch ファイルから（rosbridge + roseus ノード + HTTP サーバ）
roslaunch auto_dem auto_dem_http_server.launch
roslaunch auto_dem auto_dem_http_server.launch port:=8302

# 個別に起動する場合（rosbridge も別途起動する）
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun auto_dem auto_dem_embrio_node.l
rosrun auto_dem auto_dem_http_server.py --port 8302
```

誘電率計 PC がつながったかは操作 PC で確認できます。`/dielectrometer_cmd` の Subscribers に
rosbridge のノードが出ていれば、計測器まで指令が届きます。出ていないまま計測を始めると、腕は動いても
計測器は動きません（指令は送りっぱなしなので、エラーにもなりません）。

```bash
rostopic info /dielectrometer_cmd
```

既存の `roslaunch auto_dem auto_dem.launch` は**同時に起動しない**でください。rosbridge のポートが
衝突するうえ、`/start_dielectrometer_manip` を受けた 2 つのノードが互いを知らずに同じ腕を動かします。

### 扱う装置を絞る（`--devices`）

既定では `dem,fr3,camera` の 3 つすべてを扱います。カンマ区切りで一部だけを指定できます。

```bash
rosrun auto_dem auto_dem_http_server.py --devices dem,fr3
roslaunch auto_dem auto_dem_http_server.launch devices:=camera

# dem も fr3 も外した場合は ROS を読み込まないので、python3 で直接起動できる
python3 3rd_party/embrio/scripts/auto_dem_http_server.py --devices camera
```

- 指定しなかった装置のパスには、404 と「disabled」というエラーを返します。
- 未知の名前を指定すると、起動時に失敗します。
- カメラの open は起動時には行いません。設定は embrio 側が持っており、接続時に送ります。

ノード名は `auto_dem_embrio_manip` と `auto_dem_http` で、既存ノードとは衝突しません。
既定ポートを 8302 にしてあるのは、研究室内の他のサーバ（auto_tma 8300 / auto_cutter 8301）と
揃えるためです。

## エンドポイント

### 計測シーケンス（`dem`）

| Method | Path | 説明 |
| --- | --- | --- |
| GET | `/health` | 扱っている装置の一覧 `{"devices": [...]}` |
| GET | `/status` | `state`（`idle` / `running` / `finished` / `failed`）、`running`、`params`、`log`、`last_error`、`started_at` / `finished_at` を返す |
| POST | `/start` | 膜をスライダからピックし、選んだ周波数を 40 → 28 → 10 GHz の固定順で計測する。body は `measure_40g` / `measure_28g` / `measure_10g`（どれを計測するか、既定はすべて true、1 つ以上必須）、各周波数の関数 `measure-40G` / `-28G` / `-10G` の引数（`thickness`, `sample_name`, `file_path`, `do_measure`, `insert_until_contact`）、`measure_times`（各周波数で関数を呼ぶ回数）。すべて省略可。`thickness` / `sample_name` / `file_path` は省略すると euslisp の関数にも渡さず、関数の既定値（保存先・試料名・厚み）がそのまま使われる。`do_measure` / `insert_until_contact` の既定は true、`measure_times` は 1。202 受理 / 400 引数不正 / 409 腕が使用中 |
| POST | `/abort` | 実行中の動作をキャンセルし、走っている処理を打ち切る。run は `failed`（`last_error: "aborted"`）になる |

計測シーケンスは上流の `pick-polymer-from-slider` と `measure-40G` / `-28G` / `-10G` を並べたものです。
上流の `pick-polymer-and-measure-40-28-10` と同じく、ピック → 40G と 28G → 10G は直接つなぎ、それ以外の移り
（ピック → 28G、40G → 28G など）は reset pose を経由します。3 つとも選んで `measure_times` が 1 なら同関数と
同じ動きです。2 以上では関数をその回数呼ぶので、`-ntimes-each` より各回の初期姿勢への移動が遅くなります。

`log` には開始と終了しか入りません（euslisp の関数に途中経過を外へ出す口が無いため）。

### FR3 の基本動作（`fr3`）

姿勢は `[x, y, z, roll, pitch, yaw]`（mm / deg、R = Rz(yaw)·Ry(pitch)·Rx(roll)）、関節は deg × 7 です。
TCP は `fr3_hand` から z 方向 +103.4 mm（指先の中心）で、`dielectrometer-manip.l` が挿入用に向きを変えた
end-coords とは独立です。

| Method | Path | 説明 |
| --- | --- | --- |
| GET | `/fr3/status` | `connected` / `enabled`（roseus ノードへの ping 結果）、`robot_mode`（franka_control の `franka_states` から。`move` / `reflex` など）、`has_errors`（ノードが応答しない、`robot_mode` が `reflex` / `user_stopped` / `automatic_error_recovery`、`current_errors` が立っている、`franka_states` が 2 s 以上届いていない、のいずれか）、`busy`、`last_error` |
| GET | `/fr3/pose` | `{"joints": [7], "position": [6], "gripper_mm"}` |
| POST | `/fr3/move-joints` | `{"joints": [7], "speed": deg/s, "linear_speed": mm/s}`。所要時間は最大関節差 ÷ speed（最短 0.5 s）。到達後に読み戻し、2 deg を超えて外れていればエラー。`linear_speed`（省略可）を付けると、先にその関節角での TCP 姿勢まで直線で動き（`move-position` と同じ区分 IK・検証）、そのあと関節角を合わせる。関節補間では途中で手先が傾くので、トレイを持って運ぶときに使う |
| POST | `/fr3/move-position` | `{"position": [6], "speed": mm/s, "step_mm": mm}`。直線を `step_mm` 刻みに分け、全点の IK が解けてから動く（解けなければ動かずエラー）。到達後に 5 mm / 2 deg で検証 |
| POST | `/fr3/gripper/move` | `{"position": mm, "speed": mm/s}` → `{"achieved": mm}`。move アクションで指定幅へ動かす。アクションが失敗を返せばエラー |
| POST | `/fr3/gripper/grasp` | `{"width": mm, "speed": mm/s, "force": N, "epsilon_inner": mm, "epsilon_outer": mm}` → `{"achieved": mm}`。grasp アクションで物に当たるまで閉じ、`force` で保持する。止まった幅が `width - epsilon_inner` 〜 `width + epsilon_outer`（既定は各 5 mm）に入らなければエラー（物が無く全閉した場合も含む） |
| GET | `/fr3/gripper/position` | `{"position": mm}` |
| POST | `/fr3/abort` | `/abort` と同じ |
| POST | `/fr3/errors/clear` | 衝突などで止まった（ランプが赤の）ロボットを、franka_control の error recovery アクションで復帰させる → `{"robot_mode", "errors"}`。異常が無ければアクションを送らず今の状態を返す（0.10.2 より前の franka_control は、異常の無いときの goal が返らない）。復帰後の状態にまだ異常があれば 500、アクションサーバが無ければ 503、腕が使用中なら 409。腕は動かない |

`achieved` はアクションの完了直後ではなく、幅が 0.3 s 動かなくなってから読みます（完了直後の値は指の動きに
遅れるため）。3 s 待っても落ち着かなければエラーです。

### カメラの中継（`camera`）

中継はキャプチャのオブジェクトを操作 PC 側に置くだけの薄い層です。カメラ設定の検証は embrio 側が行います。

| Method | Path | 説明 |
| --- | --- | --- |
| POST | `/camera/open` | `{"source", "api"}`（`api` は OpenCV の VideoCapture API 番号）。開けなければ 502 |
| POST | `/camera/set`, `/camera/get` | `{"prop", "value"}` / `{"prop"}`（`prop` は OpenCV のプロパティ番号） |
| POST | `/camera/grab` | 1 フレームを読み捨てる（転送しない） |
| POST | `/camera/read` | 1 フレームを生のバイト列で返す（`X-Frame-Shape` / `X-Frame-Dtype` ヘッダ付き）。読めなければ `{"ok": false}` |
| POST | `/camera/release` | カメラを閉じる |

疎通確認:

```bash
curl http://<操作 PC>:8302/health
curl http://<操作 PC>:8302/fr3/pose
curl -X POST http://<操作 PC>:8302/start -H 'Content-Type: application/json' \
     -d '{"thickness": 0.355, "sample_name": "sample1", "file_path": "C:/Users/test/Desktop/asano/robot_exp.csv"}'
curl http://<操作 PC>:8302/status
```

## 設計上の判断・制約

- **腕を動かせるのは一度に 1 つ。** 計測 run と `/fr3/*` の動作は同じスロットを共有し、使用中なら
  待たせずに 409 を返します。`/abort` だけはスロットを取らずに通ります。
- **run スロットは 1 つ。** 実行中の `/start` は 409 を返します。
- **`/start_dielectrometer_manip` で起動した処理も同じ腕を使います。** roseus ノードは腕を使っている間、
  `ping` 以外の呼び出しを「the arm is busy」で拒否します（HTTP 側のスロットは空いて見えるため、
  HTTP では 409 ではなく 500 になります）。腕が使用中のときに届いたトピックは、エラーをログに出して捨てます。
  トピックで起動した処理は `/status` に現れませんが、`/abort` では止められます。
  `auto_dem_embrio_node.l` の `embrio-start-dielectrometer-manip` は `cb` と同じ処理です。
- **`/abort` は動作中の `:angle-vector` をキャンセルし、走っている euslisp の処理を打ち切ります。**
  計測器への指令（`dielectrometer_cmd`）は取り消せないので、計測中の誘電率計はそのまま動き続けます。
- **roseus ノードは単一スレッドです。** 処理中は `:wait-interpolation` の中でしか ROS のメッセージを
  受け取らないため、中断が届くのは腕が動いている区間だけです。
- **引数の検証は HTTP サーバ側と roseus ノード側の両方で行います。** 不正な値を euslisp 側へ渡すと
  デバッガに落ちてノードが止まるため、euslisp 側ではエラーハンドラを差し替えて JSON のエラーとして返します。
- **run の状態はプロセス内メモリのみ保持します。** サーバを再起動すると走行中 run の状態は失われます
  （シーケンス自体は roseus ノード側で最後まで走ります）。
- 認証はありません。ラボ内 LAN での利用を前提としています。
