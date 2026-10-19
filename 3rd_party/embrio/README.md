# embrio 連携用 HTTP サーバ

auto_dem の計測シーケンス（ポリマー膜のピックアップ → 10 / 28 / 40 GHz の誘電率計測）を
**別 PC から起動・監視する**ための HTTP サーバと、それを euslisp の実装へつなぐ roseus ノードです。
株式会社 embrio 側のラボオーケストレータから auto_dem をリモート実行するために追加しました。

同じサーバが **FR3 の基本動作**（姿勢の読み出し・関節移動・直線移動・グリッパ）と、
この PC に接続した **USB カメラ**の中継も行います。embrio 側は AMR で運んだトレイを FR3 で受け取り、
スライダの上で傾けて膜を落とし、トレイを AMR へ戻すところまでを自分で振り付け、
その後の計測シーケンスを `/start` で起動します。`--devices` を指定すると、扱う装置を絞れます。

**auto_dem 本体のファイルは一つも変更していません。** 追加物はすべてこのディレクトリ
（`3rd_party/embrio/`）に閉じており、明示的に起動しない限り何も動きません。既存の
`roslaunch auto_dem auto_dem.launch` の使い方は従来どおりです。

| ファイル | 内容 |
| --- | --- |
| `euslisp/auto_dem_embrio_node.l` | `dielectrometer-manip.l` を読み込み、計測一括関数と FR3 の基本動作を 1 本の ROS サービス（`roseus/StringString`）で公開する roseus ノード |
| `scripts/auto_dem_http_server.py` | 上記サービスを JSON/HTTP で包む rospy ノード。カメラの中継も同じポートで提供する |
| `scripts/embrio_camera.py` | USB カメラの中継処理（上記サーバから import される） |
| `launch/auto_dem_http_server.launch` | 上記 2 ノードだけを起動する launch ファイル |

**この PC への追加インストールはありません。** 依存は次のとおりで、いずれも既にあるものです。

- 計測・FR3（`dem` / `fr3`）: Python 3.8 の標準ライブラリ、rospy、`roseus`（StringString サービス型）、auto_dem とその依存（panda_eus など）
- カメラ（`camera`）: ROS Noetic に含まれる `python3-opencv`（cv2 / numpy）

カメラを使う場合、起動ユーザが `video` グループに入っている必要があります（`/dev/video*` を開くため）。
roseus ノードは `dielectrometer-manip.l` と同じく irtviewer のウィンドウを開くので、**DISPLAY のある端末**
（またはログイン済みのデスクトップセッション）で起動してください。

## なぜ必要か

auto_dem の計測は `dielectrometer-manip.l` の euslisp 関数で、ROS トピック
`/start_dielectrometer_manip` で起動するだけです。完了の通知も、途中で止める口も、別 PC から
FR3 を動かす口もありませんでした。このサーバはその入口を足すだけのものです。

## 起動方法

ロボット側（tiger: `roscore` と `roslaunch auto_dem fr3_dielec.launch`）と誘電率計側（WSL の
`aet_instrument`）は**従来どおり**起動してください。その上で、操作 PC（lion）の catkin workspace を
source した端末から:

```bash
# launch ファイルから（roseus ノード + HTTP サーバ）
roslaunch auto_dem auto_dem_http_server.launch
roslaunch auto_dem auto_dem_http_server.launch port:=8302

# 個別に起動する場合
rosrun auto_dem auto_dem_embrio_node.l
rosrun auto_dem auto_dem_http_server.py --port 8302
```

既存の `roslaunch auto_dem auto_dem.launch`（`dielectrometer_manip` ノード）は**同時に起動しない**
でください。同時に起動すると、プレス PC からのトピックでも腕が動きます。

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
| POST | `/start` | body は `pick-polymer-and-measure-40-28-10` の引数（`thickness`, `sample_name`, `file_path`, `do_measure`, `insert_until_contact`）と `measure_times`（1 なら同関数、2 以上なら `-ntimes-each`）。すべて省略可で、既定値は euslisp 側と同一。202 受理 / 400 引数不正 / 409 腕が使用中 |
| POST | `/abort` | 実行中の動作をキャンセルし、走っている処理を打ち切る。run は `failed`（`last_error: "aborted"`）になる |

`log` には開始と終了しか入りません（euslisp の関数に途中経過を外へ出す口が無いため）。

### FR3 の基本動作（`fr3`）

姿勢は `[x, y, z, roll, pitch, yaw]`（mm / deg、R = Rz(yaw)·Ry(pitch)·Rx(roll)）、関節は deg × 7 です。
TCP は `fr3_hand` から z 方向 +103.4 mm（指先の中心）で、`dielectrometer-manip.l` が挿入用に向きを変えた
end-coords とは独立です。

| Method | Path | 説明 |
| --- | --- | --- |
| GET | `/fr3/status` | `connected` / `enabled` / `has_errors` / `busy` / `last_error`（roseus ノードへの ping 結果） |
| GET | `/fr3/pose` | `{"joints": [7], "position": [6], "gripper_mm"}` |
| POST | `/fr3/move-joints` | `{"joints": [7], "speed": deg/s}`。所要時間は最大関節差 ÷ speed（最短 0.5 s）。到達後に読み戻し、1 deg を超えて外れていればエラー |
| POST | `/fr3/move-position` | `{"position": [6], "speed": mm/s, "step_mm": mm}`。直線を `step_mm` 刻みに分け、全点の IK が解けてから動く（解けなければ動かずエラー）。到達後に 2 mm / 2 deg で検証 |
| POST | `/fr3/gripper/move` | `{"position": mm, "speed": mm/s, "force": N}` → `{"achieved": mm}`。閉じる向きは grasp アクション（物で止まっても `force` で保持）、開く向きは move アクション |
| GET | `/fr3/gripper/position` | `{"position": mm}` |
| POST | `/fr3/abort` | `/abort` と同じ |

### カメラの中継（`camera`）

中継はキャプチャのオブジェクトをこの PC 側に置くだけの薄い層です。カメラ設定の検証は embrio 側が行います。

| Method | Path | 説明 |
| --- | --- | --- |
| POST | `/camera/open` | `{"source", "api"}`（`api` は OpenCV の VideoCapture API 番号）。開けなければ 502 |
| POST | `/camera/set`, `/camera/get` | `{"prop", "value"}` / `{"prop"}`（`prop` は OpenCV のプロパティ番号） |
| POST | `/camera/grab` | 1 フレームを読み捨てる（転送しない） |
| POST | `/camera/read` | 1 フレームを生のバイト列で返す（`X-Frame-Shape` / `X-Frame-Dtype` ヘッダ付き）。読めなければ `{"ok": false}` |
| POST | `/camera/release` | カメラを閉じる |

疎通確認:

```bash
curl http://<lion>:8302/health
curl http://<lion>:8302/fr3/pose
curl -X POST http://<lion>:8302/start -H 'Content-Type: application/json' \
     -d '{"thickness": 0.355, "sample_name": "sample1", "file_path": "C:/Users/test/Desktop/asano/robot_exp.csv"}'
curl http://<lion>:8302/status
```

## 設計上の判断・制約

- **腕を動かせるのは一度に 1 つ。** 計測 run と `/fr3/*` の動作は同じスロットを共有し、使用中なら
  待たせずに 409 を返します。`/abort` だけはスロットを取らずに通ります。
- **run スロットは 1 つ。** 実行中の `/start` は 409 を返します。
- **`/abort` は動作中の `:angle-vector` をキャンセルし、走っている euslisp の処理を打ち切ります。**
  計測器への指令（`dielectrometer_cmd`）は取り消せないので、計測中の誘電率計はそのまま動き続けます。
- **roseus ノードは単一スレッドです。** 処理中は `:wait-interpolation` の中でしか ROS のメッセージを
  受け取らないため、中断が届くのは腕が動いている区間だけです（下記「未検証」）。
- **引数の検証は HTTP サーバ側と roseus ノード側の両方で行います。** 不正な値を euslisp 側へ渡すと
  デバッガに落ちてノードが止まるため、euslisp 側ではエラーハンドラを差し替えて JSON のエラーとして返します。
- **run の状態はプロセス内メモリのみ保持します。** サーバを再起動すると走行中 run の状態は失われます
  （シーケンス自体は roseus ノード側で最後まで走ります）。
- 認証はありません。ラボ内 LAN での利用を前提としています。

## 未検証の事項

実機でまだ確かめていない点です。確認でき次第、この節を更新します。

- `/abort` が `:wait-interpolation` の途中で届くこと（robot-interface の待ちループが `ros::spin-once` を
  回していることが前提）。届かない場合は、動作の区切りまで止まりません。
- 計測待ち（`unix::usleep` の 20 秒）の間は中断が届かないこと。
- grasp アクションで物に当たって止まったときの到達幅（`achieved`）の読み値。
- `/fr3/move-position` の直線分割（既定 5 mm）で、分割点の間の経路が直線からどれだけ外れるか。
- roseus ノードの irtviewer に必要な DISPLAY が、lion の運用環境（リモート起動時）で得られること。

## 連絡先

不明点や変更のご要望は embrio（石川）までお願いします。
