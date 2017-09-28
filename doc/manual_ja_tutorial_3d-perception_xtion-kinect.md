
## Xtion/Kinect カメラの TF キャリブレーション

3次元深度カメラの Xtion や Kinect を利用してロボット環境情報を取得する場合において，
得られた環境空間情報とロボットとの相対位置・姿勢が分からなければ意味がありません．

そのためにはカメラがロボットに対してどのような位置・姿勢で設置されているかを知る必要があります．
具体的にはカメラを設置するリンクもしくはワールド空間に対するカメラの TF を取得します．


### ロボット実機での Xtion/Kinect TF キャリブレーション

#### ロボットの準備

ROS を使用するので rtm_ros_bridge が起動している必要があります．

```
$ roslaunch nextage_ros_bridge nextage_ros_bridge_real.launch nameserver:=%HOSTNAME%
```

ロボット各関節のキャリブレーションを行います．
関節のキャリブレーションが終わったら一旦ロボットの姿勢を終了姿勢にしてサーボを切ります．

ここでは操作に iPython コンソールを使用しています．

```
$ ipython -i `rospack find nextage_ros_bridge`/script/nextage.py -- --host nextage

:
(many connection messages)
:

[hrpsys.py] initialized successfully

:

In [1]: robot.checkEncoders()
In [2]: robot.goOffPose()
In [3]: robot.servoOff()
```

#### チェッカーボードのロボットへの設置

カメラとロボットの相対位置・姿勢を定めるためにチェッカーボードをロボットに取り付けます．
このときチェッカーボードと設置するロボットのリンクとの相対位置は機械設計上既知である必要があります．

- 注意: ロボット実機への作業ですのでサーボが切れていることを確認してください．

下図のように胸部チェッカーボードもしくは腰部チェッカーボードをロボットに取り付けてください．

胸部チェッカーボード
![NEXTAGE OPEN - Chekcerboard on Chest / Top View](https://cloud.githubusercontent.com/assets/3119480/23578751/41009c9e-0121-11e7-9db1-585ce8b2f690.JPG)
![NEXTAGE OPEN - Checkerboard on Chest / Bottom View](https://cloud.githubusercontent.com/assets/3119480/23578755/4a47e9e2-0121-11e7-8b47-fb6efe2aa8ee.JPG)

腰部チェッカーボード
![NEXTAGE OPEN - Checkerboard on Waist](https://cloud.githubusercontent.com/assets/3119480/23707568/3b461486-0456-11e7-9c64-3a7c4dde4444.JPG)


#### サーボを入れる（胸部チェッカーボードの場合）

胸部チェッカーボードを使用する場合は CHEST_JOINT0 の関節角度を
0 deg（正面）で維持する必要がありますので，ロボットのサーボを入れて初期姿勢にします．

```
In [1]: robot.servoOn()
In [2]: robot.goInitial()
```

#### Xtion/Kinect カメラをチェッカーボードに向ける

Xtion/Kinect カメラにチェッカーボードが映るようにします．
頭部に Xtion/Kinect カメラを設置した場合には関節を制御して頭部姿勢を変えます．

下に頭部関節を動作・制御するコマンドを記します．

```
In [1]: robot.setJointAnglesOfGroup( 'head', [ 0.0, 45.0 ], 5.0 )
In [2]: robot.getJointAngles()
```

#### チェッカーボード検出プログラムの実行

Xtion/Kinect カメラを本作業を行っている UIコントロールボックス（Ubuntu PC/Vision PC）
もしくは 開発用コンピュータ（Ubuntu）に接続してください．

チェッカーボードを検出プログラムを実行します．
Kinect カメラを使用している場合は実行時に引数 `launch_openni2:=false` を指定してください．

Xtion カメラの場合
```
$ roslaunch nextage_calibration camera_checkerboard_chest.launch  #chest
or
$ roslaunch nextage_calibration camera_checkerboard_waist.launch  #waist
```

Kinect カメラの場合
```
$ roslaunch nextage_calibration camera_checkerboard_chest.launch launch_openni2:=false  #chest
or
$ roslaunch nextage_calibration camera_checkerboard_waist.launch launch_openni2:=false  #waist
```

Checkerboard Detector - 胸部チェッカーボード <br>
![Checkerboard Detector - Chest](https://cloud.githubusercontent.com/assets/3119480/23707535/1e2bb162-0456-11e7-800b-4df1c5055950.png)

Checkerboard Detector - 腰部チェッカーボード <br>
![Checkerboard Detector - Waist](https://cloud.githubusercontent.com/assets/3119480/23644970/2b156a44-034d-11e7-8975-aad46da11ba2.png)

#### RViz の起動

各機能の実行状況を確認するために RViz を起動します．

```
$ rviz -d `rospack find nextage_calibration`/conf/calibration.rviz
```

RViz - 胸部チェッカーボード <br>
![RViz - Checkerboard on Chest](https://cloud.githubusercontent.com/assets/3119480/23707683/9e0a8318-0456-11e7-9147-5d0150df026e.png)

RViz - 腰部チェッカーボード <br>
![RViz - Checkerboard on Chest](https://cloud.githubusercontent.com/assets/3119480/23707753/cf9967f0-0456-11e7-8556-8bcf0c561490.png)

#### カメラリンクと親リンクの TF の取得

`tf_echo` コマンドを利用して camera_link から親リンクへの TF を取得します．
ここでは camera_link から HEAD_JOINT1 への TF を表示します．

```
$ rosrun tf tf_echo HEAD_JOINT1_Link camera_link
```

`tf_echo` の結果の例
```
At time 93.641
- Translation: [0.101, -0.014, 0.199]
- Rotation: in Quaternion [-0.001, 0.704, 0.002, 0.710]
            in RPY (radian) [0.126, 1.563, 0.130]
            in RPY (degree) [7.223, 89.561, 7.477]
```

#### ロボットでの作業の終了

カメラリンクと親リンクの TF の取得ができたらロボットを終了姿勢にしてサーボを切ります．

```
In [1]: robot.goOffPose()
In [2]: robot.servoOff()
```

#### 全ノードの終了

ロボットのサーボが切れたら全てのノードを Ctrl-C で終了します．

#### カメラリンク TF の適用

取得した Xtion/Kinect カメラ TF をロボット上のカメラのパラメータとして適用します．
適用は Xtion/Kinect カメラモジュール xtion_kinect.launch 起動時に引数として渡して行います．

- 単位系
  - TF_COORD_XYZ : [m]
  - TF_COORD_RPY : [rad]

まず rtm_ros_bridge を起動します．

```
$ roslaunch nextage_ros_bridge nextage_ros_bridge_real.launch nameserver:=%HOSTNAME%
```

Xtion カメラの場合

```
$ roslaunch nextage_calibration xtion_kinect.launch TF_COORD_XYZ:='0.101 -0.014 0.199' TF_COORD_RPY:='0.126 1.563 0.130'
```

Kinect カメラの場合（`launch_openni2:=false` のみの違い）

```
$ roslaunch nextage_calibration xtion_kinect.launch TF_COORD_XYZ:='0.101 -0.014 0.199' TF_COORD_RPY:='0.126 1.563 0.130' launch_openni2:=false
```

#### Xtion カメラの画像キャリブレーション（オプション）

Xtion カメラにおいては画像のキャリブレーション情報を取得しておくことをお勧めします．
画像キャリブレーションの作業手順は下記のサイトを参照ください．

- ROS Wiki - How to Calibrate a Monocular Camera
  - [http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)


### Gazebo を用いた Xtion/Kinect TF 取得シミュレーション

実機で行う Xtion/Kinect カメラの TF キャリブレーションの作業は
Gazebo シミュレータ上でも同様のことを行うことができます．


#### Gazebo の起動

NEXTAGE OPEN ロボットモデルの Gazebo シミュレータを起動します．

```
$ roslaunch nextage_gazebo nextage_world.launch
```

- メモ: ROS / Gazebo でのシミュレーションのため rtm_ros_bridge は不要

#### Gazebo 内での Kinect とチェッカーボードモデルの設置

Gazebo 起動後の NEXTAGE OPEN ロボットの初期動作が終了しましたら，
Kinect と チェッカーボード のモデルを Gazebo 内に設置します．

```
$ roslaunch nextage_calibration gazebo_kinect_checkerboard_chest.launch   #chest
or
$ roslaunch nextage_calibration gazebo_kinect_checkerboard_waist.launch   #waist
```

Gazebo シミュレータ内の Kinect と胸部チェッカーボードモデルの設置 <br>
![Gazebo - Kinect and Checkerboard](https://cloud.githubusercontent.com/assets/3119480/23578847/50332e3c-0123-11e7-845d-ac0416a243af.png)

#### チェッカーボード検出プログラムの実行

実機ロボットの場合と同様に Xtion/Kinect カメラでのチェッカーボード検出プログラムを実行します．

Kinect の場合

```
$ roslaunch nextage_calibration camera_checkerboard_chest.launch launch_openni2:=false
or
$ roslaunch nextage_calibration camera_checkerboard_waist.launch launch_openni2:=false
```

![Checkerboard Detector - Gazebo / Checkerboard on Chest](https://cloud.githubusercontent.com/assets/3119480/23578864/8a614e90-0123-11e7-979c-5300efd3a33c.png)

#### RViz の起動

実機ロボットの場合と同様に各種チェックのために RViz を起動します．

```
$ rviz -d `rospack find nextage_calibration`/conf/calibration.rviz
```

![RViz - Checkerboard Gazebo Simulation](https://cloud.githubusercontent.com/assets/3119480/23578868/97f214ae-0123-11e7-8ce3-12cc53a6f1df.png)

#### カメラリンクと親リンクの TF の取得

実機ロボットの場合と同様に camera_link から親リンクへの TF を取得します．

```
$ rosrun tf tf_echo HEAD_JOINT1_Link camera_link
```

`tf_echo` の結果の例
```
At time 93.641
- Translation: [0.101, -0.014, 0.199]
- Rotation: in Quaternion [-0.001, 0.704, 0.002, 0.710]
            in RPY (radian) [0.126, 1.563, 0.130]
            in RPY (degree) [7.223, 89.561, 7.477]
```


<!-- EOF -->
