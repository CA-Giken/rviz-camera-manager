# Rviz カメラ操作・視点登録

# Installation

```
cd ~/catkin_ws/src
git clone https://github.com/CA-Giken/rviz_camera_manager.git
git clone -b noetic-devel https://github.com/HiraiKyo/rviz_animated_view_controller.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y

python3 -m pip install pysimplegui

catkin build
```

# Run

```
roscore
```

```
source ~/catkin_ws/devel/setup.bash
roslaunch rviz_camera_manager main.launch
```

# Import & Export config file

home ディレクトリに、`rviz_camera_manager.yaml`で設定を保存している。

この設定ファイルが無い場合には、`path/to/rviz_camera_manager/config/config.yaml`から初期値を読み込むようになっているので、初期値を変更したい場合にはここを変更する。

設定の保存は自動保存で行っており、保存タイミングは`Add`, `Edit`, `Remove`によって変更を行ったタイミングで自動保存を行う。

# 不具合メモ

## docker(VNC)環境だと、rviz の Views パネルの表示が更新されない

VNC 側の問題？ Rviz Camera Manager（当プログラム）からカメラ移動しても UI が変化しない。
パネルのトグルで表示・非表示を切り替えると反映されているので、クリックは反応しているが表示が更新されていない。
