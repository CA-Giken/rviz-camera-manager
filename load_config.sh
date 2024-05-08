#!/bin/bash

# 特定のファイルのパス
specific_file="$HOME/rviz_camera_manager.yaml"

# デフォルトのファイルのパス
default_file="$(rospack find rviz_camera_manager)/config/config.yaml"

# ファイルの存在を確認
if [ -f "$specific_file" ]; then
    echo "Loading parameters from $specific_file"
    rosparam load "$specific_file"
else
    echo "Loading default parameters from $default_file"
    rosparam load "$default_file"
fi