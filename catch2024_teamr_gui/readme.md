# Ilias GUI
## About
Ilias 2024 R1操縦用GUI

仕様は変更になる可能性がある．



## Requirements
- rosbridge-suite
```bash
sudo apt install -y ros-humble-rosbridge-suite
```

- nodejs
```bash
sudo apt update
sudo apt install -y nodejs npm
sudo npm -g install n
sudo n stable
sudo apt purge nodejs npm
sudo apt autoremove
```

- This repo
```bash
cd ~/ros2_ws/src
git clone git@github.com:KeioRoboticsAssociation/ilias-gui --recursive
cd ./ilias_gui/ilias_gui_app
npm i
```

## Launch
```bash
ros2 launch ilias_gui ilias_gui_launch.py
```
