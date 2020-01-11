on-site-tools
====================
Tools for on-site work.

Usage
-----------------------

### set_run_dialog
Publish /vehicle_cmd by specified value.

```
rosrun on-site-tool set_run_dialog.py
```

### rosbag_recorder
Rosbag frontend panel with useful functions.

```
rosrun on-site-tools rosbag_recoder.py
```


Notes for developer
-----------------------
UI is built by qt5designer. Please install by command below.

```
apt install qttools5-dev-tools
```

If you change .ui file, please run ./generate_ui.sh . Generated ui .py files are located in scripts/ui. Do not edit these files by hand.