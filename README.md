# Note de Sharaine

### First step dans le docker 

Debloquer les permissions pour la manette ps4 et enlever l'ancien rosconfr 
```bash
sudo chmod a+rw /dev/input/event* /dev/input/js*
sudo rm -rf /opt/rosconfr/
```

Installer slam toolbox et nav2
```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### Changer les perfs

Dans le fichier worlds/Piste_CoVAPSy_2025a.wbt il faut changer 

```
WorldInfo {
  basicTimeStep 8 # en ms je crois et plus c'est petit plus la simu est bien (par defaut 32)
  FPS 30 #mettre a 60 si les perf sont bonne
}
```

### Changer le controller des voitures 

Dans le fichier worlds/Piste_CoVAPSy_2025a.wbt (à la toute fin) il faut changer 

```
TT02_2025a {
  translation -1 -2 0.04
  name "blue_car"
  # controller "controller_blue" #controller webot
  controller "<extern>" #ros2
  color 0 0 0.9
}
```

### Comment lancer les controller ros2 de la voiture bleu

Lancer la simulation
```bash
ros2 launch rosconfr webots_rosconfr.launch.py
```

Téléoperation
```bash
ros2 run joy joy_node
ros2 run rosconfr ps4_teleop_ackermann
```

Algorithme réactif:
```bash
ros2 run rosconfr follow_gap_blue
ros2 run rosconfr dial_nav_blue
```

### Activer les voitures jaune et violette

Dans le fichier worlds/Piste_CoVAPSy_2025a.wbt (à la toute fin) il faut décommenté les voitures 

Et en plus pour la voiture jaune il faut décommenté yellow_webots_bridge et yellow_state_publisher dans webots_rosconfr.launch.py

### Cartographie

```bash
ros2 launch rosconfr webots_rosconfr.launch.py
ros2 run rosconfr front_scan_filter

ros2 run joy joy_node
ros2 run rosconfr ps4_teleop_ackermann

ros2 launch slam_toolbox online_async_launch.py  slam_params_file:=/home/turtle/ros2_ws/src/rosconfr/params/slam_toolbox_params.yaml
```

### Nav2

Il faut changer les path de BT dans nav2_params et le tracks.yaml dans race_params si vous ếtes pas dans /home/turtle/ros2_ws/src

```bash
ros2 launch rosconfr webots_rosconfr.launch.py
ros2 run rosconfr front_scan_filter

ros2 launch rosconfr blue_odom.launch.py

ros2 launch race_path_follower bringup_launch.py map:=/home/turtle/ros2_ws/src/race_path_follower/maps/Piste_2025a.yaml

ros2 run race_path_follower race_node --ros-args --params-file /home/turtle/ros2_ws/src/race_path_follower/params/race_params.yaml

```


