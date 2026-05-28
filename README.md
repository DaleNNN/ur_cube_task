# UR Cube Task

Automatisert system for deteksjon og lokalisering av fargekodede kuber med en UR10-robot.
Utviklet som en del av et robotikkprosjekt ved NTNU.

## Industriell kontekst

Systemet er utformet rundt en industriell problemstilling der en robot verifiserer at
riktige komponenter er plassert korrekt i en monteringsprosess. Fargekodede kuber
representerer ulike komponenttyper (rød, grønn, blå), og roboten peker dem ut i
rekkefølge. Hvis en komponent mangler, søker roboten aktivt etter den fra alternative
posisjoner. Hvis den fortsatt ikke finner den, stopper systemet og varsler operatøren.

## Forfattere

- Øystein Dale
- Borgar Kleiva

## Systemkrav

- Ubuntu 24.04
- ROS2 Jazzy
- UR10 CB3-robot med External Control URCap
- Webkamera montert på tool0

## Installasjon

    cd ~/ros2_ws/src
    git clone https://github.com/DaleNNN/ur_cube_task.git
    cd ~/ros2_ws
    colcon build --packages-select ur_cube_task
    source install/setup.bash

## Kjøring på fysisk robot

1. Start External Control-programmet på teach pendant og trykk Play
2. Kjør launch-filen:

    ros2 launch ur_cube_task task.launch.py

Tilgjengelige argumenter:

    ur_type            standard: ur10       Robottype
    robot_ip           standard: 143.25.150.94   IP-adresse til robotkontroller
    use_mock_hardware  standard: false       Bruk mock hardware

## Kjøring i simulering

    ros2 launch ur_cube_task sim.launch.py

Simulering starter mock hardware, MoveIt med RViz, fake kamera og scene med
bord og kuber. Nyttig for å teste bevegelseslogikk uten fysisk robot.

## Manuell kjøring

Flytt roboten til hjemposisjon:

    ros2 run ur_cube_task move_home

Flytt roboten til oversiktsposisjon:

    ros2 run ur_cube_task move_overview

Start kun kubedeteksjon:

    ros2 run usb_cam usb_cam_node_exe --ros-args \
      -p video_device:=/dev/video3 \
      -p image_width:=640 \
      -p image_height:=480

    ros2 run ur_cube_task cube_detector --ros-args \
      -r /camera/image_raw:=/image_raw

    ros2 topic echo /detected_cubes

## Kalibrering

### Kamerakalibrering

Kalibreringsfilen ligger i config/camera_calibration.yaml og lastes automatisk
av launch-filen. For å kalibrere på nytt:

    ros2 run camera_calibration cameracalibrator \
      --size 7x5 \
      --square 0.0355 \
      --no-service-check \
      --ros-args -r image:=/image_raw

### Pixel-til-robot mapping

Jog roboten til ønsket posisjon, plasser kubene og noter pikselkoordinater
fra /detected_cubes og TCP-koordinater fra /tcp_pose_broadcaster/pose.
Legg inn målepunktene i ur_cube_task/calibrate.py og kjør:

    python3 ur_cube_task/calibrate.py

Oppdater koeffisientene i task_manager.py.

## Systemarkitektur

    usb_cam  ->  /image_raw  ->  cube_detector  ->  /detected_cubes  ->  task_manager
                                                                               |
                                                                    move_to_pose_action
                                                                               |
                                                                          /move_action
                                                                               |
                                                                             MoveIt
                                                                               |
                                                                 scaled_joint_trajectory_controller

## Filstruktur

    ur_cube_task/
    ├── config/
    │   └── camera_calibration.yaml
    ├── launch/
    │   ├── task.launch.py
    │   └── sim.launch.py
    └── ur_cube_task/
        ├── motion.py              Baseklasse for leddvinkelbevegelse, HOME og OVERVIEW
        ├── move_home.py           Kjørbar node for hjemposisjon
        ├── move_overview.py       Kjørbar node for oversiktsposisjon
        ├── cube_detector.py       HSV-basert fargedeteksjon fra kamerabilde
        ├── move_to_pose_action.py MoveIt-basert kartesisk bevegelse
        ├── task_manager.py        Hovedlogikk, koordinerer deteksjon og bevegelse
        ├── calibrate.py           Kalibrering av pixel-til-robot mapping
        ├── fake_camera.py         Simulert kamera for testing
        └── scene_publisher.py     Publiserer bord og kuber i RViz for simulering
