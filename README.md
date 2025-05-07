# var_n7k_parkbot

## English

**var_n7k_parkbot** is a ROS 2 (Humble) Python package for autonomous parking logic simulation with TurtleBot3 in Gazebo, including LIDAR-based parking spot detection and visualization in RViz.

### Quick Start

1. **Clone the package**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/samuvarga/var_n7k_parkbot
    ```

2. **Build the package**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select var_n7k_parkbot --symlink-install
    ```

3. **Source the workspace**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

4. **Launch the simulation (Gazebo + RViz + bridges)**
    ```bash
    ros2 launch var_n7k_parkbot gazebo_with_robot.launch.py
    ```

5. **In a new terminal, source the workspace again and start the parking logic node:**
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run var_n7k_parkbot parking_logic_node
    ```

- The launch file starts Gazebo with the parking world, bridges topics, and opens RViz with the correct config.
- The parking logic node must be started manually in a separate terminal.
- LIDAR point clouds and detected parking spots are visualized in RViz.

---

## Graph / Mermaid diagram

```mermaid
flowchart TD
    N1([/parking_logic_node/]):::red --> T1[/model/turtlebot3/cmd_vel/]:::light
    N1 --> T2[/model/turtlebot3/odometry/]:::light
    N1 --> T3[/model/turtlebot3/scan/points/]:::light
    N1 --> T4[/model/turtlebot3/parking_markers/]:::light

    T1 --> N2[/cmd_vel_bridge/]:::red
    T2 --> N3[/odom_bridge/]:::red
    T3 --> N4[/points_bridge/]:::red

    N1 --> T4
    N2 --> T1
    T1 --> N2

    N3 --> T2
    T2 --> N3

    N4 --> T3
    T3 --> N4

    T5[/parameter_events/]:::light --> N1
    T5 --> N2
    T5 --> N3
    T5 --> N4
    T5 --> N5[/rviz2/]:::red
    T5 --> N7[/robot_state_publisher/]:::red
    T5 --> N10[/ros2cli_daemon_.../]:::red

    N7 --> T6[/tf/]:::light
    N7 --> T7[/robot_description/]:::light
    T9[/joint_states/]:::light --> N7
    T8[/clock/]:::light --> N7

    N5 --> T6
    T6 --> N8[/transform_listener_impl_.../]:::red
    N5 --> T11[/initialpose/]:::light
    N5 --> T12[/goal_pose/]:::light
    N5 --> T13[/clicked_point/]:::light
    N9[/rqt_gui_py_node_39022/]:::red --> T5
    N10 --> T5

    N5 --> N6[/rosout/]:::red
    N1 --> N6
    N2 --> N6
    N3 --> N6
    N4 --> N6
    N7 --> N6

    classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742;
    classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff;
```

---

## Screenshots

Here you will find images of the running project, RViz, and the parking logic results.

![Simulation world](img/world.png)
*Simulation world*

![LIDAR data in RViz](img/lidar.png)
*LIDAR data in RViz*

![Detected clusters](img/cluster.png)
*Detected clusters*


---

## Magyar

A **var_n7k_parkbot** egy ROS 2 (Humble) Python csomag, amely TurtleBot3-hoz készült parkolási logikát, LIDAR-alapú parkolóhely-felismerést és RViz vizualizációt tartalmaz Gazebo szimulációban.

### Gyors indítás

1. **Csomag klónozása**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/samuvarga/var_n7k_parkbot
    ```

2. **Fordítás**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select var_n7k_parkbot --symlink-install
    ```

3. **Workspace forrásolása**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

4. **Szimuláció indítása (Gazebo + RViz + bridge-ek)**
    ```bash
    ros2 launch var_n7k_parkbot gazebo_with_robot.launch.py
    ```

5. **Új terminálban workspace forrásolása és a parkolási logika node indítása:**
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run var_n7k_parkbot parking_logic_node
    ```

- A launch file elindítja a Gazebo szimulációt a parkoló világgal, beállítja a bridge-eket, és megnyitja az RViz-t a megfelelő konfigurációval.
- A parkolási logika node-ot külön terminálban kell elindítani.
- Az RViz-ben látható a LIDAR pontfelhő és a detektált parkolóhelyek.

---

## Graph / Mermaid diagram

```mermaid
flowchart TD
    N1([/parking_logic_node/]):::red --> T1[/model/turtlebot3/cmd_vel/]:::light
    N1 --> T2[/model/turtlebot3/odometry/]:::light
    N1 --> T3[/model/turtlebot3/scan/points/]:::light
    N1 --> T4[/model/turtlebot3/parking_markers/]:::light

    T1 --> N2[/cmd_vel_bridge/]:::red
    T2 --> N3[/odom_bridge/]:::red
    T3 --> N4[/points_bridge/]:::red

    N1 --> T4
    N2 --> T1
    T1 --> N2

    N3 --> T2
    T2 --> N3

    N4 --> T3
    T3 --> N4

    T5[/parameter_events/]:::light --> N1
    T5 --> N2
    T5 --> N3
    T5 --> N4
    T5 --> N5[/rviz2/]:::red
    T5 --> N7[/robot_state_publisher/]:::red
    T5 --> N10[/ros2cli_daemon_.../]:::red

    N7 --> T6[/tf/]:::light
    N7 --> T7[/robot_description/]:::light
    T9[/joint_states/]:::light --> N7
    T8[/clock/]:::light --> N7

    N5 --> T6
    T6 --> N8[/transform_listener_impl_.../]:::red
    N5 --> T11[/initialpose/]:::light
    N5 --> T12[/goal_pose/]:::light
    N5 --> T13[/clicked_point/]:::light
    N9[/rqt_gui_py_node_39022/]:::red --> T5
    N10 --> T5

    N5 --> N6[/rosout/]:::red
    N1 --> N6
    N2 --> N6
    N3 --> N6
    N4 --> N6
    N7 --> N6

    classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742;
    classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff;
```

---

## Képek / Screenshots

Itt láthatók a projekt futásáról, az RViz-ről és a parkolási logika eredményeiről készült képek.

![Szimulációs világ](img/world.png)
*Szimulációs világ*

![LIDAR adatok az RViz-ben](img/lidar.png)
*LIDAR adatok az RViz-ben*

![Detektált klaszterek](img/cluster.png)
*Detektált klaszterek*


---

**Fejlesztői információk, részletes leírás és konfigurációk a forráskódban találhatók.**