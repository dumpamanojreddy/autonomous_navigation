Our project is autonomous navigation using turtlebot. To acheive this we implemented following:
1. Mapping
  A. Occupancy Grid Mapping
  B. Slam GMapping
2. Localization
3. Navigation

1. Mapping using occupancy grid mapping

  a) First run the launch file to show world:
  $ roslaunch navigate newtrack.launch

  b) Run the mapping code using following command:
  $ rosrun navigate mapping.py

  c) Visualize map using RVIZ by using topic /map
  $ rosrun rviz rviz

2. Mapping using slam gmapping

  a) First run the launch file to show world:
  $ roslaunch navigate newtrack.launch

  b) Run the mapping launch file using following command:
  $ roslaunch navigate gmapping.launch

  c) Run the controller code to move robot
  $ rosrun navigate mycontroller.py

  c) Visualize map using RVIZ by using topic /map
  $ rosrun rviz rviz

3. Localizing and Navigating

  a) First run the launch file to show world:
  $ roslaunch navigate newtrack.launch

  b) Deploy the localization launch file
  $ roslaunch navigate localize.launch

  c) Deploy the move base node launch file
  $ roslaunch navigate base.launch

  d) Visualize using RVIZ and open config file navigation.rviz located at navigate/share/navigation.rviz
  $ rosrun rviz rviz

  e) Run the move base code used to navigate robot in the map
  $ rosrun navigate movebase.py
