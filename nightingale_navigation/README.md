# Mapping
launch gmapping with <roslaunch nightingale_navigation mapping.launch>
drive the robot around and generate the map
the map is publsihed to /map. you can visualize it through rviz
save the map with <rosrun map_server map_saver -f 'map'>

# Localization
launch the map server and amcl with <roslaunch nightingale_navigation localization.launch>
the /particlecloud topic shows the estimated pose array. you can visualize it through rviz

# Navigation Version 0
launch nav with <roslaunch nightingale_navigation navigation.launch>
if using sim, specify the arg location:=sim
This launch file launces amcl, move base, and the room runner nodes
Room runner advertises a ros action which consumes a patient name and navigates to the patient's
door then bedside
Features: - cross boot pose persistence
          - room_runner action server which consumes patients names and navigates to door then bedside
tip: use axclient for a GUI based action client for testing purposes
