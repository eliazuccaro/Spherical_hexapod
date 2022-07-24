##### PACKAGE DESCRIPTION #####

- "CMakeLists.txt", "package.xml" files and "include", "src" directories are automatically genetated by catkin_create_package command. They must not be modified, for the sake of ROS package integrity

# DIRECTORIES
- "config" contains .yaml files with parameter used in simulation
- "meshes" contains .stl files describing shellbot's mechanical parts
- "models" contains .urdf and .xacro files
- "rviz" contains rviz starting file to be used in simulation
- "worlds" contains Gazebo empty world
- "launch" contains .launch files
- "scripts" contains ROS nodes and libraries

# FILES
- "joints_notation" helps understanding the notation used for joints and motors
- "shellbot_rqt_graph.png" shows the rqt_graph of the ROS simulation, useful for understanding how nodes communicate