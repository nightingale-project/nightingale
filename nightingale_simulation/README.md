# Simulation
Simulation of Movo in hospital can be spawned with <roslaunch nightingale_simulation simulation.launch>
you may append "2> >(grep -v TF_REPEATED_DATA buffer_core)" to the cmd to hide the annoying spam of warnings, but this has been seen to hide more errors
alternatively, see soln here https://answers.ros.org/question/381529/suppress-tf_repeated_data-warnings/
This brings up all movo core nodes (sensors, motion, etc) and moveit
This also brings up RVIZ
There exists a script under the nightingale_simulation folder for low-level random arm articulation as a baseline verification that arms work
