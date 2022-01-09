#! /bin/sh

#open a new terminal window in the run directory and start roscore
gnome-terminal --tab --working-directory=/home/aeronaut/Workspace/game-engine/run -- roscore
sleep 7

#open a new terminal tab and load in the params
gnome-terminal --tab --working-directory=/home/aeronaut/Workspace/game-engine/run -- rosparam load paramsTwoTeams.yaml /game_engine/

#in the same terminal start rviz and wait for it to boot up
gnome-terminal --tab --working-directory=/home/aeronaut/Workspace/game-engine/run -- rosrun rviz rviz -d config.rviz
sleep 3

#create new tabs in the bin directory and start the necessay binaries to execute 
#an autonomy protocol
gnome-terminal --tab --working-directory=/home/aeronaut/Workspace/game-engine/bin -- ./mediation_layer

gnome-terminal --tab --working-directory=/home/aeronaut/Workspace/game-engine/bin -- ./physics_simulator

gnome-terminal --tab --working-directory=/home/aeronaut/Workspace/game-engine/bin -- ./visualizer

#gnome-terminal --tab --working-directory=/home/aeronaut/Workspace/game-engine/bin -- rosrun joy joy_node _autorepeat_rate:=20

