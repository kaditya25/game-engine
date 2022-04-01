#!/bin/bash
# kill roscore and layers if already running
localdir=$(pwd)
echo "All logs saved in ${localdir}"
echo "Checking if roscore is running..."
while killall roscore; do
	sleep 1
done

# get game-engine working directory
filepath=
while true ; do
    read -e -r -p "Enter path to game engine (i.e. /home/user/Workspace/game-engine): " filepath
    if [ -d "$filepath" ] ; then
        break
    fi
    echo "$filepath is not a directory..."
done
cd $filepath

# launch roscroe in background
cd $localdir
echo "launching Roscore"
roscore > roscore.log 2>&1 &
sleep 2
echo "Success. Terminal output in roscore.log..."
# load roscore params
cd $filepath/run
param=
while true ; do
    read -e -r -p "ROS param file to use in /run (i.e. params.yaml): " param   
    if [ -f "$param" ] ; then
        break
    fi
    echo "$param is not in ./run"
done
rosparam load $param /game_engine/

# load visualizer
cd $filepath/run
while true; do
    read -p "Start visualizer?[y/n] " yn
    case $yn in
        [Yy]* ) rosrun rviz rviz -d config.rviz > $localdir/rviz.log 2>&1 &
		echo "Success. Terminal output in rviz.log..."
		sleep 2
		break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done
# Mediation Layer
cd $filepath/bin
./mediation_layer > $localdir/med_layer.log 2>&1 &
echo "Mediation Layer started. Terminal output in med_layer.log.."
# Physics Symulator
cd $filepath/bin
./physics_simulator &
echo "Physics sim started..."
# pass to visualizer
cd $filepath/bin
case $yn in
        [Yy]* ) ./visualizer &
		read -p "Press enter to continue once the params are laoded in visualizer..";;
esac
# autonomy protocol
cd $filepath/bin
prot=
prot_par1=
prot_par2=
while true ; do
    read -e -r -p "Autonomy Protocol file to use in /bin (no params i.e. example_autonomy_protocol): " prot
    FILE=$filepath/bin/$prot
    if [ -f "$FILE" ] ; then
	while true ; do
        	read -e -r -p "Parameters to pass (leave empty for no params): " prot_par1 prot_par2
		echo "$filepath/bin/$prot $prot_par1 $prot_par2"
		xterm -hold -e "$filepath/bin/$prot $prot_par1 $prot_par2"
		read -p "Press enter to restart Med layer, Phys layer and run new protocol.."
		while killall $prot; do
        	sleep 1
		done
		while killall physics_simulator; do
                sleep 1
                done
		while killall mediation_layer; do
                sleep 1
                done
		# Mediation Layer
		cd $filepath/bin
		./mediation_layer > $localdir/med_layer.log 2>&1 &
		echo "Mediation Layer re-started. Terminal output in med_layer.log.."
		# Physics Symulator
		cd $filepath/bin
		./physics_simulator &
		echo "Physics sim re-started..."
		# pass to visualizer
		cd $filepath/bin
		case $yn in
        		[Yy]* ) ./visualizer &
                	read -p "Press enter to continue once the params are loaded in visualizer..";;
		esac
		break;
	done
    else
	echo "${prot} not in /bin"    
    fi
done




