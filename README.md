# Game Engine
## Structure
The Game Engine has four interacting components: the Mediation Layer (ML),
Physics Simulator (PS), Visualizer (VZ), and Autonomy Protocol (AP). 

AP maps the quadcopter's state to an intended trajectory. ML mediates the
intended trajectories, altering them if necessary to impose boundaries,
simulate interaction with other objects, etc.  PS forward-simulates the
mediated trajectory, injecting disturbances and applying
proportional-derivative control to track the trajectory, and returns the
quadcopter's state at a subsequent time point. VZ pushes visualization data to
ROS's RVIZ program.

## Installation
### Dependencies
1. [Eigen](https://eigen.tuxfamily.org)
2. [ROS](http://www.ros.org)
3. [P4](https://gitlab.com/radionavlab/public/p4)

### Clone
```bash
cd ~/Workspace
git clone https://gitlab.com/todd.humphreys/game-engine-student.git game-engine
cd game-engine
git submodule update --init --recursive
```

<!--- [Todd doesn't think any of this is necessary] ### Building Dependencies

If you are cloning or using the repository for the first time, you will need
to build the dependencies (the submodules). They all use the same build
process, included below.

<p>
<details>

Note: If you want to make the dependencies available system-wide, follow the
`make` command with `sudo make install`.

#### Eigen

```bash
cd ~/Workspace/game-engine-student/src/dependencies/P4/dependencies/eigen
mkdir build 
cd build
cmake ..
make -j4
```

#### osqp

```bash
cd ~/Workspace/game-engine-student/src/dependencies/P4/dependencies/osqp
mkdir build 
cd build
cmake ..
make -j4
sudo make install
```

#### p4

```bash
cd ~/Workspace/game-engine-student/src/dependencies/P4/
mkdir build 
cd build
cmake ..
make -j4
```

#### mg-msgs

```bash
cd ~/Workspace/game-engine-student/src/dependencies/mg-msgs/
mkdir build 
cd build
cmake ..
make -j4
```

#### yaml-cpp

```bash
cd ~/Workspace/game-engine-student/src/dependencies/yaml-cpp/
mkdir build 
cd build
cmake ..
make -j4
```

</details>
</p>

### Build

Note: you may need to tell CMake where to find the OSQP dependency. 

<p>
<details>
To do this, open the cmake curses interface:

```bash
mkdir build # (if the build directory hasn't already been created)
cd build
cmake ..
```
Navigate to the `osqp_DIR` setting and change it to the location of the osqp binaries. It may be different depending where you cloned the repositories to, but for my virtual machine, it looks something like this:
```
/home/aerial-robotics/Workspace/game-engine-student/src/dependencies/P4/dependencies/osqp/build
```
</details>
</p> -->

### Build

```bash
cd ~/Workspace/game-engine
mkdir build # (do this only if the build directory hasn't already been created)
cd build
cmake ..
make -j4
```

### Configure for ROS
After the first time you build `game-engine`, you'll need to add a command
to your your `.zshrc` configuration file so that whenever you open a terminal,
the shell will run the necessary ROS setup scripts.

First open `.zshrc` in your favorite editor; e.g., for sublime:
```bash
subl ~/.zshrc
```
Then add this line to the bottom of the file:
```bash
source ~/Workspace/game-engine/build/devel/setup.zsh
```

## Running the Game Engine
The Game Engine is composed of several interacting executables. After
building, you must ensure that the following programs are running. You are
encouraged to use a terminal multiplexer like tmux and start each program in a
separate pane. See [here](tmux/README.md) for further information on tmux.

### ROS Core
```bash
roscore
```

### Load ROS params
The ROS parameters found in `game-engine/run/params.yaml` must be loaded after
`roscore` has been started or re-loaded if any of the parameters have changed
since loading.

First, modify the file paths in `/game-engine/run/params.yaml` (e.g.,
`map_file_path`) so that they are correct for your system.  Then load the
parameters into the `game_engine` namespace as follows:
```bash
cd game-engine/run
rosparam load params.yaml /game_engine/
```

### ROS Visualizer
The ROS visualizer (RVIZ) manages a 3D visualization environment in which the
arena, obstacles, balloons, and quadcopters are displayed.
```
cd game-engine/run
rosrun rviz rviz -d config.rviz
```

### Mediation Layer
The mediation layer mediates proposed trajectories, ensuring they won't
cause a quadcopter to crash.
```
cd game-engine/bin
./mediation_layer
```

### Physics Simulator
The physics simulator forward-integrates proposed quadcopter trajectories over
a short interval into the future and publishes the resulting state.  It
applies proportional-derivative control to track the trajectories.  It also
introduces disturbance forces, e.g., due to wind.
```
cd game-engine/bin
./physics_simulator
```

### Visualizer
The visualizer sends arena, obstacle, balloon, and quadcopter data
to RVIZ for display.
```
cd game-engine/bin
./visualizer
```
Note that it may take some time (a few 10s of seconds) for all the elements of
the arena to get populated into the RVIZ display.

### Autonomy Protocol
The autonomy protocol takes the current quadcopter state and publishes a
proposed trajectory for the quadcopter to follow.
```
cd game-engine/bin
./example_autonomy_protocol
```

### Tests
```bash
cd build/test
./EXECUTABLE_OF_CHOICE
```

## Contributing
### Software Patterns
Please read the [software patterns](doc/software-patterns.md) document to
understand the naming convention and the purpose of components in this system.

