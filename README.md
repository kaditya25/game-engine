# Game Engine
## Project Structure
The Game Engine has three main components: the Mediation Layer (ML), the Physics
Simulator (PS), and the Autonomy Protocol (AP). These three programs interact
together to form the Game Engine (GE).

The PS and the AP are co-dependent --- neither can accomplish its task without
the other. The AP maps the current quadcopters' states to an intended
trajectory. The PS forward-simulates the AP's intended trajectories, injecting
distrubances, and returns the quadcopter's state at a future time point.

The ML simply displays the data that the PS and the AP are passing back and
forth.

As it stands, the Mediation Layer is a vestige of a previous iteration of the
Game Engine. Originally, the ML was supposed to forward-simulate the intended
trajectories output by the AP and inject disturbances that would force the
trajectories away from other static and dynamic objects. Since the Machine Games
rules changes, this integration is no longer necessary.

## Installation
### Prerequisites 
1. [Eigen](https://eigen.tuxfamily.org)
2. [ROS](http://www.ros.org)
3. [P4](https://gitlab.com/radionavlab/machine-games/p4)

### Clone
```bash
git clone git@gitlab.com:radionavlab/machine-games/game-engine.git
cd game-engine
git submodule update --init --recursive
```

### Build
```bash
mkdir build 
cd build
cmake ..
make -j4
```

## Running the Mediation Layer
The ML is composed of a couple of executables. After building, you must ensure
that the following programs are running. It may be helpful to use a terminal
multiplexer like tmux or terminator and start each program in a separate pane.

### ROS Core
```bash
roscore
```

### Load ROS params
ROS params need only be loaded once. This must be run after roscore has been
started or re-run if any of the parameters have been changed
```bash
cd GameEngine/run
rosparam load params.yaml /mediation_layer/
```

### ROS Visualizer
The ROS visualizer manages a 3D environment that the simulation will provides
displays for.
```
cd GameEngine/run
rosrun rviz rviz -d config.rviz
```

### Mediation Layer
The mediation layer mediates proposed trajectories and ensures that they won't
cause a quadcopter to crash.
```
cd GameEngine/bin
./mediation_layer
```

### Physics Simulator
The physics simulator forward-integrates proposed quadcopter trajectories a
small period in the future and publishes the resulting state.
```
cd GameEngine/bin
./physics_simulator
```

### Autonomy Protocol
The autonomy protocol takes the current quadcopter state and publishes a
proposed trajectory for the quadcopter to follow.
```
cd GameEngine/bin
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

