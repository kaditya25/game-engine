# Game Engine
## Project Structure
The Game Engine has three main components: the Mediation Layer (ML), the Physics
Simulator (PS), and the Autonomy Protocol (AP). These three programs interact
together to form the Game Engine (GE).

AP maps the quadcopter's state to an intended trajectory. ML mediates the
intended trajectories, altering them if necessary to impose boundaries,
simulate interaction with other objects, etc.  PS forward-simulates the
mediated trajectory, injecting disturbances, and returns the quadcopter's state
at a future time point.

## Installation
### Prerequisites 
1. [Eigen](https://eigen.tuxfamily.org)
2. [ROS](http://www.ros.org)
3. [P4](https://gitlab.com/radionavlab/public/p4)

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
cd game-engine/run
rosparam load params.yaml /mediation_layer/
```

### ROS Visualizer
The ROS visualizer manages a 3D environment that the simulation will provides
displays for.
```
cd game-engine/run
rosrun rviz rviz -d config.rviz
```

### Mediation Layer
The mediation layer mediates proposed trajectories and ensures that they won't
cause a quadcopter to crash.
```
cd game-engine/bin
./mediation_layer
```

### Physics Simulator
The physics simulator forward-integrates proposed quadcopter trajectories a
small period in the future and publishes the resulting state.
```
cd game-engine/bin
./physics_simulator
```

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

