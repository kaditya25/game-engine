# Manual Control Protocol Instructions
## Installing the ROS Joy package
Go to (http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) and follow
the given instructions. The instructions are for ROS indigo; replace instances
of "indigo" with "noetic". You will need the XBox controller to be plugged into
an open USB port on your computer in order to perform the testing steps in
the tutorial.

## Adding joy-dependency folder to dependencies
Open the src/dependencies/joy-dependency folder within game engine. This folder
should have files for joyMapper, joyDrivers, joyStructs alongside a cMake file
and readme. You should not have to make any alterations to this, but just know
that this is where the ability to read joy messages comes from.

## Running the manual protocol in simulation
1. Launch game engine as you normally would; follow the steps in the game engine
readme file.
2. Once rviz has been launched and the quad/map are visible, you will now run
the command to read in the joy inputs from the controller, given by:
```bash
rosrun joy joy_node _autorepeat_rate:=20
```
The autorepeat rate defines how quickly the controller joystick processes inputs.

3. Next, run the manual autonomy protocol as
```bash
./manual_autonomy_protocol
```
Moving the joysticks should now move the simulated quad. The yaw of the quad does
change in simulation, but the visualizer is unable to display any changes in yaw. 

## Running the manual protocol on the physical quads
1. Launch the shack mission control
2. Load the params file using
```bash
rosparam load params.yaml /game_engine/
```
Ensure that the "params" name is replaced with the name of the parameter file
you would like to load. Also ensure within the parameters file that the name of 
the quad is the same as the name of the physical quad you would like to fly.
Also ensure that the start point set within the params file is away from the 
edges of the map. Loading the "arena empty squeezed" map will work best for
the physical quads. Set joy_mode to true, set the quad safety limits to 2.

3. Get the quad in the air: take off in yaw mode and switch to position mode.
Ensure you are somewhere away from the boundaries of the arena.

4. Open a new tab and run the mediation layer binary:
```bash
cd bin
./mediation_layer
```
It may produce a couple trajectory rejection messages initially; this is normal.

5. Open a new tab and run the manual control protocol binary:
```bash
./manual_control_protocol
```
SWITCH THE QUAD INTO ROS MODE BY PRESSING B. You should now be able to fly
the quad in a safety mode where flying the quad too close to the net will
cause the quad to freeze. (To unfreeze yourself, switch to position mode and 
fly back towards the middle of the arena, then switch to the mediation layer
tab and hit Ctrl+C to stop it. Then, start it again and switch back into ROS
mode)

## Important values and information
1. Trajectory refresh rate is 10 Hz
2. Safety bounds are set to 1.25 m in quad state watchdog
3. Maximum velocity and acceleration should be set to 3.0
4. The arena_ empty _squeezed.map file is the best map file
   to load for flying the physical quads.
5. A good starting point for the quad in the params file is (-20,16,-3)
6. Ensure that "joy_mode: true" is set in the params file
7. Ensure that "quad_safety_limits: 2" in the params file; extreme mode works best

