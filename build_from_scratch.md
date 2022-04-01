### Building Dependencies

If you are cloning or using the repository for the first time, you will need to
build the dependencies (the submodules). They all use the same build process,
included below.

<p>
<details>

Note: If you want to make the dependencies available system-wide, follow the
`make` command with `sudo make install`.

#### Eigen

```bash
cd ~/Workspace/game-engine/src/dependencies/P4/dependencies/eigen
mkdir build 
cd build
cmake ..
make -j4
```

#### osqp

```bash
cd ~/Workspace/game-engine/src/dependencies/P4/dependencies/osqp
mkdir build 
cd build
cmake ..
make -j4
sudo make install
```

#### p4

```bash
cd ~/Workspace/game-engine/src/dependencies/P4/
mkdir build 
cd build
cmake ..
make -j4
```

#### mg-msgs

```bash
cd ~/Workspace/game-engine/src/dependencies/mg-msgs/
mkdir build 
cd build
cmake ..
make -j4
```

#### yaml-cpp

```bash
cd ~/Workspace/game-engine/src/dependencies/yaml-cpp/
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
/home/aerial-robotics/Workspace/game-engine/src/dependencies/P4/dependencies/osqp/build
```
</details>
</p>
