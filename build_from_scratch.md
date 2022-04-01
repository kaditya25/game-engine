### Building Dependencies

If you are cloning or using the repository for the first time, you will need to
build the dependencies (the submodules). They all use the same build process,
included below.

#### Eigen

```bash
cd ~/Workspace/game-engine/src/dependencies/P4/dependencies/eigen
mkdir build 
cd build
cmake ..
make -j
sudo make install
```

#### osqp

```bash
cd ~/Workspace/game-engine/src/dependencies/P4/dependencies/osqp
mkdir build 
cd build
cmake ..
make -j
sudo make install
```

#### p4

```bash
cd ~/Workspace/game-engine/src/dependencies/P4/
mkdir build 
cd build
cmake ..
make -j
```

#### mg-msgs

```bash
cd ~/Workspace/game-engine/src/dependencies/mg-msgs/
mkdir build 
cd build
cmake ..
make -j
```

#### yaml-cpp

```bash
cd ~/Workspace/game-engine/src/dependencies/yaml-cpp/
mkdir build 
cd build
cmake ..
make -j
sudo make install
```

### Build

Note: you may need to tell CMake where to find the OSQP dependency. To do this,
open the cmake curses interface:

```bash
cd ~/Workspace/game-engine
rm -rf build  # remove any prior build directory
mkdir build 
cd build
cmake ..
```

Navigate to the `osqp_DIR` setting and change it to the location of the osqp
binaries:
```
/home/aeronaut/Workspace/game-engine/src/dependencies/P4/dependencies/osqp/build
```

