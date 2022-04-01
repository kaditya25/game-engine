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

### Build Game Engine

```bash
cd ~/Workspace/game-engine
rm -rf build  # remove any prior build directory
mkdir build 
cd build
cmake ..
make -j
```


