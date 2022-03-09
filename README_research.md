# Additional Instructions for RNL Researchers working with Game Engine

## Clone Additional Directories
Before building Game Engine, clone additional research-related directories:
```bash
cd ~/Workspace/game-engine/src/autonomy_protocol
git clone git@gitlab.com:radionavlab/machine-games/research-autonomy-protocols.git
```

## Build with Research Extensions
### Build
```bash
cd ~/Workspace/game-engine
mkdir build # (do this only if the build directory hasn't already been created)
cd build
cmake -DRESEARCH=ON ..
make -j
```