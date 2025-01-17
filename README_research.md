# Additional Instructions for RNL Researchers and TAs working with Game Engine

## Clone Additional Directories
Before building Game Engine, clone additional research- and TA-related directories:
```bash
cd ~/Workspace/game-engine/src/autonomy_protocol
git clone git@gitlab.com:radionavlab/machine-games/research-autonomy-protocols.git
cd ~/Workspace/game-engine/src/autonomy_protocol/research-autonomy-protocols
git submodule update --init --recursive
cd ~/Workspace/game-engine/src/autonomy_protocol
git clone git@gitlab.com:radionavlab/machine-games/ta-autonomy-protocol.git
```

## Build with Research/TA Extensions
### Build
```bash
cd ~/Workspace/game-engine
mkdir build # (do this only if the build directory hasn't already been created)
cd build
cmake -DRESEARCH=ON ..
make -j
```
