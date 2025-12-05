# Rigid IPC
<b>Robust, intersection-free, simulations of rigid bodies.</b>

This is the implementation of the SIGGRAPH 2021 paper [Intersection-free Rigid Body Dynamics](https://ipc-sim.github.io/rigid-ipc/).

## Files

* `src/`: source code
* `cmake/` and `CMakeLists.txt`: CMake files
* `fixtures/`: input scripts to rerun all examples in our paper
* `meshes/`: input meshes used by the fixtures

## Build

To build the project, use the following commands from the root directory of the project:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DRIGID_IPC_WITH_UNIT_TESTS=OFF \
      -DRIGID_IPC_WITH_TOOLS=OFF \
      -DCMAKE_CXX_FLAGS="-Wno-array-bounds -Wno-stringop-overflow -Wno-maybe-uninitialized -Wno-noexcept -Wno-error" \
      ..
find _deps -name "CMakeLists.txt" -o -name "*.cmake" | xargs sed -i 's/-Werror//g'
find .. -name "CMakeLists.txt" -o -name "*.cmake" | grep -v "_deps" | xargs sed -i 's/-Werror//g'
make -j4
```

## Execution

To run the build, use the following commands from the root diretory of the project: 

```bash
./build/rigid_ipc_sim ./fixtures/<path_of_the_json>
//eg: ./rigid_ipc_sim ../fixtures/3D/fracture/cube.json
```

### Dependencies

**All dependancies are downloaded through CMake** depending on the build options.
The following libraries are used in this project:

* [IPC Toolkit](https://github.com/ipc-sim/ipc-toolkit): common IPC functions
* [Eigen](https://eigen.tuxfamily.org/): linear algebra
* [libigl](https://github.com/libigl/libigl): basic geometry functions, predicates, and viewer
* [Tight Inclusion CCD](https://github.com/Continuous-Collision-Detection/Tight-Inclusion): correct (conservative) continuous collision detection between triangle meshes in 3D
* [spdlog](https://github.com/gabime/spdlog): logging information
* [filib](https://github.com/txstc55/filib): interval arithmetic
* [Niels Lohmann's JSON](https://github.com/nlohmann/json): parsing input JSON scenes
* [tinygltf](https://github.com/syoyo/tinygltf.git): exporting simulation animation to GLTF format
* [finite-diff](https://github.com/zfergus/finite-diff): finite difference comparisons
    * Only used by the unit tests and when `RIGID_IPC_WITH_DERIVATIVE_CHECK=ON`

