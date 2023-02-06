# SoftDrone-Target-Pose-Estimator
Package for target pose estimating

*`config/` contains yaml files for tunable parameters.

*`include/` contains cpp files for plotting, keypoint detection, keypoint projection, and point cloud registration.

*`launch/` contains launch files for ros nodes.

*`msg/` contains custom keypoint message.

*`notebooks/` contains jupyter notebooks for keypoint training and pose estimation evalutation scripts.

*`src/` contains entry points for ros nodes (as well as a hacky python-keypoint detector wrapper)

*`tests/` has some data for unit tests.


## Requirements

* Tested in Ubuntu 18.04 with [ROS Melodic](http://wiki.ros.org/melodic).

* Requires [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) >10.1.
  > Note: Tested 11.5, 11.1, 10.2.

* Requires [cuDNN](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html).

* Requires libtorch:
  > **Note:** Libtorch **must** be compiled using -D_GLIBCXX_USE_CXX11_ABI=1 flag, otherwise Torch will fail to link with other libraries (ie. ROS) and compiling our package will fail in ambiguous ways.
  * Option A: Compile pytorch from source.
  * Option B: Use my precompiled binaries found here
  * Option C: Use pytorch's provided precompiled binaries [here](https://github.com/pytorch/pytorch/issues/17492#issuecomment-524692441) (**I couldn't get these to work!**)


* Requires torchvision:
  > **Note:** You must point torchvision to look for a version of libtorch that was compiled using -D_GLIBCXX_USE_CXX11_ABI=1 (see above).
  
  > **Note:** You must also upgrade cmake, but don't purge your old cmake or it will uninstall all of ROS!
  ```
  # Need to upgrade cmake
  pip install cmake
  # Refresh terminal
  source ~/.bashrc 
  export TORCH_CUDA_ARCH_LIST="Kepler;Kepler+Tesla;Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
  export CMAKE_PREFIX_PATH=your/path/libtorch
  git clone --branch v0.11.1 https://github.com/pytorch/vision/
  mkdir vision/build && cd vision/build && \
	cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_BUILD_TYPE=Release -DWITH_CUDA=on   -DTORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST && \
	make -j && make install
  ```

* Requires [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus)


## Building:
```
export Torch_DIR=your/path/libtorch
catkin build
```

## Running:
```
roslaunch softdrone_target_pose_estimator estimate_target_pose.launch
```
