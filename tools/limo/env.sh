# 팀원 .bashrc 와 같은 환경 (CUDA·TensorRT·dl_env 의 ultralytics) + 우리 워크스페이스
export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
source ~/AutoRace2026_sim2d/autorace_ws/install/setup.bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/lib/python3.10/dist-packages:$PYTHONPATH
export PYTHONPATH=/home/wego/miniconda3/envs/dl_env/lib/python3.10/site-packages:$PYTHONPATH
export TENSORRT_INSTALL=/usr/local/TensorRT-10.3.0.26
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$TENSORRT_INSTALL/lib
