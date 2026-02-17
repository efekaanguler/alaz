#step1
echo "Adding repo and apt update"

sudo add-apt-repository universe
sudo apt update

#step2
echo "Installing CARLA python library"

pip install carla==0.9.15
pip install evdev==1.6.1
pip install pynput

#step3
echo "Setting bridge repo up"

mkdir -p ~/Workspace/ros-bridge
cd ~/Workspace/ros-bridge
git clone --recurse-submodules https://github.com/ttgamage/carla-ros-bridge.git
mv carla-ros-bridge src

#step4
echo "Installing rosdeps"

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r

#step5
echo "Colcon build"

colcon build --symlink-install

#allah kurtarsın
sudo apt update
sudo apt install ros-humble-urg-node

#lidar clustering deps
pip install scikit-learn
pip uninstall numpy
pip install numpy==1.24.0

#done
echo "DONE!"