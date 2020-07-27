mkdir -p ~/envs
cd ~/envs
virtualenv -p python pbcollision --system-site-packages

source ~/envs/pbcollision/bin/activate
pip install numpy
pip install pybullet
pip install open3d
