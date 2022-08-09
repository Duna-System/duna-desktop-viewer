# Duna-Viar Porject

- This project offers a 3D Point Clouds visualization and Laser-Camera extrinsics calibration tool

# Dependencies
PCL
duna-optimizer (https://github.com/Marcus-D-Forte/duna-optimizer.git)
Use `vcs import . < modules.repos`

# build
You can build duna project isolated from viar. Make sure you also have the dependencies installed. To download duna-optimizator, simply use
`` git submodule update --init ``.
Then, build using
`` mkdir build; cd build; cmake .. ``

