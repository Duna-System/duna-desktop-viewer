# Duna-Viar Porject

- This project goals visualize 3D Point Clouds and Camera Images for laser-camera extrinsics calibration.

# Dependencies
PCL
duna-optimizator (https://github.com/Marcus-D-Forte/duna-optimizator.git)

# build
You can build duna project isolated from viar. Make sure you also have the dependencies installed. To download duna-optimizator, simply use
`` git submodule update --init ``.
Then, build using
`` mkdir build; cd build; cmake .. ``

