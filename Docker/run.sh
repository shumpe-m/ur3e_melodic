docker run --gpus all --rm -it  \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
    -v $HOME/UR:/root/UR \
    ur-ubuntu18 bash 