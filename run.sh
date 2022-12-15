docker run --gpus all --rm -it  \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
#    -v $HOME/ur3e_melodic:/root/ur3e_melodic \
    ur3e-melodic bash
