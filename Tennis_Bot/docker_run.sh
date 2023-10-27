docker run \
            -it \
            --rm \
            --name ros2 \
            --env="DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
	    --net=host \
            -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            -e ROS_DOMAIN_ID=2  \
            --entrypoint /bin/bash \
            -v $(pwd):/home/G03/workspace/ \
            -v /dev:/dev \
            --privileged \
            -w /home/G03/workspace \
            image_tag:latest
