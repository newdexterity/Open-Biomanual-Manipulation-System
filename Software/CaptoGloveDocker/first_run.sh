  GNU nano 4.8                              first_run_cont.sh                                        
#!/usr/bin/bash 

docker run -it --network host --privileged --gpus all \
       -e DISPLAY=$DISPLAY \
       -v /dev:/dev \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       --name captoglove_cont captoglove_img:latest


