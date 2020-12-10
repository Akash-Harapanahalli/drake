docker run -it \
	-v $(pwd):/app \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e DISPLAY=$DISPLAY \
	--gpus 'all,capabilities=graphics' \
	--privileged \
	drake:v2 bin/bash -c "echo \"hello\" && /bin/bash"

