export containerId=$(docker ps -l -q)
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
docker start $containerId