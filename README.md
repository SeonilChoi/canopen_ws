# canopen_ws

1. Be active can port
```
sudo slcand -o -s8 -t hw -S 1000000 /dev/canable can0 && \
  sudo ip link set can0 up && \
  sudo ip link set can0 txqueuelen 1000
```

2. Connect can to container (bash format)
```
if ! ip link show can0 &>/dev/null; then
  exit 1
fi

CONTAINER_NAME="ros_foxy_canopen"
CONTAINER_ID=$(docker ps -qf "name=$CONTAINER_NAME")

if [ -z "$CONTAINER_ID" ]; then
  exit 1
fi

CONTAINER_PID=$(docker inspect -f '{{.State.Pid}}' $CONTAINER_ID)

sudo ip link set can0 netns $CONTAINER_PID

if [ $? -eq 0 ]; then
  docker exec $CONTAINER_NAME ip link set can0 up
  if [ $? -eq 0 ]; then
    docker exec $CONTAINER_NAME ip -details link show can0
  fi
fi
```
