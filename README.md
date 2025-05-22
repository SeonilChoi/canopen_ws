# canopen_ws

1. Be active can port
```
sudo slcand -o -s8 -t hw -S 1000000 /dev/canable can0 && sudo ip link set can0 up && sudo ip link set can0 txqueuelen 1000
```

2. Connect can to container
```
# can0 인터페이스가 존재하는지 확인
if ! ip link show can0 &>/dev/null; then
  echo "오류: can0 인터페이스가 존재하지 않습니다."
  echo "slcand를 사용하여 먼저 can0 인터페이스를 활성화하세요."
  exit 1
fi

# obserbot 컨테이너가 실행 중인지 확인
CONTAINER_NAME="ros_foxy_canopen"
CONTAINER_ID=$(docker ps -qf "name=$CONTAINER_NAME")

if [ -z "$CONTAINER_ID" ]; then
  echo "오류: $CONTAINER_NAME 컨테이너가 실행 중이지 않습니다."
  exit 1
fi

# 컨테이너 PID 가져오기
CONTAINER_PID=$(docker inspect -f '{{.State.Pid}}' $CONTAINER_ID)
echo "컨테이너 PID: $CONTAINER_PID"

# can0 인터페이스를 컨테이너 네임스페이스로 이동
echo "can0 인터페이스를 $CONTAINER_NAME 컨테이너로 이동 중..."
sudo ip link set can0 netns $CONTAINER_PID

if [ $? -eq 0 ]; then
  echo "성공: can0 인터페이스가 $CONTAINER_NAME 컨테이너로 이동되었습니다."
  echo "주의: 이제 호스트에서는 can0 인터페이스에 접근할 수 없습니다."

  # 컨테이너 내에서 can0 인터페이스 활성화
  echo "컨테이너 내에서 can0 인터페이스 활성화 중..."
  docker exec $CONTAINER_NAME ip link set can0 up

  if [ $? -eq 0 ]; then
    echo "성공: 컨테이너 내에서 can0 인터페이스가 활성화되었습니다."

    # 인터페이스 상태 확인
    echo "컨테이너 내 can0 인터페이스 상태:"
    docker exec $CONTAINER_NAME ip -details link show can0
  else
    echo "오류: 컨테이너 내에서 can0 인터페이스를 활성화하지 못했습니다."
  fi
else
  echo "오류: can0 인터페이스를 컨테이너로 이동하지 못했습니다."
fi
```
