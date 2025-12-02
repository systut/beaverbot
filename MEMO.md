1. Open Docker Desktop

2. Open Xlaunch

3. Start docker container
```
cd beaverbot_dockerfiles
docker compose -f .\docker-compose.windows.yml up -d 
```

4. Start gazebo 
First enter the docker container 
```
docker exec -it beaverbot bash
```
To start gazebo (must be after entering the docker container)
```
roslaunch beaverbot_launch bringup.launch
```

5. Run robot 
To run robot, open another terminal and enter docker container 
```
docker exec -it beaverbot bash
```
To start the robot (must be after entering the docker container)
```
roslaunch beaverbot_control beaverbot_control.launch
```
