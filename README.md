## Lidar odometry under development
This repository is for lidar odometry development.

## Install
You have to run the code in Ubuntu and need docker to setup the environment.
1. Clone the repo
  ```
  # on terminal A in your host,
  git clone https://github.com/YuheiSugano4/24785-project.git
  ```

2. Set your UID and GID in /lo_dev/docker/setup.env. You can check your uid and gid in Ubuntu bu using the following command:
  ```
  # on terminal A in your host,
  id -u  # Get User ID (UID)
  id -g  # Get Group ID (GID)
  ```

3. Run the following commands to build a docker image and start a container:
  ```
  # on terminal A in your host,
  cd /lo_dev/docker
  chmod +x run_docker.sh
  ./run_docker.sh --build
  ```
  After the command, you are in a docker container where all the necessary packages are installed.

4. Create a ros2 workspace directory and clone the repo again in your container:
  ```
  # on terminal A in your container,
  cd /sandbox/{YOUR_USER_NAME}                # put your username in {YOUR_USER_NAME}
  mkdir ros2_ws
  cd ros2_ws
  mkdir src
  cd src
  git clone https://github.com/YuheiSugano4/24785-project.git
  ```

5. Build the ros2 package and launch the package in your container:
  ```
  # on terminal A in your container,
  cd /sandbox/{YOUR_USERNAME}/ros2_ws      # put your username in {YOUR_USERNAME}
  source /opt/ros/humble/setup.bash         # to source the necessary ros2 package
  colcon build                              # to build the ros2 package
  source install/setup.bash                 # to source the package built
  ros2 launch lo_dev launch.py              # launch the pacakge
  ```
  Then, a Rviz2 window will pop up. Now that the ros node is waiting for the sensor data published.

6. Play a ros2 bag in another terminal:
  Put a ros2 bag file in the followin directory in your host
  ```
  /home/{YOUR_USERNAME}/data/  # Put the ros2 bag file name in data folder.
  ```

  In your host environment, open a new terminal. Then, command the following:
  ```
  # on terminal B in your host,
  cd /lo_dev/docker                         # to go to the repo in your host
  ./run_docker.sh                           # to go into the container that is already running via the above process
  ```
  Now that you are in the same docker container but connecting via another terminal.
  In the terminal, command the followings:
  ```
  # on terminal B in your container,
  cd /sandbox/{YOUR_USERNAME}/ros2_ws      # to go to the ros2 workspace
  source /opt/ros/humble/setup.bash         # to source the necessary ros2 package
  ros2 bag play ../data/{ROS2_BAG_DATASET}  # to play ros2 bag data(recorded measurement sensor data). Put the ros2 bag file name in {ROS2_BAG_DATASET}.
  ```
  Then, the sensor data starts being published and the ros2 node starts working.

## Folder Structure in your host environment
  ```
24785-project/
  │── docker/
  │   │── docker-compose.yml
  │   │── Dockerfile
  │   │── run_docker.sh
  │   │── setup.env
  │── sandbox/
  ```
  
## Folder Structure in your docker container
  ```
  sandbox/
  │── YOUR_USERNAME/
     │── data/
         │── YOUR_ROSBAG_FILE/ 
     │── ros2_ws/
         │── build/ 
         │── install/ 
         │── log/ 
         │── src/ 
              |── 24785-project/
                   │── cmake/
                   │── config/
                   │── docker/
                   │── include/
                   │── launch/
                   │── src/       # This is the source code. You should modify the code in the docker container.
                   │── CMakeLists.txt
                   │── package.xml
                   │  
  ```

## Code Update
Please push the script to the git from the container. Don't push the code from the host, which could cause a conflict issue.