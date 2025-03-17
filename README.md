<h1 style="font-size: 3em;">16662: Pixel Art, by team CopyPasta</h1>

Updated instructions 16.03.25. Author: Sreeharsha

1. **Clone the Repositories:**
    ```bash
    git clone https://github.com/sreeharshaparuchur1/pixel-art-franka.git
    cd pixel-art-franka
    ```
2. **Save the current user id into a file:**
    ```bash
    echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
    ```
    It is needed to mount the folder from inside the Docker container.
3. **Use the docker compose file and the dockerfile to build the image**
image name: franka_ros2-pixel-art

container name: pixel-art-franka

4. **Enter the container**
    ```bash
    docker exec -it pixel-art-franka bash
    ```


# About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

For convenience, we provide Dockerfile and docker-compose.yml files. While it is possible to build **franka_ros2** directly on your local machine, this approach requires manual installation of certain dependencies, while many others will be automatically installed by the **ROS 2** build system (e.g., via **rosdep**). This can result in a large number of libraries being installed on your system, potentially causing conflicts. Using Docker encapsulates these dependencies within the container, minimizing such risks. Docker also ensures a consistent and reproducible build environment across systems. For these reasons, we recommend using Docker.

# Docker Container Installation
The **franka_ros2** package includes a `Dockerfile` and a `docker-compose.yml`, which allows you to use `franka_ros2` packages without manually installing **ROS 2**.

1. **Clone the Repositories:**
    ```bash
    git clone https://github.com/sreeharshaparuchur1/pixel-art-franka.git
    cd pixel-art-franka
    ```
2. **Save the current user id into a file:**
    ```bash
    echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
    ```
    It is needed to mount the folder from inside the Docker container.

3. **Build the container:**
    ```bash
    docker compose build
    ```
4. **Run the container:**
    ```bash
    docker compose up -d
    ```
5. **Open a shell inside the container:**
    ```bash
    docker exec -it franka_ros2 /bin/bash
    ```
6. **Clone the latests dependencies:**
    ```bash
    vcs import src < src/franka.repos --recursive --skip-existing
    ```
7. **Build the workspace:**
    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
7. **Source the built workspace:**
    ```bash
    source install/setup.bash
    ```
8.  **Test the build**
    ```bash
    colcon test
    ```
    >NOTE: Warnings can be expected. 
9. **When you are done, you can exit the shell and shut down the container**:
    ```bash
    docker compose stop
    ```
    or exit the shell and delete the container
    ```bash
    docker compose down -t 0
    ```

 

# Run a sample ROS2 application

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```


# Troubleshooting
#### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).