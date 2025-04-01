# Line Following Robot Challenge

This challenge involves running the Robotics Academy in a Docker container, uploading the provided Python script, and executing it to control a line-following robot using PID control.

## Prerequisites

Ensure you have the following installed:

- **Docker** ([Download from Docker](https://www.docker.com/))
- **NVIDIA Drivers** (Optional) if using GPU acceleration
- **Robotics Academy Docker Image**

## Steps to Run the Challenge

### 1. Pull the Robotics Academy Docker Image

Open a terminal (or PowerShell on Windows) and run:

```bash
docker pull jderobot/robotics-academy:latest
```

### 2. Run the Robotics Academy Container

Run the following command to start the Robotics Academy environment:

```bash
docker run --rm -it \
    -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7164:7164 \
    jderobot/robotics-academy:latest
```

If you have an NVIDIA GPU, enable GPU support with:

```bash
docker run --rm -it --gpus all \
    -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7164:7164 \
    jderobot/robotics-academy:latest
```

### 3. Access the GUI

Open a browser and go to:  
[http://localhost:7164](http://localhost:7164)

This opens a virtual desktop where you can interact with the challenge.

### 4. Upload Your Code

Navigate to the challenge folder inside the container.  
Upload your script (`line_following.py`) to the correct location.

You can use `scp` (Linux/Mac) or Docker volume mounting to transfer files.

To upload via Docker volume mounting, run:

```bash
docker run --rm -it -v /path/to/local/folder:/root/shared \
    -p 6080:6080 -p 1108:1108 -p 7163:7163 -p 7164:7164 \
    jderobot/robotics-academy:latest
```

This will mount your local folder to `/root/shared` inside the container.

### 5. Run the Line Following Robot Script

Inside the container's virtual desktop:

1. Open a terminal.
2. Navigate to the challenge directory:

     ```bash
     cd /root/RoboticsAcademy/exercises/follow_line
     ```

3. Run the script:

     ```bash
     python3 line_follower.py
     ```

### 6. Observe the Robot's Behavior

The robot will now try to follow the line using derivative control (KD). If needed, you can tweak the `Kd` value in the script for better performance.

### 7. Stop the Container

To stop the Robotics Academy container, press `Ctrl + C` in the terminal where you started it or run:

```bash
docker ps  # Find the container ID
docker stop <CONTAINER_ID>
```

## Troubleshooting

- **Cannot access GUI?**  
    Check if Docker is running and that port `6080` is not in use.

- **Robot not moving?**  
    Ensure the correct script is uploaded and executing without errors.

- **GPU acceleration not working?**  
    Make sure NVIDIA drivers and CUDA are installed.

## Additional Resources

- [Robotics Academy Documentation](https://jderobot.github.io/RoboticsAcademy/user_guide/)
- [Docker Guide](https://docs.docker.com/)  