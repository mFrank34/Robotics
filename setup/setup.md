# ROS2 Setup Guide on Docker
A guide for using ROS2 through a Docker container with Dev Containers in PyCharm.

## Requirements
* Docker Desktop / Docker Engine
* PyCharm Professional
* Dev Containers Plugin (install from PyCharm Plugins)

## Project Layout
```
your-project/
├── .devcontainer/
│   ├── devcontainer.json
│   └── setup-wayland.sh (optional, for Linux/Wayland)
└── ros2_ws/
    └── src/
```

## Initial Setup

### 1. Install PyCharm Dev Containers Plugin
1. Open PyCharm
2. **File → Settings → Plugins**
3. Search for "**Dev Containers**"
4. Click **Install** and restart PyCharm

### 2. Create Dev Container Configuration

Create `.devcontainer/devcontainer.json` in your project root:
```json
{
  "name": "ROS2 Humble Desktop Full",
  "image": "osrf/ros:humble-desktop-full",
  "remoteUser": "root",
  
  "runArgs": [
    "--network=host",
    "-e", "DISPLAY=${env:DISPLAY}",
    "-e", "WAYLAND_DISPLAY=${env:WAYLAND_DISPLAY}",
    "-e", "XDG_RUNTIME_DIR=${env:XDG_RUNTIME_DIR}",
    "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
    "-v", "${env:XDG_RUNTIME_DIR}/${env:WAYLAND_DISPLAY}:${env:XDG_RUNTIME_DIR}/${env:WAYLAND_DISPLAY}:rw",
    "--cap-add=SYS_PTRACE",
    "--security-opt=seccomp=unconfined"
  ],
  
  "customizations": {
    "jetbrains": {
      "backend": "PyCharm"
    }
  },
  
  "remoteEnv": {
    "PYTHONPATH": "/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages",
    "ROS_DISTRO": "humble"
  },
  
  "postCreateCommand": "git config --global --add safe.directory /workspace && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc",
  
  "workspaceFolder": "/workspace"
}
```

### 3. Enable GUI Support

#### Linux (Wayland/X11)

Create `.devcontainer/setup-wayland.sh`:
```bash
#!/bin/bash

# Allow Docker to connect to Wayland
if [ -n "$XDG_RUNTIME_DIR" ]; then
    if [ -S "$XDG_RUNTIME_DIR/wayland-0" ]; then
        chmod 777 "$XDG_RUNTIME_DIR/wayland-0" 2>/dev/null || true
    fi
    xhost +local:docker 2>/dev/null || true
fi

echo "Wayland/X11 access configured for Docker"
```

Make it executable:
```bash
chmod +x .devcontainer/setup-wayland.sh
```

Add to your `~/.bashrc`:
```bash
# Docker GUI access
xhost +local:docker 2>/dev/null || true
```

#### macOS

1. Install XQuartz:
```bash
brew install --cask xquartz
```

2. Launch XQuartz → **Preferences → Security**
   - ✅ Enable "Allow connections from network clients"

3. Restart XQuartz

4. In terminal:
```bash
export DISPLAY_IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost + $DISPLAY_IP
export DISPLAY=$DISPLAY_IP:0
```

5. Update `devcontainer.json` DISPLAY line:
```json
"-e", "DISPLAY=host.docker.internal:0",
```

#### Windows

1. Install VcXsrv from: https://sourceforge.net/projects/vcxsrv/

2. Launch VcXsrv:
   - Display number: **0**
   - ✅ **Disable access control**

3. Update `devcontainer.json` DISPLAY line:
```json
"-e", "DISPLAY=host.docker.internal:0",
```

## Usage

### Connect to Dev Container

1. Open your project in PyCharm
2. Look at **bottom-right corner** for the Dev Container widget
3. Click → **Create Dev Container and Mount Sources**
4. Wait for container to build (first time: ~5-10 minutes)

### Configure Python Interpreter

PyCharm should auto-detect the interpreter. To verify:

1. **Settings → Project → Python Interpreter**
2. Should show: **Python 3.10 (Docker: osrf/ros...)**
3. Path: `/usr/bin/python3`

If not detected:
- Click gear icon → **Add Interpreter → On Docker**
- Select your container
- Python path: `/usr/bin/python3`

### Enable Code Inspections

1. **Settings → Editor → Inspections → Python**
2. Enable inspections you want:
   - ✅ PEP 8 coding style violation
   - ✅ Type checker
   - ✅ Shadowing names from outer scopes

PyCharm's built-in inspections work great - no external linters needed!

## Verify Setup

### Test ROS2 Commands

In PyCharm terminal:
```bash
# Should auto-source ROS2
ros2 topic list

# Check Python can import ROS2
python3 -c "import rclpy; print('ROS2 works!')"
```

### Test GUI Applications
```bash
# Test turtlesim
ros2 run turtlesim turtlesim_node

# Test RViz
rviz2

# Test Gazebo
gazebo
```

If GUI windows appear, you're all set! ✅

### Test Git
```bash
cd /workspace
git status  # Should work without safe.directory error
```

## Working with ROS2

### Create a Package
```bash
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_python my_package
```

### Build Workspace
```bash
cd /workspace/ros2_ws
colcon build
source install/setup.bash
```

### Run Your Node
```bash
ros2 run my_package my_node
```

## Troubleshooting

### GUI Not Working?

Check DISPLAY variable:
```bash
echo $DISPLAY  # Should show :0 or host.docker.internal:0
```

Test X connection:
```bash
xeyes  # Should open a window with eyes
```

### ROS2 Imports Not Found?

Check PYTHONPATH:
```bash
echo $PYTHONPATH
# Should include: /opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
```

If missing, rebuild container:
- Bottom-right widget → **Rebuild Dev Container**

### Git Safe Directory Error?

Should be fixed by `postCreateCommand`, but if not:
```bash
git config --global --add safe.directory /workspace
```

### Slow Performance?

For better file sync performance on macOS/Windows, use cached consistency in mount options (already configured in the provided config).

## Tips

* Terminal automatically sources ROS2 environment
* Python interpreter has full access to ROS2 packages
* All your local files are mounted at `/workspace`
* Container persists between PyCharm restarts
* Use `colcon build` inside the container
* PyCharm inspections work with ROS2 code

## Useful Commands
```bash
# List ROS2 topics
ros2 topic list

# Build workspace
cd /workspace/ros2_ws && colcon build

# Source workspace
source /workspace/ros2_ws/install/setup.bash

# Run a node
ros2 run <package> <node>

# Launch a file
ros2 launch <package> <launch_file>
```

## References

* [ROS2 Documentation](https://docs.ros.org/en/humble/)
* [PyCharm Dev Containers](https://www.jetbrains.com/help/pycharm/connect-to-devcontainer.html)
* [Docker Documentation](https://docs.docker.com/)