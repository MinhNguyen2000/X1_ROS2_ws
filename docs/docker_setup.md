# Docker Setup

This project uses **two separate Docker environments** — one for the development machine and one for the physical robot. Both are defined by Dockerfiles that live inside the git-tracked `X1_ROS2_ws` repository, but they serve completely different purposes and run on different hardware.

---

## Repository layout

```
X1_ROS2_ws/
├── .devcontainer/
│   └── Dockerfile          # Dev machine container (x86_64, VSCode Dev Container)
├── robot_ws/
│   └── Dockerfile          # Robot container (aarch64 Jetson Nano, hardware deployment)
└── src/
    └── ...                 # ROS2 packages shared by both environments
```

> **Why `robot_ws` is inside `X1_ROS2_ws`:** `robot_ws` was originally a standalone directory on the robot host and was not git-tracked. It has since been moved inside `X1_ROS2_ws` so that the robot Dockerfile and any robot-specific configuration are version-controlled alongside the rest of the project.

---

## Environment 1 — Development machine (`.devcontainer/`)

| Property | Value |
|---|---|
| Host OS | Ubuntu 22.04 (x86_64) |
| Purpose | RViz visualization & ROS2 inspection |
| Entry point | VSCode Dev Containers (`Ctrl+Shift+P` → *Reopen in Container*) |

The dev container is managed entirely by VSCode using the `.devcontainer/Dockerfile`. VSCode bind-mounts the workspace automatically — no manual `docker run` is needed. The container communicates with the robot over the local network via ROS2 DDS (same `ROS_DOMAIN_ID`), allowing topics like `/scan`, `/odom`, `/cmd_vel`, and `/face_pose` to be visualized in RViz on the dev machine while the robot is running.
Rebuilt the dev container from VSCode:Ctrl+Shift+P → "Dev Containers: Rebuild Container"

---

## Environment 2 — Robot (Jetson Nano) (`robot_ws/`)

| Property | Value |
|---|---|
| Host OS | Ubuntu 18.04 (aarch64, Jetson Nano 4GB) |
| Purpose | Hardware bringup, DRL policy inference, face detection (TensorRT) |
| Entry point | Auto-start via `~/.bashrc` => `docker exec` on SSH login |
| CUDA | 10.2 (mounted read-only from host) |

The robot container is built from `robot_ws/Dockerfile` and run with the script below. It is configured with `--restart unless-stopped` so it survives reboots, and SSH is started inside the container to allow VSCode *Connect to Host* to attach directly to the container.

### Build the robot image

Run this on the **Jetson Nano host** (outside the container):

```bash
cd ~/X1_ROS2_ws/robot_ws
docker build -t ros2_humble_img .
```

> **Note:** The Dockerfile must be edited on the Jetson Nano **host** (not from inside the container) to avoid bind-mount permission issues. The file is at `~/X1_ROS2_ws/robot_ws/Dockerfile` on the host.

### Start the robot container

Method 1: start the robot container with the starting shell script

```bash
cd ~/X1_ROS2_ws/robot_ws
. start_ros2_container.sh
```

Method 2: start the container with the following bash command (content of the above shell script)
```bash
docker run -d --name ros2_humble --restart unless-stopped \
  -u ros \
  --net=host \
  --privileged \
  --runtime nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /dev:/dev --device-cgroup-rule='c *:* rmw' \
  -v /usr/lib/aarch64-linux-gnu/libEGL.so.1:/usr/lib/aarch64-linux-gnu/libEGL.so.1:ro \
  -v /usr/local/cuda-10.2:/usr/local/cuda-10.2:ro \
  -v ~/X1_ROS2_ws:/X1_ROS2_ws \
  -v ~/robot_ws:/robot_ws \
  ros2_humble_img bash -c "sudo mkdir -p /run/sshd && sudo /usr/sbin/sshd -D"
```

Key flags:

- `--net=host` — shares the host network stack so ROS2 DDS discovery works without extra configuration
- `--privileged` + `-v /dev:/dev` — exposes all host device nodes (LiDAR on `/dev/ydlidar`, STM32 board on `/dev/myserial`) to the container
- `--runtime nvidia` + CUDA volume mount — enables TensorRT inference for the face detection node
- `-v ~/X1_ROS2_ws:/X1_ROS2_ws` — bind-mounts the full git repository so ROS2 packages built inside the container persist on the host
- `-v ~/robot_ws:/robot_ws` — retained for backwards compatibility with any scripts that reference the original `robot_ws` path

### Rebuild after Dockerfile changes

```bash
# On the Jetson Nano host:
docker stop ros2_humble && docker rm ros2_humble
cd ~/X1_ROS2_ws/robot_ws
docker build -t ros2_humble_img .
# Then re-run the docker run command above
```

> Workspace source files survive a container rebuild because they live in the bind-mounted `~/X1_ROS2_ws` directory on the host, not inside the container image.

---

## Connecting to the robot container

**Option A — SSH directly into the container** (from the dev machine or any machine on the network):

```bash
ssh jetson@192.168.0.141
# ~/.bashrc on the Jetson host runs `docker exec` to drop you into the container automatically
```

**Option B — VSCode Remote: Connect to Host** - some extra setup, as detailed in the next section, is required

---
 
## VSCode SSH configuration
 
Recent versions of VSCode's *Remote: Connect to Host* extension require a sufficiently modern OS on the remote host to install the VSCode Server. The Jetson Nano runs Ubuntu 18.04, which no longer meets this requirement. The workaround is to connect **directly to the container** (which runs Ubuntu 22.04) rather than to the Jetson Nano host, by forwarding an SSH port from inside the container to the host network.
 
### 1. Expose the container's SSH port on the host
 
The robot's `Dockerfile` already handles the `sshd` configuration at image build time:
 
```dockerfile
RUN mkdir -p /run/sshd \
    && sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config \
    && sed -i 's/#PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config
```
 
This configures `sshd` inside the container to listen on port `2222` instead of the default `22`. Because the container uses `--net=host`, port `2222` inside the container is directly reachable on the Jetson Nano's network interface — no explicit `-p` port mapping is needed. Using port `2222` also avoids conflicting with any `sshd` already running on the Jetson Nano host on port `22`.
 
The `docker run` entrypoint then starts `sshd` at container launch:
```bash
bash -c "sudo mkdir -p /run/sshd && sudo /usr/sbin/sshd -D"
```
 
No changes to the `docker run` command are required for SSH to work.
 
### 2. Add entries to `~/.ssh/config` on the dev machine
 
Open `~/.ssh/config` (create it if it doesn't exist) and add one entry per network the robot may be on:
 
```
# Robot container — lab WiFi (update with the robot's IP address in the network)
Host X1_ROS2_container_lab_wifi
  HostName 192.168.x.x
  Port 2222
  User ros
 
# Robot container — eduroam
Host X1_ROS2_container_eduroam
  HostName 10.9.x.x
  Port 2222
  User ros
```
 
The `Host` alias is what appears in VSCode's *Connect to Host* dropdown and can be named anything descriptive.
 
### 3. Connect from VSCode
 
1. Open the Command Palette (`Ctrl+Shift+P`)
2. Select **Remote-SSH: Connect to Host...**
3. Pick `X1_ROS2_container_lab_wifi` (or whichever alias matches your current network)
VSCode will SSH into the container directly (Ubuntu 22.04) and install the VSCode Server there, bypassing the host OS version restriction entirely.
 
### 4. Verify SSH access from the terminal first
 
Before connecting via VSCode, confirm the SSH tunnel is working:
 
```bash
ssh X1_ROS2_container_lab_wifi          # uses the alias from ~/.ssh/config
# or explicitly:
ssh ros@192.168.0.141 -p 2222
```
 
If the connection is refused, check that `sshd` is running inside the container:
 
```bash
# From the Jetson Nano host:
docker exec ros2_humble pgrep -a sshd
```

---

## Peripheral device symlinks

The robot container relies on udev rules set up on the Jetson Nano host to provide stable device paths:

| Device | Symlink |
|---|---|
| YDLiDAR (USB) | `/dev/ydlidar` |
| STM32 low-level board (USB) | `/dev/myserial` |

These symlinks are visible inside the container because of the `-v /dev:/dev` bind mount.
