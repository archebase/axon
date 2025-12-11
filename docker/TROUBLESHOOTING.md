# Docker Build Troubleshooting

## Common Issues

### Apache Arrow Package Not Found

**Symptom:**
```
E: Unable to locate package libarrow-dev
E: Unable to locate package libarrow-glib-dev
E: Unable to locate package libparquet-dev
```

**Root Cause:**
- Apache Arrow packages are not in default Ubuntu repositories
- Need to add Apache Arrow official PPA

**Solution:**
- Already fixed in Dockerfiles - they automatically add Apache Arrow PPA
- If building manually, add the PPA:
  ```bash
  curl -fsSL https://apache.jfrog.io/artifactory/arrow/ubuntu/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb -o /tmp/apache-arrow-apt-source.deb
  sudo apt-get install -y /tmp/apache-arrow-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y libarrow-dev libarrow-glib-dev libparquet-dev
  ```

### 403 Forbidden Error

**Symptom:**
```
ERROR: failed to solve: ros:humble-desktop: failed to resolve source metadata 
for docker.io/library/ros/humble-desktop: unexpected status from HEAD request 
to https://docker.m.daocloud.io/...: 403 Forbidden
```

**Root Cause:**
- Docker is using a mirror registry (e.g., daocloud.io) that has rate limits or doesn't have the image
- The mirror may require authentication or have restrictions

**Solutions:**

1. **Use ros-base instead (Recommended)**
   - Smaller images, faster pulls
   - Already configured in Dockerfiles
   - Run: `make docker-build-ros2-humble`

2. **Configure Docker to use official registry**
   ```bash
   # For Docker Desktop (Mac/Windows)
   # Go to Settings > Docker Engine
   # Add or modify:
   {
     "registry-mirrors": [],
     "insecure-registries": []
   }
   
   # For Linux, edit /etc/docker/daemon.json
   sudo nano /etc/docker/daemon.json
   # Remove or comment out registry-mirrors
   sudo systemctl restart docker
   ```

3. **Login to Docker Hub**
   ```bash
   docker login
   # This may help with rate limits
   ```

4. **Use explicit registry in Dockerfile**
   ```dockerfile
   FROM docker.io/ros:humble-ros-base
   ```

5. **Pull image manually first**
   ```bash
   docker pull ros:humble-ros-base
   make docker-build-ros2-humble
   ```

### Image Size Issues

**ros-base vs ros-desktop:**
- `ros-base`: ~500MB, minimal ROS installation
- `ros-desktop`: ~2GB+, includes GUI tools and visualization

**We use ros-base because:**
- We only need build tools (colcon, catkin)
- We install dependencies explicitly
- Faster builds and pulls
- Smaller final images

### Build Failures

**Missing dependencies:**
- Ensure all apt packages are listed in Dockerfile
- Check if rosdep needs initialization (already handled)

**Rust build failures:**
- Check internet connection (Rust toolchain download)
- Verify Rust installation in Dockerfile

**CMake errors:**
- Ensure Arrow and yaml-cpp are installed
- Check ROS environment is sourced correctly

### Rust Compilation Errors

**Symptom:**
```
error[E0034]: multiple applicable items in scope
arrow-arith-37.0.0/src/temporal.rs:261:47
```

**Root Cause:**
- Arrow 50 has compatibility issues with `arrow-arith` and chrono versions
- Method name conflicts between `Datelike` and `ChronoDateExt` traits

**Solution:**
- Already fixed in `Cargo.toml` - using arrow 49 instead of 50
- If issues persist, try:
  ```bash
  cd src/bridge
  cargo update
  cargo build --release
  ```

### Network Issues

**Slow downloads:**
- Use ros-base (smaller)
- Pre-pull base images: `docker pull ros:humble-ros-base`
- Use local registry mirror if available

**Timeouts:**
- Increase Docker build timeout
- Check firewall/proxy settings

## Quick Fixes

```bash
# 1. Clean and rebuild
docker system prune -a
make docker-build-ros2-humble

# 2. Pull base image first
docker pull ros:humble-ros-base
make docker-build-ros2-humble

# 3. Use official registry
docker pull docker.io/ros:humble-ros-base
docker tag docker.io/ros:humble-ros-base ros:humble-ros-base
make docker-build-ros2-humble
```

## Verification

After building, verify the image:
```bash
docker run --rm ros:humble-ros-base /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 --help"
```

