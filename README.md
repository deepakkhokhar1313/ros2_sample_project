ROS 2 Docker Project: A Setup & Troubleshooting Guide
This document chronicles the end-to-end process of setting up a robust, containerized development environment for a ROS 2 project. It serves as a practical guide, detailing the real-world challenges encountered—from networking and permissions to build system quirks and CI/CD—and provides the solutions that led to a successful and efficient workflow.

Chapter 1: The Dockerized Development Environment
The foundation of modern robotics development is a consistent, portable environment. Our first step was to create this using Docker, but it came with its own set of initial hurdles.

Challenge 1.1: Slow Internet Speed in Container
Symptom: After setting up the container, any network activity inside it (e.g., apt-get update or speedtest-cli) was incredibly slow (KB/s), while the host machine's internet was fast (MB/s).

Root Cause: The issue was traced to the host machine's Wi-Fi Power Saving Mode. This feature causes tiny latency spikes as the Wi-Fi card wakes up to handle network packets. While unnoticeable for normal browsing, these delays accumulate and cripple the performance of Docker's virtualized networking. A custom /etc/docker/daemon.json file created during troubleshooting was also found to conflict with the final network configuration.

Solution: The problem was resolved at the host level.

Disable Wi-Fi Power Saving: This was the critical fix. The following command turns off power saving for the current session.

sudo iwconfig <your_interface_name> power off

Remove Conflicting Docker Config: We removed the custom Docker daemon configuration to ensure it didn't interfere with the network_mode: "host" setting.

sudo rm /etc/docker/daemon.json
sudo systemctl restart docker

Challenge 1.2: File Permission Errors on Host
Symptom: When trying to create a file (e.g., talker.py) in VS Code on the host machine inside a directory that was created by a command inside the container, we received an EACCES: permission denied error.

Root Cause: A command like ros2 pkg create run by the rosuser inside the container creates a new directory. On the host machine, this directory is owned by the root user. The code editor, running as a normal user, doesn't have permission to write to a root-owned directory.

Solution: Take ownership of the new directory on the host machine. This is a one-time action required only after creating a new package from within the container.

sudo chown -R $USER:$USER path/to/your/new/package

Chapter 2: Mastering the Colcon Build System
colcon is the standard build tool for ROS 2, but its interaction with mixed-language packages and its own caching can be tricky.

Challenge 2.1: Initial Build Failures
Symptom: Our first colcon build attempts failed with PermissionError: [Errno 13] Permission denied: 'log' and add_custom_target cannot create target ... because another target with the same name already exists.

Root Cause:

The first error was because the rosuser inside the container didn't own the workspace directory, so it couldn't create the build/, install/, and log/ folders.

The second error was caused by stale files left over from a previously failed build, which confused CMake.

Solution:

Fix Ownership: We added a line to our Dockerfile to ensure rosuser owns the workspace: RUN sudo chown -R rosuser:rosuser /home/rosuser/ros2_ws.

Clean Workspace: We learned a critical ROS 2 development habit: always clean your workspace after a build failure or when changing build configurations.

# From your workspace root (~/ros2_ws)
rm -rf build/ install/ log/

Challenge 2.2: The Mixed Python/Interface Package Puzzle
Symptom: When trying to add a custom service (.srv file) to our Python package, the build system failed with a cascade of confusing errors like package name does not match current PROJECT_NAME.

Root Cause: This revealed a core ROS 2 design principle: a single package is not designed to both generate custom interfaces (which requires build_type: ament_cmake) and contain Python nodes (which uses build_type: ament_python). The two build systems conflict.

Solution: We adopted the ROS 2 best practice of separating concerns into two packages:

my_interfaces Package: A simple ament_cmake package whose only job is to contain our .srv and .action files.

py_pubsub Package: Our main ament_python package, which now simply adds <exec_depend>my_interfaces</exec_depend> to its package.xml to use the custom interfaces. This resolved all build conflicts.

Chapter 3: Automating with GitHub Actions CI/CD
Continuous Integration (CI) is essential for professional development. Our goal was to automatically build, test, and publish our Docker image.

Challenge 3.1: The Out-of-Memory Killer
Symptom: Our GitHub Actions workflow consistently failed on the colcon build step with Error: Process completed with exit code 137.

Root Cause: Exit code 137 indicates the process was terminated by the system's Out-of-Memory (OOM) Killer. Even with an empty workspace, the cumulative memory usage of building the large ROS 2 Docker image and then starting a container from it was enough to exhaust the limited RAM of the standard GitHub runner.

Solution: We implemented a two-part solution in our .github/workflows/ros-ci.yml file:

Increase Swap Space: We added a step to provide more virtual memory to the runner, creating a buffer to prevent OOM errors.

- name: Increase Runner Swap Space
  uses: pierotofy/set-swap-space@master
  with:
    swap-size-gb: 16

Efficient Execution: We replaced the inefficient docker run -d followed by docker exec with a single, self-contained docker run --rm command. This executes all build and test steps in one process and automatically cleans up the container upon completion, significantly reducing the overall memory pressure.

- name: Build and Test ROS 2 Workspace
  run: |
    docker run --rm \
      -v ${{ github.workspace }}/src:/home/rosuser/ros2_ws/src \
      ${{ env.IMAGE_NAME }}:latest \
      bash -c "colcon build ... && colcon test ... && colcon test-result ..."

This journey, filled with practical challenges and solutions, resulted in a robust, professional, and efficient development environment ready for any ROS 2 project.