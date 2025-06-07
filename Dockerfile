# Stage 1: Use the official ROS 2 Jazzy image as a lightweight base.
FROM osrf/ros:jazzy-desktop-full

# Set the shell to bash for all subsequent commands.
SHELL ["/bin/bash", "-c"]

# Set environment variable to prevent interactive prompts during package installation.
ENV DEBIAN_FRONTEND=noninteractive

# Combine update, install, and cleanup into a single RUN layer to reduce image size.
# This is a key Docker optimization best practice.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    nano \
    vim \
    terminator \
    # Clean up the apt cache to keep the image lean.
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user 'rosuser' for enhanced security.
# This updated command gracefully handles cases where the GID already exists.
ARG USER_ID=1013
ARG GROUP_ID=1013
RUN if ! getent group $GROUP_ID; then \
        addgroup --gid $GROUP_ID rosuser; \
    fi && \
    adduser \
    --uid $USER_ID \
    --gid $GROUP_ID \
    --disabled-password \
    --gecos '' \
    rosuser && \
    usermod -aG sudo rosuser && \
    echo 'rosuser ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the non-root user for all subsequent instructions.
USER rosuser
WORKDIR /home/rosuser

# Set up the ROS 2 workspace directory structure.
RUN mkdir -p /home/rosuser/ros2_ws/src

# --- FIX for Permission Denied ---
# Change ownership of the entire workspace to the non-root user.
# This allows colcon to create build/, install/, and log/ directories.
RUN sudo chown -R rosuser:rosuser /home/rosuser/ros2_ws

# Set the default working directory for the container.
WORKDIR /home/rosuser/ros2_ws

# Copy the entrypoint script into the container and make it executable.
COPY --chown=rosuser:rosuser entrypoint.sh /home/rosuser/entrypoint.sh
RUN sudo chmod +x /home/rosuser/entrypoint.sh

# Set the entrypoint to our custom script.
ENTRYPOINT ["/home/rosuser/entrypoint.sh"]

# Set the default command to launch a bash shell.
CMD ["bash"]
