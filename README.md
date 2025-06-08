# ros2_sample_project
## Some important Commands:
# 1. To launch a launch file: ros2 launch py_pubsub pubsub.launch.py 
# 2. command to use a service: ros2 interface show py_pubsub/srv/AddTwoInts
# 3. command to run service server : ros2 run py_pubsub service_server
# 4. command to run service client : ros2 run py_pubsub service_client














## Problem 1: 
Unable to write file '/home/deepak/Desktop/Projects/ros2_ws1/ros2_sample_project/src/py_pubsub/py_pubsub/talker.py' (NoPermissions (FileSystemError): Error: EACCES: permission denied, open '/home/deepak/Desktop/Projects/ros2_ws1/ros2_sample_project/src/py_pubsub/py_pubsub/talker.py')

## Cause : 
What's Happening
You ran the command ros2 pkg create py_pubsub from inside the Docker container.
That command, running as the rosuser inside the container, created a new directory inside your mounted src/ folder.
On your host machine, the operating system sees that this new directory was created by the Docker daemon's process, which often means the new directory is now owned by the root user on your host.
Your VS Code application is running as your normal user(not as root), and the normal user does not have permission to create files inside a directory owned by root.

## Solution : 
You need to take ownership of the newly created directory on your host machine.

Open a terminal on your host machine and run the following command. This will recursively change the owner of the py_pubsub directory (and everything inside it) to be your current user.
command: sudo chown -R $USER:$USER ~/Desktop/Projects/ros2_ws1/ros2_sample_project/src/py_pubsub