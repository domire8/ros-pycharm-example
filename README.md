# Developing Python ROS in PyCharm

This is an example project to illustrate how use PyCharm as a fully integrated development environment for a ROS Python
project by encapsulating the dependencies in a docker container.

## The turtle example

This example installs dependencies for the [`turtlesim`](http://wiki.ros.org/turtlesim) package on top of the base ROS
image. It defines a new package called [`turtle_example`](./turtle_example) with a simple Python node
called [`circler`](./turtle_example/scripts/circler)
which publishes twist commands for a running turtlesim node to drive it in a circle.

## Base ROS Docker images

The majority of the background work is done in the
repository [aica-technology/docker-images](https://github.com/aica-technology/docker-images). There, docker images are
defined for specific versions of ROS and ROS2 that contain all of the necessary development dependencies, including an
inbuilt SSH server. PyCharm will use the SSH server to access the remote environment.

To begin, clone the [docker-images](https://github.com/aica-technology/docker-images) repository and follow the
instructions to pull or build your base ROS workspace of choice.

## PyCharm setup

Prerequisites: Docker, PyCharm Professional 2021 or newer, base ROS docker image from AICA

Step 1: Build the docker image with `docker build -t domire8/ros-pycharm-example .` (this will take a while the first
time around).

Step 2: Use the [`server.sh` script](https://github.com/aica-technology/docker-images/blob/master/scripts/server.sh) in
the aica-technology/docker-images repository to start a background container for the appropriate user with a desired
port number.

In this example, we will use user `ros` for a noetic version image. We will also choose to use port 3456, though the
specific number does not matter.

```shell script
path/to/aica-technology/docker-images/scripts/server.sh domire8/ros-pycharm-example --user ros --port 3456
```

Step 3: Open this directory in PyCharm or via command line

```bash
cd path/to/ros-pycharm-example && pycharm-professional .
```

Step 4: Go to `File` &rarr; `Settings` &rarr; `Project: ros-pycharm-example` &rarr; `Python Interpreter` and choose the
option to add a new interpreter.

![Add Interpreter](img/add_interpreter.png)

Step 5: Choose to add a new `SSH Interpreter`, enter the server configuration as shown in the picture below, and click
next. There might be a warning popping up if you want to continue connecting to the SSH server. Confirm with yes.

![Configure Interpreter](img/configure_interpreter.png)

Step 6: In the next window, change the interpreter path to `usr/bin/python3` and make sure that the option *
Automatically upload project files to the server* is activated. Do not worry about the rest for now and click finish.

![Configure Interpreter 2](img/configure_interpreter_2.png)

Step 7: Go to `File` &rarr; `Settings` &rarr; `Build, Execution & Deployment` &rarr; `Deployment`. The created ssh
connection should now show up with type *SFTP*. Click on `AUTODETECT` in the connection tab. You should see `/home/ros`.

![Deployment Settings](img/deployment_settings.png)

Step 8: In the mappings tab, change the local path, deployment path, and web path as shown in the picture below. This
steps make sure that the syncing of files between the host and server is correct. Click apply, and finally make sure
that the new connection is used as default (`ros@localhost:3456` in the list of connections should be bold, otherwise
use the tick above to set it as default).

![Deployment Settings 2](img/deployment_settings_2.png)

Step 9: Finally, go to `File` &rarr; `Settings` &rarr; `Build, Execution & Deployment` &rarr; `Deployment`
&rarr; `Options` and change to `Yes` under *Preserve original file permissions (SFTP only)*. Apply and close.

![SFTP Settings](img/sftp_settings.png)

Step 10 (for testing): Right click on your project folder, go to *Deployment* and click on *Upload to ros@localhost:
3456*. You should get a message in green saying that the files have been transferred.

Step 11 (optional): This example can easily be extended to a project with several ROS packages. Assuming a folder
structure like

```bash
ros-pycharm-example
   └──source
       ├──package1
       └──package2
```

you would just need to `Add new mapping` in step 8 such that you have one mapping per package. Additionally, after you
have built the package(s) the first time, you can go to `File` &rarr; `Settings` &rarr; `Project: ros-pycharm-example`
&rarr; `Python Interpreter` and choose *Show All*.

![Add dist packages](img/add_dist_packages.png)

Then, select your new interpreter and click on *Show paths for selected interpreter* and then click on the plus.

![Add dist packages 2](img/add_dist_packages_2.png)

Finally, choose to add `/home/ros/ros_ws/devel/lib/python3/dist-packages` as additional remote path. This will enable
full IDE capabilities even for the package(s) currently under development in the project.

![Add dist packages 3](img/add_dist_packages_3.png)

## Using the example

Once the server is running in the background, you can use
the [`connect.sh` script](https://github.com/aica-technology/docker-images/blob/master/scripts/connect.sh)
to attach new terminal windows by specifying the container name and user. You can use `docker container ls` to lookup
the names of running containers. In the case of this example:

```shell script
/.../aica-technology/docker-images/scripts/connect.sh domire8-ros-pycharm-example-ssh --user ros
```

Using three terminals, invoke `roscore` in one, then `rosrun turtlesim turtlesim_node` in a second, and finally
`catkin_make && rosrun turtle_example circler` in the third. You should see the turtle begin to swim in circles.

You can also simply use one terminal and invoke `catkin_make && roslaunch turtle_example circler.launch` directly.

With the PyCharm environment properly established, you will also be able to modify the `circler` source script from
within the IDE instead of from the third terminal and still see the effect on the turtlesim window.

Note that for GUI applications through Docker on MacOS, you will need to follow the additional
[display forwarding instructions](https://github.com/aica-technology/docker-images#notes-on-x11-display-forwarding-for-mac)
in the docker-images repository.
