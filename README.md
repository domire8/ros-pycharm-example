# Developing Python ROS2 in PyCharm

This is an example project to illustrate how use PyCharm as a fully integrated development environment (IDE) for a ROS2
Python project by encapsulating the dependencies in a docker container.

## The turtle example

This example installs dependencies for the [`turtlesim`](http://wiki.ros.org/turtlesim) package on top of the base ROS2
image. It defines a new package called [`turtle_example`](./turtle_example) with a simple Python node
called [`circler`](./turtle_example/scripts/circler) which publishes twist commands for a running turtlesim node to
drive it in a circle.

## Base ROS2 Docker images

The majority of the background work is done in the
repository [aica-technology/docker-images](https://github.com/aica-technology/docker-images). There, docker images are
defined for specific versions of ROS and ROS2 that contain all the necessary development dependencies, including an
inbuilt SSH server. PyCharm will use the SSH server to access the remote environment.

To begin, clone the [docker-images](https://github.com/aica-technology/docker-images) repository to have access to the
helper scripts and additionally pull the ROS workspace image with

```bash
docker pull ghcr.io/aica-technology/ros-ws:foxy
```

## PyCharm setup

Prerequisites: Docker, PyCharm Professional 2021 or newer, base ROS2 docker image from AICA

**Step 1:** Build the docker image with `docker build -t domire8/ros2-pycharm-example .`

**Step 2:** Use the [`server.sh` script](https://github.com/aica-technology/docker-images/blob/master/scripts/server.sh)
in the aica-technology/docker-images repository to start a background container for the appropriate user with a desired
port number.

In this example, we will use user `ros2` for a foxy version image. We will also choose to use port 4567, though the
specific number does not matter.

```shell script
path/to/aica-technology/docker-images/scripts/server.sh domire8/ros2-pycharm-example --user ros2 --port 4567
```

**Step 3:** Open this directory in PyCharm or via command line

```bash
cd path/to/ros-pycharm-example && pycharm-professional .
```

**Step 4:** Go to `File` &rarr; `Settings` &rarr; `Project: ros-pycharm-example` &rarr; `Python Interpreter` and choose
the option to add a new interpreter.

![Add Interpreter](img/add_interpreter.png)

**Step 5:** Choose to add a new `SSH Interpreter`, enter the server configuration as shown in the picture below (**
replace ros by ros2 and 3456 by 4567**), and click next. There might be a warning popping up if you want to continue
connecting to the SSH server. Confirm with yes.

![Configure Interpreter](img/configure_interpreter.png)

**Step 6:** In the next window, change the interpreter path to `/usr/bin/python3` and make sure that the option
*Automatically upload project files to the server* is activated. Do not worry about the rest for now and click finish.

![Configure Interpreter 2](img/configure_interpreter_2.png)

**Step 7:** Go to `File` &rarr; `Settings` &rarr; `Build, Execution & Deployment` &rarr; `Deployment`. The created ssh
connection should now show up with type *SFTP*. Click on `AUTODETECT` in the connection tab. You should see `/home/ros2`
.

![Deployment Settings](img/deployment_settings.png)

**Step 8:** In the mappings tab, change the local path, deployment path, and web path as shown in the picture below.
These steps make sure that the syncing of files between the host and server is correct. Click apply, and finally make
sure that the new connection is used as default (`ros2@localhost:4567` in the list of connections should be bold,
otherwise use the tick above to set it as default).

![Deployment Settings 2](img/deployment_settings_2.png)

**Step 9:** Finally, go to `File` &rarr; `Settings` &rarr; `Build, Execution & Deployment` &rarr; `Deployment`
&rarr; `Options` and change to `Yes` under *Preserve original file permissions (SFTP only)*. Apply and close.

![SFTP Settings](img/sftp_settings.png)

**Step 10 (for testing):** Right-click on your project folder, go to *Deployment* and click on
*Upload to ros2@localhost:4567*. You should get a message in green saying that the files have been transferred.

Step 11 (optional): This example can easily be extended to a project with several ROS2 packages. Assuming a folder
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

Finally, choose to add `/home/ros2/ros2_ws/install/<your package>/lib/python3/site-packages` as additional remote path
for each package. This will enable full IDE capabilities even for the package(s) currently under development in the
project.

![Add dist packages 3](img/add_dist_packages_3.png)

## Using the example

Once the server is running in the background, you can use
the [`connect.sh` script](https://github.com/aica-technology/docker-images/blob/master/scripts/connect.sh)
to attach new terminal windows by specifying the container name and user. You can use `docker container ls` to look up
the names of running containers. In the case of this example:

```shell script
path/to/aica-technology/docker-images/scripts/connect.sh domire8-ros2-pycharm-example-ssh --user ros
```

Note that for GUI applications through Docker on MacOS, you will need to follow the additional
[display forwarding instructions](https://github.com/aica-technology/docker-images#notes-on-x11-display-forwarding-for-mac)
in the docker-images repository.

### Running the example outside the IDE

Using two terminals, invoke `ros2 run turtlesim turtlesim_node` in one, and `ros2 run turtle_example circler` in the
second one. You should see the turtle begin to swim in circles.

You can also simply use one terminal and invoke `ros2 launch turtle_example circler.launch.py` directly.

With the PyCharm environment established as explained above and automatic upload to the server enabled, you can modify
the code in `circler`, press save, go to the connected terminal, and immediately do `ros2 run turtle_example circler` to
see the changes.

### Running and debugging the example within the IDE

Code insight and completion are just two advantages of configuring the Python interpreter in the IDE. Another big
advantage is that you can actually run and **debug** your code from within the IDE. To do this, follow these steps:

1. In a terminal, invoke `ros2 run turtlesim turtlesim_node`. You should see the turtle in the bottom left corner of the
   window (not moving yet).
2. Go to the `circler.py` script and press the play button next to the line `if __name__ == '__main__':`. This won't
   work the first time around. But it will create a configuration *CIRCLER* that you should be able to see in the top
   right corner of PyCharm (see picture below). If this is not the case, activate `View` &rarr; `Appearance`
   &rarr; `Navigation Bar`.
   ![Add dist packages](img/edit_configuration.png)
3. Click on *CIRCLER*  &rarr; `Edit Configurations`. Under `Environment` &rarr; `Environment Variables` click on
   `Edit environment variables` and add an environment variable with
    - Name: PYTHONPATH
    - Value: $PYTHONPATH:/opt/ros/foxy/lib/python3.8/site-packages:/home/ros2/ros2_ws/install/turtle_example/lib/python3.8/site-packages

   Note that if you are working with several packages, you have to append 
   /home/ros2/ros2_ws/install/<package_name>/lib/python3.8/site-packages to the PYTHONPATH for each package.
   ![Add dist packages](img/env_variables.png)
4. Apply the settings and close the window. Then, you can use the play and debug icons to execute the script either in
   run or debug mode. For more information on debugging with PyCharm,
   visit [this](https://www.jetbrains.com/help/pycharm/debugging-your-first-python-application.html) website.

## Credits

Credits for the code of this example go to **[@eeberhard](https://github.com/eeberhard)** and his
equivalent [C++/CLion example](https://github.com/eeberhard/ros-clion-example).
