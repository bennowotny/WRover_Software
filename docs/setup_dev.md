# Setup: Development Environment

This document describes the setup process for a development environment machine.
This includes the base station, as well as any other machine (e.g. a team member's laptop) which is used for ROS development.

## Prerequisites

You'll need various software tools before you can start working on the WR software system.

### Installing Linux

You will need to install a Linux distribution that supports ROS Noetic.
Ubuntu 20.04 ("Focal Fossa") is recommended, as this is the officially-supported OS for ROS Noetic.
Usually, it is recommended that it be installed natively, either [on a dedicated machine](https://help.ubuntu.com/community/Installation) or [as a dual-booted OS](https://help.ubuntu.com/community/WindowsDualBoot).
If necessary, it can also be installed as a virtual machine on Windows or macOS via software such as [VirtualBox](https://www.virtualbox.org/), although this comes at the cost of performance.

In theory, [the Windows Subsystem for Linux (WSL)](https://docs.microsoft.com/en-us/windows/wsl/about) can also run ROS, but it has some problems.
Most notably, the lack of PCI passthrough makes it difficult to interface with hardware devices, which may pose a problem while trying to develop software that interacts with rover hardware.
If you go this route, you may want to [recompile the WSL kernel](https://gist.github.com/cerebrate/d40c89d3fa89594e1b1538b2ce9d2720) to enable the kernel modules related to interfacing with hardware.
This is very tedious, and so WSL is usually not recommended.

From here onwards, it is assumed that you have some flavour of Ubuntu 20.04 installed.

### Installing Python 3

A significant portion of the WRover software system is built on Python 3, so you'll need to ensure that it, as well as some necessary packages, are installed.
Most Ubuntu 20.04 distributions should come with Python 3 by default, and you can check this using the command:

```sh
$ python3 --version # should print "3.(something)"
```

You'll want to ensure that `pip`, Python's package manager, is installed, since that's what we'll be using to install most Python packages.
This can be done using the commands:

```sh
$ sudo apt install python3-pip  # install pip
$ python3 -m pip install -U pip # ensure that it's up-to-date
```

Next, you'll want to use `pip` to install `virtualenv`, a tool that lets us isolate separate Python workspaces.
This can be done using the command:

```sh
$ python3 -m pip install -U virtualenv
```

### Installing ROS

The WRover software system is built using the Robotics Operating System (ROS) framework, which you'll need to install.
See [the official instructions on the ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) for more details.
You can check that everything has installed properly using the commands:

```sh
$ echo $ROS_DISTRO  # should print "noetic"
$ export | grep ROS # should print various env vars related to ROS
```

## Setting Up the Workspace

This repository consists of the ROS catkin workspace for the WRover software system.
You'll want to clone the repo to wherever you wish to store your workspaces (conventionally, this would be `~/catkin_ws`).
Once you have a local copy, you'll want to open a terminal in the workspace.
To initialize the workspace, run the command:

```sh
$ ./assemble.py init
```

This will initialize the catkin project and create the Python virtual environment.
Next, you'll want to run the command:

```
$ source setup.sh
```

This will configure your current terminal's environment to work in this workspace.
Now, you can run the command:

```sh
$ ./assemble.py build
```

This will download any dependencies, then build the whole project.
Once this is finished, your workspace setup is complete!

Every time you want to work in the workspace, you'll have to run `source setup.sh` again to set up your shell with the workspace environment.
This is equivalent to sourcing the `devel/setup.bash` and `venv/bin/activate` scripts, which will respectively configure the ROS package path and activate the Python venv.
Alternatively, you can do this automatically in your `.bashrc` if you don't anticipate working in any other ROS workspaces.

## Working in the Workspace

### Sourcing `setup.sh`

When working on the software system, you'll want to ensure that your terminal window is configured to work in the catkin workspace.
This is done by sourcing the `setup.sh` script, which is done using one of the commands:

```sh
$ source setup.sh # configure the environment
$ . setup.sh      # "." is a shorthand for "source"
```

If successful, you should see "`(venv)`" somewhere in your terminal's prompt line, which indicates that the workspace's Python virtual environment has been activated.
If sourcing `setup.sh` for every new terminal is too tedious, you can automatically do it by adding the command to `.bashrc`.
This could be done, for example, using the command:

```sh
$ echo "source ~/catkin_ws/WRover21_Software/setup.sh" >> ~/.bashrc
```

Note that if you intend to use multiple catkin workspaces on the same machine, this might not be the best idea, as you'll always automatically be put into that one workspace.

### Building the Workspace

To build the workspace, you can either use `assemble.py` or directly invoke `catkin_make`.
Using `assemble.py`, you would run one of:

```sh
$ ./assemble.py build    # update all dependencies, then build
$ ./assemble.py build -o # build without updating dependencies
$ ./assemble.py build -f # delete and re-download all dependencies, then build
```

The `build` task of `assemble.py` checks dependencies, then delegates to `catkin_make`.
If you don't want to bother with dependencies, you can just invoke `catkin_make` directly.
For more info on `assemble.py`, check out the `build_tools.md` document.

## Connecting to the Rover

TODO: stuff about public key auth
