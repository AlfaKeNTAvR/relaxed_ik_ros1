# Relaxed IK ROS1 for Kinova Gen3 Robotic Arm

## Introduction

This repository is based on the [RelaxedIK](https://github.com/uwgraphics/relaxed_ik) repository, with additional config files to use relaxedIK with Kinova Gen3 Robotic Arm.

**NOTE:** the required Kinova config files needed to implement RelacedIK have already been generated and are provided to you in this repository. If you want to generate them yourself, you will need to clone [this repo](https://github.com/uwgraphics/relaxed_ik.git) and follow the guide [there](https://github.com/uwgraphics/relaxed_ik/blob/dev/src/start_here.py).

## Dependencies

Make sure you install the following dependencies:

### Python Dependencies
**readchar**:
```bash
sudo pip install readchar
```
**fcl collision library**: https://github.com/BerkeleyAutomation/python-fcl
```bash
sudo pip install python-fcl
```
**scipy**:
```bash
sudo pip install scipy
```
**Pyyaml**:
```bash
sudo pip install PyYaml
```
**kdl urdf parser**:
```bash
sudo apt-get install ros-noetic-urdfdom-py
sudo apt-get install ros-noetic-kdl-parser-py
sudo apt-get install ros-noetic-kdl-conversions
```
**Pypcd**: https://github.com/dimatura/pypcd
Lastly, update your version of **numpy**:
```bash
sudo pip install --upgrade numpy
```

### Rust Dependencies
To use this wrapper, you will first need to install Rust. Please go to https://www.rust-lang.org/learn/get-started for more infomation.
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```
Downgrade to rust 1.58:
```bash
rustup install 1.58 
rustup default 1.58
```

## Getting Started

1. Make sure that you have installed all the dependencies.
1. Create a new workspace.
1. Clone the [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) repo to the *scr* directory in your catkin workspace.
    ```bash
    git clone https://github.com/Kinovarobotics/ros_kortex.git
    ```
1. Clone the [kinova_positional_control](https://github.com/AlfaKeNTAvR/kinova_positional_control) repo to the *src* directory in your catkin workspace.
    ```bash
    git clone https://github.com/AlfaKeNTAvR/kinova_positional_control.git
    ```
1. Clone [kinova_pid](https://github.com/AlfaKeNTAvR/kinova_pid) repo to the *src* directory in your catkin workspace.
    ```bash
    git clone -b joint-controllers https://github.com/AlfaKeNTAvR/kinova_pid.git
    ```
1. Clone the [relaxed_ik_ros1](https://github.com/AlfaKeNTAvR/relaxed_ik_ros1) repo to the *src* directory in your catkin workspace.
    ```bash
    git clone https://github.com/AlfaKeNTAvR/relaxed_ik_ros1.git
    ```
1. Build your catkin workspace by using `catkin_make` in the workspace root directory. 
1. Initialize relaxed_ik_core (the Rust library of Relaxed IK) as a submodule by running the following command from the project directory *(/src/relaxed_ik_ros1)*:
    ```bash
    git submodule update --init
    ``` 
    
    1. If the previous command didn't return an error, you can skip this step. Else, you might need to Setup a SSH key for Git: 
    
    Paste the text below, substituting in your GitHub email address.
    ```bash
    sh-keygen -t ed25519 -C "your_email@example.com" 
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_ed25519
    ``` 
    Copy the SSH key to your clipboard.
    ```bash
    clip < ~/.ssh/id_ed25519.pub
    ```

    Login to your GitHub account > Settings.

    In the "Access" section of the sidebar, click SSH and GPG keys.

    Click New SSH key or Add SSH key.

    Create a Title and paste the SSH Key under "Key", then click Add SSH key.

1. After the submodule has been initialized successfully, navigate to the *relaxed_ik_core* folder and compile the core:
    ```bash
    cargo build
    ``` 
1. Look at <settings.yaml> in the folder *relaxed_ik_core/config* and follow the instructions there to customize the parameters, and personalize the environment obstacles. Note that you don't need to recompile *relaxed_ik_core* every time you change the parameters in <settings.yaml>.
1. Look at <kortex_info.yaml> in the folder *relaxed_ik_ros1/relaxed_ik_core/config/info_files* and edit the starting configuration of the robot.

### Possible Errors

One of the possible errors you might encounter is when running the last command *rosrun relaxed_ik_ros1 keyboard_ikgoal_driver.py* for testing purposes:

*thread '<unnamed>' panicked at 'assertion failed: min_bound == min_bound", /home Valfakentavr/.cargo/registry/src/github.com-1ecc6299db9ec823/ncollide3d-0.21.0/s rc/query/algorithms/gjk.rs: 124:9 note: run with RUST_BACKTRACE=1' environment variable to display a backtrace*

To solve this error, go to RVIZ and change the fixed frame from common_world to base_link. Then Save Config As relaxed_ik_viewer.rviz with replacement.
