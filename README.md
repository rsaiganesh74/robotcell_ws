# Bin Picking

## Table of Contents

- [Requirements](#requirements)
- [Setup](#setup)
  - [Python Virtual Environment](#python-virtual-environment)
  - [Installing Dependencies](#installing-dependencies)
  - [Bash Aliases and .bashrc Modifications](#bash-aliases-and-bashrc-modifications)
- [Running the Project](#running-the-project)
  - [ROS Node Execution](#ros-node-execution)

## Requirements

- Python 3 (ensure you have Python 3.6+)
- ROS2 (e.g., ROS2 Humble or Foxy)
- Git

## Setup

### Python Virtual Environment

It is recommended that you use a Python virtual environment to manage your dependencies. You can use the built-in `venv` module. If you don't have `venv` installed, you may need to install it (on some systems, it's provided by the package `python3-venv`).

Create and activate a virtual environment:

```bash
# In the project root directory
cd ~/robotcell_ws
python3 -m venv .venv
source .venv/bin/activate
```

### Installing Dependencies
Once your virtual environment is activated, install the required Python packages:
```bash
pip install -r requirements.txt
```

### Bash Aliases and Dependencies
If you already have a .bash_aliases file then add the following to it:
```bash
alias door_open='cd ~/robotcell_ws && source install/local_setup.bash && ros2 service call /door_status_set std_srvs/srv/SetBool data:\ false'
alias door_close='cd ~/robotcell_ws && source install/local_setup.bash && ros2 service call /door_status_set std_srvs/srv/SetBool data:\ true'
alias estop_press='cd ~/robotcell_ws && source install/local_setup.bash && ros2 service call /estop_status_set std_srvs/srv/SetBool data:\ true'
alias estop_disengage='cd ~/robotcell_ws && source install/local_setup.bash && ros2 service call /estop_status_set std_srvs/srv/SetBool data:\ false'
```
If you do not have a bash alias file then do the following:
```bash
cd
touch .bash_aliases
sudo nano .bash_aliases
```
Then enter the above aliases in the newly created .bash_aliases file.

For .bashrc, following modifications are needed:
```bash
cd
sudo nano .bashrc
```
Add these lines to the end of your .bashrc file:
```bash
source /opt/ros/humble/setup.bash

pick_request() {

    local pick_id="$1"
    local quantity="$2"
    curl -X POST -H "Content-Type: application/json" \
         -d "{\"pick_id\": \"${pick_id}\", \"quantity\": ${quantity}}" \
         http://localhost:8080/pick
}
```

Also check if these lines exist in your .bashrc file:
```bash
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi
```
If they do, then check if the alias file name is the same as you have created.

### Running the Project

First execute the following in your terminal:
```bash
cd ~/robotcell_ws
source .venv/bin/activate
```
Verify that the virtual environment is activated by checking if the (.venv) appears at the beginning of the line in the terminal.

If so then, execute:
```bash
colcon build
```

Then open three aditional terminals.

In terminal number 1:
```bash
source ~/.bashrc
source install/setup.bash
ros2 launch launcher launcher_launch.py 
```

You should see the HMI.

In terminal number 2:
```bash
source ~/.bashrc
pick_request <pick_id> <quantity>
```
Fill in the place holders with actual integer values.

FInally in terminal number 3:
```bash
source ~/.bashrc
AND THEN
doop_open
OR
door_close
OR
estop_press
OR
estop_disengaged
```
execute the relevant code depending on what you want to simulate.
