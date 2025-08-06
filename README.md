# Robotic Bin Picking Cell Control

This project focuses on the development and implementation of a control system for a robotic cell designed for bin picking applications in a warehouse environment. The system is built to facilitate safe and efficient communication between a Warehouse Management System (WMS) and the robotic cell.

## Project Components

The solution is comprised of the following key components:

* **API Call Handler:** An API was implemented to handle communication between the WMS and the robotic cell. A server-side component sends picking requests, and a client-side component receives these requests, simulates the picking process, and sends a confirmation response.
* **ROS 2 Nodes:** Several ROS 2 nodes were developed to monitor and manage the state of the robotic cell's hardware:
    * **Scanner Node:** Publishes a constant stream of 5-digit random numbers to simulate a barcode scanner and offers a service to return the most recent barcode.
    * **Door Handle Node:** Publishes a boolean value representing the state of the cell's door handle (true for closed, false for open).
    * **Emergency Button Node:** Publishes a boolean value indicating if the emergency button has been pressed (true for pressed, false for not pressed).
    * **Stack-light Node:** Publishes an integer value to indicate the system's current state: `0` for operational, `1` for paused, and `-1` for emergency.
* **Human-Machine Interface (HMI):** A real-time HMI was created to provide workers with a visual representation of the cell's status. It displays the details of requests and responses, the current states of the emergency button and door handle, and the stack-light status, which is visualized with corresponding colored shapes (green for operational, yellow for paused, red for emergency).

This project demonstrates the integration of various software components to create a robust control system, ensuring the safe and functional operation of an automated bin picking cell.
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

### Bash Aliases and .bashrc Modifications
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
## ROS Node Execution

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
source .venv/bin/activate
source install/setup.bash
ros2 launch launcher launcher_launch.py 
```

You should see the HMI.

In terminal number 2:
```bash
source ~/.bashrc
source .venv/bin/activate
pick_request <pick_id> <quantity>
```
Fill in the place holders with actual integer values.

FInally in terminal number 3:
```bash
source ~/.bashrc
source .venv/bin/activate
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
