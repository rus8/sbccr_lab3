# ecn_baxter_vs
Sensor-based control lab 3 at École Centrale de Nantes.


## Prepare environment
Instructions are provided for Ubuntu 16 and ROS Kinetic.

At first follow steps 1, 3-5 from [Lab 2: Prepare Environment](https://github.com/rus8/sbccr_lab2/blob/lab2/README.md)

Then do the following:
1.  Clone repo with baxter packages in some temporary location: <br>
    ```git clone https://github.com/RethinkRobotics/baxter_common.git```
    Then move content of *baxter_common* folder to your workspace src/ folder.
2.  Clone this repo in workspace src/ folder: <br>
    ```git clone https://github.com/rus8/sbccr_lab3.git```
3.  Build your workspace. Run in workspace root folder: <br>
    ``` catkin build # using catkin_tools```


## Run the lab

1. Being in your workspace root folder source environment: <br>
    ```source devel/setup.bash```
2. Launch VREP: <br>
    ```roslaunch ecn_baxter_vs vrep.launch```
3. Run control node: <br>
    ```rosrun ecn_baxter_vs baxter_vs```
