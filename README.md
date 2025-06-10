# Unitree g1 Whoble Body Control (WBC) Deploy

In this repo, we use an apple vision pro to control upper body of Unitree g1, and use `Homie` Algorithm to control lower body.

![](demos.gif)


## 👏 Acknowledgements

- [OpenHomie](https://github.com/OpenRobotLab/OpenHomie/tree/main/HomieDeploy): Our robot deployment code is based on `OpenHomie`.

- [avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate): We use `avp_teleoperate` library to control upper body of the robot.

## Deploy

For controlling robot, we use the cpp-based Unitree SDK2, which means you should first compile the `g1_control.cpp` for `Unitree G1` and `hand_control.cpp` for `Dex-3`. We have prepared the required `CMakeLists.txt` for you, so you only need to run the following command lines:
```
cd unitree_sdk2
rm -rf build
mkdir build && cd build
cmake ..
make
```
Then the runnable binary files will be in `unitree_sdk2/build/bin`.
You also need to install the  `g1_gym_deploy` by running:
```
cd g1_gym_deploy && pip install -e .
```


**Before deployment, please run L1+A L2+R2 L2+A L2+B to close G1's initial control process, if successful, you will see the robot hang up its arm after L2+A and lose efforts after L2+B.**

For TCP communications, you should determine the IP address of your PC and robot by running:
```
ifconfig | grep inet
```
Set the IP addresses in the code to the correct value.


A. Run the robot control program on `robot` (robot terminal 1):
```
cd unitree_sdk2/build/bin && ./g1_control eth0 (or eth1)
```
B. Run the inference thread to make policy control robot on `robot` (robot terminal 2):
```
python g1_gym_deploy/scripts/deploy_policy.py
```
C. After putting the robot on the ground, push the `R2` button of the joysticker, make the robot stand on the ground, and push `R2` again.

***NOTE:*** We strongly recommend you to really deploy the system after you really understand function of all files, otherwise there can be some troubles.


## AVP

D. Run the robot image server (robot terminal 3):
```
cd avp_teleoperate/teleop/image_server
python image_server.py
```

E. Run avp_teleoperate (Your PC terminal 1):

   ```bash
   # G1 (29DoF) Robot + Dex3-1 Dexterous Hand (Note: G1_29 is the default value for --arm, so it can be omitted)
   (tv) unitree@Host:~/avp_teleoperate/teleop$ python teleop_data_collecting.py --arm=G1_29 --hand=dex3 --record
   ```

