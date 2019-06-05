---
id: network
title: Networking
sidebar_label: Networking
---


## Networking Setup

This is a tutorial for setting up the networking so that you can connect your **desktop** to the **NUC** machine on the mobile robot.

---

### Method 1

This method allows you to make use of the local machine to control the robot. In this mode, the NUC machine on the robot is sending the robot information to the local machine. And most of the jobs (especially those that require lots of computation) are done in the local machine. The local machine and the NUC machine will be on the same network and they share the robot information. So you will run all the robot driver or PyRobot code on your local machine (hence, it can be much faster and it also means that you should have **all the software setup in your local machine**), and then the commands are sent to the NUC machine to control the robot.

#### Step 1: Connect the desktop and the NUC to a router

To connect the desktop to the NUC machine, we suggest using a **wireless router**. And your desktop and the NUC should be connected to the router individually.

#### Step 2: Determine the IP address of the desktop and the NUC

Use the command `ifconfig` to get the IP address for the desktop (**IP_DESKTOP**) and the NUC (**IP_NUC**). The ip address should be something like `192.168.0.100`.

Then check the connectivity between the desktop and the NUC machine.

On the desktop, run
```bash
ping <IP_NUC>   
```

On the NUC machine, run
```bash
ping <IP_DESKTOP> 
```

#### Step 3 Setup the NUC machine

On the NUC machine, run (replace the **IP_NUC** with the actual IP value):
```bash
echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
echo export ROS_HOSTNAME=IP_NUC >> ~/.bashrc
```

#### Step 4 Setup the Desktop

On the destop, run (replace the **IP_DESKTOP** with the actual IP value):
```bash
echo export ROS_MASTER_URI=http://IP_NUC:11311 >> ~/.bashrc
echo export ROS_HOSTNAME=IP_DESKTOP >> ~/.bashrc
```

#### Step 5: Test connection

Verify from the NUC machine to the desktop

On the NUC machine, run (**Send message**):
```bash
rostopic pub -r10 /hello std_msgs/String "hello"
```

On the desktop, run (**Display message**):
```bash
rostopic echo /hello
```
If the connection is successful, you should see output as shown below on the desktop:
```bash
data: "hello"
---
data: "hello"
---
data: "hello"
---
```

You can verify the connection from the desktop to the NUC machine in the same way (**Send message** on the desktop and **Display message** on the NUC machine).

---

### Method 2

You can also just use `ssh` to connect to the NUC machine from your local machine. And then run the code on the NUC machine. So in this mode, all the computation is performed in the NUC machine and you don't need to setup the software in your local machine.
