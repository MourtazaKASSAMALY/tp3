# Install ROS Melodic

http://wiki.ros.org/melodic/Installation/Ubuntu. 

Beware to choose the ”Desktop-Full Install” version.

To check if the setup was successful, launch the roscore program:

> roscore

You should see something like that:

> SUMMARY
> 
> \======================
> 
> PARAMETERS
> 
> \* / rosdistro : melodic
> 
> \* / rosversion : x . x . x
> 
> NODES
> 
> auto - starting new master
> 
> process [ master ]: started with pid [28153]
> 
> ROS_MASTER_URI = http :// votre_machine :11311/
> 
> setting / run_id to 30 fa6508 - fb20 -11 e5 -846 b - d4bed93065d2
> 
> process [ rosout -1]: started with pid [28166]
> 
> started core service [/ rosout ]

# Create the ROS Workspace

> mkdir -p ~/workspace_ros/src
> 
> cd ~/workspace_ros

Build the ROS environment using:
> catkin_make
> 
> echo "source ~/workspace_ros/devel/setup.bash" >> ~/.bashrc
>
> source ~/.bashrc

You should see in the ~/workspace_ros folder:
- a build folder
- a devel folder
- a logs folder
- a src folder

# Connect with GitHub

> cd ~/workspace_ros/src
> 
> git clone https://github.com/MourtazaKASSAMALY/tp3.git

You should see in the ~/workspace_ros/src a tp3 folder which directly connects to the tp3.git repository on GitHub. 

# Usage

Compile using:
 
> cd ~/workspace_ros
> 
> catkin_make
>
> roslaunch tp3 file.launch

# Work with Framagit

You want to push your modifications to the GitHub repository

> cd ~/workspace_ros/src/tp3
> 
> git add *
> 
> git commit -m "my_modifications"
> 
> git push

To pull files from the GitHub repository

> cd ~/workspace_ros/src/tp3
> 
> git pull
