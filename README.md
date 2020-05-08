# RoboDrawer

This **Matlab** script generates a simulated **UR5** manipulator from **Universal Robots**, capable of drawing given black and white 
pictures, by using [Peter Corke](https://github.com/petercorke) **machine vision** and **robotics** toolboxes. By the use of these, generates the mathematical model 
from the robot (forward and inverse kinematics) for its use for the drawing algorithm. This algorithm works by using a point matrix
approach isntead of parameterizing curves, which has advantages and disadvantages that we will discuss later.

With this logic, a connection to **CoppeliaSim** is stablished (through the **Remote CoppeliaSim-Matlab API**), in order to make
a more realistic implementation and every reached point is shown by the use of a "dummy", a graphical three-dimensional 
element from **CoppeliaSim**.

## Requirements

- Matlab 2010a or newer

- CoppeliaSim Matlab API (included in this project)

- Peter Corke's Robotics Toolbox v9.10 (included in this project)

- Peter Corke's Machine Vision Toolbox v4.3 (included in this project)

- CoppeliaSim v4 (not included), free educational version available at [CoppeliaSim's page](https://www.coppeliarobotics.com/)

- UR5 custom scene (included in this project)

- A high-end computer that can affords rough processes, since CoppeliaSim consumpts many resources.

## Preliminary steps

All the extra files are self-executable in the main script, except for the **Machine Vision Toolbox**, which has to be manually
installed. For this matter, all you need to do is click on open, in your main **Matlab** interface and select the **MVTB-4.3.mltbx**
file. A new window asking for installation allowing will show up, just click in **install** and we are ready to go.

## Instructions

1. With the CoppeliaSim's scene already opened, open the main file **robo_drawer_v3.m** in Matlab.

2. In the **coordinates = point_detector('C:\Users\**YourUser**\**YourRD3Folder**\RoboDrawer V3\Pictures\**B&WPicture**');** statement
(may change depending on your operating system), replace the black and white picture file for your own (or you can run the included 
samples). Remember that **must** be in Black and White, colored **will not work**. I strongly recommend to use pictures with well 
defined edges and without black colored areas (these last ones may work, but will take a lot of time in perform the drawing). The
scripts recognises must of the main picture files, such as jpg, jpeg, png, etc.

3. Now, just press the **play** button on Matlab's editor and we are done.

4. Back in CoppeliaSim's scene, the manipulator will start moving to the defined working plane. A Matlab figure will show up with 
the pointed Matlab plot from the picture.

5. Finally, you just need a lot of patience, since the arm will take from 20 to 45 minutes drawing (it is planned to improve this
in future releases), depending of the complexity of the picture and your hardware.

## Functioning

1. First, the script executes **startup_rvc**, which calls the robotics toolbox.

2. The scripts executes **ur5_coppelia_com**, which makes the connection through the Matlab remote API. Such script, was developed
based on this [video](https://www.youtube.com/watch?v=WaYBTA6QPY0) and the work developed by [Radica R](https://github.com/radica1113). It also builds some
necessary stuff for correctly handling the UR5 and some other components in CoppeliaSim.

3. Later, **point_detector** is executed. This function reads the given picture, reduces its size for a faster handling and reducing
the point rate and finally detects where are black-alike pixels and storages them in a matrix. Later there is an algorithm for making
offsets in x or y axis, depending on needs.

4. With the usage of robotics toolbox, the **SerialLink Object**, regarding to UR5 is built and the end-effector new point and working
plane is defined.

5. A resizing factor is multiplied by the obtained points from the picture.

6. The forward kinematics of the final position is calculated with **fkine**.

7. With the use of the API, the end-effector is moved to the desired position.

8. The rotation matrix and the position vector are calculated from the forward kinematics, in order to set the position of the frame
where the end-effector is going to place.

9. Now, it comes the point logic. Homogeneous Transformation Matrices are calculated, by using the points given by the matrix
built from the given picture and the desired end-effector orientation. The inverse kinematics are obtained by the use of "ikunc"
and the joints' positions regarding to these, storaged.

10. The manipulator is asked to move according to the storaged joints' positions. However, always, befor moving between points,
the arm is asked to keep in the previous point, but moving backwards in its working plane for avoiding drawing mistakes in real
life.

11. The entire process is repeated for each point in the picture.

12. Finally, the manipulator goes back to its regular setup.

**Notes:** A system for detecting when points calculted were reached by the arm was designed but did not work. Instead (bad practice)
Matlab pauses were stablished for this purpose. As it was already said some of the functions were retrieved from [Radica R](https://github.com/radica1113) and its
team, these were: **copyf**, which is used for copying dummies and other spacial stuff in CoppeliaSim and a part of **ur5_coppelia_com**,
which stablishes connection with the API. It is also replicated its usage of their function **movef**, but translated to use Robotics
Toolkit. All the remaining algorithms and logic were developed by me and my team (except from the functions belonging to the toolboxes).
We also designed our fixed version of the CoppeliaSim UR5 scene, adapted to behave as the Robotics Toolbox SerialLink object. The **vrchk**
function for checking the Remote API return status was implemented by [Radica R](https://github.com/radica1113), but he, at the same time, based on **Renaud Detry**
work.

## Parameters setup

As different experiments may be done, different setups may be set.

In **robo_drawer_v3.m**, by changing **v_final** automatically a different working plane will be set. Just take into account the 
orientation of the end-effector, which z-axis must be oriented to the working plane. And the fact that not all the working planes
will be reacheable.

In **point_detector**, according to your desired quality, you may change the **factor** variable. Keep verifying your results.

Also, in **point_detector** there is an offset section. This may or may not be used in your application. It depends on your
results in the manipulator and the resulting picture points. If you see your manipulator is not being capable of reaching the 
points without collapsing, you should make adjustments in these offsets, according to the manipulator's performance.

Related to this, some other variable that you might want to change is the **resize_factor** wich is the resize appliable in the
main file. 

All these parameters depend on your results. So you can freely experiment with these, according to your needs.

## Advantages against parameterizing approach

- With point matrix approach, you can make the maths approach much faster.

- You could draw really complex shapes, without having to analyze each single line in it.

- By changing parameters, you could get an extreme quality.

## Disadvantages against parameterizing approach

- Since there are going to be many points and the manipulator is not designed to move so fast, the drawing process may take too
much time.

- The algorithm logic is quite outdated, compared to some others.

## Results

Here, are some examples of the drawings I have tested.

![alt text](https://github.com/JMRMEDEV/RoboDrawer/blob/master/Pictures/bart.jpg "Input Picture: Bart")
![alt text](https://github.com/JMRMEDEV/RoboDrawer/blob/master/Pictures_results/bart.jpg "Drawing Result: Bart")

Video of the process: [bart](https://youtu.be/DJkrqklCV6Y)

![alt text](https://github.com/JMRMEDEV/RoboDrawer/blob/master/Pictures/catbug.jpg "Input Picture: Catbug")
![alt text](https://github.com/JMRMEDEV/RoboDrawer/blob/master/Pictures_results/catbug.jpg "Drawing Result: Catbug")

Video of the process: [catbug](https://youtu.be/kPVfW9TcqEc)

## Future improvements

- Algorithm for reducing drawing time
- Make some limits for joints
- Calculation of best joints' positions for reaching points
- Correcting reached points verification
