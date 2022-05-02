# golf-smart :golf:


The goal of all the scripts we made to control the robot is to make one time array and positon and velocity arrays for each joint. What is actually sent to the robot is a series of waypoint goals:<br>
``` {[S_pos,L_pos, ...], time (float), [S_vel,L_vel, ...]}```<br>
The robot will get to that position at that time with that instaneous velocity. Between waypoints I don't know what is happening, but I would guess it is solving some boundary conditons problem between two waypoints. If the time intervals are close together and so are the positions and velocities, you can achieve smooth controlled motion. For motion of any kind, all you need to do is build these arrays and send them to the robot. 


### Scripts Functionality Descriptions:

|Script | Description | 
 | ----|-----|
 |[swing_superPoints_wfigures.py](scripts/swing_superPoints_wfigures)| Base swing model. Exactly follows the Super Points which shape the base swing.<br>Builds trajectory using waypoints and executes motion to robot.|
 |[swing_centerPointVariations.py](scripts/swing_centerPointVariations.py)| Load in `CenterPoints.csv` file where each row is a swings center (ball impact) point. Loops through all and executes each swing on robot.|
 |[generate_CenterPoints.py](scripts/generate_CenterPoints.py)| Generate ~100 different center points and export as a csv file.<br>Each center point is a semi-random variation (joint-space) on the SuperPoints swing.|
 |[buildSwingPath_Gauss.py](scripts/buildSwingPath_Gauss.py)| :file_folder: Generates, plots, executes a swing using the **Gaussian** curve model. |
 |[buildSwingPath_Tophat.py](scripts/buildSwingPath_Tophat.py)| :file_folder: Generates, plots, executes a swing using the **TopHat** curve model. |
 

### Examples for what the Gauss and Tophat models ouput:
 
| Gauss Model | TopHat Model | 
| ----|-----|
|![image](/docs/pathPlan_GaussModel.png)|![image](/docs/pathPlan_TopHatModel.png)|


### Misc Notes on Each File:
 
`swing_superPoints_wfigures.py`
  - This script is less commented, but it does have some nice plots. So if you'd like to see them or move the plotting to another script, they can be found here

`swing_centerPointVariations.py`
  - The robot class as well as the drawRobot2 and makeTime functions are in this script 
  - Next the CenterPoints.csv file is read in as "Matrix"
  - The while loop then asks the user for an index input or just enter will select the next index (Know that idx = 10 will be excel's row 9 etc.) 
  - Each loop, the robot swings that particular path and asks for user input again

`generate_CenterPoints.py`
  - At the top are the "Super Points" that outline the base swing. The interp function just linearly interpolates them (in joint space) to be 90 points
  - The robot class as well as the drawRobot2 functions are in this script. The visualization here can be helpful so if you want to see the hill climb,  use the plot3D 
    function to plot the vectors from drawRobot2. I eliminated that part for this script because I was told I made too many graphs.
  - Then a vector of angles + or – 5 degrees from the base swing is created for B and T. The number of steps in the range will decide how many center points are made. 
    This is order 2 though so 10  steps makes 100 center points and 20 would make 400. This can take 5 minutes or so for about 100 center points. 
  - For each angle a hill climb is performed to put the end of the club back at the correct location. 
  - These center point values for [S,L,U,R,B,T] are saved and written to a csv file 
  
  *IMPORTANT NOTE*: The angles convention for the robot is not zeroed as a straight aligned vector. 
  For this reason, the anglesDesired would take say [0,0,0,0,0,0] and the anglesConvention switches these to the proper convention of the robot where it will be in the normal robot zeroed position. 



---
### 2021-2022 Project Closing Comments:
 
If the end goal is some kind of very accurate machine learning based chipping robot, I think there are going to be few important things to consider:
 - The hardware needs to be really consistent and repeatable. The way the club is loaded into the holder is currently based on judgment in terms of 
   how the club face is angled. This really should be exact and repeatable or data collected two different days won't agree. 
 - I think it would also be worthwhile to go back to the "base" swing and spend some time deciding exactly where the tee should be for that shot. Ideally
   That would be the perfect straight shot and everything else would be some variation from this. I don't think we spent quite enough time to be precise on this.
 - From the machine learning perspective, it is going to be important to get more reliable shot data for both training and validation. 
 - Also from the machine learning side, it is important to remember that the hill climb inverse kinematics has some random aspect to it so two hill climbs will not
   always find the same minumum. Whatever learned policy that maps desired shot charateristics to swing needs to output center point angles for all six angles
   instead of just B and T and hill climb to find the others. 

 
