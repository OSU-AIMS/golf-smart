# golf-smart



The goal of all the scripts we made to control the robot is to make one time array and positon and velocity arrays for each joint. What is actually sent to the 
robot is a series of waypoint goals {[S_pos,L_pos, ...], time (float), [S_vel,L_vel, ...]}. The robot will get to that position at that time with that instaneous
velocity. Between waypoints I don't know what is happening, but I would guess it is solving some boundary conditons problem between two waypoints. If the time 
intervals are close together and so are the positions and velocities, you can achieve smooth controlled motion. For motion of any kind, all you need to do is 
biuld these arrays and send them to the robot. 

Description of all the scripts:

|Script | Description | 
 | ----|-----|
 |DifferentCenterSwings.py|This is the most recent code that is used to loop through the points in CenterPoints.csv and swing the robot in all 100 different swings|
 |MakeDifferentCenterFile.py|The point of this script is to generate the 100 or whoever many different center points and print them to a csv file to be read in later|
 |makeSwing.py|This is an old script that does the same thing as DifferentCenterSwings.py but for only the "Base" Swing.|
 |buildSwingPath_Gauss.py| This script runs the old Gaussian curve model for a swing. |
 |buildSwingPath_Tophat.py| This script runs the old Tophat model for a swing.|
 
 Examples for what the Gauss and Tophat models ouput:
 
 |Model | Output | 
 | ----|-----|
 |Gauss|<img src="https://github.com/OSU-AIMS/golf-smart/blob/refactor/docs/pathPlan_GaussModel.png?raw=true" width="500">|
 |Tophat|<img src="https://github.com/OSU-AIMS/golf-smart/blob/refactor/docs/pathPlan_TopHatModel.png?raw=true" width="500">|


Extra Notes:
 
DifferentCenterSwings.py
  - The robot class as well as the drawRobot2 and makeTime functions are in this script 
  - Next the CenterPoints.csv file is read in as "Matrix"
  - The while loop then asks the user for an index input or just enter will select the next index (Know that idx = 10 will be excel's row 9 etc.) 
  - Each loop, the robot swings that particular path and asks for user input again

MakeDifferentCenterFile.py
  - At the top are the "Super Points" that outline the base swing. The interp function just linearly interpolates them (in joint space) to be 90 points
  - The robot class as well as the drawRobot2 functions are in this script. The visualization here can be helpful so if you want to see the hill climb,  use the plot3D 
    function to plot the vectors from drawRobot2. I eliminated that part for this script because I was told I made too many graphs.
  - Then a vector of angles + or – 5 degrees from the base swing is created for B and T. The number of steps in the range will decide how many center points are made. 
    This is order 2 though so 10  steps makes 100 center points and 20 would make 400. This can take 5 minutes or so for about 100 center points. 
  - For each angle a hill climb is performed to put the end of the club back at the correct location. 
  - These center point values for [S,L,U,R,B,T] are saved and written to a csv file 
  
  IMPORTANT NOTE: The angles convention for the robot is not zeroed as a straight aligned vector. 
  For this reason, the anglesDesired would take say [0,0,0,0,0,0] and the anglesConvention switches these to the proper convention of the robot where it will be in 
  the normal robot zeroed position. 

makeSwing.py
  - This script is less commented, but it does have some nice plots. So if you'd like to see them or move the plotting to another script, they can be found here

Additional Notes on the Project:
 
If the end goal is some kind of very accurate machine learning based chipping robot, I think there are going to be few important things to consider:
 - The hardware needs to be really consistent and repeatable. The way the club is loaded into the holder is currently based on judgment in terms of 
   how the club face is angled. This really should be exact and repeatable or data collected two different days won't agree. 
 - I think it would also be worthwhile to go back to the "base" swing and spend some time deciding exactly where the tee should be for that shot. Ideally
   That would be the perfect straight shot and everything else would be some variation from this. I don't think we spent quite enough time to be precise on this.
 - From the machine learning perspective, it is going to be important to get more reliable shot data for both training and validation. 
 - Also from the machine learning side, it is important to remember that the hill climb inverse kinematics has some random aspect to it so two hill climbs will not
   always find the same minumum. Whatever learned policy that maps desired shot charateristics to swing needs to output center point angles for all six angles
   instead of just B and T and hill climb to find the others. 

 
