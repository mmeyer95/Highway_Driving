# Autonomous Highway Driving

The goal of this project was to safely navigate a 3-lane highway in simulation, without colliding with other cars or exceeding the speed limit of 50 mph. Lane changes should only be made when they are both safe and help the car (the "ego" car) progress through traffic. Acceleration should remain under 10 m/s^2, and jerk under 10 m/s^3. The simulator accepts trajectory points, in x & y coordinates, of the car's target position every 0.02 seconds.

<center><img src="https://i.ibb.co/LhpM5jD/HWDriving-Trajectory.png"></center>
<center><i>A screenshot of the simulator. Calculated trajectory points are shown in green.</i></center>

The process for successful implementation of a self-driving car consists of many steps happening at overlapping times, as displayed by the below graphic.

<center><img src="https://i.ibb.co/qJyNHLS/Process-overview.png" width="300"></center>
<center><i>Diagram of process flow from Udacity.com</i></center>

Sensor fusion, localization, and motion control are each completed in a separate project. For this purposes of this project, these 3 steps are already accounted for.
Therefore, the main task of the project can be split into 3 parts: <b>prediction</b>, <b>behavior planning</b>, and <b>trajectory generation</b>. It was accomplished in C++.

### Prediction
Prediction uses sensor fusion data to identify nearby cars and predict where they will be in the future. For the prediction step, since the simulation is run on a highway, I did not have to account for turns. Also, for the purpose of this project I assumed I would not have to account for sudden stops. The sensor fusion data includes the cars' x and y positions, in map coordinate, as well as the x and y components of their velocities. I used simple kinematics to calculate a predicted position and velocity for the other cars on the road at a future time t. Since I pass 1 second of trajectory data to the simulator at a time, t is less than or equal to 1 second (depending on how much time passes between execution of the code). 

It was also helpful in this step to use a different coordinate system, s & d. 

<center><img src="https://live.staticflickr.com/65535/47955424321_06b97d9469.jpg" width="500" height="269" alt="XY_sd"></center>
<center><i>Visualization of s&d vs x&y coordinate system from Udacity.com</i></center>

While x & y coordinates provide the absolute position on a map, s & d coordinates describe the position along a road, and the distance from the center line. This is helpful for calculating net travel distance, as well as which lane the car is in. Each lane is 4m wide, therefore the center of the first lane is located at d=2m. Likewise, lanes 2 and 3 are centered at d=6m and d=10m, respectively.

Mathematically, the prediction step involved the following equations:
<center><img src="https://live.staticflickr.com/65535/47955520731_3d1085bd24.jpg"></center>

where velocities are given in m/s, and positions are in terms of the coordinate "s". Predictions for the ego car were similarly calculated.

The output of the prediction step is used in the next step, behavior planning.

### Behavior Planning
Once I defined the presence of other cars, I decided on a move and car speed. I used boolean logic to determine if a right or left lane change was appropriate:

<code>bool left_move = current_lane>0 && (!car_left || ((ref_vels[current_lane-1] > ref_vels[current_lane]+4) && ref_dists[current_lane-1]>20)) && behaviors.back()!=1;</code>

<code>bool right_move = current_lane<2 && (!car_right || ((ref_vels[current_lane+1] > ref_vels[current_lane]+4) && ref_dists[current_lane+1]>20)) && behaviors.back()!=-1;</code>

The conditions for a lane switch, as defined by the above code are:
* A lane change would not put the ego car off the road
* There is no car in the target lane OR 
    * The neighboring lane is moving at 5 mph faster than the current lane AND the neighboring car is a safe distance away
* The ego car is not in the middle of a lane change in the opposite direction

<center><img src="https://i.ibb.co/YQ2XdFL/Behavior-Planning.jpg"></center>

Red spaces in the above graphic indicate positions of neighboring cars which would make a left or right lane shift not allowed. The target speed is ideally that of either: the car in front of me, or the maximum speed allowed. However, to avoid sudden large accels or decels, I only incremented the velocity, rather than setting it to the nearby car’s speed immediately.

The output of the behavior planning step for this project is the target lane (1, 2, or 3) and speed (<50 mph), then used in the next step.

### Trajectory Generation
After determining a behavior as a goal lane and move speed, the trajectory generation step determines the actual X & Y values for the trajectory of the car. To do so, I:

1. Find 5 points that the car will visit as it travels down the road towards its goal position
2. Define the 5 points in relative time and extrapolate those points with a spline function
3. Send the simulator the x & y points for each time step of 0.02 seconds

These steps are described below.

1. The 5 points I used as car visit points consist of 2 previous points and 3 future points. The previous points are the last x & y points from the previous path, if there are unprocessed points. If there is no previous path, likely at the start of the simulator, the previous points are the car’s current x & y plus one point behind the car’s current s value, in the same lane. The 3 future x & y points I defined as 30, 60, and 90 meters down the road in the middle of the target lane.

2. The timing of the points involve calculations. The most recent point is time 0, which is straight-forward. The timing of the remaining points must be based upon the car’s commanded velocity. I calculated the time stamp from the distance between the two points and the velocity at which the car should be travelling, based on: Time = Distance/ Speed. <br>The important part of this step is interpolation, as the simulator accepts positions in increments of 0.02 seconds. To find these exact points, I chose a spline function, which is a curve that guarantees passage through all of the given points. However, since the track is circular, 2 spline functions are necessary. Take, for example, a circle. The points along that circle look something like this:<br><center><img src="https://live.staticflickr.com/65535/47955659341_6a95600f12.jpg"></center><br>No matter how you connect the dots, there will be times when one of the values is increasing. The spline algorithm I used in C++ does not accept decreasing values. Therefore, I calculated 2 splines: one for x vs. time, one for y vs. time.

3. Finally, I sent the x & y points to the simulator. First, I filled the vectors with any remaining, un-processed points from the previous move. I always send the simulator 1 second, or 50 points, of data. Therefore, any open spots after the remaining points are added I filled with points from the splines calculated in step 2

You can see from the screenshots that the car decides to switch lanes when it makes sense.
<center><img src="https://i.ibb.co/THFCg5N/right-shift.png"><img src="https://i.ibb.co/WxKMPyM/left-shift.png"></center>
<center><i>Screenshots of left and right lane changes.</i></center>

[Watch the Video](https://www.youtube.com/embed/DQPjBo8q4xA)

