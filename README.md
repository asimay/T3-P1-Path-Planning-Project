# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

----------------

[//]: # (Image References)
[image1]: ./output/final.gif
[image2]: ./output/trajectory.PNG
[image3]: ./output/1.PNG
[image4]: ./output/2.PNG
[image5]: ./output/best.PNG

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


#### Final result:

![alt text][image1]


## Procedure

### 1. Get the car's localization data, and construct previous path data.

Above step will compensate the future path according to vehicle's velocity.

the highway data with drawing as belows:

![alt text][image2]

Above pic, green point is start point, the blue point is end point of map waypoints.

### 2. Get nearest car lists around ego car.

In this part, I didn't follow the original `sensor_fusion` loop code thoughts of project, because actually, we only need to care about the nearest car around ego car, so we just loop between nearest car list, this will also remove the invalid sensor fusion data which is actually very far away if you look into the data, and this also promote the handle speed of our program.

Also, in nearest car list, we only look ahead 60m and 40m backwards first, and with all frenet `d` data should be big than 0.

Function `get_nearest_car_list()` illustrate the thoughts of step 2. 

Use the nearest_car_list to speed up the progress speed.

### 3. Handle the lane change and behavior planning algothrim.

First, check the car in same `d` lane, this only check the car of front or behind ego car, if front/behind cars are in the same lane, and their speed is small than our ego car, this need us to take lane change actions.

Before lane change, we need to check ego car's both sides has other cars or not. This is equivalent to prepare lane change procedure, as `PLCL` or `PLCR`. Before lane change, I used function `check_side_has_car_or_not()` to check both sideway has car or not, and remember whether left lane or right lane has car, this will help us decide which way is a better way to do lane change afterwards.

Also, I wrote some lane change code, which is different from FSM in udacity course, but the basic core thoughts are same. This can be find in function `lane_change_has_side_car()` and  function `lane_change_has_side_car()`, actually this is a simple FSM part, which determine the which side to do lane change. In the lane change steps, ego car also check 25m in front and 20m behind of pretended lane, in case of collisions. This part is more robust than demo exercises provided. About safe and fast, there is a balance between them, this is an art to adjust the distance parameters to behave more safe or more fast.

Following is the parameters config, especially for `REF_VEL`, in classroom courses, which is defined to 0.224, but in some cases, if ego car do lane change continuous twice, this will exceed max jerk limitation, so decrease this value to 0.2 can solve the problem very well. Also, during lane change step, we only care about distance 20m front or 14m behind of our ego car, this is a safe distance to do lane change despite of there is a car 30m in front of ego car. This method can greatly promote lane change efficience, rather than waiting for behind car to pass. 

```cpp

#define MAX_VEL (49.7)
#define MIN_VEL (32)
#define REF_VEL (0.2)  //0.224, 0.224 will happened to jerk when change lane continuous.
#define LEFT  (0)
#define RIGHT (1)
#define FRONT_CONSTRAINT (20)
#define BACK_CONSTRAINT (14) //when change lane, d is at least 4m, so actually s is 10m

```

I also define some states, like `too close` and `too slow` and other velocity limit states, this can help ego car to decrease speed not too much, with these status flags, I can limit the ego car to decrease velocity only to 30-32 mph, if without these condition, the ego car will always decelerate to about 20 mph, this is too slow for ego car.

some screenshots of code running, all is running at the max velocity limitation:


this is the first pic of running:


![alt text][image3]


the cmd output the sensor fusion data and nearest car list data, and other lane change info.


![alt text][image4]


this is the best running result. successfully running 28 miles without incident.


![alt text][image5]


### 4. Use spline to fit the path.

This part is the same as instructions of classroom. use 2 points to compose a path, and use frenet coordinate transform to or from map x,y coordinate, and then push all the fit points into `ptsx/ptsy`. this step including local transform between map and car coordinate.

### 5. other specs:

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 


## Tips

The original Frenet to x,y transform is not very accurate, so I used code from ericlavigne, who is live mentor from slack of term 3, his code provide more accuracy of transform, especially when car is at curve turning places.

## Further to promote

During testing, I find actually when there are many many cars around ego car, and algothrim still has more spaces to promote, I understand that in real case, it is really hard to do self-driving lane change. 

Anyway, from this project, I learned a lot more than I expected, I code my own style of lane change logic, which was I deemed as impossible before, and solved many problems during this project.

