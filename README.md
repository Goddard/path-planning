### Goals
In this project my goal was to safely navigate around a virtual highway. To do that a few constraints were also placed on the simulation.  They are listed below.
 1. Don't crash, that includes accidents where vehicles rear end you.
 2. Don't break the speed limit.
 3. Don't Accelerate too quickly.
 4. Don't jerk the car about too vigorously.
 6. Make lane changes.
 7. Drive about 4.5 miles at least.
 
 
### Considerations
To complete this project I first tried to define the simpliest path to my goals.  Maybe recommendations were given, but the task seemed easy enough without using overly complicated methods.  First I made sure I understood the data that was given from the simulator and played around with distance measurements.  Since I had the ability to use Frenet coordinate system it made space calculations that much easier.  I could check if a car was on the left, right, or in front of my very easily.  After that it was just a matter of defining the correct logic to make sure the vehicle only turned at approaiate times on its simulated highway.

### Implimentation Details
In order to accomplish the task without jerking about I had to make sure the car took the waypoints given and smooth them out.  A couple methods to do this were suggested, but using a spline was the one I used.  Before the waypoints could be used though I had to define lane information that was important for lane change considerations.  I first calculated distances of cars in the lanes around my car and also in front of me.  Using a simple boolean flag I triggered the turn even to process as long as a few condidtions were meet.  First I wanted to make sure we were traveling at a good speed, meaning not too slow and not too fast.  I made sure our event took place where a lane change was warranted, meaning we were our max_s distance away from a car ahead of the vehicle which I set to 30.  After those conditions were meet I made sure we don't make any other changes during the step process until a lane change has completed and we are a safe distance away along with speeding up to a safe speed.  At this point I took reference points of where my car was and plotted a tangential set of points along the longitudinal axis.  To do this I made sure when starting if our vector was too small to just use some simulated points from our starting position and vehicle yaw after that I could draw on previous and projected points about 30 "meter" increments apart.  Once the tanget points were created I could feed it into the spline to smooth out or points.  Once my spline points were created I broke them up using the same 30 "meters" and using a Euclidean distance formula.  At this point I rotated the points back from zero using covariant transformation.  Then we just send our generated line off to the simulator and watch the car make it's way around and around.

### Conclusion
This method is far from perfect and it could no doubt get into accidents from reckless cars on the road.  I think more considerations could be taken into account in a real world scenario and using costs for these situations might be more useful.  This method is just robust enough to past this simulation with the constraints given.

### Data Details

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Acknowledgements

Smoothing trajectories was made easier using splines and a header file from http://kluge.in-chemnitz.de/opensource/spline/

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
