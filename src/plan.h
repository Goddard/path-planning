#ifndef PLAN_H
#define PLAN_H

#include "spline.h"
#include <map>
#include <queue>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

double max_s = 6945.554;

// start in lane 1
int lane = 1;

// have a reference velocity to target
double ref_vel = 0.0; // mph

int prev_size = previous_path_x.size();

if ( prev_size > 0 ) {
    car_s = end_path_s;
}

bool too_close = false;

// find rev_v to use
for ( int i = 0; i < sensor_fusion.size(); i++ ) {
    //left to right lanes 0, 1, 2
    bool left = false;
    bool front = false;
    bool rear = false;
    bool right = false;
    
    bool lane_one = true;
    bool lane_two = true;
    bool lane_three = true;
    
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt ( vx * vx + vy * vy );
    double check_car_s = sensor_fusion[i][5];
    
    // car is in my lane
    float d = sensor_fusion[i][6];
    
    int oc_lane = -1;
    for ( int i = 0; i < 3; i++ ) {
        if ( d < ( 2 + 4 * i + 2 ) && d > ( 2 + 4 * i - 2 ) ) {
            oc_lane = i;
            // is the car in i lane ahead of us && is it less then 50 meters
            
            // need to change this to create zones around car to determine if any where around the car is blocked
            // since we don't have some kind of radar this is the best method to avoid accidents i think :D
            double safe_distance = distance( car_x, car_y, vx, vy );
            
            cout << "distance " << safe_distance << endl;
            
            if ( safe_distance < (car_x - 200)  &&  safe_distance > (car_x + 200)) {
                //                                 if( ( ( sensor_fusion[i][5] - car_s ) > 50 ) ) { //sensor_fusion[i][5] > car_s ) && (
                if ( oc_lane == 0 ) {
                    lane_three = true;
                    cout << ", lane 0 " << lane_three;
                }
                
                if ( oc_lane == 1 ) {
                    lane_two = true;
                    cout << ", lane 1 " << lane_two;
                }
                
                if ( oc_lane == 2 ) {
                    lane_one = true;
                    cout << ", lane 2 " << lane_one;
                }
                
                cout << endl;
                
            } else {
                if ( oc_lane == 0 ) {
                    lane_three = false;
                }
                
                if ( oc_lane == 1 ) {
                    lane_two = false;
                }
                
                if ( oc_lane == 2 ) {
                    lane_one = false;
                }
            }
        }
    }
    
    //                         if(car_s < sensor_fusion[i][5]) {
    //                             cout << "lane : " << lane << endl;
    //                             cout << "pos : " << ( 2 + 4 * lane + 2 ) << endl;
    //                             cout << "car : " << car_s << ", " << car_d << endl;
    //                             //                             cout << "other car : " << sensor_fusion[i][3] << ", " << sensor_fusion[i][4] << endl << endl;
    //                             cout << "other car lane : " << oc_lane << endl;
    //                             cout << "other car : " << sensor_fusion[i][5] << ", " << sensor_fusion[i][6] << endl << endl;
    //                         }
    
    if ( d < ( 2 + 4 * lane + 2 ) && d > ( 2 + 4 * lane - 2 ) ) {
        // if using previous points can project s value out
        check_car_s += ( ( double ) prev_size * 0.02 * check_speed );
        
        //check s values greater than mine and s gap
        if ( ( check_car_s > car_s ) && ( ( check_car_s - car_s ) < 30 ) ) {
            ref_vel -= 0.224;
            
            too_close = true;
            
            cout << "lane, l1, l2, l3 : " << lane << ", " << lane_one << ", " << lane_two << ", " << lane_three << endl << endl;
            
            // check left lane and see if we can move over
            // if not lets move to another
            if ( lane == 0 ) {
                if ( lane_two ) {
                    lane = 1;
                }
                continue;
            }                                
            
            else if ( lane == 1 ) {
                if ( lane_three ) {
                    lane = 0;
                    cout << "lane 1 move to 0" << endl;
                    continue;
                }
                
                else if ( lane_one ){
                    lane = 2;
                    cout << "lane 2 move to 1" << endl;
                    continue;
                }
            }
            
            else if ( lane == 2 ) {
                if ( lane_two ) {
                    lane = 1;
                }
                continue;
            }
        }
    }
}

if ( too_close ) {
    ref_vel -= 0.224;
}

else if ( ref_vel < 49.5 ) {
    ref_vel += 0.224;
}

vector<double> ptsx;
vector<double> ptsy;

// reference x, y, yaw states
// either we will reference the starting point as where
// the car is or at the previous paths end point
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad ( car_yaw );

// if previous size is almost empty, use the car as starting reference.
if ( prev_size < 2 ) {
    // use two points that make the path tangent to the car
    double prev_car_x = car_x - cos ( car_yaw );
    double prev_car_y = car_y - sin ( car_yaw );
    
    ptsx.push_back ( prev_car_x );
    ptsy.push_back ( prev_car_y );
    
    ptsx.push_back ( car_x );
    ptsy.push_back ( car_y );
}

// use the previous paths end point as starting reference
else {
    // redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2 ( ref_y - ref_y_prev, ref_x - ref_x_prev );
    
    // use two points that make the path tangent to the previous paths end point
    ptsx.push_back ( ref_x_prev );
    ptsy.push_back ( ref_y_prev );
    
    ptsx.push_back ( ref_x );
    ptsy.push_back ( ref_y );
}

// In Frenet add evenly 30m space points ahead of the starting reference
vector<double> next_wp0 = getXY ( car_s + 30, ( 2 + 4 * lane ), map_waypoints_s, map_waypoints_x, map_waypoints_y );
vector<double> next_wp1 = getXY ( car_s + 60, ( 2 + 4 * lane ), map_waypoints_s, map_waypoints_x, map_waypoints_y );
vector<double> next_wp2 = getXY ( car_s + 90, ( 2 + 4 * lane ), map_waypoints_s, map_waypoints_x, map_waypoints_y );

ptsx.push_back ( next_wp0[0] );
ptsy.push_back ( next_wp0[1] );

ptsx.push_back ( next_wp1[0] );
ptsy.push_back ( next_wp1[1] );

ptsx.push_back ( next_wp2[0] );
ptsy.push_back ( next_wp2[1] );

for ( int i = 0; i < ptsx.size(); i++ ) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    
    ptsx[i] = ( shift_x * cos ( 0 - ref_yaw ) - shift_y * sin ( 0 - ref_yaw ) );
    ptsy[i] = ( shift_x * sin ( 0 - ref_yaw ) + shift_y * cos ( 0 - ref_yaw ) );
}

// create spline
tk::spline s;

// set(x, y) points to the spline
s.set_points ( ptsx, ptsy );

// define the actual (x, y) points we will use for the Planner
vector<double> next_x_vals;
vector<double> next_y_vals;

// start with all of the previous path points from the last time
for ( int i = 0; i < previous_path_x.size(); i++ ) {
    next_x_vals.push_back ( previous_path_x[i] );
    next_y_vals.push_back ( previous_path_y[i] );
}

// Calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30.0;
double target_y = s ( target_x );
double target_dist = sqrt ( ( target_x ) * ( target_x ) + ( target_y ) * ( target_y ) );

double x_add_on = 0;

// Fill up the rest of our path plannner after filling it with previous points here we will output 50 points
for ( int i = 1; i <= 50 - previous_path_x.size(); i++ ) {
    double N = ( target_dist / ( 0.02 * ref_vel / 2.24 ) );
    double x_point = x_add_on + ( target_x ) / N;
    double y_point = s ( x_point );
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // cout << x_ref << ", " << y_ref;
    
    // rotate back to normal after rotating it earlier
    // basis transform?
    x_point = ( x_ref * cos ( ref_yaw ) - y_ref * sin ( ref_yaw ) );
    y_point = ( x_ref * sin ( ref_yaw ) + y_ref * cos ( ref_yaw ) );
    
    x_point += ref_x;
    y_point += ref_y;
    
    next_x_vals.push_back ( x_point );
    next_y_vals.push_back ( y_point );
}
