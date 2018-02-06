#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <algorithm>

#define MAX_VEL (49.7)
#define MIN_VEL (32)
#define REF_VEL (0.2)  //0.224, 0.224 will happened to jerk when change lane continuous.
#define LEFT  (0)
#define RIGHT (1)
#define FRONT_CONSTRAINT (20)
#define BACK_CONSTRAINT (14) //when change lane, d is at least 4m, so actually s is 10m

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

int findindexbyID(int car_id, vector<vector<double>>& sensor_fusion)
{
    int index = -1;
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
        if(car_id == (int)sensor_fusion[i][0])
        {
            index = i;
        }
    }

    if(index == -1)
    {
        cout << "error! cannot find index by id." << endl;
    }

    return index;
}

struct DistanceCmp
{
    bool operator()(std::pair<int, double>& e1, std::pair<int, double>& e2) const
    {
        return (e1.second < e2.second);
    }
};

vector<std::pair<int, double>> get_nearest_car_list(double car_s, double car_d, vector<vector<double>>& sensor_fusion, int prev_size)
{
    vector<std::pair<int, double>> nearest_cars;

    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double v_s = sensor_fusion[i][5];
        double v_d = sensor_fusion[i][6];

        v_s += prev_size * 0.02 * check_speed;

        double dist1 = sqrt((car_s-v_s)*(car_s-v_s) + (car_d-v_d)*(car_d-v_d));
        double dist2 = abs(car_s-v_s) + abs(car_d-v_d);
        double dist = (dist1 + dist2)/2.0;   //cost function, two heuristic function combined


        //debug
        cout << i << "  sensor_fusion, id = " << sensor_fusion[i][0] << " distance = " << dist << " v_s: " << v_s << " v_d: " << v_d << endl;

        if((v_s - car_s > 60) || (car_s - v_s > 40) || (v_d < 0)) //remove illegal value, only see s within 60m head or 40m backwards, and d>0
        {
            continue;
        }
        else
        {
            //only push <60m car's id and distance into vector
            nearest_cars.push_back(std::make_pair(sensor_fusion[i][0], dist));
        }

    }

    if(nearest_cars.size() > 1)
    {
        std::sort(nearest_cars.begin(), nearest_cars.end(), DistanceCmp()); //ascending order
    }

    cout << "----------------------------------------------------------------" << endl;
    for (int i = 0; i < nearest_cars.size(); i++)
    {
        cout << "--" << i << "  nearest_cars, id = " << nearest_cars[i].first << " distance = " << nearest_cars[i].second << endl;
    }

    return nearest_cars;
}


//only check both adjacent side of ego car.
vector<std::pair<int, int>> check_side_has_car_or_not(double car_s, double car_d, int lane, vector<vector<double>>& sensor_fusion, vector<std::pair<int, double>>& nearest_cars, bool& hascarflag, int prev_size)
{
    //with car's id and position at left/right flag
    vector<std::pair<int, int>> side_car_vec;

    if(nearest_cars.size() > 1) //verify the nearest car is at beginning, so the side car is also from nearest to farer
    {
        std::sort(nearest_cars.begin(), nearest_cars.end(), DistanceCmp()); //ascending order
    }

    // check nearest car
    for(int i = 0; i < nearest_cars.size(); i++)
    {
        // id & left/right flag
        std::pair<int, int> car_side = std::make_pair(nearest_cars[i].first, -1);

        int j = findindexbyID(car_side.first, sensor_fusion);

        double vx = sensor_fusion[j][3];
        double vy = sensor_fusion[j][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[j][5]; // s in frenet
        double check_car_d = sensor_fusion[j][6]; // d in frenet

        check_car_s += prev_size * 0.02 * check_speed;

        //if car in same lane, continue
        if(check_car_d < 4*lane+4 && check_car_d > 4*lane)
        {
            continue;
        }

        //in other lanes, d is 2.5-5.5m, s within 6m, 6m is a car's length gap //10m
        if( (fabs(car_d - check_car_d) <= 5.5 && fabs(car_d - check_car_d) >= 2.5) &&
                (((check_car_s - car_s  < FRONT_CONSTRAINT) &&  check_car_s >= car_s) ||    //check front ,in 25m is not safe for passing
                 (((car_s - check_car_s  < BACK_CONSTRAINT)) && car_s >= check_car_s)       //check behind
                )   //d < (4*lane+2+2) && d > 4*lane
          )
        {
            hascarflag = true;

            if( car_d > check_car_d)
            {
                car_side.second = LEFT; //other car is at left side
            }
            else if( car_d < check_car_d)
            {
                car_side.second = RIGHT;  //other car is at right side
            }

            side_car_vec.push_back(car_side); //already sort
        }

    }

    return side_car_vec;

}

bool checklanecar(int check_lane, double car_s, vector<vector<double>>& sensor_fusion, int prev_size)
{
    bool has_car = false;

    for(int i = 0; i < sensor_fusion.size(); i++)
    {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double v_s = sensor_fusion[i][5];
        double v_d = sensor_fusion[i][6];

        v_s += prev_size * 0.02 * check_speed;

        if((v_s - car_s > 60) || (car_s - v_s > 40) || (v_d < 0)) //remove illegal value, only see s within 90m head or 40m backwards, and d>0
        {
            continue;
        }

        if(v_d < 4*check_lane+4 && v_d > 4*check_lane)
        {
            if(v_s - car_s < FRONT_CONSTRAINT || car_s - v_s < BACK_CONSTRAINT) //front, in 25m, back in 20m
            {
                has_car = true;
            }
        }
    }

    return has_car;
}

int lane_change_no_side_car(int lane, double car_s, vector<vector<double>>& sensor_fusion, int prev_size)
{

    int to_lane = lane;

    //FSM
    switch (to_lane)
    {
    case 0:
    {
        to_lane = 1;   // (lane+1)%3;
        break;
    }
    case 1:
    {
        //change to left or right
        //if in 60m, lane 0 has car, lane 2 no car, to lane 2,
        //if in 60m, lane 2 has car, lane 0 no car, to lane 0,
        //else default left change;
        bool lane0_hascar = checklanecar(0, car_s, sensor_fusion, prev_size);
        bool lane2_hascar = checklanecar(2, car_s, sensor_fusion, prev_size);

        if(lane0_hascar == true && lane2_hascar == false)
        {
            to_lane = 2;
        }
        else if(lane0_hascar == false && lane2_hascar == true)
        {
            to_lane = 0;
        }
        else if(lane0_hascar == false && lane2_hascar == false)  //default
        {
            to_lane = 0; //left change
        }

        break;
    }
    case 2:
    {
        to_lane = 1;
        break;
    }
    default:
        break;
    }

    cout << "*** lane_change_no_side_car, from cur lane = " << lane << " to lane = " << to_lane << endl;

    return to_lane;
}

int lane_change_has_side_car(int lane, double car_s, vector<std::pair<int, int>> side_cars_vec, vector<vector<double>>& sensor_fusion, int prev_size)
{
    int to_lane = lane;

    // lane change FSM
    switch (to_lane)
    {
    case 0:
    {

        //only care about nearest car of lane 1
        for(int i = 0; i < side_cars_vec.size(); i++)
        {
            int v_id =  side_cars_vec[i].first;
            int index = findindexbyID(v_id, sensor_fusion);

            double vx = sensor_fusion[index][3];
            double vy = sensor_fusion[index][4];
            double check_speed = sqrt(vx*vx + vy*vy);

            double v_s = sensor_fusion[index][5];
            double v_d = sensor_fusion[index][6];

            v_s += prev_size * 0.02 * check_speed;

            if( ((v_s > car_s) && (v_s - car_s > FRONT_CONSTRAINT))  //front
                    || ((v_s < car_s) && (car_s - v_s > BACK_CONSTRAINT)) //back
              )  // || (car_s - v_s > 20))
            {
                to_lane = 1;
                break;
            }

            break;
            //default lane keep
        }

        break;
    }
    case 1:
    {
        //change to left or right
        //if side_cars_vec.size == 1, means only 1 car, only check 1 car.
        //if side_cars_vec.size > 1, means many cars, only check 2 car.
        if(side_cars_vec.size() == 0)
        {
            to_lane = 0; //default left change
            break;
        }

        if(side_cars_vec.size() == 1)
        {
            int v_id =  side_cars_vec[0].first;
            int index = findindexbyID(v_id, sensor_fusion);

            double vx = sensor_fusion[index][3];
            double vy = sensor_fusion[index][4];
            double check_speed = sqrt(vx*vx + vy*vy);

            double v_s = sensor_fusion[index][5];
            double v_d = sensor_fusion[index][6];

            v_s += prev_size * 0.02 * check_speed;

            if(side_cars_vec[0].second == LEFT)
            {
                to_lane = 2; //change lane to right
            }
            else if(side_cars_vec[0].second == RIGHT)
            {
                to_lane = 0;
            }

            break;
        }

        if(side_cars_vec.size() > 1)
        {
            int left_count = 0;
            int right_count = 0;

            for(int i = 0; i < side_cars_vec.size(); i++)
            {
                if(side_cars_vec[i].second == LEFT)
                {
                    left_count++;
                }
                else if(side_cars_vec[i].second == RIGHT)
                {
                    right_count++;
                }
            }

            //int changeto = (left_count > right_count)? 1:-1;

            if(left_count > right_count)
            {
                //loop right side car
                //if abs(car_s - v_s) > 20, change lane, lane+1
                for(int i = 0; i < side_cars_vec.size(); i++)
                {
                    if(side_cars_vec[i].second == RIGHT)
                    {
                        int v_id =  side_cars_vec[i].first;
                        int index = findindexbyID(v_id, sensor_fusion);
                        double vx = sensor_fusion[index][3];
                        double vy = sensor_fusion[index][4];
                        double check_speed = sqrt(vx*vx + vy*vy);

                        double v_s = sensor_fusion[index][5];

                        v_s += prev_size * 0.02 * check_speed;

                        if (((v_s > car_s) && (v_s - car_s > FRONT_CONSTRAINT))  //front
                            || ((v_s < car_s) && (car_s - v_s > BACK_CONSTRAINT)) //back
                            )
                        {
                            to_lane = 2;
                            break;
                        }

                        break;  //lane keep
                    }
                }
            }
            else   //left_count <= right_count
            {
                //loop left side car
                //if abs(car_s - v_s) > 20, change lane, lane+1
                for(int i = 0; i < side_cars_vec.size(); i++)
                {
                    if(side_cars_vec[i].second == LEFT)
                    {
                        int v_id =  side_cars_vec[i].first;
                        int index = findindexbyID(v_id, sensor_fusion);
                        double vx = sensor_fusion[index][3];
                        double vy = sensor_fusion[index][4];
                        double check_speed = sqrt(vx*vx + vy*vy);
                        double v_s = sensor_fusion[index][5];
                        v_s += prev_size * 0.02 * check_speed;

                        if (((v_s > car_s) && (v_s - car_s > FRONT_CONSTRAINT))  //front
                            || ((v_s < car_s) && (car_s - v_s > BACK_CONSTRAINT)) //back
                            )
                        {
                            to_lane = 0;
                            break;
                        }

                        break;  //lane keep

                    }
                }
            }
        }

        break;
    }
    case 2:
    {
        //only care about nearest car of lane 1
        for(int i = 0; i < side_cars_vec.size(); i++)
        {
            int v_id =  side_cars_vec[i].first;
            int index = findindexbyID(v_id, sensor_fusion);
            double vx = sensor_fusion[index][3];
            double vy = sensor_fusion[index][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double v_s = sensor_fusion[index][5];
            double v_d = sensor_fusion[index][6];

            v_s += prev_size * 0.02 * check_speed;

            if( ((v_s > car_s) && (v_s - car_s > FRONT_CONSTRAINT))  //front
                    || ((v_s < car_s) && (car_s - v_s > BACK_CONSTRAINT)) //back
              )  // || (car_s - v_s > 20))
            {
                to_lane = 1;
                break;
            }

            break;
        }

        break;
    }
    default:
        break;
    }

    cout << "*** lane_change_has_side_car, lane = " << lane << " to lane = " << to_lane << endl;

    return to_lane;
}

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    //start lane
    int lane = 1;
    // reference velocity
    double ref_vel = 0.0; //49.5;

    h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {

            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
                    int prev_size = previous_path_x.size();

                    if(previous_path_x.size() > 0)
                    {
                        car_s = end_path_s;
                        car_d = end_path_d;
                    }

                    bool too_close = false;
                    bool too_slow = false;
                    double front_car_vel = MIN_VEL;

                    //find legal nearest cars list, with cars' id and distance, within 60m front and 40 backwards, and d>0
                    vector<std::pair<int, double>> nearest_cars = get_nearest_car_list(car_s, car_d, sensor_fusion, prev_size);

                    //from nearest car and then far car.
                    //no need to care about all sensor_fusion data, only need to care about cars around ego car, and promote efficiency.
                    for(int i = 0; i < nearest_cars.size(); i++)
                    {
                        //cars' id
                        int car_id = nearest_cars[i].first;

                        //this is closest car's index
                        int index = findindexbyID(car_id, sensor_fusion);

                        //check current lane's all car's status, sensor_fusion: [id, x, y, vx, vy, s, d]
                        double vx = sensor_fusion[index][3];
                        double vy = sensor_fusion[index][4];
                        double check_speed = sqrt(vx*vx + vy*vy);
                        double check_car_s = sensor_fusion[index][5]; // s in frenet
                        double check_car_d = sensor_fusion[index][6]; // d in frenet
                        check_car_s += previous_path_x.size() * 0.02 * check_speed; //0.02s
                        front_car_vel = check_speed;

                        //when nearest car in same d lane
                        if( check_car_d < (4*lane+2+2) && check_car_d > 4*lane )
                        {
                            // check front car, <= 30m, and front car's velocity < ego car
                            if(check_car_s > car_s && (check_car_s - car_s <= 30) && check_speed < min(MAX_VEL, ref_vel))
                            {
                                too_close = true;

                                //prepare to change lane
                                //check side-way car, front 25m and back 20m, have car or not, if yes, stop pass, and reduce velocity
                                //if no, pass car.
                                bool has_car_flag = false;
                                //check sideway has car or not, return with side car's id and position relative to ego car. in 25m
                                vector<std::pair<int, int>> side_cars_vec = check_side_has_car_or_not(car_s, car_d, lane, sensor_fusion, nearest_cars,  has_car_flag, prev_size);

                                //debug
                                for(int i = 0; i<side_cars_vec.size(); i++)
                                {
                                    string position = (side_cars_vec[i].second == 0)?"left":"right";
                                    cout << "------00: " << "side_cars_vec, id = " << side_cars_vec[i].first << " position at: \t" << position << endl;
                                }

                                if (has_car_flag == true)
                                {
                                    lane = lane_change_has_side_car(lane, car_s, side_cars_vec, sensor_fusion, prev_size);
                                    break; //reduce velocity
                                }
                                else //in safe passing distance, but s distance in (25m, 30m)
                                {
                                    too_close = false;  // reset flag

                                    lane = lane_change_no_side_car(lane, car_s, sensor_fusion, prev_size);

                                    break;
                                }

                            }
                            else //if((check_car_s > car_s) && (check_car_s - car_s > 30))
                            {
                                too_close = false; //accelerate the velocity promote rate
                            }

                            //if s distance > 30m, lane keep.

                            // check behind and side car, in 6m,
                            if((check_car_s < car_s) && (car_s - check_car_s < BACK_CONSTRAINT))
                            {
                                //speed up
                                too_slow = true;
                                break;
                            }
                        }
                        else   //different d lane, adjacent lane d
                        {
                            too_close = false;
                            continue;
                        }

                    }

                    if(too_close == true)
                    {
                        if(ref_vel > MIN_VEL)
                        {
                            ref_vel -= REF_VEL; //0.224;
                        }
                    }
                    else if(ref_vel < MAX_VEL)
                    {
                        ref_vel += REF_VEL;//0.224;
                    }
                    //else if(ref_vel - MAX_VEL <= 0.224) // to avoid velocity vibrate
                    //{
                    //    ref_vel = MAX_VEL;
                    //}
                    else if(ref_vel >= MAX_VEL)
                    {
                        ref_vel -= REF_VEL; //0.224;
                    }
                    else if(too_slow == true)
                    {
                        if(ref_vel < MAX_VEL)
                        {
                            ref_vel += REF_VEL;//0.224;
                        }
                    }

                    cout << "----------------------------------------------------------------" << endl;
                    cout << "car_x:  " << car_x << "\t car_y:  " << car_y << endl;
                    cout << "car_s:  " << car_s << "\t car_d:  " << car_d << endl;
                    cout << "car_yaw:  " << car_yaw << "\t car_speed:  " << car_speed << endl;
                    cout << "previous_path_x size is:  " << previous_path_x.size() << endl;
                    cout << "sensor_fusion size is:  " << sensor_fusion.size() << endl;
                    cout << endl;

                    vector<double> ptsx;
                    vector<double> ptsy;

                    //referenct for starting points
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    if(previous_path_x.size() < 2)
                    {
                        // construct two points to make path tangent to car
                        double prev_car_x = car_x - 1 * cos(car_yaw);
                        double prev_car_y = car_y - 1 * sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);

                    }
                    else
                    {
                        // use previous path end points to make path
                        ref_x = previous_path_x[previous_path_x.size()-1];
                        ref_y = previous_path_y[previous_path_x.size()-1];

                        double prev_path_x2 = previous_path_x[previous_path_x.size()-2];
                        double prev_path_y2 = previous_path_y[previous_path_x.size()-2];

                        ref_yaw = atan2(ref_y-prev_path_y2, ref_x-prev_path_x2);

                        ptsx.push_back(prev_path_x2);
                        ptsx.push_back(ref_x);
                        ptsy.push_back(prev_path_y2);
                        ptsy.push_back(ref_y);

                    }

                    vector<double> next_wp0 = getXY(car_s+30, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s+60, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s+90, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    //convert waypoints from map coordinate to car coordinate to make math simple
                    for(int i = 0; i < ptsx.size(); i++)
                    {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;
                        ptsx[i] = shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw);
                        ptsy[i] = shift_x * (-sin(ref_yaw)) + shift_y * cos(ref_yaw);
                    }

                    // use spline to fit car points
                    tk::spline stk;
                    stk.set_points(ptsx, ptsy);

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // start with the previous path points from last time
                    for(int i = 0; i < previous_path_x.size(); i++)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    //calculate spline x, y, d and number of points.
                    double target_x = 30.0;
                    double target_y = stk(target_x);
                    double target_dist = sqrt(target_x*target_x + target_y*target_y);

                    double x_add_on = 0;
                    double num_n = target_dist / (0.02 * ref_vel * 0.447); //mph to m/s : 1/2.24 or 0.447

                    //fill with the rest of our path planner after filling with previous points, always output 50 points
                    for(int i = 0; i < 50 - previous_path_x.size(); i++)
                    {

                        double x_point = x_add_on + target_x/num_n;
                        double y_point = stk(x_point);

                        x_add_on = x_point;
                        double x_ref = x_point;
                        double y_ref = y_point;

                        //rotate back to map coordinate.
                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);

                    }

                    json msgJson;
                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t)
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}