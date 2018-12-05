
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include "object.cpp"
#include <obstacle_detection/Obstacle.h>
#include <object_identification/ObjectList.h>
#include <std_msgs/Bool.h>

#define LOAD_MAP_FROM_FILE 1
#define RESOLUTION 0.02
#define USE_RAY_TRACE 1
#define ADD_OBSTACLES 1


class MapNode
{

private:
    nav_msgs::OccupancyGrid map_msg;
    object_identification::ObjectList object_msg;
    int nRows;
    int nColumns;
    double map_resolution;
    double object_size;
    bool isMoving;
    bool obstacleInFront;

public:
    ros::NodeHandle n;
    ros::Publisher pub_gridmap;
    ros::Publisher pub_known_objects;
    ros::Subscriber sub_known_objects;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_obstacle;
    ros::Subscriber sub_position;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_wall;
    tf::TransformListener *tf_listener;
    std::vector<ValuableObject> foundObjects;



    MapNode(ros::NodeHandle node, double width,double height, double res)
    {
        //ValuableObject myobj(1,1,1);
        n = node;
        tf_listener = new tf::TransformListener();

        map_resolution = res; //Every element corresponds to a res*res cm area
        object_size = 0.05; // side length of square object
        nColumns = (int) round((width/map_resolution)+1); // +0.5
        nRows = (int) round((height/map_resolution)+1); // +0.5
        isMoving = 1;
        obstacleInFront = 0;
        ROS_INFO("Grid map rows: %i, cols: %i",nRows,nColumns);

        map_msg.data.resize((nRows)*(nColumns));
        map_msg.header.frame_id = "/map";
        map_msg.info.resolution = map_resolution;
        map_msg.info.height = nRows;
        map_msg.info.width = nColumns;


        pub_gridmap = n.advertise<nav_msgs::OccupancyGrid>("/grid_map",1);
        pub_known_objects = n.advertise<object_identification::ObjectList>("/known_objects",1);
        sub_vel = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist", 1, &MapNode::velocityCallback,this);
        sub_known_objects = n.subscribe<object_identification::ObjectList>("/known_objects",1,&MapNode::objectCallback,this);
        sub_laser = n.subscribe<sensor_msgs::LaserScan>("/scan_modified",1,&MapNode::laserCallback,this);
        sub_obstacle = n.subscribe<obstacle_detection::Obstacle>("/found_obstacle_perception",10,&MapNode::obstacleCallback,this);
        sub_position = n.subscribe<nav_msgs::Odometry>("/particle_position",1,&MapNode::positionCallback,this);
        sub_wall = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &MapNode::wallCallback,this);
        pub_known_objects = n.advertise<object_identification::ObjectList>("/saved_objects",1);
    }

    MapNode(ros::NodeHandle node)
    {
        n = node;
        tf_listener = new tf::TransformListener();

        loadMap();

        isMoving = 1;
        obstacleInFront = 0;
        map_resolution = map_msg.info.resolution;
        object_size = 0.05; // side length of square object
        nColumns = map_msg.info.width;
        nRows = map_msg.info.height;
        ROS_INFO("Grid map rows: %i, cols: %i",nRows,nColumns);


        pub_gridmap = n.advertise<nav_msgs::OccupancyGrid>("/grid_map",1);
        pub_known_objects = n.advertise<object_identification::ObjectList>("/saved_objects",1);
        sub_vel = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist", 1, &MapNode::velocityCallback,this);
        //sub_objectsToAdd = n.subscribe<geometry_msgs::PointStamped>("/found_object",1,&MapNode::objectCallback,this);
        sub_known_objects = n.subscribe<object_identification::ObjectList>("/known_objects",1,&MapNode::objectCallback,this);
        sub_laser = n.subscribe<sensor_msgs::LaserScan>("/scan",1,&MapNode::laserCallback,this);
        sub_obstacle = n.subscribe<obstacle_detection::Obstacle>("/found_obstacle_perception",10,&MapNode::obstacleCallback,this);
        sub_position = n.subscribe<nav_msgs::Odometry>("/particle_position",1,&MapNode::positionCallback,this);
        sub_wall = n.subscribe<std_msgs::Bool>("/wall_detected", 1, &MapNode::wallCallback,this);
    }

    ~MapNode()
    {
        delete tf_listener;
    }


    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      if (msg->linear.x == 0 && msg->angular.z == 0)
      {
        isMoving = 0;
      }else
      {
        isMoving = 1;
      }
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {

        if (!USE_RAY_TRACE || isMoving || !obstacleInFront)
        {
            return;
        }

        laser_geometry::LaserProjection projector;
        tf::StampedTransform laser_tf;
        sensor_msgs::PointCloud cloud;

        try
        {
            tf_listener->waitForTransform("/map","/laser",scan_msg->header.stamp+ros::Duration().fromSec(scan_msg->ranges.size()*scan_msg->time_increment),ros::Duration(2.0));
            projector.transformLaserScanToPointCloud("/map",*scan_msg,cloud,*tf_listener);
            tf_listener->lookupTransform("/map", "/laser",scan_msg->header.stamp+ros::Duration().fromSec(scan_msg->ranges.size()*scan_msg->time_increment), laser_tf);
        }catch(tf::TransformException ex)
        {
            ROS_ERROR("Transform error in map node: %s", ex.what());
            return;
        }


        for(int i = 0; i < cloud.points.size(); i++)
        {
            rayTrace(laser_tf.getOrigin().x(), laser_tf.getOrigin().y(),cloud.points[i].x,cloud.points[i].y);
        }
    }

    bool clearLineOfSight(double x0, double y0, double x1, double y1)
    {
    // Returns 1 if no wall was in the way. Prevents from adding occupancy behind known walls.
        int index;
        double phi = atan2(y1-y0,x1-x0);
        double dist = sqrt(pow(x0-x1,2) + pow(y0-y1,2));
        double current_dist = 0;

        double x = x0;
        double y = y0;

        while (current_dist < dist)
        {
            index = mToCell(y)*nColumns+mToCell(x);
            if (0 <= index && index < nRows*nColumns)
            {
                if (map_msg.data[mToCell(y)*nColumns+mToCell(x)] == 100)
                {
                    return 0;
                }
                decreaseOccupancy(mToCell(x),mToCell(y));
                x = x+map_resolution*cos(phi);
                y = y+map_resolution*sin(phi);
                current_dist = current_dist+map_resolution;
            }else
            {
                return 0;
            }
        }

        return 1;
    }


    void objectCallback(const object_identification::ObjectList::ConstPtr& msg)
    {
      int x,y;
      for(int i = 0; i < msg->positions.size(); i++)
      {
        x = mToCell(msg->positions[i].point.x);
        y = mToCell(msg->positions[i].point.y);
        clearArea(x,y, object_size+0.03);
        addObject(x,y);
        foundObjects.push_back(*(new ValuableObject(x,y,(int) msg->object_class[i],(int) msg->id[i])));
      }

      object_msg = *msg;

      saveMap();
    }


    void removeObject(int id)
    {
      for(int i = 0; i < foundObjects.size(); i++)
      {
        if(foundObjects[i].ID == id)
        {
          clearArea(foundObjects[i].x,foundObjects[i].y, object_size);
          return;
        }
      }
    }


    void obstacleCallback(const obstacle_detection::Obstacle::ConstPtr& msg)
    {
        if (!ADD_OBSTACLES)
        {
            return;
        }
        //ROS_INFO("Adding obstacle.");
        addLine(msg->positions[0].point.x,msg->positions[0].point.y,msg->positions[1].point.x,msg->positions[1].point.y,"fill");
        //addObject(mToCell(msg->positions[2].point.x),mToCell(msg->positions[2].point.y));
    }

    void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        clearArea(mToCell(msg->pose.pose.position.x),mToCell(msg->pose.pose.position.y),0.15);
    }

    void wallCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data == 1)
        {
          obstacleInFront = 1;

        }else
        {
          obstacleInFront = 0;
        }
    }

    void clearArea(int x, int y, float area)
    {
        int size = mToCell(area);
        int x_start = x-round(size/2);
        int y_start = y-round(size/2);

        if (x_start < 0)
        {
          x_start = 0;
        }
        if (y_start < 0)
        {
          y_start = 0;
        }

        for (int index_x = x_start; index_x <= x+round(size/2); index_x++)
        {
            for (int index_y = y_start; index_y<= y+round(size/2); index_y++)
            {
                clearOccupancy(index_x,index_y);
            }
        }
    }

    void addObject(int x, int y)
    {
        // Assumes coordinates is already in map frame
        int size = mToCell(object_size);
        int x_start = x-round(size/2);
        int y_start = y-round(size/2);

        if (x_start < 0)
        {
          x_start = 0;
        }
        if (y_start < 0)
        {
          y_start = 0;
        }

        for (int index_x = x_start; index_x <= x+round(size/2); index_x++)
        {
            for (int index_y = y_start; index_y<= y+round(size/2); index_y++)
            {
                addOccupancy(index_x,index_y,100);
            }
        }
    }

    void rayTrace(double x_robot, double y_robot, double x, double y)
    {
        if (sqrt(pow(x_robot-x,2) + pow(y_robot-y,2)) > 0.8)
        {
          // dont add measurements that are too far away
          return;
        }else if (clearLineOfSight(x_robot,y_robot,x,y)){
            // if no known walls was in the way.
            increaseOccupancy(mToCell(x),mToCell(y));
        }
    }


    int mToCell(double x)
    {
        // converts x from meters to grid cell coordinate
        return (int) round(x/map_resolution);
    }


    void addLineLow(double x0_d, double y0_d, double x1_d, double y1_d, std::string action)
    {
        // Implements Bresenhams line algorithm for integers
        int x0,x1,y0,y1,dx,dy,D,y,yi;
        x0 = mToCell(x0_d);
        x1 = mToCell(x1_d);
        y0 = mToCell(y0_d);
        y1 = mToCell(y1_d);

        dx = x1-x0;
        dy = y1-y0;
        yi = 1;

        if (dy <0)
        {
            yi = -1;
            dy = -dy;
        }

        D = 2*dy-dx;
        y = y0;

        for (int x = x0; x<=x1; x++)
        {
            if (action == "increase")
            {
                increaseOccupancy(x,y);
            }else if (action == "decrease")
            {
                decreaseOccupancy(x,y);
            }else
            {
                addOccupancy(x,y,100);
            }

            if (D > 0)
            {
                y = y+yi;
                D = D-2*dx;
            }
            D = D+2*dy;
        }
    }

    void addLineHigh(double x0_d, double y0_d, double x1_d, double y1_d, std::string action)
    {
        // Implements Bresenhams line algorithm for integers
        int x0,x1,y0,y1,dx,dy,D,x,xi;
        x0 = mToCell(x0_d);
        x1 = mToCell(x1_d);
        y0 = mToCell(y0_d);
        y1 = mToCell(y1_d);

        dx = x1-x0;
        dy = y1-y0;
        xi = 1;

        if (dx < 0)
        {
            xi = -1;
            dx = -dx;
        }

        D = 2*dx-dy;
        x = x0;

        for (int y = y0; y<=y1; y++)
        {
            if (action == "increase")
            {
                increaseOccupancy(x,y);
            }else if (action == "decrease")
            {
                decreaseOccupancy(x,y);
            }else
            {
                addOccupancy(x,y,100);
            }

            if (D > 0)
            {
                x = x+xi;
                D = D-2*dy;
            }
            D = D+2*dx;
        }

    }


    void addLine(double x0, double y0, double x1, double y1, std::string action)
    {
        // Decides how to add line depending on derivative and starting point x0,y0
        if (fabs(y1-y0) < fabs(x1-x0))
        {
            if (x0 > x1)
            {
                addLineLow(x1,y1,x0,y0,action);
            }else
            {
                addLineLow(x0,y0,x1,y1,action);
            }
        }else
        {
            if (y0 > y1)
            {
                addLineHigh(x1,y1,x0,y0,action);
            }else
            {
                addLineHigh(x0,y0,x1,y1,action);
            }
        }
    }


    void addOccupancy(int x, int y, int value)
    {
        int index = y*nColumns+x;
        if ((value < 0) || (value > 100))
        {
            ROS_INFO("Map recieved invalid occupancy value [%i]. Value must be in [0,100]", value);
        }
        else if (0<= index && index < nRows*nColumns)
        {
            map_msg.data[index] = value;
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
        }
    }

    void clearOccupancy(int x, int y)
    {
        int index = y*nColumns+x;

        if (0<= index && index < nRows*nColumns)
        {
          map_msg.data[index] = 0;
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
        }
    }

    void increaseOccupancy(int x, int y)
    {
        int index = y*nColumns+x;

        if (0 <= index && index < nRows*nColumns)
        {
            if (map_msg.data[index] == 100)
            {
                // dont mess with occupancy 100 cells!
                return;
            }
            else if (map_msg.data[index] <= 79)
            {
                map_msg.data[index] = map_msg.data[index]+20;
            }else
            {
                // occupancy 100 reserved for fixed walls and objects
                map_msg.data[index] = 99;
            }
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
        }
    }

    void decreaseOccupancy(int x, int y)
    {
        int index = y*nColumns+x;

        if (0 <= index && index < nRows*nColumns)
        {
            if (map_msg.data[index] == 100)
            {
                // fixed wall or object, dont decrease
                return;
            }
            else if (map_msg.data[index] > 10)
            {
                map_msg.data[index] = map_msg.data[index]-10;
            }else
            {
                map_msg.data[index] = 0;
            }
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
        }
    }

    void loadMap(){
        std::fstream mapfile("/home/ras16/catkin_ws/src/robot_map/map.txt");


        float line;
        mapfile>>line;
        int n_objects = (int) line;

        object_msg.positions.resize(n_objects);
        object_msg.object_class.resize(n_objects);
        object_msg.id.resize(n_objects);

        // get objects
        for(int i = 0; i < n_objects; i++)
        {
          mapfile>>line;
          object_msg.positions[i].point.x = line;
          mapfile>>line;
          object_msg.positions[i].point.y = line;
          mapfile>>line;
          object_msg.object_class[i] = (int) line;
          mapfile>>line;
          object_msg.id[i] = (int) line;
        }

        // get map data
        mapfile>>line;
        map_msg.info.height = (int) line;
        mapfile>>line;
        map_msg.info.width = (int) line;
        mapfile>>line;
        map_msg.info.resolution = line;
        map_msg.data.resize(map_msg.info.height*map_msg.info.width);
        map_msg.header.frame_id = "/map";

        int i = 0;
        while(mapfile >> line)
        {
            map_msg.data[i] = (int) line;
            i++;
        }

        mapfile.close();
    }

    void competitionObject()
    {
      ROS_INFO("Saved competition object");
      object_msg.positions.resize(1);
      object_msg.object_class.resize(1);
      object_msg.id.resize(1);
      object_msg.positions[0].point.x = 2.0;
      object_msg.positions[0].point.y = 2.32;
      object_msg.object_class[0] = 5;
      object_msg.id[0] = 0;

    }

    void saveMap(){

        std::ofstream mapfile;
        //mapfile.open("/home/ras/catkin_ws/src/robot_map/map.txt");
        mapfile.open("/home/ras16/catkin_ws/src/robot_map/map.txt");

        if (mapfile.is_open()){
            ROS_INFO("File open");
        }

        mapfile << object_msg.positions.size() << "\n";

        for(int i = 0; i < object_msg.positions.size(); i++)
        {
          mapfile << object_msg.positions[i].point.x << "\n";
          mapfile << object_msg.positions[i].point.y << "\n";
          mapfile << object_msg.object_class[i] << "\n";
          mapfile << object_msg.id[i] << "\n";
        }

        mapfile << map_msg.info.height << "\n";
        mapfile << map_msg.info.width << "\n";
        mapfile << map_msg.info.resolution << "\n";

        for (int i = 0; i < map_msg.data.size(); i++)
        {
            mapfile << (int) map_msg.data[i] << "\n";
        }

        mapfile.close();
        ROS_INFO("Map saved to file.");
    }


    void publishMapToTopic()
    {
        pub_gridmap.publish(map_msg);
    }

    void publishObjects()
    {
        pub_known_objects.publish(object_msg);
    }


};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(5);
    MapNode *map_node;

    if(LOAD_MAP_FROM_FILE)
    {

        ROS_INFO("Reading old map from file.");
        map_node = new MapNode(n);

    }else{
        ROS_INFO("Creating new map.");
        std::string _map_file;
        n.param<std::string>("map_file", _map_file, "lab_maze_2018.txt");

        ROS_INFO_STREAM("Loading the maze map from " << _map_file);

        std::ifstream map_fs; map_fs.open(_map_file.c_str());
        if (!map_fs.is_open()){
            ROS_ERROR_STREAM("Could not find map file: "<<_map_file);
            return -1;
        }

        std::vector<double> x1_points;
        std::vector<double> x2_points;
        std::vector<double> y1_points;
        std::vector<double> y2_points;
        double max_x = -100;
        double max_y = -100;
        double min_x = 100;
        double min_y = 100;

        std::string line;
        while(getline(map_fs, line))
        {
            if( ! (line[0]=='#') )
            {
                std::istringstream line_stream(line);
                double x1, y1, x2, y2;
                line_stream >> x1 >> y1 >> x2 >> y2;
                x1_points.push_back(x1);
                x2_points.push_back(x2);
                y1_points.push_back(y1);
                y2_points.push_back(y2);
                //ROS_INFO("Got wall x1,y1,x2,y2: [%f, %f, %f, %f]", x1,y1,x2,y2);
                max_x = fmax(max_x,x1);
                max_x = fmax(max_x,x2);
                max_y = fmax(max_y,y1);
                max_y = fmax(max_y,y2);

                min_x = fmin(min_x,x1);
                min_x = fmin(min_x,x2);
                min_y = fmin(min_y,y1);
                min_y = fmin(min_y,y2);
            }
        }
        ROS_INFO("Map dimensions: (%f x %f)",max_x-min_x,max_y-min_y);

       map_node = new MapNode(n,max_x-min_x,max_y-min_y, RESOLUTION);


        for (int i = 0; i < x1_points.size(); ++i)
        {
            map_node->addLine(x1_points[i]-min_x,y1_points[i]-min_y,x2_points[i]-min_x,y2_points[i]-min_y, "fill");
        }

        //map_node->saveMap();
    }

    //map_node->competitionObject(); // used for hard coded object position during competition
    while(ros::ok())
    {
        ros::spinOnce();
        map_node->publishMapToTopic();
        //map_node->publishObjects();
        if (LOAD_MAP_FROM_FILE)
        {
            map_node->publishObjects();
        }
        loop_rate.sleep();
    }

  return 0;
}
