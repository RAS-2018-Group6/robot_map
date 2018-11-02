
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <sstream>
#include <string>
#include <fstream>

//using namespace std;

class MapNode
{

public:
    ros::NodeHandle n;
    ros::Publisher pub_gridmap;
    ros::Subscriber sub_objectsToAdd;
    ros::Subscriber sub_wall;
    ros::Subscriber sub_laser;
    tf::TransformListener *tf_listener;




    MapNode(ros::NodeHandle node, double height,double width, double res)
    {
        n = node;

        tf_listener = new tf::TransformListener();

        map_resolution = res; //Every element corresponds to a res*res cm area
        object_size = 0.1; // side length of square object
        nColumns = (int) round((height/map_resolution)+0.5);
        nRows = (int) round((width/map_resolution)+0.5);
        ROS_INFO("Grid map rows: %i, cols: %i",nRows,nColumns);

        //std::vector<int8_t> initialmap;
        //initialmap.resize((nRows)*(nColumns));

        //map_msg.data = initialmap;
        map_msg.data.resize((nRows)*(nColumns));
        map_msg.header.frame_id = "/map";
        map_msg.info.resolution = map_resolution;
        map_msg.info.height = nColumns; //nRows;
        map_msg.info.width = nRows; //nColumns;

        // grid msg has different default origin compared to given map
        map_msg.info.origin.orientation.y = 1;
        map_msg.info.origin.orientation.x = 1;

        pub_gridmap = n.advertise<nav_msgs::OccupancyGrid>("/grid_map",1);
        sub_objectsToAdd = n.subscribe<geometry_msgs::Pose>("/found_object",1,&MapNode::objectCallback,this);
        //sub_wall = n.subscribe<geometry_msgs::PointStamped>("/wall_point",10000,&MapNode::pointCallback,this);
        sub_laser = n.subscribe<sensor_msgs::LaserScan>("/scan",1,&MapNode::laserCallback,this);


    }
    ~MapNode()
    {
        delete tf_listener;
    }



    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
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
            rayTrace(laser_tf.getOrigin().x()+0.2, laser_tf.getOrigin().y()+0.2,cloud.points[i].x+0.2,cloud.points[i].y+0.2);
        }
        //publishMapToTopic();


    }

    void clearLineOfSight(double x0, double y0, double x1, double y1)
            {
                double phi = atan2(y1-y0,x1-x0);
                double dist = sqrt(pow(x0-x1,2) + pow(y0-y1,2));
                double current_dist = 0;

                double x = x0;
                double y = y0;

                while (current_dist <= dist)
                {
                    decreaseOccupancy(mToCell(x),mToCell(y));
                    x = x+map_resolution*cos(phi);
                    y = y+map_resolution*sin(phi);
                    current_dist = current_dist+map_resolution;

                }
    }

    /*
    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        tf::TransformListener tf_listener;
        tf::StampedTransform laser_tf;


        try
        {

            tf_listener.waitForTransform("map", "laser", ros::Time(0),ros::Duration(2));
            tf_listener.lookupTransform("map", "laser",ros::Time(0), laser_tf);
            //ROS_INFO("Line: [%i, %i -> %i, %i]",mToCell(laser_tf.getOrigin().x()), mToCell(laser_tf.getOrigin().y()),mToCell(msg->point.x),mToCell(msg->point.y));

            rayTrace(mToCell(laser_tf.getOrigin().x()+0.2), mToCell(laser_tf.getOrigin().y()+0.2),mToCell(msg->point.x+0.2),mToCell(msg->point.y+0.2));

        }catch(tf::TransformException ex)
        {
            ROS_ERROR("Transform exception in map_node.");
        }
        publishMapToTopic();

    }
    */

    void objectCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        int x,y;
        x = mToCell(msg->position.x);
        y = mToCell(msg->position.y);
        addObject(x,y);

    }

    void addObject(int x, int y)
    {
        // Assumes coordinates is already in map frame


        int size = mToCell(object_size);

        for (int index_x = x-round(size/2); index_x <= x+round(size/2); index_x++)
        {
            for (int index_y = y-round(size/2); index_y<= y+round(size/2); index_y++)
            {
                addOccupancy(index_x,index_y,100);
            }
        }
    }

    void rayTrace(double x_robot, double y_robot, double x, double y)
    {
        // decrease line of sight to point
        //addLine(x_robot,y_robot,x,y,"decrease");
        // increase laser point
        clearLineOfSight(x_robot,y_robot,x,y);
        increaseOccupancy(mToCell(x),mToCell(y));
        addObject(mToCell(x_robot),mToCell(y_robot));
    }

    double getXofY(double x, double k,double m)
    {
        return k*x+m;
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
        int index = x*nRows+y;
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

    void increaseOccupancy(int x, int y)
    {
        int index = x*nRows+y;

        if (0 <= index && index < nRows*nColumns)
        {
            if (map_msg.data[index] < 80)
            {
                map_msg.data[index] = map_msg.data[index]+20;
            }else
            {
                map_msg.data[index] = 100;
            }
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
        }
    }

    void decreaseOccupancy(int x, int y)
    {
        int index = x*nRows+y;

        if (0 <= index && index < nRows*nColumns)
        {
            if (map_msg.data[index] > 10)
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



    void publishMapToTopic()
    {
        pub_gridmap.publish(map_msg);
    }


private:
    nav_msgs::OccupancyGrid map_msg;
    int nRows;
    int nColumns;
    double map_resolution;
    double object_size;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(5);

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
    double max_x = 0;
    double max_y = 0;

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
            ROS_INFO("Got wall x1,y1,x2,y2: [%f, %f, %f, %f]", x1,y1,x2,y2);
            max_x = fmax(max_x,x1);
            max_x = fmax(max_x,x2);
            max_y = fmax(max_y,y1);
            max_y = fmax(max_y,y2);
        }
    }
    ROS_INFO("Map dimensions: (%f x %f)",max_x,max_y);

    double map_resolution = 0.01;
    MapNode map_node = MapNode(n,max_x,max_y, map_resolution);


    for (int i = 0; i < x1_points.size(); ++i)
    {
        map_node.addLine(x1_points[i],y1_points[i],x2_points[i],y2_points[i], "fill");
    }
    map_node.publishMapToTopic();
    //ros::sleep(1);
    //map_node.addObject(50,20);

    //ros::spin();


    while(ros::ok())
    {
    ros::spinOnce();
    map_node.publishMapToTopic();
    loop_rate.sleep();
    }



  return 0;
}

