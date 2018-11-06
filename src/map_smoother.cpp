
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include "object.cpp"

//using namespace std;


class SmoothMap
{

public:
    ros::NodeHandle n;
    ros::Publisher pub_gridmap;
    ros::Subscriber sub_gridmap;


    SmoothMap(ros::NodeHandle node)
    {
        n = node;

        kernelSize = 4;
        pub_gridmap = n.advertise<nav_msgs::OccupancyGrid>("/smooth_map",1);
        sub_gridmap = n.subscribe<nav_msgs::OccupancyGrid>("/grid_map",1,&SmoothMap::mapCallback,this);

    }


    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        map_resolution = map_msg->info.resolution;
        nColumns = map_msg->info.height;
        nRows = map_msg->info.width;

        nav_msgs::OccupancyGrid map;

        smooth_map = map;

        smooth_map.info.resolution = map_resolution;
        smooth_map.info.height = nColumns;
        smooth_map.info.width = nRows;
        smooth_map.info.origin.orientation.y = 1; // grid msg has different default origin compared to given map
        smooth_map.info.origin.orientation.x = 1;


        smooth_map.data.resize(nRows*nColumns);

        //boundaries??
        ROS_INFO("Smoothing map");
        for (int x = 0; x <= nColumns; x++)
        {
            for (int y = 0; y<= nRows; y++)
            {
                if (map_msg->data[x*nRows+y] >= 99)
                {
                    smoothArea(x,y);
                    addOccupancy(x,y,100);
                }
            }
        }

        pub_gridmap.publish(smooth_map);
        ROS_INFO("Map published");
    }


    void smoothArea(int x, int y)
    {
        // Assumes coordinates is already in map frame
        int size = kernelSize;
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

        float k;
        for (int index_x = x_start; index_x <= x+round(size/2); index_x++)
        {
            for (int index_y = y_start; index_y<= y+round(size/2); index_y++)
            {
                k = 1 / (1+sqrt(pow(x-index_x,2)+pow(y-index_y,2)));
                //ROS_INFO("K: %f",k);
                increaseOccupancy(index_x,index_y,round(k*15));

            }
        }
    }


    void addOccupancy(int x, int y, int value)
    {
        int index = x*nRows+y;
        if ((value < 0) || (value > 100))
        {
            //ROS_INFO("Map recieved invalid occupancy value [%i]. Value must be in [0,100]", value);
            return;
        }
        else if (0<= index && index < nRows*nColumns)
        {
            smooth_map.data[index] = value;
        }else
        {
            //ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
            return;
        }
    }

    void increaseOccupancy(int x, int y, int inc_value)
    {
        int index = x*nRows+y;
        int value;

        if (0<= index && index < nRows*nColumns)
        {
            value = smooth_map.data[index]+inc_value;
            if (value <= 100)
            {
                smooth_map.data[index] = value;
            }else
            {
                smooth_map.data[index] = 100;
            }
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
        }
    }


private:
    nav_msgs::OccupancyGrid smooth_map;
    int kernelSize;
    int nRows;
    int nColumns;
    double map_resolution;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_smoother");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(2);

    SmoothMap map_node = SmoothMap(n);


    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
