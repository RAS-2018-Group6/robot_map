
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <math.h>
#include <sstream>
#include <string>
#include <fstream>

//using namespace std;

class MapNode
{

public:
    ros::NodeHandle n;
    ros::Publisher pub_wall;
    //std::string initial_map;



    MapNode(ros::NodeHandle node, double width,double height, double res){

        n = node;

        map_resolution = res; //Every element corresponds to a 2x2 cm area
        cols = (int) round(width/map_resolution)+4;
        rows = (int) round(height/map_resolution)+4;
        ROS_INFO("Grid map rows: %i, cols: %i",rows,cols);


        std::vector<int8_t> initialmap;
        initialmap.resize((rows)*(cols));

        map_msg.data = initialmap;
        map_msg.header.frame_id = "/map";
        map_msg.info.resolution = map_resolution;
        map_msg.info.height = rows;
        map_msg.info.width = cols;
        map_msg.info.origin.position.y = height;
        map_msg.info.origin.orientation.x = 180;

        pub_wall = n.advertise<nav_msgs::OccupancyGrid>("/grid_array",1);
    }

    void wallCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        float a = 0;
    }

    double getXofY(double x, double k,double m)
    {
        return k*x+m;
    }

    void addWall(double x1, double y1, double x2, double y2)
    {
        double k, m, X, Xmax, Y, Ymax, temp;
        int x_, y_;
        int occupancy_value = 100;

        if (sqrt(pow(x1,2) + pow(y1,2)) > sqrt(pow(x2,2) + pow(y2,2)))
        {
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }


        if (x2-x1 < y2-y1){
            Y = fmin(y1,y2);
            Ymax = fmax(y1,y2);
            k = (x2-x1)/(y2-y1);
            m = x1-k*y1;

            ROS_INFO("Adding Wall as function of Y...");
            while(Y < Ymax)
            {
                x_ = (int) round(getXofY(Y,k,m)/map_resolution);
                y_ = (int) round(Y/map_resolution);
                ROS_INFO("Adding: %i, %i. X = %f",x_,y_,Y);
                addOccupancy(x_, y_,occupancy_value);
                Y = Y+map_resolution/2;
            }
            ROS_INFO("Wall added.");


        }else
        {
            k = (y2-y1)/(x2-x1);
            m = y1-k*x1;
            X = fmin(x1,x2);
            Xmax = fmax(x1,x2);

            ROS_INFO("Adding Wall as function of X...");
            while(X < Xmax)
            {
                x_ = (int) round(X/map_resolution);
                y_ = (int) round(getXofY(X,k,m)/map_resolution);
                ROS_INFO("Adding: %i, %i. X = %f",x_,y_,X);
                addOccupancy(x_, y_, occupancy_value);
                X = X+map_resolution/2;
            }
            ROS_INFO("Wall added.");
        }

    }


    void addOccupancy(int x, int y, int value)
    {
        int index = x*cols+y;
        if (value < 0 || value < 100)
        {
            ROS_INFO("Map recieved invalid occupancy value [%i]. Value must be in [0,100]", value);
        }
        else if (index <= rows*cols)
        {
            map_msg.data[index] = value;
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, rows*cols-1);
        }
    }



    void publishMapToTopic()
    {
        pub_wall.publish(map_msg);
    }


private:
    nav_msgs::OccupancyGrid map_msg;
    int rows;
    int cols;
    double map_resolution;
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

    double map_resolution = 0.02;
    MapNode map_node = MapNode(n,max_x,max_y, map_resolution);

    for (int i = 0; i < x1_points.size(); ++i)
    {
        map_node.addWall(x1_points[i],y1_points[i],x2_points[i],y2_points[i]);
    }


    while(ros::ok())
    {
    ros::spinOnce();
    map_node.publishMapToTopic();
    loop_rate.sleep();
    }


  return 0;
}

