
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



    MapNode(ros::NodeHandle node, double height,double width, double res)
    {
        n = node;

        map_resolution = res; //Every element corresponds to a res*res cm area
        nColumns = (int) round((height/map_resolution)+0.5);
        nRows = (int) round((width/map_resolution)+0.5);
        ROS_INFO("Grid map rows: %i, cols: %i",nRows,nColumns);

        std::vector<int8_t> initialmap;
        initialmap.resize((nRows)*(nColumns));

        map_msg.data = initialmap;
        map_msg.header.frame_id = "/map";
        map_msg.info.resolution = map_resolution;
        map_msg.info.height = nColumns; //nRows;
        map_msg.info.width = nRows; //nColumns;

        // grid msg has different default origin compared to given map
        map_msg.info.origin.orientation.y = 1;
        map_msg.info.origin.orientation.x = 1;

        pub_wall = n.advertise<nav_msgs::OccupancyGrid>("/grid_array",1);
    }

    void wallCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        // TODO
        float a = 0;
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


    void addLineLow(double x0_d, double y0_d, double x1_d, double y1_d)
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
            addOccupancy(x,y,100);

            if (D > 0)
            {
                y = y+yi;
                D = D-2*dx;
            }
            D = D+2*dy;
        }
    }

    void addLineHigh(double x0_d, double y0_d, double x1_d, double y1_d)
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
            addOccupancy(x,y,100);

            if (D > 0)
            {
                x = x+xi;
                D = D-2*dy;
            }
            D = D+2*dx;
        }

    }


    void addLine(double x0, double y0, double x1, double y1)
    {
        // Decides how to add line depending on derivative and starting point x0,y0
        if (fabs(y1-y0) < fabs(x1-x0))
        {
            if (x0 > x1)
            {
                addLineLow(x1,y1,x0,y0);
            }else
            {
                addLineLow(x0,y0,x1,y1);
            }
        }else
        {
            if (y0 > y1)
            {
                addLineHigh(x1,y1,x0,y0);
            }else
            {
                addLineHigh(x0,y0,x1,y1);
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
        else if (index <= nRows*nColumns)
        {
            map_msg.data[index] = value;
        }else
        {
            ROS_INFO("Map index out of bounds: %i, Max: %i", index, nRows*nColumns-1);
        }
    }



    void publishMapToTopic()
    {
        pub_wall.publish(map_msg);
    }


private:
    nav_msgs::OccupancyGrid map_msg;
    int nRows;
    int nColumns;
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
        map_node.addLine(x1_points[i],y1_points[i],x2_points[i],y2_points[i]);
    }


    while(ros::ok())
    {
    ros::spinOnce();
    map_node.publishMapToTopic();
    loop_rate.sleep();
    }


  return 0;
}

