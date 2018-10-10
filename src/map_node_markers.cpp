
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <string>
#include <fstream>
#include <math.h>

//using namespace std;

class MapNode
{

public:
    ros::NodeHandle n;
    ros::Publisher pub_map;
    ros::Publisher pub_wall;
    std::string initial_map;



    MapNode(ros::NodeHandle node, double width,double height){

        n = node;

        map_resolution = 0.02; //Every element corresponds to a 2x2 cm area

        cols = (int) round(width/map_resolution);
        rows = (int) round(height/map_resolution);
        ROS_INFO("r c %i %i",rows, cols);
        cell_id = 0;

        grid_map[rows][cols];
        map_as_array.resize(rows*cols);

        //node.param<string>("map_file",initial_map,"lab_maze_2018.txt");
        pub_wall = n.advertise<nav_msgs::OccupancyGrid>("/grid_array",1);
        pub_map = n.advertise<visualization_msgs::MarkerArray>("/grid_map", 1);
    }

    void wallCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        //int newreadings[] = msg->data;
        for (int r = 0; r < rows; r++)
        {
            //add newreading to data
        }

    }


    void initMap()
    {
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                ROS_INFO("Tryink %i %i",rows,cols);
                if ( (r == (rows-1)) || (r == 0) || (c == (cols-1)) || (c == 0) )
                {
                    grid_map[r][c] = 1;
                    addCellMarker(r,c);
                    ROS_INFO("Added %i %i",r,c);
                }else
                {
                    grid_map[r][c] = 0;
                    ROS_INFO("Not Added %i %i",r,c);
                }
            }
        }
        ROS_INFO("Map initialized");

        for(int index = 0; index<(rows*cols); ++index)
        {
            map_msg.data[index] = 50;
        }
    }


    void publishMap()
    {
        pub_map.publish(all_markers);
    }

    void addCellMarker(float x, float y)
    {
        cell_marker.pose.position.x = x*map_resolution;
        cell_marker.pose.position.y = y*map_resolution;
        cell_marker.id = cell_id;
        all_markers.markers.push_back(cell_marker);
        cell_id++;
    }

    void initCellMarker()
    {
        cell_marker.header.frame_id = "/map";
        cell_marker.header.stamp = ros::Time();
        //cell_marker.ns = "world";
        cell_marker.type = visualization_msgs::Marker::CUBE;
        cell_marker.action = visualization_msgs::Marker::ADD;
        cell_marker.scale.x = map_resolution;
        cell_marker.scale.y = map_resolution;
        cell_marker.scale.z = 0.1;
        cell_marker.color.a = 1.0;
        cell_marker.color.r = 1.0;
        cell_marker.color.g = 0;
        cell_marker.color.b = 0;
        cell_marker.id = cell_id;

        //wall_marker.pose.position.z = 0.2;
    }

    int cmToCell(double x)
    {
        return (int) round(x/map_resolution);
    }

    void publishMapToTopic()
    {
        //map_msg.data = map_as_array;
        pub_wall.publish(map_msg);
    }


private:
    nav_msgs::OccupancyGrid map_msg;
    std::vector<int> map_as_array;
    visualization_msgs::Marker cell_marker;
    visualization_msgs::MarkerArray all_markers;
    int cell_id;
    int grid_map[200][200];
    int rows;
    int cols;
    int sizeX;
    int sizeY;
    float map_resolution;
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
            //addWall(x1,y1,x2,y2);
            ROS_INFO("Added wall x1,y1,x2,y2: [%f, %f, %f, %f]", x1,y1,x2,y2);
            max_x = fmax(max_x,x1);
            max_x = fmax(max_x,x2);
            max_y = fmax(max_y,y1);
            max_y = fmax(max_y,y2);
        }
    }
    ROS_INFO("Map dimensions: (%f x %f)",max_x,max_y);

    MapNode map_node = MapNode(n,max_x,max_y);
    map_node.initCellMarker();
    map_node.initMap();



    while(ros::ok())
    {
    ros::spinOnce();
    map_node.publishMap();
    map_node.publishMapToTopic();
    loop_rate.sleep();
    }


  return 0;
}

