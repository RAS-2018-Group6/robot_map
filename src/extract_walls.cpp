
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <tf/transform_listener.h>


class LaserRANSAC
{

public:
    ros::NodeHandle n;
    ros::Publisher pub_wall;
    ros::Subscriber sub_laser;




    LaserRANSAC(ros::NodeHandle node)
    {
        n = node;
        //private std::vector<float> ranges_x{0,1,2,3,0,0,0,0,1,2};
        //private std:::vector<float> ranges_y{0,0,0,0,0,1,2,3,2,1};
        cppsucks();

        pub_wall = n.advertise<geometry_msgs::PointStamped>("/wall_point",10000);
        sub_laser = n.subscribe<sensor_msgs::LaserScan>("/scan",1,&LaserRANSAC::laserCallback,this);
    }

    void cppsucks()
    {
        ranges_x.push_back(2);
        ranges_x.push_back(3);
        ranges_x.push_back(4);
        ranges_x.push_back(5);
        ranges_x.push_back(0);
        ranges_x.push_back(0);
        ranges_x.push_back(0);
        ranges_x.push_back(0);
        ranges_x.push_back(1);
        ranges_x.push_back(2);


        ranges_y.push_back(0);
        ranges_y.push_back(0);
        ranges_y.push_back(0);
        ranges_y.push_back(0);
        ranges_y.push_back(2);
        ranges_y.push_back(3);
        ranges_y.push_back(4);
        ranges_y.push_back(5);
        ranges_y.push_back(2);
        ranges_y.push_back(1);

    }



    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        //float ranges[] = msg->ranges;
        range_min = msg-> range_min;
        range_max = msg-> range_max;
        angle_min = msg-> angle_min;
        angle_increment = msg-> angle_increment;

        tf::TransformListener tf_listener;
        tf::StampedTransform laser_tf;

        tf_listener.waitForTransform("map", "laser", ros::Time(0),ros::Duration(2));
        tf_listener.lookupTransform("map", "laser",ros::Time(0), laser_tf);


        geometry_msgs::PointStamped laser_point;
        geometry_msgs::PointStamped map_point;

        laser_point.header.frame_id = "laser";
        laser_point.header.stamp = ros::Time(0);

        //ranges_x.clear();
        //ranges_y.clear();
        float r;
        for (int i = 0; i < msg->ranges.size(); i++)
        {
            r = msg->ranges[i];
            if (r > range_min && r < range_max)
            {
                laser_point.point.x = r*cos(angle_min+i*angle_increment);
                laser_point.point.y = r*sin(angle_min+i*angle_increment);
                laser_point.point.z = 0.0;


                try
                {
                    tf_listener.transformPoint("/map",laser_point,map_point);
                    pub_wall.publish(map_point);

                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("Got exception when transforming point from laser to map.");
                }

                //ranges_x.push_back(r*cos(angle_min+i*angle_increment));
                //ranges_y.push_back(r*sin(angle_min+i*angle_increment));
            }
        }

    }

    void findWalls()
    {
        //ranges_x = {0,1,2,3,0,0,0,0,1,2}; // test
        //ranges_y = {0,0,0,0,0,1,2,3,2,1};

        float a,b,c,dist,thresh,x1,x2,y1,y2,x0,y0;
        int rand1,rand2,nInliers,nPoints,maxInliers,M,N;
        ROS_INFO("Starting findWalls");

        nInliers = 100; // init > M
        N = 20; // iterations
        M = 3; // required inliers for wall
        thresh = 0.02; // distance from line to be inlier
        std::vector<int> inliers;
        std::vector<int> best_inliers;
        float best_line[3];


        // while walls can still be found
        while (nInliers > M)
        {
            ROS_INFO("Looking for new wall");
            maxInliers = 0;
            //inliers.clear();
            nPoints = ranges_x.size();

            // RANSAC N iterations
            for (int n = 0; n < N; n++)
            {
                //ROS_INFO("RANSAC interation %i",n);
                nInliers = 0;
                inliers.clear();


                rand1 = rand() % nPoints;
                rand2 = rand() % nPoints;

                x1 = ranges_x[rand1];
                y1 = ranges_y[rand1];
                x2 = ranges_x[rand2];
                y2 = ranges_y[rand2];

                a = y1-y2;
                b = x2-x1;
                c = x1*y2-x2*y1;

                // for points find nInliers
                for (int i = 0; i < nPoints; i++)
                {
                    x0 = ranges_x[i];
                    y0 = ranges_y[i];
                    if (abs(a*x0+b*y0+c)/sqrt(pow(a,2)+pow(b,2)) < thresh)
                    {
                        nInliers++;
                        inliers.push_back(i); // point index i is an inlier
                    }
                }

                if (nInliers > maxInliers)
                {
                    //ROS_INFO("Found best line with %i inliers.",nInliers);
                    best_line[0] = a;
                    best_line[1] = b;
                    best_line[2] = c;
                    best_inliers = inliers;
                    maxInliers = nInliers;
                }


            }
             // remove inliers for best line, repeat
            //ROS_INFO("Removing inliers from ranges %i", best_inliers.size());

            std::sort(best_inliers.begin(), best_inliers.end());
            for(int i = best_inliers.size()-1; i>-1; --i)
            {
                //ROS_INFO("%i",i);
                ranges_x.erase(ranges_x.begin() + best_inliers[i]);
                ranges_y.erase(ranges_y.begin() + best_inliers[i]);
                //ROS_INFO("DONE");
            }
            //ROS_INFO("Clearing");
            best_inliers.clear();
            //ROS_INFO("DONE");
            ROS_INFO("Got line: a: %f, b: %f, c: %f with %i inliers",best_line[0], best_line[1],best_line[2], maxInliers);
            //best_line = {};
        }
    }



private:
    //nav_msgs::OccupancyGrid wall_msg;

    //float ranges[];
    std::vector<float> ranges_x;
    std::vector<float> ranges_y;
    float range_min;
    float range_max;
    float angle_min;
    float angle_max;
    float angle_increment;
    std::vector<float> wall_x;
    std::vector<float> wall_y;


};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_finder");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(2);


    LaserRANSAC ransac = LaserRANSAC(n);
    //ransac.findWalls();

    //ros::spin();

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }



  return 0;
}

