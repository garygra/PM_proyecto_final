#include <ros/ros.h>
#include <QtGui>
#include <QApplication>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <boost/bind.hpp>
#include "timer.h"
#include "soccerview.h"
#include "graphical_client/Pose2D_Array.h"

#define NOROBOTS 12
#define RATE_HZ 2

GLSoccerView *view;
int max_deb_pts = 10;
QVector <vector2d> dest_points(max_deb_pts);
int des_p_i = 0;

class ThreadROS : public QThread{
	protected:
		void run()
		{
			// ros::Rate rate(RATE_HZ);
			while(ros::ok()) 
			{
				ros::spinOnce();
				// rate.sleep();			
			}
		}
};

void updateBall(const geometry_msgs::Pose2D& msg)
{
	vector2d ball;
	ball.set(msg.x, msg.y);
	view->updateBall(ball);
}

void updateRobot(const geometry_msgs::Pose2D::ConstPtr& msg, std::string robotID)
{
	Robot robot;
	size_t found = robotID.find("/y_r");
	if(found!=std::string::npos)
	{
		robot.team = teamYellow;
	}
	else
	{
		robot.team = teamBlue;
	}
	robot.conf = 0.0;
	robot.loc.set(msg->x, msg->y);
	robot.angle = DEG(msg->theta);
	robot.id = atoi(robotID.substr(4).c_str());
	robot.hasAngle = false;
	robot.vel.set(msg->x, msg->y, msg->theta);

	view->updateRobot(robot);
}

void updateRobotTraj(const geometry_msgs::Pose2D::ConstPtr& msg, std::string robotID)
{
        Robot robot; 
        size_t found = robotID.find("/y_r");
        if(found!=std::string::npos)
                robot.team = teamYellow;
        else    
                robot.team = teamBlue;
	robot.id = atoi(robotID.substr(4).c_str());
	robot.vel.set(msg->x, msg->y, msg->theta);
 
        view->updateRobotTraj(robot);
}

// void updatePoints(const PointCloud::ConstPtr& msg)
// {
// 	int max = msg->height * msg->width ;//- 2;
// 	QVector <vector2d> points(max);
// 	int i = 0;

// 	vector2d loc;
// 	// printf("\ngrapg_client\n" );
// 	BOOST_FOREACH (const pcl::PointXYZ& pt, msg -> points)
//   	{
//   		// if(i < max)
//   		// {
//   			// printf("(%f,%f)",pt.x, pt.y );
//   			points[i].set(1*pt.x, 1*pt.y); 
//   			// loc.set(pt.x, pt.y);
//   			// view -> updateBall(loc);
//   			i++;
//   		// }
//   	}
//   		// ROS_INFO_STREAM("Point: ( " << pt.x << " , " << pt.y << " ) ");
//   		// vector2d loc;
//   		//loc.set(100, 100);//pt.x, pt.y);
//     	view -> updatePoints(points);//Point(loc);
  	
// }

void update_dest_point(const geometry_msgs::Pose2D &msg)
{
	view -> update_dest_point(1*msg.x, 1*msg.y);
}

void update_target_point_debug(const geometry_msgs::Pose2D &msg)
{
	dest_points[des_p_i].set(msg.x, msg.y);
	des_p_i = (des_p_i + 1) % max_deb_pts ; 
	ROS_INFO_STREAM(msg.x << ", " <<  msg.y);
	view -> update_target_point(dest_points);
}
void update_trajectory(const graphical_client::Pose2D_Array &msg)
{
	QVector <vector3d> points;
	for(auto const& pose: msg.poses) 
	{
		vector3d v(pose.x, pose.y, pose.theta);
  		points.append(v); 
	}



	view -> update_target_points(points);

}
// void update_target_point(const PointCloud::ConstPtr& msg)
// {
// 	int max = msg->height * msg->width ;//- 2;
// 	QVector <vector2d> points(max);
// 	int i = 0;
// 	BOOST_FOREACH (const pcl::PointXYZ& pt, msg -> points)
//   	{
//   		// if(i < max)
//   		// {
//   			// printf("(%f,%f)",pt.x, pt.y );
//   			points[i].set(1*pt.x, 1*pt.y); 
//   			// loc.set(pt.x, pt.y);
//   			// view -> updateBall(loc);
//   			i++;
//   		// }
//   	}

// 	view -> update_target_points(points);
// }
int main(int argc, char **argv){
        ros::init(argc,argv, "graphical_client_node");
        ros::NodeHandle nh;
        ROS_INFO_STREAM("graphical_client_node initialized");
	ros::Subscriber sub_ball_pos;
        std::vector<ros::Subscriber> vision_sub_yr;
        std::vector<ros::Subscriber> vision_sub_br;
	// std::vector<ros::Subscriber> traj_sub_yr;
 //        std::vector<ros::Subscriber> traj_sub_br;

	// subscribes
    ROS_INFO_STREAM("At ball and robot subscribes");
	sub_ball_pos = nh.subscribe("/ball", 1, updateBall);
	for(int i = 0; i < (NOROBOTS - 1); i++){
            std::stringstream yss, bss, yss2, bss2;
            yss << "/y_r" << i;
            bss << "/b_r" << i;
	    vision_sub_yr.push_back(nh.subscribe<geometry_msgs::Pose2D>(yss.str(), 1, boost::bind(&updateRobot, _1, yss.str())));
	    vision_sub_br.push_back(nh.subscribe<geometry_msgs::Pose2D>(bss.str(), 1, boost::bind(&updateRobot, _1, bss.str())));
	    // yss2 << "/y_r" << i << "_pos";
     //        bss2 << "/b_r" << i << "_pos";
	    // traj_sub_yr.push_back(nh.subscribe<geometry_msgs::Pose2D>(yss2.str(), 1, boost::bind(&updateRobotTraj, _1, yss.str())));
	    // traj_sub_br.push_back(nh.subscribe<geometry_msgs::Pose2D>(bss2.str(), 1, boost::bind(&updateRobotTraj, _1, bss.str())));
        }
    ROS_INFO_STREAM("At points and target subscribers");
    // ros::Subscriber sub_points = nh.subscribe<PointCloud>("/point_cloud", 1, &updatePoints);                     
    // printf("Line: %d\n", __LINE__);
    // ros::Subscriber sub_target = nh.subscribe("/target", 1, &update_target_point);
	ros::Subscriber sub_dest = nh.subscribe("/current_destination", 1, &update_dest_point);
	ros::Subscriber sub_trajectory = nh.subscribe("/trajectory", 1, &update_trajectory);

	// ros::Subscriber sub_targ_debug = nh.subscribe("/target_topic_debug", 1, &update_target_point_debug);
	// graphics
	ROS_INFO_STREAM("QApplication");
	QApplication app(argc, argv);
	view = new GLSoccerView();
	view->show();	
	ROS_INFO_STREAM("Before ThreadROS");
	// ROS_DEBUG_STREAM
	ThreadROS tr;

	ROS_INFO_STREAM("Before tr.start");
	tr.start();
	int retVal = app.exec();
	tr.wait();
	ROS_INFO_STREAM("Before return");

        return retVal;
}


