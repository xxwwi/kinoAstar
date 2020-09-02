#include <global_planner/global_planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_test_node");

    tf::TransformListener tf(ros::Duration(10));

    // costmap_2d::Costmap2DROS lcr("costmap", tf);

    // KinoAstar_planner::PlannerWithCostmap pppp("planner", &lcr);

    // ros::spin();
    // std::cout<<"ros run here 1"<<std::endl;
    costmap_2d::Costmap2DROS* planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf);
    // std::cout<<"ros run here 1_1"<<std::endl;
    global_planner::GlobalPlanner* global_planner_ = new global_planner::GlobalPlanner();
    // std::cout<<"ros run here 1_2"<<std::endl;
    global_planner_->initialize("global_planner", planner_costmap_ros_);
    // std::cout<<"ros run here 1_3"<<std::endl;
    ros::Time plan_time = ros::Time::now();

    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    // cout<<"Frame "<<global_frame<<endl;

    geometry_msgs::PoseStamped start;
    start.header.stamp = plan_time;
    start.header.frame_id = global_frame;
    start.pose.position.x = 0.0;
    start.pose.position.y = 0.0;
    start.pose.position.z = 0.0;
    start.pose.orientation.x = 0.0;
    start.pose.orientation.y = 0.0;
    start.pose.orientation.z = 0.0;
    start.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = plan_time;
    goal.header.frame_id = global_frame;
    goal.pose.position.x = 24.76;
    goal.pose.position.y = -4.89;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    std::vector<geometry_msgs::PoseStamped> plan;

    ros::Publisher goal_pub_;

    ros::NodeHandle n;

    goal_pub_=n.advertise<geometry_msgs::PoseStamped>("goal", 1);
    // std::cout<<"ros run here 2"<<std::endl;
    while(1){
        global_planner_->makePlan(start, goal, plan);
        goal_pub_.publish(goal);
        ros::Duration(5).sleep();
        // std::cout<<"Sleep"<<endl;
        // std::cout<<"ros run here 3"<<std::endl;
    }

  

    return 0;
}