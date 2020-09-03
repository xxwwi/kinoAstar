#include <plan_test/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>

geometry_msgs::PoseStamped goal;
std::string global_frame;


void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal_){
    goal = *goal_;
    goal.header.frame_id = global_frame;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_test_node");

    tf::TransformListener tf(ros::Duration(10));
    
    // costmap_2d::Costmap2DROS lcr("costmap", tf);

    // KinoAstar_planner::PlannerWithCostmap pppp("planner", &lcr);

    // ros::spin();
    // std::cout<<"ros run here 1"<<std::endl;
    costmap_2d::Costmap2DROS* planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf);
    // std::cout<<"ros run here 1_1"<<std::endl;
    
    global_frame = planner_costmap_ros_->getGlobalFrameID();

    KinoAstar_planner::KinoAstarPlanner* kinoastar_planner_ = new KinoAstar_planner::KinoAstarPlanner();
    // std::cout<<"ros run here 1_2"<<std::endl;
    // kinoastar_planner_->initialize("KinoAstar", planner_costmap_ros_);
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
    // geometry_msgs::PoseStamped goal;
    goal.header.stamp = plan_time;
    goal.header.frame_id = global_frame;
    goal.pose.position.x = 4.37;//-7.95; //12.17; //24.76;
    goal.pose.position.y = 15.57;//17.84; //12.87; //-4.89;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.9996;//0.7046;//0.0;
    goal.pose.orientation.w = -0.0087;//0.7095;//1.0;
    
    ros::Publisher goal_pub_;
    ros::Subscriber goal_sub_;

    ros::NodeHandle n;

    goal_pub_ = n.advertise<geometry_msgs::PoseStamped>("goal", 1);
    goal_sub_ = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, goalCB);

    std::vector<geometry_msgs::PoseStamped> plan;
    // std::cout<<"ros run here 2"<<std::endl;
    while(1){
        kinoastar_planner_->initialize("KinoAstarPlanner", planner_costmap_ros_);
        kinoastar_planner_->makePlan(start, goal, plan);
        goal_pub_.publish(goal);
        // ros::Duration(5).sleep();
        // std::cout<<"Sleep"<<endl;
        // std::cout<<"ros run here 3"<<std::endl;
        ros::spinOnce();
    }

  

    return 0;
}
