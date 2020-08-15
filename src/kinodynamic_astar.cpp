#include <KinoAstar_planner/kinodynamic_astar.h>
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <sstream>

using namespace std;
using namespace Eigen;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(KinoAstar_planner::KinoAstarPlanner, nav_core::BaseGlobalPlanner)

namespace KinoAstar_planner {

    //Constuctor
    KinoAstarPlanner::KinoAstarPlanner() : costmap_(NULL), initialized_(false), allow_unkown_(true) {

    }

    KinoAstarPlanner::KinoAstarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id);
    
    }

    //Deconstuctor
    KinoAstarPlanner::~KinoAstarPlanner(){

    }

    void KinoAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void KinoAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!intialized_){
            //???ros::NodeHandle private_nh("~/" + name);
            costmap_ = costmap;
            frame_id_ = frame_id;

            unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            make_plan_srv_ = private_nh.advertiseService("make_plan", &KinoAstarPlanner::makePlanService, this);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

    }











}