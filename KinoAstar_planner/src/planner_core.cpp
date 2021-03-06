/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <KinoAstar_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <KinoAstar_planner/kinodynamic_astar.h>
#include <iostream>

// #include <global_planner/dijkstra.h>
// #include <global_planner/astar.h>
// #include <global_planner/grid_path.h>
// #include <global_planner/gradient_path.h>
// #include <global_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(KinoAstar_planner::KinoAstarPlanner, nav_core::BaseGlobalPlanner)

namespace KinoAstar_planner {

void KinoAstarPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

KinoAstarPlanner::KinoAstarPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

KinoAstarPlanner::KinoAstarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

KinoAstarPlanner::~KinoAstarPlanner() {
    // if (p_calc_)
    //     delete p_calc_;
    // if (planner_)
    //     delete planner_;
    // if (path_maker_)
    //     delete path_maker_;
    // if (dsrv_)
    //     delete dsrv_;
}

void KinoAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void KinoAstarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        // bool use_quadratic;
        // private_nh.param("use_quadratic", use_quadratic, true);
        // if (use_quadratic)
        //     p_calc_ = new QuadraticCalculator(cx, cy);
        // else
        //     p_calc_ = new PotentialCalculator(cx, cy);

        // bool use_dijkstra;
        // private_nh.param("use_dijkstra", use_dijkstra, true);
        // if (use_dijkstra)
        // {
        //     DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
        //     if(!old_navfn_behavior_)
        //         de->setPreciseStart(true);
        //     planner_ = de;
        // }
        // else
        //     planner_ = new AStarExpansion(p_calc_, cx, cy);

		//--------------new--------------------
        //ros::NodeHandle nh;
        data_vel << 0.0, 0.0, 0.0;
        data_acc << 0.0, 0.0, 0.0;
        acc_flag = false;
		kino_planner_ = new KinodynamicAstar();
		kino_planner_->setParam(private_nh);
		kino_planner_->setEnvironment(costmap_);
        kino_planner_->init();
        //-------------------------------------

        // bool use_grid_path;
        // private_nh.param("use_grid_path", use_grid_path, false);
        // if (use_grid_path)
        //     path_maker_ = new GridPath(p_calc_);
        // else
        //     path_maker_ = new GradientPath(p_calc_);

        // orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        // potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        // planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &KinoAstarPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<KinoAstar_planner::KinoAstarPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<KinoAstar_planner::KinoAstarPlannerConfig>::CallbackType cb = boost::bind(
                &KinoAstarPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ros::NodeHandle nh;

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&KinoAstarPlanner::odomCB, this, _1));

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void KinoAstarPlanner::reconfigureCB(KinoAstar_planner::KinoAstarPlannerConfig& config, uint32_t level) {
    // planner_->setLethalCost(config.lethal_cost);
    // path_maker_->setLethalCost(config.lethal_cost);
    // planner_->setNeutralCost(config.neutral_cost);
    // planner_->setFactor(config.cost_factor);
    // publish_potential_ = config.publish_potential;
    // orientation_filter_->setMode(config.orientation_mode);
    // orientation_filter_->setWindowSize(config.orientation_window_size);
    
    kino_planner_->setLethalCost(config.lethal_cost);
    kino_planner_->setMaxTau(config.max_tau);
    kino_planner_->setInitMaxTau(config.init_max_tau);
    kino_planner_->setMaxVel(config.max_vel);
    kino_planner_->setMaxAcc(config.max_acc);
    kino_planner_->setWTime(config.w_time);
    kino_planner_->setHorizon(config.horizon);
    kino_planner_->setTimeResolution(config.time_resolution);
    kino_planner_->setLambdaHeu(config.lambda_heu);
    kino_planner_->setCheckNum(config.check_num);
    kino_planner_->setCeffi(config.c_effi);

}

void KinoAstarPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool KinoAstarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void KinoAstarPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool KinoAstarPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();
   // cout<<"planner core resolution is "<<resolution<<" and origin is "<<origin_x<<"  "<<origin_x<<endl;
    // double origin_x = -50;
    // double  origin_y =-50;

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool KinoAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool KinoAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    // cout<<"acc_flag "<<acc_flag<<endl;
    cout<<"start position from planner code "<<wx<<" "<<wy<<endl;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    // if(old_navfn_behavior_){
    //     start_x = start_x_i;
    //     start_y = start_y_i;
    // }else{
         if(!worldToMap(wx, wy, start_x, start_y))
            cout<<"code core world to map failure";
    // }

    cout<<"start id from planner core "<<start_x<<" "<<start_y<<endl;


    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    // if(old_navfn_behavior_){
    //     goal_x = goal_x_i;
    //     goal_y = goal_y_i;
    // }else{
         worldToMap(wx, wy, goal_x, goal_y);
    // }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    // p_calc_->setSize(nx, ny);
    // planner_->setSize(nx, ny);
    // path_maker_->setSize(nx, ny);
    // potential_array_ = new float[nx * ny];

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    // bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
    //                                                 nx * ny * 2, potential_array_);
	
	//Eigen::Vector3d start_pt, start_vel, start_acc, goal_pt, goal_vel;
	//??????????start_pt << start_x << start_y << 0.0;
	Eigen::Vector3d start_pt(start.pose.position.x, start.pose.position.y, 0.0);
	Eigen::Vector3d goal_pt(goal.pose.position.x, goal.pose.position.y, 0.0);	
	Eigen::Vector3d goal_vel(0.0, 0.0, 0.0);
    Eigen::Vector3d start_vel;
    Eigen::Vector3d start_acc;

    if(!acc_flag){
        start_vel<<0.0, 0.0, 0.0;
        start_acc<<0.0, 0.0, 0.0;
    }
    else{
        start_vel = data_vel;
        start_acc = data_acc;

    }

	if ((start_pt - goal_pt).norm() < 0.4) {
		// cout << "Close goal" << endl;
        plan.push_back(start);
        plan.push_back(goal);
        publishPlan(plan);
		return true;
	}
	
    kino_planner_->reset();

	int status = kino_planner_->search(start_pt, start_vel, start_acc, goal_pt, goal_vel, true);
    // cout<<"acc "<<start_acc(0)<<" "<<start_acc(1)<<endl;
    // cout<<"odom_acc "<<data_acc(0)<<" "<<data_acc(1)<<endl;
	if (status == KinodynamicAstar::NO_PATH) {
		cout << "[kino plan]: kinodynamic search fail!" << endl;

		// retry searching with discontinuous initial state
		kino_planner_->reset();
		status = kino_planner_->search(start_pt, start_vel, start_acc, goal_pt, goal_vel, false);
        //status = KinodynamicAstar::NO_PATH;
		if (status == KinodynamicAstar::NO_PATH) {
			cout << "[kino replan]: Can't find path." << endl;
			return false;
		}
		else {
			cout << "[kino replan]: retry search success." << endl;
		}

	}
	else {
		cout << "[kino replan]: kinodynamic search success." << endl;
	}

    acc_flag = true;

    

//----------------------------------------------------------------------------------------------------
//     if(!old_navfn_behavior_)
//         planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
//     if(publish_potential_)
//         publishPotential(potential_array_);

    // if (found_legal) {
    if (status != KinodynamicAstar::NO_PATH) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            //cout<<"get path done"<<endl;
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    // orientation_filter_->processPath(start, plan);

    //publish the plan for visualization purposes
    publishPlan(plan);
    //delete potential_array_;

    
    return !plan.empty();
}

void KinoAstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool KinoAstarPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

//     if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
//         ROS_ERROR("NO PATH!");
//         return false;
//     }
    bool path_flag = false;

    if(path_flag){
        if (!kino_planner_->getPath(start_x, start_y, goal_x, goal_y, path)) {
		    ROS_ERROR("NO PATH!");
		    return false;
        }
        cout<<path.size()<<endl;
        for(int i=0;i<path.size();i++){
            cout<<path[i].first<<" "<<path[i].second<<endl;
        }
        cout<<"Done"<<endl;

        ros::Time plan_time = ros::Time::now();
        for (int i = path.size() -1; i>=0; i--) {
            std::pair<float, float> point = path[i];
            //convert the plan to world coordinates
            double world_x, world_y;
            mapToWorld(point.first, point.second, world_x, world_y);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }   

    }
    else{
        vector<Eigen::Vector3d> st = kino_planner_->getKinoTraj(0.01);
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i <= st.size() - 1; i++){
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame;
            pose.pose.position.x = st[i](0);
            pose.pose.position.y = st[i](1);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }
    }


//     if(old_navfn_behavior_){
//             plan.push_back(goal);
//     }
//cout<<"Get"<<endl;
    return !plan.empty();
}

void KinoAstarPlanner::odomCB(const nav_msgs::Odometry::ConstPtr &odom){
    double cur_t = odom->header.stamp.toSec();
    Eigen::Vector3d cur_v;
    cur_v(0) = odom->twist.twist.linear.x;
    cur_v(1) = odom->twist.twist.linear.y;
    cur_v(2) = 0;
    // cout<<"cur_v"<<cur_v(0)<<" "<<cur_v(1)<<endl;
    data_acc(0) = (cur_v(0) - data_vel(0))/(cur_t - acc_time);
    data_acc(1) = (cur_v(1) - data_vel(1))/(cur_t - acc_time);
    data_acc(2) = 0;
    if(data_acc(0) < 1e-3){
        data_acc(0) = 0;
    }
    if(data_acc(1) < 1e-3){
        data_acc(1) = 0;
    }
    // cout<<"cur_a"<<data_acc(0)<<" "<<data_acc(1)<<endl;

    data_vel(0) = cur_v(0);
    data_vel(1) = cur_v(1);
    data_vel(2) = cur_v(2);

    acc_time = cur_t;

}




}//end namespace KinoAstar_planner