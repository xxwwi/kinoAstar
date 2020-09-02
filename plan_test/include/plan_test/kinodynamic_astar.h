#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
// #include "grad_spline/sdf_map.h"
// #include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include <queue>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include<costmap_2d/cost_values.h>

using namespace std;

namespace KinoAstar_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode {
public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;
  double cost;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
};
typedef PathNode* PathNodePtr;

class NodeComparator {
public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    // return node1->f_score+node1->cost > node2->f_score+node2->cost;
       return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>> data_4d_;

public:
  NodeHashTable(/* args */) {
  }
  ~NodeHashTable() {
  }
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstar {
private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
  std::vector<PathNodePtr> path_nodes_;
  vector<Eigen::Vector3d> state_lists;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
//   shared_ptr<SDFMap> sdf_map;
//   EDTEnvironment::Ptr edt_environment_;
  //use costmap
  costmap_2d::Costmap2D* costmap_;

  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_ = 0.25;
  double init_max_tau_ = 0.8;
  double max_vel_ = 3.0;
  double max_acc_ = 3.0;
  double w_time_ = 10.0;
  double horizon_;
  double lambda_heu_;
  double margin_;
  int allocate_num_;
  int check_num_;
  double tie_breaker_ = 1.0 + 1.0 / 10000;
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;
  unsigned int cx_, cy_;
  unsigned char* costs;
  unsigned char lethal_cost_;
  bool unknown_;
  //collision coefficient
  double c_effi_;


  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);
  
  double index2cost(Eigen::Vector3i index);

  double collisioncost(PathNodePtr cur_node, double t, Eigen::Vector3d um);

public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3 };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);
  void setLethalCost(unsigned char lethal_cost){
    lethal_cost_ = lethal_cost;
  };
  void setMaxTau(double max_tau){
    max_tau_ = max_tau;
  };
  void setInitMaxTau(double init_max_tau){
    init_max_tau_ = init_max_tau;
  };
  void setMaxVel(double max_vel){
    max_vel_ = max_vel;
  };
  void setMaxAcc(double max_acc){
    max_acc_ = max_acc;
  };
  void setWTime(double w_time){
    w_time_ = w_time;
  };
  void setHorizon(double horizon){
    horizon_ = horizon;
  };
  void setTimeResolution(double time_resolution){
    time_resolution_ = time_resolution;
  };
  void setLambdaHeu(double lambda_heu){
    lambda_heu_ = lambda_heu;
  };
  void setCheckNum(int check_num){
    check_num_ = check_num;
  };
  void setCeffi(double c_effi){
    c_effi_ = c_effi;
  };
//   void setEnvironment(const EDTEnvironment::Ptr& env);


  //costmap
  void setEnvironment(costmap_2d::Costmap2D* costmap_);
  //////////////////////
  bool getPath(double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
  int toIndex(int x,int y);

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar> Ptr;
};

}  // namespace KinoAstar_planner

#endif