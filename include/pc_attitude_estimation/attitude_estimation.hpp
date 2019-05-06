#include <ros/ros.h>
#include <ros/package.h>


#include <geometry_msgs/Pose.h>
#include <manipulation_mapping/Flower.h>
#include <manipulation_mapping/FlowerMap.h>
#include <manipulation_mapping/UpdateFlowerMap.h>

#include <tuple>
#include <string>
#include <cassert>
#include <utility>
#include <Eigen/Dense>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace manipulation
{

class FlowerMapper {
public:
FlowerMapper();

bool update_flower_map (manipulation_mapping::UpdateFlowerMap::Request  &req,
                        manipulation_mapping::UpdateFlowerMap::Response &res);

gtsam::Pose3 pose_msg_to_gtsam(const geometry_msgs::Pose& fp);
unsigned int get_flower_index(const gtsam::Pose3& fp);
void update_pose_graph(const unsigned int& flower_index, const gtsam::Pose3 flower_pose_est, const double& flower_prob);
void update_position_graph(const unsigned int& flower_index, const gtsam::Point3 flower_position_est, const double& flower_prob);
void send_flower_map(manipulation_mapping::UpdateFlowerMap::Response &res, const ros::Publisher& pub);
bool load_parameters(const ros::NodeHandle& nh);

ros::NodeHandle nh;

ros::ServiceServer flowerMapperServ;

// {flower_index, graph_key, factor graph, optimized values, number of times observed, accumulated prob. of flower }
typedef std::vector< std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double> > factor_graphs;
// {flower_index,  flower_pose}
typedef std::vector< std::pair<unsigned int, gtsam::Pose3> >flower_map;


private:

factor_graphs graphs_;
flower_map map_;

ros::Publisher map_pub_;

typedef gtsam::noiseModel::Isotropic isoNoise;
typedef gtsam::noiseModel::Diagonal diagNoise;

// number of flowers currently represented in the map.
unsigned int flower_count_ = 0;

// filter noise params
double noise_position_classifier, noise_attitude_classifier, noise_position_process, noise_attitude_process, flower_distance_thresh;

bool position_only; // Only estimate position -- default is full 3d pose
bool make_robust; // place holder for now -- will add robust cost function later.
std::string world_frame;
int observation_thresh;

const double DISTANCE_THRESHOLD = 5.0;

};

}
