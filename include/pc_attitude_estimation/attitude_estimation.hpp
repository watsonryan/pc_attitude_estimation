#include <ros/ros.h>
#include <ros/package.h>


#include <geometry_msgs/Pose.h>
#include <attitude_estimation/Flower.h>
#include <attitude_estimation/UpdateFlowerMap.h>

#include <tuple>
#include <string>
#include <cassert>
#include <utility>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

using namespace pcl;
using namespace pcl::io;

typedef SampleConsensusModelPlane<PointXYZ>::Ptr SampleConsensusModelPlanePtr;

namespace attitude
{

class AttitudeEstimation {
public:
AttitudeEstimation();

bool get_attitude_est(attitude_estimation::UpdateFlowerMap::Request  &req,
                      attitude_estimation::UpdateFlowerMap::Response &res);

void send_attitude_est(attitude_estimation::UpdateFlowerMap::Response &res, const ros::Publisher& pub);

bool load_parameters(const ros::NodeHandle& nh);

ros::NodeHandle nh;
ros::ServiceServer attitudeEstServ;

typedef std::vector< std::pair<unsigned, Eigen::Matrix3d> > clusters_att;

private:

ros::Publisher att_pub_;

// yaml defined input params.
double distance_thresh;
unsigned int estimator_type;
};

}
