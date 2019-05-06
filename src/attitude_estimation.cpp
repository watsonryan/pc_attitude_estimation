#include <pc_attitude_estimation/attitude_estimation.hpp>
#include <parameter_utils/ParameterUtils.h>

namespace pu = parameter_utils;

namespace attitude
{

//----------------------------------------------------------------------------
/**
 * Constructor
 */
AttitudeEstimation::
AttitudeEstimation()
{
        load_parameters(nh);
        //server
        attitudeEstServ = nh.advertiseService("pc_attitude_estimation", &AttitudeEstimation::get_attitude_est, this);
}

//----------------------------------------------------------------------------
/**
 * Server for estimating attide of point cloud.
 */
bool AttitudeEstimation::
get_attitude_est(attitude_estimation::GetAttitudeEst::Request  &req,
                 attitude_estimation::GetAttitudeEst::Response &res)
{
        res.status = 0;
        for(int i=0; i<req.point_clouds.size(); i++)
        {
                auto curr_point_cloud = req.point_clouds[i];

                SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (curr_point_cloud));

                switch (estimator_type)
                {
                case 0:
                        // RRAANSAC
                        // Chum, Ondrej, and Jirı Matas. "Randomized RANSAC with Td, d test." Proc. British Machine Vision Conference. Vol. 2. 2002.
                        RandomizedRandomSampleConsensus<PointXYZ> sac (model, distance_thresh);
                        break;
                case 1:
                        // LMeDs
                        // Fusiello, Andrea. "Elements of geometric computer vision." Available fro m: http://homepages. inf. ed. ac. uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO4/tutorial. html (2006).
                        // See Section 7.3.3
                        LeastMedianSquares<PointXYZ> sac (model, distance_thresh);
                        break;
                case 2;
                        // MLESAC
                        // Torr, Philip HS, and Andrew Zisserman. "MLESAC: A new robust estimator with application to estimating image geometry." Computer vision and image understanding 78.1 (2000): 138-156.
                        MaximumLikelihoodSampleConsensus<PointXYZ> sac (model, distance_thresh);
                        break;
                default:
                        LeastMedianSquares<PointXYZ> sac (model, distance_thresh);
                        break;
                }
                std::vector<int> sample, inliers;
                Eigen::VectorXf coeff, coeff_refined;

                sac.getModel (sample);
                sac.getInliers (inliers);
                sac.getModelCoefficients (coeff);

                model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
                cout << coeff_refined << endl;

                Vector3d plane_normal = (Vector3d() << coeff_refined(0), coeff_refined(1), coeff_refined(2)).finished();

                Vector3d frame_normal = (Vector3d() << 0,0,1).finished();

                Vector3d plane_attitude = get_attitude_euler(plane_normal, frame_normal);
        }


        // att_pub_ = nh.advertise<attitude_estimation::attitude_est>( "pc_attitude_estimation", 10, false);
        // send_attitude_est(res, att_pub_);

        res.status = 1;
        return true;
}

//////////////////////////
// Start Helper Methods //
//////////////////////////
// void AttitudeEstimation::send_attitude_est(attitude_estimation::GetAttitudeEst::Response &res, const ros::Publisher& pub)
// {
//         attitude_estimation::attitude_est est;
//         auto timestamp = ros::Time::now();
//         for (int i=0; i<map_.size(); i++)
//         {
//                 flower.id = map_[i].first;
//                 unsigned int flower_index = map_[i].first;
//                 gtsam::Pose3 flower_pose_est = map_[i].second;
//                 auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double>&g) {return std::get<0>(g) == flower_index; });
//
//                 if (std::get<4>(*graph_iter)>=observation_thresh)
//                 {
//                         gtsam::Vector3 position = flower_pose_est.translation();
//                         gtsam::Vector4 attitude = flower_pose_est.rotation().quaternion();
//
//                         flower.header.frame_id = world_frame;
//                         flower.header.stamp = timestamp;
//
//                         flower.num_obs = std::get<4>(*graph_iter);
//                         flower.prob = std::get<5>(*graph_iter);
//
//                         flower.pose.position.x = position[0];
//                         flower.pose.position.y = position[1];
//                         flower.pose.position.z = position[2];
//
//                         flower.pose.orientation.w = attitude[0];
//                         flower.pose.orientation.x = attitude[1];
//                         flower.pose.orientation.y = attitude[2];
//                         flower.pose.orientation.z = attitude[3];
//
//                         flower_map.map.push_back(flower);
//                         res.map.push_back(flower);
//                 }
//         }
//         if(pub.getNumSubscribers() != 0) { pub.publish(flower_map); }
//         flower_map.map.clear();
// }

Eigen::Vector3d AttitudeEstimation::get_attitude_euler(Eigen::Vector3f n1, Eigen::Vector3f n2)
{

        // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        Eigen::Vector3d v = n1.cross(n2);
        double c = n1.dot(n2);
        double s = v.norm();

        Eigen::Matrix3d skew_symm;
        skew_symm << (Matrix3d() << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0 ).finished();

        Eigen::Matrix3d rot_m =  Eigen::MatrixXd::Identity(3,3) + skew_symm + (skew_symm.pow(2) * ((1-c)/s*s) );

        return Vector3d euler = rot_m.eulerAngles(2, 1, 0);
}

bool AttitudeEstimation::load_parameters(const ros::NodeHandle& nh)
{
        // // Load filter noise specs
        if(!pu::Get("model/type", estimatior_type)) return false;
        if(!pu::Get("model/distance_thresh", distance_thresh)) return false;
        return true;
}
///////////////////////
// End Helper Methods//
///////////////////////



} // End Namespace
