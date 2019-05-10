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

                Eigen::Vector3d plane_normal = (Eigen::Vector3d() << coeff_refined(0), coeff_refined(1), coeff_refined(2)).finished();

                Eigen::Vector3d frame_normal = (Eigen::Vector3d() << 0,0,1).finished();

                Eigen::Matrix3d rot_mat = get_rotation_mat(plane_normal, frame_normal);

                unsigned int id = 0;
                rot_mat_vec_.push_back(std::make_pair(id, rot_mat));
        }


        att_pub_ = nh.advertise<attitude_estimation::AttitudeEstVec>( "pc_attitude_estimation", 10, false);
        send_attitude_est(res, att_pub_);

        rot_mat_vec_.clear();
        res.status = 1;
        return true;
}

//////////////////////////
// Start Helper Methods //
//////////////////////////

// publish attitude message
void AttitudeEstimation::send_attitude_est(attitude_estimation::GetAttitudeEst::Response &res, clusters_att& att_est, const ros::Publisher& pub)
{
        attitude_estimation::AttitudeEst est;
        attitude_estimation::AttitudeEstVec est_vec;

        auto timestamp = ros::Time::now();
        for (int i=0; i<att_est.size(); i++)
        {
                est.id = rot_mat_vec_[i].first;

                est.header.frame_id = "world";
                est.header.stamp = timestamp;

                auto r = rot_mat_vec_[i].second;

                auto e = get_euler(r);
                est.euler.x = e[0];
                est.euler.y = e[1];
                est.euler.z = e[2];

                auto q = get_quat(r);
                est.quat.orientation.w = q[0];
                est.quat.orientation.x = q[1];
                est.quat.orientation.y = q[2];
                est.quat.orientation.z = q[3];

                est_vec.AttitudeEstVec.push_back(est);
                res.AttitudeEstVec.push_back(est);
        }
        if(pub.getNumSubscribers() != 0) { pub.publish(est_vec); }
        est_vec.AttitudeEstVec.clear();
}


// rotate estimated normal vector to the basis normal vector
Eigen::Matrix3d AttitudeEstimation::get_rotation_mat(Eigen::Vector3f n1, Eigen::Vector3f n2)
{

        // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        Eigen::Vector3d v = n1.cross(n2);
        double c = n1.dot(n2);
        double s = v.norm();

        Eigen::Matrix3d skew_symm;
        skew_symm << (Matrix3d() << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0 ).finished();

        // Rodrigues' Formula for rotation matrix
        // see "Constructing Rotation Matrices using Power Series" p.g., 11
        return Eigen::MatrixXd::Identity(3,3) + skew_symm + (skew_symm.pow(2) * ((1-c)/s*s) );
}

// get quaternion representation from rotation matrix
Eigen::Quaterniond AttitudeEstimation::get_quat(Eigen::Matrix3d rot) {
        Quaterniond q;
        return q=mat;
}

// get euler representation from rotation matrix
Eigen::Vector3d AttitudeEstimation::get_euler(Eigen::Matrix3d rot) {
        return rot.eulerAngles(2,1,0);
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
