#include <manipulation_mapping/flower_mapper.hpp>
#include <parameter_utils/ParameterUtils.h>

namespace pu = parameter_utils;
using gtsam::Rot3;
using gtsam::Pose3;
using gtsam::Point3;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::Vector4;
using gtsam::Vector6;
using gtsam::PriorFactor;
using gtsam::BetweenFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::LevenbergMarquardtOptimizer;


namespace manipulation
{

//----------------------------------------------------------------------------
/**
 * Constructor
 */
FlowerMapper::
FlowerMapper()
{
        load_parameters(nh);
        //server
        flowerMapperServ = nh.advertiseService("flower_mapper", &FlowerMapper::update_flower_map, this);
}

//----------------------------------------------------------------------------
/**
 * Server for updating flower map given flower observations
 */
bool FlowerMapper::
update_flower_map (manipulation_mapping::UpdateFlowerMap::Request  &req,
                   manipulation_mapping::UpdateFlowerMap::Response &res)
{
        res.status = 0;
        std::vector<double> flower_probs; //this will be loaded from req
        for(int i=0; i<req.poses.size(); i++)
        {
                gtsam::Pose3 flower_pose_est = pose_msg_to_gtsam(req.poses[i]);
                int flower_index = get_flower_index(flower_pose_est);//flower_probs[i]);
                if (position_only) {
                        gtsam::Point3 flower_position_est = flower_pose_est.translation();
                        update_position_graph(flower_index, flower_position_est, 0.5);
                }
                else{ update_pose_graph(flower_index, flower_pose_est, 0.5); }
        }

        map_pub_ = nh.advertise<manipulation_mapping::FlowerMap>( "flower_map", 10, false);
        send_flower_map(res, map_pub_);

        res.status = 1;
        return true;
}

//////////////////////////
// Start Helper Methods //
//////////////////////////
gtsam::Pose3 FlowerMapper::pose_msg_to_gtsam(const geometry_msgs::Pose& flower_pose_est)
{

        gtsam::Point3 point(flower_pose_est.position.x,
                            flower_pose_est.position.y,
                            flower_pose_est.position.z);

        gtsam::Rot3 rot = gtsam::Rot3::quaternion( flower_pose_est.orientation.w,
                                                   flower_pose_est.orientation.x,
                                                   flower_pose_est.orientation.y,
                                                   flower_pose_est.orientation.z);

        return gtsam::Pose3(rot, point);
}

unsigned int FlowerMapper::get_flower_index(const gtsam::Pose3& flower_pose_est)
{
        // If this is the first flower observed, then create map
        if (map_.size() == 0)
        {
                int flower_index = ++flower_count_;
                map_.push_back(std::make_pair(flower_index, flower_pose_est));
                return flower_index;
        }
        else
        {
                // Look at all flowers in map to search for a match
                for (int i=0; i<map_.size(); i++)
                {
                        unsigned int current_flower_index = map_[i].first;
                        gtsam::Pose3 current_flower_pose = map_[i].second;

                        gtsam::Point3 current_position = current_flower_pose.translation();
                        gtsam::Point3 test_position = flower_pose_est.translation();

                        double dist = current_position.distance(test_position);
                        if (dist <= flower_distance_thresh)
                        {
                                auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [current_flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double>&g) {return std::get<0>(g) == current_flower_index; });
                                if ( graph_iter != graphs_.end() )
                                {
                                    ++std::get<4>(*graph_iter);
                                }
                                return current_flower_index;

                        }
                }
                // If newly observed flower, then add to map
                unsigned int flower_index = ++flower_count_;
                map_.push_back(std::make_pair(flower_index, flower_pose_est));
                return flower_index;
        }
}

void FlowerMapper::update_pose_graph(const unsigned int& flower_index, const gtsam::Pose3 flower_pose_est, const double& flower_prob)
{

        using gtsam::symbol_shorthand::X;

        auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double>&g) {return std::get<0>(g) == flower_index; });

        auto map_iter = std::find_if(map_.begin(), map_.end(), [flower_index] (const std::pair<unsigned int, gtsam::Pose3>&g) {return std::get<0>(g) == flower_index; });


        if ( graph_iter == graphs_.end() )
        {

                unsigned int key = 0;
                using gtsam::symbol_shorthand::X; // Pose3 {point, quat.}
                gtsam::Values values;
                gtsam::NonlinearFactorGraph graph;

                diagNoise::shared_ptr measurement_noise = diagNoise::Sigmas(( gtsam::Vector(6) << noise_attitude_classifier, noise_attitude_classifier, noise_attitude_classifier, noise_position_classifier, noise_position_classifier, noise_position_classifier ).finished());

                values.insert(X(key), flower_pose_est);

                // add prior factor from classifier
                graph.add(PriorFactor<gtsam::Pose3>(X(key), flower_pose_est, measurement_noise));

                graphs_.push_back( std::make_tuple(flower_index, key, graph, values, 1, flower_prob ));
                return;
        }
        else
        {

                ++std::get<1>(*graph_iter);

                diagNoise::shared_ptr measurement_noise = diagNoise::Sigmas(( gtsam::Vector(6) << noise_attitude_classifier, noise_attitude_classifier, noise_attitude_classifier, noise_position_classifier, noise_position_classifier, noise_position_classifier ).finished());

                diagNoise::shared_ptr process_noise = diagNoise::Sigmas(( gtsam::Vector(6) << noise_attitude_process, noise_attitude_process, noise_attitude_process, noise_position_process, noise_position_process, noise_position_process ).finished());

                unsigned int key = std::get<1>(*graph_iter);
                (std::get<3>(*graph_iter)).insert(X(key), flower_pose_est);


                // Add prior from classifier
                (std::get<2>(*graph_iter)).add(PriorFactor<gtsam::Pose3>(X(key), flower_pose_est, measurement_noise));

                // Process noise update
                gtsam::Pose3 process_est(gtsam::Rot3::RzRyRx(0,0,0), gtsam::Point3(0, 0, 0));
                (std::get<2>(*graph_iter)).add(BetweenFactor<gtsam::Pose3>( X(key), X(key-1), process_est, process_noise));

                // Optimizer graph
                std::get<3>(*graph_iter) = LevenbergMarquardtOptimizer(std::get<2>(*graph_iter), std::get<3>(*graph_iter)).optimize();

                std::get<5>(*graph_iter)*=flower_prob;
                gtsam::Pose3 pose_est = std::get<3>(*graph_iter).at<gtsam::Pose3>(X(key));

                if ( map_iter == map_.end() )
                {
                  map_.push_back(std::make_pair(flower_index, pose_est));
                }
                else
                {
                  std::get<1>(*map_iter) = pose_est;
                }
                // map_.push_back(std::make_pair(flower_index, pose_est));
        }
        return;


}


void FlowerMapper::update_position_graph(const unsigned int& flower_index, const gtsam::Point3 flower_position_est, const double& flower_prob)
{

        using gtsam::symbol_shorthand::X;

        auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double>&g) {return std::get<0>(g) == flower_index; });

        auto map_iter = std::find_if(map_.begin(), map_.end(), [flower_index] (const std::pair<unsigned int, gtsam::Pose3>&g) {return std::get<0>(g) == flower_index; });


        if ( graph_iter == graphs_.end() )
        {
                unsigned int key = 0;
                diagNoise::shared_ptr measurement_noise = diagNoise::Sigmas(( gtsam::Vector(3) <<  noise_position_classifier, noise_position_classifier, noise_position_classifier ).finished());

                gtsam::Values values;
                gtsam::NonlinearFactorGraph graph;

                values.insert(X(key), flower_position_est);

                // add prior factor from classifier
                graph.add(PriorFactor<gtsam::Point3>(X(key), flower_position_est, measurement_noise));

                graphs_.push_back( std::make_tuple(flower_index, key, graph, values, 1, flower_prob ));
                return;
        }
        else
        {
                gtsam::Values values;
                gtsam::NonlinearFactorGraph graph;

                ++std::get<1>(*graph_iter);

                diagNoise::shared_ptr measurement_noise = diagNoise::Sigmas(( gtsam::Vector(3) <<    noise_position_classifier, noise_position_classifier, noise_position_classifier ).finished());

                diagNoise::shared_ptr process_noise = diagNoise::Sigmas(( gtsam::Vector(3) <<   noise_position_process, noise_position_process, noise_position_process ).finished());

                unsigned int key = std::get<1>(*graph_iter);
                std::get<3>(*graph_iter).insert(X(key), flower_position_est);

                // Add prior from classifier
                (std::get<2>(*graph_iter)).add(PriorFactor<gtsam::Point3>(X(key), flower_position_est, measurement_noise));

                // Process noise update
                gtsam::Point3 process_est(gtsam::Point3(0, 0, 0));
                (std::get<2>(*graph_iter)).add(BetweenFactor<gtsam::Point3>( X(key), X(key-1), process_est, process_noise));

                // Optimizer graph
                std::get<3>(*graph_iter) = LevenbergMarquardtOptimizer(std::get<2>(*graph_iter), std::get<3>(*graph_iter)).optimize();


                gtsam::Point3 position_est = std::get<3>(*graph_iter).at<gtsam::Point3>(X(key));
                gtsam::Pose3 pose_est(gtsam::Rot3::RzRyRx(0,0,0), position_est);

                std::get<5>(*graph_iter)*=flower_prob;
                if ( map_iter == map_.end() )
                {
                  map_.push_back(std::make_pair(flower_index, pose_est));
                }
                else
                {
                  std::get<1>(*map_iter) = pose_est;
                }
        }
        return;

}


void FlowerMapper::send_flower_map(manipulation_mapping::UpdateFlowerMap::Response &res, const ros::Publisher& pub)
{
        manipulation_mapping::Flower flower;
        manipulation_mapping::FlowerMap flower_map;
        auto timestamp = ros::Time::now();
        for (int i=0; i<map_.size(); i++)
        {
                flower.id = map_[i].first;
                unsigned int flower_index = map_[i].first;
                gtsam::Pose3 flower_pose_est = map_[i].second;
                auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double>&g) {return std::get<0>(g) == flower_index; });

                if (std::get<4>(*graph_iter)>=observation_thresh)
                {
                  gtsam::Vector3 position = flower_pose_est.translation();
                  gtsam::Vector4 attitude = flower_pose_est.rotation().quaternion();

                  flower.header.frame_id = world_frame;
                  flower.header.stamp = timestamp;

                  flower.num_obs = std::get<4>(*graph_iter);
                  flower.prob = std::get<5>(*graph_iter);

                  flower.pose.position.x = position[0];
                  flower.pose.position.y = position[1];
                  flower.pose.position.z = position[2];

                  flower.pose.orientation.w = attitude[0];
                  flower.pose.orientation.x = attitude[1];
                  flower.pose.orientation.y = attitude[2];
                  flower.pose.orientation.z = attitude[3];

                  flower_map.map.push_back(flower);
                  res.map.push_back(flower);
              }
        }
        if(pub.getNumSubscribers() != 0) { pub.publish(flower_map); }
        flower_map.map.clear();
}

bool FlowerMapper::load_parameters(const ros::NodeHandle& nh){

        // // Load filter noise specs
        if(!pu::Get("filter/noise_position_classifier", noise_position_classifier)) return false;
        if(!pu::Get("filter/noise_attitude_classifier", noise_attitude_classifier)) return false;
        if(!pu::Get("filter/noise_position_process", noise_position_process)) return false;
        if(!pu::Get("filter/noise_attitude_process", noise_attitude_process)) return false;
        if(!pu::Get("filter/flower_distance_thresh", flower_distance_thresh)) return false;
        if(!pu::Get("filter/position_only", position_only)) return false;
        if(!pu::Get("filter/make_robust", make_robust)) return false;
        if(!pu::Get("filter/world_frame", world_frame)) return false;
        if(!pu::Get("filter/observation_thresh", observation_thresh)) return false;

        return true;
}
///////////////////////
// End Helper Methods//
///////////////////////



} // End Namespace
