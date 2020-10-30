#ifndef BT_POLICY_HPP
#define BT_POLICY_HPP

#include <iostream>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <bt_minimal_example/bt_nodes.hpp>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

struct BTPolicy {
public:
    BTPolicy()
    {
        BT::BehaviorTreeFactory factory;
        using namespace bt_learning_nodes;
        factory.registerNodeType<SetMGGoal>("SetMGGoal");
        factory.registerNodeType<EEYThreshold>("EEYThreshold");
        factory.registerNodeType<EEZThreshold>("EEZThreshold");

        _tree = std::make_shared<BT::Tree>(factory.createTreeFromFile("./res/bt.xml"));
    }

    Eigen::VectorXd next(const Eigen::VectorXd& state)
    {
        BT::StdCoutLogger logger_cout(*_tree.get());
     
        _tree->rootBlackboard()->set("ee_y", state[0]);
        _tree->rootBlackboard()->set("ee_z", state[1]);

        BT::NodeStatus status = _tree->tickRoot();
        
        Eigen::VectorXd action{Eigen::VectorXd::Zero(1)};
        try {
            _tree->rootBlackboard()->get("mg_trans_x", action[0]);
            std::cout << "Action is: " << action << std::endl;
        } catch (const std::runtime_error& e) {
            std::cout << "ERROR: Could not retrieve MG config from blackboard." << std::endl;
        }
        return action;
    }

    void set_params(const Eigen::VectorXd& params)
    {
        _params = params;
        double x_offset = -0.5;
        _tree->rootBlackboard()->set("z_threshold", _params[0]);
        _tree->rootBlackboard()->set("y_threshold", _params[1]);

        _tree->rootBlackboard()->set("mp_1_x", _params[2]);
        _tree->rootBlackboard()->set("mp_2_x", _params[3]);
        _tree->rootBlackboard()->set("mp_3_x", _params[4]);
    }

protected:
    std::shared_ptr<BT::Tree> _tree;
    Eigen::VectorXd _params {Eigen::VectorXd::Zero(5)};
};
#endif
