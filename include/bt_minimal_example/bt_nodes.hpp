#ifndef BT_NODES_HPP
#define BT_NODES_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/Pose.h>

namespace bt_learning_nodes {

    class EEYThreshold : public BT::ConditionNode {
    public:
        EEYThreshold(const std::string& name, const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() { 
            return {
                BT::InputPort<double>("threshold"),
                BT::InputPort<double>("ee_y")};
        }

        BT::NodeStatus tick() override {
            BT::Optional<double> threshold = getInput<double>("threshold");
            BT::Optional<double> ee_y = getInput<double>("ee_y");
            if (!threshold || !ee_y)
            {
                throw BT::RuntimeError("Missing required input [threshold] or [ee_y].");
            }
            if (ee_y.value() > threshold.value()) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }
    };

    class EEZThreshold : public BT::ConditionNode {
    public:
        EEZThreshold(const std::string& name, const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) {}

        static BT::PortsList providedPorts() { 
            return {
                BT::InputPort<double>("threshold"),
                BT::InputPort<double>("ee_z")};
        }

        BT::NodeStatus tick() override {
            BT::Optional<double> threshold = getInput<double>("threshold");
            BT::Optional<double> ee_z = getInput<double>("ee_z");
            if (!threshold || !ee_z)
            {
                throw BT::RuntimeError("Missing required input [threshold] or [ee_z].");
            }
            if (ee_z.value() > threshold.value()) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }
    };

    class SetMGGoal : public BT::ActionNodeBase
    {
    public:
        SetMGGoal(const std::string& name, const BT::NodeConfiguration& config) :
            BT::ActionNodeBase(name, config){}

        static BT::PortsList providedPorts() { return {
            BT::InputPort<double>("mp_param_x"),
            BT::OutputPort<double>("mg_trans_x"),
        }; }

        void halt()
        {
            setStatus(BT::NodeStatus::IDLE);
        }

        BT::NodeStatus tick() override
        {
            BT::Optional<double> param_x = getInput<double>("mp_param_x");
            if (!param_x)
            {
                throw BT::RuntimeError("missing required input [mg_param_x] ");
            }
            setOutput("mg_trans_x", param_x.value());
            return BT::NodeStatus::RUNNING;
        }
    };
} // namespace bt_learning_nodes
#endif
