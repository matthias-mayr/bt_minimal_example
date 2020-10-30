#include <bt_minimal_example/bt_policy.hpp>

int main() {
    BTPolicy policy;
    Eigen::VectorXd params{Eigen::VectorXd::Zero(5)};
    params << 1,2,3,4,5;

    policy.set_params(params);

    Eigen::VectorXd state{Eigen::VectorXd::Zero(2)};

    std::cout << "First tick. Expected action is 3." << std::endl;
    policy.next(state);

    std::cout << "\nSecond tick. Expected action is 3." << std::endl;
    policy.next(state);

    std::cout << "\nThird tick. Expected action is 3." << std::endl;
    policy.next(state);

    state[0] = 3;
    std::cout << "\n4th tick. Expected action is 5." << std::endl;
    policy.next(state);

    std::cout << "\n5th tick. Expected action is 5." << std::endl;
    policy.next(state);

    return 0;
}