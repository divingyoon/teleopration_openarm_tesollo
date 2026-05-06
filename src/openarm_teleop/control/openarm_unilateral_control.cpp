// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <thread>
#include <yamlloader.hpp>

#include <fstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting loop..." << std::endl;
        keep_running = false;
    }
}

class LeaderArmThread : public PeriodicTimerThread {
public:
    LeaderArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_l,
                    double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_l_(control_l) {}

protected:
    void before_start() override { std::cout << "leader start thread " << std::endl; }

    void after_stop() override { std::cout << "leader stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();

        control_l_->unilateral_step();

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;
        (void)elapsed_us;

        // std::cout << "[Leader] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_l_;
};

class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_f,
                      double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f) {}

protected:
    void before_start() override { std::cout << "follower start thread " << std::endl; }

    void after_stop() override { std::cout << "follower stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();

        control_f_->unilateral_step();

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;
        (void)elapsed_us;

        // std::cout << "[Follower] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_f_;
};

class AdminThread : public PeriodicTimerThread {
public:
    AdminThread(std::shared_ptr<RobotSystemState> leader_state,
                std::shared_ptr<RobotSystemState> follower_state, Control *control_l,
                Control *control_f, std::shared_ptr<rclcpp::Node> ros_node,
                const std::string &arm_side, double hz = 500.0)
        : PeriodicTimerThread(hz),
          leader_state_(leader_state),
          follower_state_(follower_state),
          control_l_(control_l),
          control_f_(control_f),
          first_tick_(true),
          ros_node_(ros_node),
          arm_side_(arm_side)
    {
        prefix_ = (arm_side_ == "left_arm") ? "left" : "right";
        const std::string base_topic = "/openarm/" + prefix_;
        leader_joint_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
            base_topic + "/leader/joint_states", 10);
        follower_joint_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
            base_topic + "/follower/joint_states", 10);
        joint_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
            base_topic + "/joint_states", 10);
        leader_gripper_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
            base_topic + "/leader/gripper_state", 10);

        std::cout << ">>> [ROS 2] Publisher created on topic: "
                  << base_topic + "/leader/joint_states" << std::endl;
        std::cout << ">>> [ROS 2] Publisher created on topic: "
                  << base_topic + "/follower/joint_states" << std::endl;
        std::cout << ">>> [ROS 2] Publisher created on topic: "
                  << base_topic + "/joint_states" << std::endl;
        std::cout << ">>> [ROS 2] Publisher created on topic: "
                  << base_topic + "/leader/gripper_state" << std::endl;
    }

protected:
    void before_start() override { 
        std::cout << "admin start thread " << std::endl; 
        
        csv_file_.open("follower_demo_data.csv");
        if(csv_file_.is_open()) {
            std::cout << "[Logger] Started recording to follower_demo_data.csv" << std::endl;
        } else {
            std::cerr << "[Logger Error] Failed to open CSV file!" << std::endl;
        }
        start_time_ = std::chrono::steady_clock::now();
        first_tick_ = true;
    }

    void after_stop() override { 
        std::cout << "admin stop thread " << std::endl; 
        
        if (csv_file_.is_open()) {
            csv_file_.close();
            std::cout << "[Logger] Stopped recording. Data saved." << std::endl;
        }
    }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();

        auto leader_arm_resp = leader_state_->arm_state().get_all_responses();
        auto follower_arm_resp = follower_state_->arm_state().get_all_responses();
        auto leader_hand_resp = leader_state_->hand_state().get_all_responses();
        auto follower_hand_resp = follower_state_->hand_state().get_all_responses();

        sensor_msgs::msg::JointState follower_msg;
        sensor_msgs::msg::JointState leader_msg;
        follower_msg.header.stamp = ros_node_->now();
        leader_msg.header.stamp = follower_msg.header.stamp;

        for(size_t i = 0; i < follower_arm_resp.size(); ++i) {
            follower_msg.name.push_back(prefix_ + "_follower_arm_joint_" + std::to_string(i));
            follower_msg.position.push_back(follower_arm_resp[i].position);
            follower_msg.velocity.push_back(follower_arm_resp[i].velocity);
            follower_msg.effort.push_back(follower_arm_resp[i].effort);
        }

        for(size_t i = 0; i < follower_hand_resp.size(); ++i) {
            follower_msg.name.push_back(prefix_ + "_follower_hand_joint_" + std::to_string(i));
            follower_msg.position.push_back(follower_hand_resp[i].position);
            follower_msg.velocity.push_back(follower_hand_resp[i].velocity);
            follower_msg.effort.push_back(follower_hand_resp[i].effort);
        }

        for (size_t i = 0; i < leader_arm_resp.size(); ++i) {
            leader_msg.name.push_back(prefix_ + "_leader_arm_joint_" + std::to_string(i));
            leader_msg.position.push_back(leader_arm_resp[i].position);
            leader_msg.velocity.push_back(leader_arm_resp[i].velocity);
            leader_msg.effort.push_back(leader_arm_resp[i].effort);
        }

        for (size_t i = 0; i < leader_hand_resp.size(); ++i) {
            leader_msg.name.push_back(prefix_ + "_leader_hand_joint_" + std::to_string(i));
            leader_msg.position.push_back(leader_hand_resp[i].position);
            leader_msg.velocity.push_back(leader_hand_resp[i].velocity);
            leader_msg.effort.push_back(leader_hand_resp[i].effort);
        }

        leader_joint_pub_->publish(leader_msg);
        follower_joint_pub_->publish(follower_msg);
        joint_pub_->publish(follower_msg);

        if (leader_gripper_pub_ && !leader_hand_resp.empty()) {
            sensor_msgs::msg::JointState gripper_msg;
            gripper_msg.header.stamp = leader_msg.header.stamp;
            gripper_msg.name.push_back(prefix_ + "_leader_gripper_joint_0");
            gripper_msg.position.push_back(leader_hand_resp[0].position);
            gripper_msg.velocity.push_back(leader_hand_resp[0].velocity);
            gripper_msg.effort.push_back(leader_hand_resp[0].effort);
            leader_gripper_pub_->publish(gripper_msg);
        }

        static int pub_count = 0;
        if (pub_count++ % 500 == 0) { // 500Hz loop, print about every second
            std::cout << ">>> [ROS 2] Publishing /openarm/" << prefix_
                      << "/leader|follower|joint_states... (" << pub_count << " times)"
                      << std::endl;
        }

        if (csv_file_.is_open()) {
            // 첫 루프에서 CSV 헤더(첫 줄) 작성
            if (first_tick_) {
                csv_file_ << "time_sec";
                for(size_t i = 0; i < follower_arm_resp.size(); ++i) {
                    csv_file_ << ",follower_arm_pos_" << i
                              << ",follower_arm_vel_" << i
                              << ",follower_arm_eff_" << i;
                }
                for(size_t i = 0; i < follower_hand_resp.size(); ++i) {
                    csv_file_ << ",follower_hand_pos_" << i
                              << ",follower_hand_vel_" << i
                              << ",follower_hand_eff_" << i;
                }
                csv_file_ << "\n";
                first_tick_ = false;
            }

            double t = std::chrono::duration<double>(now - start_time_).count();
            csv_file_ << std::fixed << std::setprecision(6) << t;

            for(const auto& joint : follower_arm_resp) {
                csv_file_ << "," << joint.position << "," << joint.velocity << "," << joint.effort;
            }
            for(const auto& joint : follower_hand_resp) {
                csv_file_ << "," << joint.position << "," << joint.velocity << "," << joint.effort;
            }
            csv_file_ << "\n";
        }

        // set referense
        leader_state_->arm_state().set_all_references(follower_arm_resp);
        leader_state_->hand_state().set_all_references(follower_hand_resp);

        follower_state_->arm_state().set_all_references(leader_arm_resp);
        follower_state_->hand_state().set_all_references(leader_hand_resp);

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;
        (void)elapsed_us;

        // std::cout << "[Admin] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> leader_state_;
    std::shared_ptr<RobotSystemState> follower_state_;
    Control *control_l_;
    Control *control_f_;

    std::ofstream csv_file_;
    std::chrono::steady_clock::time_point start_time_;
    bool first_tick_;

    std::shared_ptr<rclcpp::Node> ros_node_;
    std::string arm_side_;
    std::string prefix_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr leader_joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr follower_joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr leader_gripper_pub_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("openarm_teleop_node");

    std::cout << ">>> ROS 2 Node Successfully Created: " << ros_node->get_name() << std::endl;  

    try {
        std::signal(SIGINT, signal_handler);

        // default configration
        std::string arm_side = "right_arm";
        std::string leader_urdf_path;
        std::string follower_urdf_path;
        std::string leader_can_interface = "can0";
        std::string follower_can_interface = "can2";

        if (argc < 3) {
            std::cerr
                << "Usage: " << argv[0]
                << " <leader_urdf_path> <follower_urdf_path> [arm_side] [leader_can] [follower_can]"
                << std::endl;
            return 1;
        }

        // Required: URDF paths
        leader_urdf_path = argv[1];
        follower_urdf_path = argv[2];

        // Optional: arm_side
        if (argc >= 4) {
            arm_side = argv[3];
            if (arm_side != "left_arm" && arm_side != "right_arm") {
                std::cerr << "[ERROR] Invalid arm_side: " << arm_side
                          << ". Must be 'left_arm' or 'right_arm'." << std::endl;
                return 1;
            }
        }

        // Optional: CAN interfaces
        if (argc >= 6) {
            leader_can_interface = argv[4];
            follower_can_interface = argv[5];
        }

        // URDF file existence check
        if (!std::filesystem::exists(leader_urdf_path)) {
            std::cerr << "[ERROR] Leader URDF not found: " << leader_urdf_path << std::endl;
            return 1;
        }
        if (!std::filesystem::exists(follower_urdf_path)) {
            std::cerr << "[ERROR] Follower URDF not found: " << follower_urdf_path << std::endl;
            return 1;
        }

        // Setup dynamics
        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        // Output confirmation
        std::cout << "=== OpenArm Unilateral Control ===" << std::endl;
        std::cout << "Arm side         : " << arm_side << std::endl;
        std::cout << "Leader CAN       : " << leader_can_interface << std::endl;
        std::cout << "Follower CAN     : " << follower_can_interface << std::endl;
        std::cout << "Leader URDF path : " << leader_urdf_path << std::endl;
        std::cout << "Follower URDF path: " << follower_urdf_path << std::endl;
        std::cout << "Root link         : " << root_link << std::endl;
        std::cout << "Leaf link         : " << leaf_link << std::endl;

        const std::string cfg_dir = "config/" + arm_side + "/";
        YamlLoader leader_loader(cfg_dir + "leader.yaml");
        YamlLoader follower_loader(cfg_dir + "follower.yaml");

        // Leader parameters
        std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> leader_k = leader_loader.get_vector("LeaderArmParam", "k");
        std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");

        // Follower parameters
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");

        Dynamics *leader_arm_dynamics = new Dynamics(leader_urdf_path, root_link, leaf_link);
        leader_arm_dynamics->Init();

        Dynamics *follower_arm_dynamics = new Dynamics(follower_urdf_path, root_link, leaf_link);
        follower_arm_dynamics->Init();

        std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *leader_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(leader_can_interface, true);

        std::cout << "=== Initializing Follower OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *follower_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(follower_can_interface, true);

        size_t leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
        size_t follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
        size_t leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
        size_t follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();

        std::cout << "leader arm motor num : " << leader_arm_motor_num << std::endl;
        std::cout << "follower arm motor num : " << follower_arm_motor_num << std::endl;
        std::cout << "leader hand motor num : " << leader_hand_motor_num << std::endl;
        std::cout << "follower hand motor num : " << follower_hand_motor_num << std::endl;

        // Declare robot_state (Joint and motor counts are assumed to be equal)
        std::shared_ptr<RobotSystemState> leader_state =
            std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);

        std::shared_ptr<RobotSystemState> follower_state =
            std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);

        Control *control_leader = new Control(
            leader_openarm, leader_arm_dynamics, follower_arm_dynamics, leader_state,
            1.0 / FREQUENCY, ROLE_LEADER, arm_side, leader_arm_motor_num, leader_hand_motor_num);
        Control *control_follower =
            new Control(follower_openarm, leader_arm_dynamics, follower_arm_dynamics,
                        follower_state, 1.0 / FREQUENCY, ROLE_FOLLOWER, arm_side,
                        follower_arm_motor_num, follower_hand_motor_num);

        control_leader->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv,
                                     leader_Fo);

        control_follower->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k,
                                       follower_Fv, follower_Fo);

        // set home postion
        std::thread thread_l(&Control::AdjustPosition, control_leader);
        std::thread thread_f(&Control::AdjustPosition, control_follower);
        thread_l.join();
        thread_f.join();

        std::thread ros_spin_thread([ros_node]() {
            std::cout << ">>> ROS 2 Spin Thread Started! Node: " << ros_node->get_name() << std::endl;
            rclcpp::executors::MultiThreadedExecutor executor;
            executor.add_node(ros_node);
            executor.spin(); 
        });
        
        LeaderArmThread leader_thread(leader_state, control_leader, FREQUENCY);
        FollowerArmThread follower_thread(follower_state, control_follower, FREQUENCY);
        AdminThread admin_thread(leader_state, follower_state, control_leader, control_follower,
                                 ros_node, arm_side, FREQUENCY);

        leader_thread.start_thread();
        follower_thread.start_thread();
        admin_thread.start_thread();

        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        leader_thread.stop_thread();
        follower_thread.stop_thread();
        admin_thread.stop_thread();

        leader_openarm->disable_all();
        follower_openarm->disable_all();

        rclcpp::shutdown();
        if (ros_spin_thread.joinable()) {
            ros_spin_thread.join();
        }

    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }

    rclcpp::shutdown();
    return 0;
}
