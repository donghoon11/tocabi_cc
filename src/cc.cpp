#include "cc.h"

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    // ControlVal_.setZero();

    // if (is_write_file_)
    // {
    //     if (is_on_robot_)
    //     {
    //         writeFile.open("/home/dyros/catkin_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
    //     }
    //     else
    //     {
    //         writeFile.open("/home/yong20/ros_ws/ros1/tocabi_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
    //     }
    //     writeFile << std::fixed << std::setprecision(8);
    // }

    initVariable();
    arm_sub_ = nh_.subscribe("/tocabi/Arm_pose", 100, &CustomController::ArmCallback, this);
}

// Eigen::VectorQd CustomController::getControl()
// {
//     return ControlVal_;
// }

void CustomController::initVariable()
{    
    q_dot_lpf_.setZero();

    torque_bound_ << 333, 232, 263, 289, 222, 166,
                    333, 232, 263, 289, 222, 166,
                    303, 303, 303, 
                    64, 64, 64, 64, 23, 23, 10, 10,
                    10, 10,
                    64, 64, 64, 64, 23, 23, 10, 10;  
                    
    q_init_ << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
                0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
                0.0, 0.0, 0.0,
                0.3, 0.3, 1.5, -1.27, -3.14, 0.0, -3.14, 0.0,
                0.0, 0.0,
                -0.3, -0.3, -1.5, 1.27, 3.14, 0.0, 3.14, 0.0;

    kp_.setZero();
    kv_.setZero();
    kp_.diagonal() <<   2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        6000.0, 10000.0, 10000.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                        100.0, 100.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
    // kp_.diagonal() /= 9.0;
    kv_.diagonal() << 15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        200.0, 100.0, 100.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                        2.0, 2.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;
    // kv_.diagonal() /= 3.0;
}

void CustomController::processNoise()
{
    time_cur_ = rd_cc_.control_time_us_ / 1e6;
    if (is_on_robot_)
    {
        q_vel_noise_ = rd_cc_.q_dot_virtual_.segment(6,MODEL_DOF);
        q_noise_= rd_cc_.q_virtual_.segment(6,MODEL_DOF);
        if (time_cur_ - time_pre_ > 0.0)
        {
            q_dot_lpf_ = DyrosMath::lpf<MODEL_DOF>(q_vel_noise_, q_dot_lpf_, 1/(time_cur_ - time_pre_), 4.0);
        }
        else
        {
            q_dot_lpf_ = q_dot_lpf_;
        }
    }
    else
    {
        std::random_device rd;  
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.00001, 0.00001);
        for (int i = 0; i < MODEL_DOF; i++) {
            q_noise_(i) = rd_cc_.q_virtual_(6+i) + dis(gen);
        }
        if (time_cur_ - time_pre_ > 0.0)
        {
            q_vel_noise_ = (q_noise_ - q_noise_pre_) / (time_cur_ - time_pre_);
            q_dot_lpf_ = DyrosMath::lpf<MODEL_DOF>(q_vel_noise_, q_dot_lpf_, 1/(time_cur_ - time_pre_), 4.0);
        }
        else
        {
            q_vel_noise_ = q_vel_noise_;
            q_dot_lpf_ = q_dot_lpf_;
        }
        q_noise_pre_ = q_noise_;
    }
    time_pre_ = time_cur_;
}


void CustomController::writeDesiredPos()
{
    arm_desired_pos_(0) = l_arm_shoulder1;
    arm_desired_pos_(1) = l_arm_shoulder2;
    arm_desired_pos_(2) = l_arm_shoulder3;
    arm_desired_pos_(3) = l_arm_arm;
    arm_desired_pos_(4) = l_arm_elbow;
    arm_desired_pos_(5) = l_arm_forearm;
    arm_desired_pos_(6) = l_arm_wrist1;
    arm_desired_pos_(7) = l_arm_wrist2;

    arm_desired_pos_(8) = r_arm_shoulder1;
    arm_desired_pos_(9) = r_arm_shoulder2;
    arm_desired_pos_(10) = r_arm_shoulder3;
    arm_desired_pos_(11) = r_arm_arm;
    arm_desired_pos_(12) = r_arm_elbow;
    arm_desired_pos_(13) = r_arm_forearm;
    arm_desired_pos_(14) = r_arm_wrist1;
    arm_desired_pos_(15) = r_arm_wrist2;
}


void CustomController::computeSlow()
{
    copyRobotData(rd_);
    if (rd_cc_.tc_.mode == 7)
    {
        if (rd_cc_.tc_init)
        {
            //Initialize settings for Task Control! 
            start_time_ = rd_cc_.control_time_us_;
            q_noise_pre_ = q_noise_ = q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
            time_cur_ = start_time_ / 1e6;
            time_pre_ = time_cur_ - 0.005;
            time_inference_pre_ = rd_cc_.control_time_us_ - (1/249.9)*1e6;

            rd_.tc_init = false;
            std::cout<<"cc mode 7"<<std::endl;
            torque_init_ = rd_cc_.torque_desired;

            processNoise();
        }
        processNoise();

        if ((rd_cc_.control_time_us_ - time_inference_pre_)/3.14e6 >= 1/250.0 - 1/10000.0)
        {
            writeDesiredPos();
            
            time_inference_pre_ = rd_cc_.control_time_us_;
        }

        for (int i = 0; i < MODEL_DOF; i++)
        {
            torque_desired_(i) = kp_(i,i) * (q_init_(i) - q_noise_(i)) - kv_(i,i)*q_vel_noise_(i);
        }
        
        // left 15 ~ 22
        for (int i = 15; i < 23; i++)
        {
            torque_desired_(i) = kp_(i,i) * (arm_desired_pos_(i-15) - q_noise_(i)) - kv_(i,i)*q_vel_noise_(i);
        }

        // right 25 ~ 32
        for (int i = 25; i < 33; i++)
        {
            torque_desired_(i) = kp_(i,i) * (arm_desired_pos_(i-17) - q_noise_(i)) - kv_(i,i)*q_vel_noise_(i);
        }

        rd_.torque_desired = torque_desired_;
    }
}

void CustomController::computeFast()
{
    // if (tc.mode == 10)
    // {s
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::ArmCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    l_arm_shoulder1 = DyrosMath::minmax_cut(msg->data[0], -2.09, 1.57);
    l_arm_shoulder2 = DyrosMath::minmax_cut(msg->data[1], -3.14, 3.14);
    l_arm_shoulder3 = DyrosMath::minmax_cut(msg->data[2], -1.92, 1.92);
    l_arm_arm = DyrosMath::minmax_cut(msg->data[3], -3.14, 3.14);
    l_arm_elbow = DyrosMath::minmax_cut(msg->data[4], -2.6, 1.57);
    l_arm_forearm = DyrosMath::minmax_cut(msg->data[5], -3.14, 3.14);
    l_arm_wrist1 = DyrosMath::minmax_cut(msg->data[6], -3.14, 3.14);
    l_arm_wrist2 = DyrosMath::minmax_cut(msg->data[7], -1.0, 1.0);

    r_arm_shoulder1 = DyrosMath::minmax_cut(msg->data[8], -1.57, 2.09);
    r_arm_shoulder2 = DyrosMath::minmax_cut(msg->data[9], -3.14, 3.14);
    r_arm_shoulder3 = DyrosMath::minmax_cut(msg->data[10], -1.92, 1.92);
    r_arm_arm = DyrosMath::minmax_cut(msg->data[11], -3.14, 3.14);
    r_arm_elbow = DyrosMath::minmax_cut(msg->data[12], -1.57, 2.6);
    r_arm_forearm = DyrosMath::minmax_cut(msg->data[13], -3.14, 3.14);
    r_arm_wrist1 = DyrosMath::minmax_cut(msg->data[14], -3.14, 3.14);
    r_arm_wrist2 = DyrosMath::minmax_cut(msg->data[15], -1.0, 1.0);
}