#include <NormalEKF.h>
#include <fstream>
namespace ly
{
    NormalEKF::NormalEKF(/* args */)
    {
        is_kalman_init = false;
        // 6个状态量，3个观测量
        kalman_filter = new ExtendedKalman<double, 6, 3>();
        // 后验状态
        posteriori_pose = Eigen::Vector3d::Zero();
        posteriori_speed = Eigen::Vector3d::Zero();
        // 过程噪声
        process_noice = Eigen::Matrix3d::Identity();
        process_noise_matrix = Eigen::Matrix<double, 6, 3>::Zero();
    }

    NormalEKF::~NormalEKF()
    {
    }
    void NormalEKF::rebootKalman(const Eigen::Vector3d &new_armor_pose)
    {
        // reboot Kalman
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->posteriori_state_estimate[i * 2] = new_armor_pose[i];
            kalman_filter->posteriori_state_estimate[i * 2 + 1] = 0;

            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];
            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 2 + 1];
        }
        kalman_filter->error_cov_post = Eigen::Matrix<double, 6, 6>::Identity();

        resetTransitionMatrix();
    }

    void NormalEKF::resetTransitionMatrix()
    {
        // x,x_v,y,y_v,z,z_v
        kalman_filter->transition_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    }
    void NormalEKF::setUpdateTime(const double &delta_t)
    {
        if (fabs(delta_t) < 1e-4) // 防止时间差为0
        {
            update_time = 8.0 / 1000.0;
        }
        else
        {
            update_time = delta_t / 1000.0;
        }
    }

    Eigen::Vector3d NormalEKF::runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t) // 量测有效更新
    {

        // 第一次初始化
        if (!is_kalman_init)
        {
            // set signal values
            is_kalman_init = true;

            // reset kalman
            rebootKalman(new_armor_pose);

            // return values
            return new_armor_pose;
        }
        else
        {
            // set update time
            setUpdateTime(delta_t);

            // update transition matrix
            setTransitionMatrix();

            // 返回滤波后的位置
            return correct(new_armor_pose);
        }
    }

    // 设置状态转移矩阵
    void NormalEKF::setTransitionMatrix()
    {
        Eigen::Matrix2d transition;
        transition << 1, update_time, 0, 1;
        for (int i = 0; i < 3; i++)
        {
            kalman_filter->transition_matrix.block<2, 2>(i * 2, i * 2) = transition;
        }
    }
    Eigen::Vector3d NormalEKF::correct(const Eigen::Vector3d &armor_pose)
    {   
        // 设置过程噪声
        setProcessNoise();

        // 设置测量噪声
        setMeasurementNoise(armor_pose); // 设置测量噪声
        
        // 测量得到pitch yaw distance
        Eigen::Vector3d pyd = measure(armor_pose);

        // std::fstream ss("../pyd.txt", std::ios::app);
        // ss << pyd << std::endl;
        // ss.close();
        // std::cout << "update_time:" << update_time << std::endl;

        if (update_time > 0.3) // 大于0.2s没有观测到数据，选择重启卡尔曼滤波
        {
            rebootKalman(armor_pose);
            return armor_pose;
        }

        // 左边那是转换函数func, 右边是基于当前测量值进行预测
        kalman_filter->predict(xyz_to_pyd, pyd); // 量测有效更新

        // 进行卡方检验
        detect_param = kalman_filter->ChiSquaredTest();

        /********************反小陀螺状态检测开始*******************/
        
        if (detect_param > VERIFY_THRESH) // 检验失败
        {
            rebootKalman(armor_pose);
            return armor_pose;
        }

        // 进行更新操作， 计算卡尔曼增益，计算后验状态误差估计
        kalman_filter->update();

        // 后验状态
        for (int i = 0; i < 3; i++)
        {
            // update armor status and return
            posteriori_pose[i] = kalman_filter->posteriori_state_estimate[i * 2];
            posteriori_speed[i] = kalman_filter->posteriori_state_estimate[i * 2 + 1];
        }
        return posteriori_pose;
    }

    Eigen::Vector3d NormalEKF::measure(const Eigen::Vector3d &armor_pose)
    {
        pyd[2] = armor_pose.norm();
        pyd[0] = ceres::atan2(armor_pose[2], sqrt(armor_pose[0] * armor_pose[0] + armor_pose[1] * armor_pose[1])); // pitch
        pyd[1] = ceres::atan2(armor_pose[0], armor_pose[1]);
        return pyd;
    }

    void NormalEKF::setMeasurementNoise(const Eigen::Vector3d &armor_pose)
    {
        // pitch,yaw,distance的噪声
        double measurement_noise_pose_pitch = 0.0001;
        double measurement_noise_pose_yaw = 0.0001;

        double distance = armor_pose.norm();
        double measurement_noise_pose_distance;

        if (distance < 1.5) // 统计方法计算，分段线性
        {
            measurement_noise_pose_distance = pow(distance * 0.01, 2);
        }
        else if (distance < 4.5)
        {
            measurement_noise_pose_distance = pow(0.015 + 0.058 * (distance - 1.5), 2);
        }
        else
        {
            measurement_noise_pose_distance = pow(0.189 + 0.03 * (distance - 4.5), 2);
        }

        kalman_filter->measurement_noise_cov.diagonal() << measurement_noise_pose_pitch,
            measurement_noise_pose_yaw,
            measurement_noise_pose_distance; // 3个轴的测量噪声，感觉三个轴的噪声需要根据PNP距离来计算
    }

    void NormalEKF::setProcessNoise()
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix.block<2, 1>(2 * i, i) = process_noice_vec;
        }

        process_noice.diagonal() << process_noise_pose_x,
            process_noise_pose_y,
            process_noise_pose_z; // 3个轴的过程噪声

        kalman_filter->process_noise_cov = process_noise_matrix * process_noice * process_noise_matrix.transpose();
    }
    void NormalEKF::setProcessNoise(double x, double y, double z)
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix.block<2, 1>(2 * i, i) = process_noice_vec;
        }
        process_noice.diagonal() << x, y, z;
        // DLOG(WARNING) << "Q++++++++++++++++++++++++++++++++++++++++++" << process_noise_matrix * process_noice * process_noise_matrix.transpose() << endl;
        kalman_filter->process_noise_cov = process_noise_matrix * process_noice * process_noise_matrix.transpose();
    }
    
    // 预测
    Eigen::Vector3d NormalEKF::predict(const double &predict_t)
    {
        return posteriori_pose + posteriori_speed * predict_t;
    }

}