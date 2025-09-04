namespace delto_gripper_helper {


double GetLibraryVersion();

//current Unit [mA]
std::vector<double> ConvertEffort(const std::vector<double>& current);

std::vector<double> CurrentControl(int joint_count,
                                   const std::vector<int> &actual_current,
                                   const std::vector<double> &target_torque,
                                   std::vector<int> &current_limit_flag,
                                   std::vector<double> &current_integral);

std::vector<double> ConvertDuty(int joint_count,
                                std::vector<double> target_torque);
                                
}  // namespace delto_gripper_helper
