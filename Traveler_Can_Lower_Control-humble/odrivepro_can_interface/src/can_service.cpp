#include "odrivepro_can_includes/can_service.hpp"

CanService::CanService(/* args */) : Node("can_service"),
                                     socket_get_motor_error_(odrive_can::Msg::MSG_GET_MOTOR_ERROR),
                                     socket_get_encoder_error_(odrive_can::Msg::MSG_GET_ENCODER_ERROR),
                                     socket_get_encoder_estimates_(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES),
                                     socket_get_encoder_count_(odrive_can::Msg::MSG_GET_ENCODER_COUNT),
                                     socket_get_iq_(odrive_can::Msg::MSG_GET_IQ),
                                     socket_get_temperature_(odrive_can::Msg::MSG_GET_TEMPERATURE),
                                     socket_get_vbus_voltage_(odrive_can::Msg::MSG_GET_VBUS_VOLTAGE),
                                     socket_set_position_(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID, 0x7FF, 500),
                                     socket_generic_write_(0x00)  //构造函数，初始化canservice对象
{
    service_odrive_estop_ = this->create_service<odrive_pro_srvs_msgs::srv::OdriveEstop>("odrive/odrive_estop", std::bind(&CanService::odrive_estop_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_motor_error_ = this->create_service<odrive_pro_srvs_msgs::srv::GetMotorError>("odrive/get_motor_error", std::bind(&CanService::get_motor_error_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_encoder_error_ = this->create_service<odrive_pro_srvs_msgs::srv::GetEncoderError>("odrive/get_encoder_error", std::bind(&CanService::get_encoder_error_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_axis_node_id_ = this->create_service<odrive_pro_srvs_msgs::srv::SetAxisNodeId>("odrive/set_axis_node_id", std::bind(&CanService::set_axis_node_id_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_axis_requested_state_ = this->create_service<odrive_pro_srvs_msgs::srv::SetAxisRequestedState>("odrive/set_axis_requested_state", std::bind(&CanService::set_axis_requested_state_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_axis_startup_config_ = this->create_service<odrive_pro_srvs_msgs::srv::SetAxisStartupConfig>("odrive/set_axis_startup_config", std::bind(&CanService::set_axis_startup_config_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_encoder_estimates_ = this->create_service<odrive_pro_srvs_msgs::srv::GetEncoderEstimates>("odrive/get_encoder_estimates", std::bind(&CanService::get_encoder_estimates_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_encoder_count_ = this->create_service<odrive_pro_srvs_msgs::srv::GetEncoderCount>("odrive/get_encoder_count", std::bind(&CanService::get_encoder_count_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_controller_modes_ = this->create_service<odrive_pro_srvs_msgs::srv::SetControllerModes>("odrive/set_controller_modes", std::bind(&CanService::set_controller_modes_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_input_pos_ = this->create_service<odrive_pro_srvs_msgs::srv::SetInputPos>("odrive/set_input_pos", std::bind(&CanService::set_input_pos_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_input_vel_ = this->create_service<odrive_pro_srvs_msgs::srv::SetInputVel>("odrive/set_input_vel", std::bind(&CanService::set_input_vel_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_input_torque_ = this->create_service<odrive_pro_srvs_msgs::srv::SetInputTorque>("odrive/set_input_torque", std::bind(&CanService::set_input_torque_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_vel_limit_ = this->create_service<odrive_pro_srvs_msgs::srv::SetVelLimit>("odrive/set_vel_limit", std::bind(&CanService::set_vel_limit_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_start_anticogging_ = this->create_service<odrive_pro_srvs_msgs::srv::StartAnticogging>("odrive/start_anticogging", std::bind(&CanService::start_anticogging_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_traj_vel_limit_ = this->create_service<odrive_pro_srvs_msgs::srv::SetTrajVelLimit>("odrive/set_traj_vel_limit", std::bind(&CanService::set_traj_vel_limit_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_traj_accel_limits_ = this->create_service<odrive_pro_srvs_msgs::srv::SetTrajAccelLimits>("odrive/set_traj_accel_limits", std::bind(&CanService::set_traj_accel_limits_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_traj_inertia_ = this->create_service<odrive_pro_srvs_msgs::srv::SetTrajInertia>("odrive/set_traj_inertia", std::bind(&CanService::set_traj_inertia_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_iq_ = this->create_service<odrive_pro_srvs_msgs::srv::GetIq>("odrive/get_iq", std::bind(&CanService::get_iq_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_temperature_ = this->create_service<odrive_pro_srvs_msgs::srv::GetTemperature>("odrive/get_temperature", std::bind(&CanService::get_temperature_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_reset_odrive_ = this->create_service<odrive_pro_srvs_msgs::srv::ResetOdrive>("odrive/reset_odrive", std::bind(&CanService::reset_odrive_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_vbus_voltage_ = this->create_service<odrive_pro_srvs_msgs::srv::GetVbusVoltage>("odrive/get_vbus_voltage", std::bind(&CanService::get_vbus_voltage_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_clear_errors_ = this->create_service<odrive_pro_srvs_msgs::srv::ClearErrors>("odrive/clear_errors", std::bind(&CanService::clear_errors_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_absolute_pos_ = this->create_service<odrive_pro_srvs_msgs::srv::SetAbsolutePos>("odrive/set_absolute_pos", std::bind(&CanService::set_absolute_pos_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_pos_gain_ = this->create_service<odrive_pro_srvs_msgs::srv::SetPosGain>("odrive/set_pos_gain", std::bind(&CanService::set_pos_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_vel_gains_ = this->create_service<odrive_pro_srvs_msgs::srv::SetVelGains>("odrive/set_vel_gains", std::bind(&CanService::set_vel_gains_callback, this, std::placeholders::_1, std::placeholders::_2));
}

CanService::~CanService()
{
}

void CanService::odrive_estop_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::OdriveEstop::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::OdriveEstop::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = odrive_can::Msg::MSG_ODRIVE_ESTOP;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::get_motor_error_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::GetMotorError::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::GetMotorError::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
    send_frame.can_id += odrive_can::Msg::MSG_GET_MOTOR_ERROR;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_get_motor_error_.writeFrame(send_frame);
    can_frame recv_frame;
    if (socket_get_motor_error_.readFrame(&recv_frame) < 0) {
        RCLCPP_INFO(this->get_logger(), "No ODrive Response Received");
    }
    RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
    response->motor_error = odrive_can::can_getSignal<int32_t>(recv_frame, 0, 32, true);
}
void CanService::get_encoder_error_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::GetEncoderError::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::GetEncoderError::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
    send_frame.can_id += odrive_can::Msg::MSG_GET_ENCODER_ERROR;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_get_encoder_error_.writeFrame(send_frame);
    can_frame recv_frame;
    if (socket_get_encoder_error_.readFrame(&recv_frame) < 0) {
        RCLCPP_INFO(this->get_logger(), "No ODrive Response Received");
    }
    RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
    response->encoder_error = odrive_can::can_getSignal<int32_t>(recv_frame, 0, 32, true);
}
void CanService::set_axis_node_id_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAxisNodeId::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAxisNodeId::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 2;
    send_frame.data[0] = request->node_id;
    send_frame.data[1] = request->node_id >> 8;
    send_frame.can_id = odrive_can::Msg::MSG_SET_AXIS_NODE_ID;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_axis_requested_state_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAxisRequestedState::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAxisRequestedState::Response> response)
{
   
    can_frame send_frame;
    send_frame.can_dlc = 8;
    send_frame.data[0] = request->requested_state;
    send_frame.data[1] = request->requested_state >> 8;
    send_frame.data[2] = request->requested_state >> 16;
    send_frame.data[3] = request->requested_state >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_AXIS_REQUESTED_STATE;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_axis_startup_config_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAxisStartupConfig::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAxisStartupConfig::Response> response)
{
    // Not yet implemented in ODrive Firmware
    response->success = false;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::get_encoder_estimates_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::GetEncoderEstimates::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::GetEncoderEstimates::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
    send_frame.can_id += odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_get_encoder_estimates_.writeFrame(send_frame);
    can_frame recv_frame;
    if (socket_get_encoder_estimates_.readFrame(&recv_frame) < 0) {
        RCLCPP_INFO(this->get_logger(), "No ODrive Response Received");
    }
    RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
    response->pos_estimate = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    response->vel_estimate = odrive_can::can_getSignal<float>(recv_frame, 32, 32, true);
}
void CanService::get_encoder_count_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::GetEncoderCount::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::GetEncoderCount::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
    send_frame.can_id += odrive_can::Msg::MSG_GET_ENCODER_COUNT;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_get_encoder_count_.writeFrame(send_frame);
    can_frame recv_frame;
    if (socket_get_encoder_count_.readFrame(&recv_frame) < 0) {
        RCLCPP_INFO(this->get_logger(), "No ODrive Response Received");
    }
    RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
    response->shadow_count = odrive_can::can_getSignal<int32_t>(recv_frame, 0, 32, true);
    response->cpr_count = odrive_can::can_getSignal<int32_t>(recv_frame, 32, 32, true);
}
void CanService::set_controller_modes_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetControllerModes::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetControllerModes::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;
    send_frame.data[0] = request->control_mode;
    send_frame.data[1] = request->control_mode >> 8;
    send_frame.data[2] = request->control_mode >> 16;
    send_frame.data[3] = request->control_mode >> 24;
    send_frame.data[4] = request->input_mode;
    send_frame.data[5] = request->input_mode >> 8;
    send_frame.data[6] = request->input_mode >> 16;
    send_frame.data[7] = request->input_mode >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_CONTROLLER_MODES;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_input_pos_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetInputPos::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetInputPos::Response> response)
{   
    int can_channel = request->can_channel;
    
    can_frame send_frame;
    send_frame.can_dlc = 8;
    std::memcpy(&send_frame.data[0], &request->input_position, sizeof(request->input_position));
    std::memcpy(&send_frame.data[4], &request->vel_ff, sizeof(request->vel_ff));
    std::memcpy(&send_frame.data[6], &request->torque_ff, sizeof(request->torque_ff));
    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID;
    
    socket_set_position_.writeFrame(send_frame);
    //// socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_input_vel_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetInputVel::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetInputVel::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;
    send_frame.data[0] = request->input_vel;
    send_frame.data[1] = request->input_vel >> 8;
    send_frame.data[2] = request->input_vel >> 16;
    send_frame.data[3] = request->input_vel >> 24;
    send_frame.data[4] = request->torque_ff;
    send_frame.data[5] = request->torque_ff >> 8;
    send_frame.data[6] = request->torque_ff >> 16;
    send_frame.data[7] = request->torque_ff >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_VEL;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_input_torque_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetInputTorque::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetInputTorque::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 4;
    send_frame.data[0] = request->input_torque;
    send_frame.data[1] = request->input_torque >> 8;
    send_frame.data[2] = request->input_torque >> 16;
    send_frame.data[3] = request->input_torque >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_TORQUE;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_vel_limit_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetVelLimit::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetVelLimit::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 4;

    uint32_t float_bytes;
    std::memcpy(&float_bytes, &request->vel_limit, sizeof float_bytes);
    send_frame.data[0] = float_bytes;
    send_frame.data[1] = float_bytes >> 8;
    send_frame.data[2] = float_bytes >> 16;
    send_frame.data[3] = float_bytes >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_VEL_LIMIT;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::start_anticogging_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::StartAnticogging::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::StartAnticogging::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
    send_frame.can_id += odrive_can::Msg::MSG_START_ANTICOGGING;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_traj_vel_limit_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetTrajVelLimit::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetTrajVelLimit::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 4;

    uint32_t float_bytes;
    std::memcpy(&float_bytes, &request->traj_vel_limit, sizeof float_bytes);
    send_frame.data[0] = float_bytes;
    send_frame.data[1] = float_bytes >> 8;
    send_frame.data[2] = float_bytes >> 16;
    send_frame.data[3] = float_bytes >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_TRAJ_VEL_LIMIT;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_traj_accel_limits_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetTrajAccelLimits::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetTrajAccelLimits::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;

    uint32_t float_bytes;
    std::memcpy(&float_bytes, &request->traj_accel_limit, sizeof float_bytes);
    send_frame.data[0] = float_bytes;
    send_frame.data[1] = float_bytes >> 8;
    send_frame.data[2] = float_bytes >> 16;
    send_frame.data[3] = float_bytes >> 24;
    std::memcpy(&float_bytes, &request->traj_decel_limit, sizeof float_bytes);
    send_frame.data[4] = float_bytes;
    send_frame.data[5] = float_bytes >> 8;
    send_frame.data[6] = float_bytes >> 16;
    send_frame.data[7] = float_bytes >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_TRAJ_VEL_LIMIT;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_traj_inertia_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetTrajInertia::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetTrajInertia::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 4;

    uint32_t float_bytes;
    std::memcpy(&float_bytes, &request->traj_inertia, sizeof float_bytes);
    send_frame.data[0] = float_bytes;
    send_frame.data[1] = float_bytes >> 8;
    send_frame.data[2] = float_bytes >> 16;
    send_frame.data[3] = float_bytes >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_TRAJ_INERTIA;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::get_iq_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::GetIq::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::GetIq::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    //send_frame.can_id = 1 << 30; // Set ID to have RTR
    //std::cout << send_frame.can_id << std::endl;
    send_frame.can_id = odrive_can::Msg::MSG_GET_IQ;
    std::cout << send_frame.can_id << std::endl;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
       send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
       std::cout << send_frame.can_id << std::endl;
    }
    else
    {
        return;
    }
    std::cout<<send_frame.can_id<<std::endl;
    socket_get_iq_.writeFrame(send_frame);
    can_frame recv_frame;
    if (socket_get_iq_.readFrame(&recv_frame) < 0) {
        RCLCPP_INFO(this->get_logger(), "No ODrive Response Received");
    }
    RCLCPP_INFO(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
    response->iq_setpoint = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    response->iq_measured = odrive_can::can_getSignal<float>(recv_frame, 32, 32, true);
}
void CanService::get_temperature_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::GetTemperature::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::GetTemperature::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
    send_frame.can_id += odrive_can::Msg::MSG_GET_TEMPERATURE;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_get_temperature_.writeFrame(send_frame);
    can_frame recv_frame;
    if (socket_get_temperature_.readFrame(&recv_frame) < 0) {
        RCLCPP_INFO(this->get_logger(), "No ODrive Response Received");
    }
    RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
    response->temp_inverter = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    response->temp_motor = odrive_can::can_getSignal<float>(recv_frame, 32, 32, true);
}

void CanService::reset_odrive_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::ResetOdrive::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::ResetOdrive::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0 || odrive_can::AXIS::AXIS_1)
    {
        can_frame send_frame;
        send_frame.can_dlc = 0;
        send_frame.can_id = odrive_can::AXIS::AXIS_0_ID;
        send_frame.can_id += odrive_can::Msg::MSG_REBOOT_ODRIVE;
        socket_generic_write_.writeFrame(send_frame);
        response->success = true;
    }
    else
    {
        response->success = false;
        return;
    }
}
void CanService::get_vbus_voltage_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::GetVbusVoltage::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::GetVbusVoltage::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0 || odrive_can::AXIS::AXIS_1)
    {
        can_frame send_frame;
        send_frame.can_dlc = 0;
        send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
        send_frame.can_id += odrive_can::Msg::MSG_GET_VBUS_VOLTAGE;
        socket_get_vbus_voltage_.writeFrame(send_frame);
        can_frame recv_frame;
        if (socket_get_vbus_voltage_.readFrame(&recv_frame) < 0) {
            RCLCPP_INFO(this->get_logger(), "No ODrive Response Received");
        }
        RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
        response->vbus_voltage = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    }
    else
    {
        return;
    }
}
void CanService::clear_errors_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::ClearErrors::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::ClearErrors::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = odrive_can::Msg::MSG_CLEAR_ERRORS;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_absolute_pos_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAbsolutePos::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetAbsolutePos::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 4;

    uint32_t float_bytes;
    std::memcpy(&float_bytes, &request->absolute_pos, sizeof float_bytes);
    send_frame.data[0] = float_bytes;
    send_frame.data[1] = float_bytes >> 8;
    send_frame.data[2] = float_bytes >> 16;
    send_frame.data[3] = float_bytes >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_ABSOLUTE_POS;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_pos_gain_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetPosGain::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetPosGain::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 4;

    uint32_t float_bytes;
    std::memcpy(&float_bytes, &request->pos_gain, sizeof float_bytes);
    send_frame.data[0] = float_bytes;
    send_frame.data[1] = float_bytes >> 8;
    send_frame.data[2] = float_bytes >> 16;
    send_frame.data[3] = float_bytes >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_POSITION_GAIN;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_vel_gains_callback(const std::shared_ptr<odrive_pro_srvs_msgs::srv::SetVelGains::Request> request, std::shared_ptr<odrive_pro_srvs_msgs::srv::SetVelGains::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;

    uint32_t float_bytes;
    std::memcpy(&float_bytes, &request->vel_gain, sizeof float_bytes);
    send_frame.data[0] = float_bytes;
    send_frame.data[1] = float_bytes >> 8;
    send_frame.data[2] = float_bytes >> 16;
    send_frame.data[3] = float_bytes >> 24;
    std::memcpy(&float_bytes, &request->vel_integrator_gain, sizeof float_bytes);
    send_frame.data[4] = float_bytes;
    send_frame.data[5] = float_bytes >> 8;
    send_frame.data[6] = float_bytes >> 16;
    send_frame.data[7] = float_bytes >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_VEL_GAINS;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_generic_write_.writeFrame(send_frame);
    response->success = true;
}
