#include "odrivepro_can_includes/can_publisher.hpp"

CanPublisher::CanPublisher() : Node("can_publisher"),
                               socket_axis0_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_0_ID, 0x7FF, 500),
                               socket_axis1_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_1_ID, 0x7FF, 110000),
                               socket_get_iq_(odrive_can::Msg::MSG_GET_IQ | odrive_can::AXIS::AXIS_0_ID, 0x7FF, 500),
                               socket_get_encoder_estimates_(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES | odrive_can::AXIS::AXIS_0_ID, 0x7FF, 500),
                               socket_set_position_(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID, 0x7FF, 500),
                               socket_set_velocity_(odrive_can::Msg::MSG_SET_INPUT_VEL | odrive_can::AXIS::AXIS_0_ID, 0x7FF, 500),
                               socket_set_torque_(odrive_can::Msg::MSG_SET_INPUT_TORQUE | odrive_can::AXIS::AXIS_0_ID, 0x7FF, 500)
{
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
    publisher_ = this->create_publisher<odrive_pro_srvs_msgs::msg::OdriveStatus>("/odrive/odrive_status", 5);
    input_pos_subscription_ = this->create_subscription<odrive_pro_srvs_msgs::msg::SetInputPos>(
        "/odrive/odrive_input_pos", 10, std::bind(&CanPublisher::set_input_pos_callback, this, std::placeholders::_1));
    input_vel_subscription_ = this->create_subscription<odrive_pro_srvs_msgs::msg::SetInputVel>(
        "/odrive/odrive_input_vel", 10, std::bind(&CanPublisher::set_input_vel_callback, this, std::placeholders::_1));
    input_torque_subscription_ = this->create_subscription<odrive_pro_srvs_msgs::msg::SetInputTorque>(
        "/odrive/odrive_input_torque", 10, std::bind(&CSanPublisher::set_input_torque_callback, this, std::placeholders::_1));
    odrive_status_msg.axis_error = 0;
    odrive_status_msg.axis_state = 0;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CanPublisher::timerCallback, this));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CanPublisher::updateStatusCallback, this));
}

CanPublisher::~CanPublisher()
{
}

void CanPublisher::timerCallback()
{

    odrive_status_msg.axis = 0;

    can_frame axis0_frame;
    int axis0_read = socket_axis0_read_.readFrame(&axis0_frame);
    if (axis0_read < 0)
    {
        RCLCPP_INFO(this->get_logger(), "No Axis 0 Heartbeat message");
        odrive_status_msg.is_axis_alive = false;
    }
    else
    {
        odrive_status_msg.is_axis_alive = true;
        RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", axis0_frame.data[0], axis0_frame.data[1], axis0_frame.data[2], axis0_frame.data[3], axis0_frame.data[4], axis0_frame.data[5], axis0_frame.data[6], axis0_frame.data[7]);
        odrive_status_msg.axis_error = odrive_can::can_getSignal<uint32_t>(axis0_frame, 0, 32, true);
        odrive_status_msg.axis_state = odrive_can::can_getSignal<uint32_t>(axis0_frame, 32, 32, true);
    }

    // odrive_status_msg.axis = 1;
    // can_frame axis1_frame;
    // int axis1_read = socket_axis1_read_.readFrame(&axis1_frame);
    // if (axis1_read < 0)
    //{
    //    RCLCPP_INFO(this->get_logger(), "No Axis 1 Heartbeat message");
    //    odrive_status_msg.is_axis_alive = false;
    //}
    // else
    //{
    //  odrive_status_msg.is_axis_alive = true;
    // RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", axis1_frame.data[0], axis1_frame.data[1], axis1_frame.data[2], axis1_frame.data[3], axis1_frame.data[4], axis1_frame.data[5], axis1_frame.data[6], axis1_frame.data[7]);
    // odrive_status_msg.axis_error = odrive_can::can_getSignal<uint32_t>(axis1_frame, 0, 32, true);
    // odrive_status_msg.axis_state = odrive_can::can_getSignal<uint32_t>(axis1_frame, 32, 32, true);
    //}
    // publisher_->publish(odrive_status_msg);
}

void CanPublisher::updateStatusCallback()
{

    can_frame recv_frame;
    if (socket_get_encoder_estimates_.readFrame(&recv_frame) < 0)
    {
        RCLCPP_INFO(this->get_logger(), "No Encoder Response Received");
    }
    RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);

    RCLCPP_DEBUG(this->get_logger(), "%f %f", odrive_can::can_getSignal<float>(recv_frame, 0, 32, true), odrive_can::can_getSignal<float>(recv_frame, 32, 32, true));
    odrive_status_msg.pos_estimate = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    odrive_status_msg.vel_estimate = odrive_can::can_getSignal<float>(recv_frame, 32, 32, true);
    can_frame iq_recv_frame;
    if (socket_get_iq_.readFrame(&iq_recv_frame) < 0)
    {
        RCLCPP_INFO(this->get_logger(), "No Current Response Received");
    }
    RCLCPP_DEBUG(this->get_logger(), "%f %f", odrive_can::can_getSignal<float>(recv_frame, 0, 32, true), odrive_can::can_getSignal<float>(recv_frame, 32, 32, true));
    odrive_status_msg.iq_setpoint = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    odrive_status_msg.iq_measured = odrive_can::can_getSignal<float>(recv_frame, 32, 32, true);
    publisher_->publish(odrive_status_msg);
}


void CanPublisher::set_input_pos_callback(const odrive_pro_srvs_msgs::msg::SetInputPos::SharedPtr  msg)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;
    std::memcpy(&send_frame.data[0], &msg->input_position, sizeof(msg->input_position));
    std::memcpy(&send_frame.data[4], &msg->vel_ff, sizeof(msg->vel_ff));
    std::memcpy(&send_frame.data[6], &msg->torque_ff, sizeof(msg->torque_ff));
    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID;
    
    socket_set_position_.writeFrame(send_frame);

}

void CanPublisher::set_input_vel_callback(const odrive_pro_srvs_msgs::msg::SetInputVel::SharedPtr  msg)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;
    std::memcpy(&send_frame.data[0], &msg->input_vel, sizeof(msg->input_vel));
    std::memcpy(&send_frame.data[4], &msg->torque_ff, sizeof(msg->torque_ff));
    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_VEL | odrive_can::AXIS::AXIS_0_ID;
    socket_set_velocity_.writeFrame(send_frame);

}

void CanPublisher::set_input_torque_callback(const odrive_pro_srvs_msgs::msg::SetInputTorque::SharedPtr  msg)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;
    std::memcpy(&send_frame.data[0], &msg->input_torque, sizeof(msg->input_torque));
    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_TORQUE | odrive_can::AXIS::AXIS_0_ID;
    
    socket_set_torque_.writeFrame(send_frame);

}