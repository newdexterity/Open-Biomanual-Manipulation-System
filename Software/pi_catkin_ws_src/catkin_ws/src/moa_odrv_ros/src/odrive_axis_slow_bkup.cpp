#include "odrive_axis.hpp"

namespace odrive {
    ODriveAxis::ODriveAxis(ros::NodeHandle *node,  std::string axis_name, int axis_can_id, std::string direction) {
        axis_name_ = axis_name;
        axis_can_id_ = axis_can_id;
        if (direction == "forward") {
            direction_ = 1;
        } else {
            direction_ = -1;
        }
        calibrated = false;
        ready2calibrate = false;
        node->param<std::string>("can_rx_topic", can_rx_topic_, "/can/received_messages");
        node->param<std::string>("can_tx_topic", can_tx_topic_, "/can/sent_messages");
        node->param<double>("update_rate", update_rate_, DEFAULT_UPDATE_RATE);
        node->param<bool>("engage_on_startup", engage_on_startup_, false);
        received_messages_sub_ = node->subscribe<can_msgs::Frame>(can_rx_topic_, 1000,
            std::bind(&ODriveAxis::canReceivedMessagesCallback, this, std::placeholders::_1));
        sent_messages_pub_ = node->advertise<can_msgs::Frame>(can_tx_topic_, 1000);
        /* target_velocity_sub_ = node->subscribe<std_msgs::Float64>("/" + axis_name_ + "/target_velocity", 1000, std::bind(&ODriveAxis::velocityReceivedMessagesCallback, this, std::placeholders::_1));
        */
        target_position_sub_ = node->subscribe<std_msgs::Float64>("/" + axis_name_ + "/target_position",
            1000, std::bind(&ODriveAxis::positionReceivedMessagesCallback, this, std::placeholders::_1));
        leftarm_position_sub_ = node->subscribe<std_msgs::Float64MultiArray>("/leftarm/position_controller/command",
            1000, std::bind(&ODriveAxis::leftarmPositionReceivedMessagesCallback, this, std::placeholders::_1));
        
        rightarm_position_sub_ = node->subscribe<std_msgs::Float64MultiArray>("/rightarm/position_controller/command",
            1000, std::bind(&ODriveAxis::rightarmPositionReceivedMessagesCallback, this, std::placeholders::_1));  
            
        axis_angle_pub_ = node->advertise<std_msgs::Float64>("/" + axis_name + "/angle", 1);
        axis_velocity_pub_ = node->advertise<std_msgs::Float64>("/" + axis_name + "/current_velocity", 1);
        axis_voltage_pub_ = node->advertise<std_msgs::Float64>("/" + axis_name + "/voltage", 1);
        axis_current_pub_ = node->advertise<std_msgs::Float64>("/" + axis_name + "/current", 1);
//        ros::spin();        
        axis_angle_ = 0.0;
        axis_velocity_ = 0.0;
        axis_current_ = 0.0;
        axis_voltage_ = 0.0;
        axis_update_timer_ = node->createTimer(ros::Duration(1 / update_rate_),
            std::bind(&ODriveAxis::updateTimerCallback, this, std::placeholders::_1));
        // TODO: design the logic behind engage on startup behaviour
        
        /*if (engage_on_startup_) {
        
            ROS_INFO("test : --------------------------------------------------------------------------");
            //calibrate();
            //ros::Duration(1).sleep();
            engage();
        }*/
        
    }

    void ODriveAxis::canReceivedMessagesCallback(const can_msgs::Frame::ConstPtr& msg) {
        // TODO: implement CAN message parsing and processing
        uint32_t ID = msg->id;
        uint32_t NODE_ID = (ID >> 5);
        std_msgs::Float64 angle_msg;
        std_msgs::Float64 velocity_msg;
        std_msgs::Float64 current_msg;
        std_msgs::Float64 voltage_msg;
        float velocityEstimate;
        float positionEstimate;
        float axisVoltage;
        float axisCurrent;
        uint8_t *ptrVelocityEstimate = (uint8_t *)&velocityEstimate;
        uint8_t *ptrPositionEstimate = (uint8_t *)&positionEstimate;
        uint8_t *ptrAxisVoltage = (uint8_t *)&axisVoltage;
        uint8_t *ptrAxisCurrent = (uint8_t *)&axisCurrent;

        if (NODE_ID == axis_can_id_) {
            uint32_t CMD_ID = (ID & 0x01F);
            //ROS_INFO("CALLBACK CALLED FROM ID %u, CMD %u", NODE_ID, CMD_ID);
            switch (CMD_ID) {
            case ODriveCommandId::HEARTBEAT_MESSAGE:
               // ROS_DEBUG("Axis %02x: HEARTBEAT_MESSAGE", axis_can_id_);
                this->axis_motor_state_ = msg->data[4];
                //ROS_INFO("HEARBEAT MESSAGE. DATA[4] VALUE %u", msg->data[4]);
		//if (!calibrated){
		//	calibrated = true;
		//	ready2calibrate = true;
		//}
                break;
            case ODriveCommandId::GET_ENCODER_ESTIMATE:
                ROS_DEBUG("Axis %02x: GET_ENCODER_ESTIMATE", axis_can_id_);
                ptrPositionEstimate[0] = msg->data[0];
                ptrPositionEstimate[1] = msg->data[1];
                ptrPositionEstimate[2] = msg->data[2];
                ptrPositionEstimate[3] = msg->data[3];
                ptrVelocityEstimate[0] = msg->data[4];
                ptrVelocityEstimate[1] = msg->data[5];
                ptrVelocityEstimate[2] = msg->data[6];
                ptrVelocityEstimate[3] = msg->data[7];
                axis_angle_ = (double)positionEstimate * 2 * M_PI * direction_;
                axis_velocity_ = (double)velocityEstimate * 2 * M_PI * direction_;
                angle_msg.data = axis_angle_;
                velocity_msg.data = axis_velocity_;
                axis_angle_pub_.publish(angle_msg);
                axis_velocity_pub_.publish(velocity_msg);
                break;
            case ODriveCommandId::GET_BUS_VOLTAGE_AND_CURRENT:
                ROS_DEBUG("Axis %02x: GET_BUS_VOLTAGE_AND_CURRENT", axis_can_id_);
                ptrAxisVoltage[0] = msg->data[0];
                ptrAxisVoltage[1] = msg->data[1];
                ptrAxisVoltage[2] = msg->data[2];
                ptrAxisVoltage[3] = msg->data[3];
                ptrAxisCurrent[0] = msg->data[4];
                ptrAxisCurrent[1] = msg->data[5];
                ptrAxisCurrent[2] = msg->data[6];
                ptrAxisCurrent[3] = msg->data[7];
                axis_current_ = (double)axisCurrent;
                axis_voltage_ = (double)axisVoltage;
                current_msg.data = axis_current_;
                voltage_msg.data = axis_voltage_;
                axis_current_pub_.publish(current_msg);
                axis_voltage_pub_.publish(voltage_msg);
                break;
            default:
                ROS_DEBUG("Axis %02x: unsupported command %02x", axis_can_id_, CMD_ID);
                break;
            }
        }
    }

    void ODriveAxis::velocityReceivedMessagesCallback(const std_msgs::Float64::ConstPtr& msg) {
        ROS_DEBUG("Received velocity message");
        double targetVelocity = msg->data / (2.0 * M_PI) * direction_;
        setInputVelocity(targetVelocity);
    }
    void ODriveAxis::positionReceivedMessagesCallback(const std_msgs::Float64::ConstPtr& msg) {
        ROS_DEBUG("Received position message");
        double targetPosition = msg->data * direction_;
        setInputPosition(targetPosition);
    }
    
    void ODriveAxis::rightarmPositionReceivedMessagesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        ROS_DEBUG("Received position message");
        //ROS_INFO("I got : %f %f %f %f %f", msg->data[0] ,msg->data[1],msg->data[2],msg->data[3],msg->data[4]);
        double targetPosition1 = msg->data[0] * direction_;
        networkInputPosition(1,targetPosition1);
        double targetPosition2 = msg->data[1] * direction_;
        networkInputPosition(2,targetPosition2);
        double targetPosition3 = msg->data[2] * direction_;
        networkInputPosition(3,targetPosition3);
        double targetPosition4 = msg->data[3] * direction_;
        networkInputPosition(4,targetPosition4);
        double targetPosition5 = msg->data[4] * direction_;
        networkInputPosition(5,targetPosition5);

    }
    void ODriveAxis::leftarmPositionReceivedMessagesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        ROS_DEBUG("Received position message");
        //double targetPosition = msg->data * direction_;
        //setInputPosition(targetPosition);
    }

    void ODriveAxis::updateTimerCallback(const ros::TimerEvent& event) {
        //requestEncoderEstimate();
        //requestBusVoltageAndCurrent();
    }

    void ODriveAxis::requestEncoderEstimate() {
        can_msgs::Frame request_msg;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::GET_ENCODER_ESTIMATE);
        request_msg.is_extended = false;
        request_msg.is_rtr = true;
        request_msg.dlc = 8;
        sent_messages_pub_.publish(request_msg);
    }

    void ODriveAxis::requestBusVoltageAndCurrent() {
        can_msgs::Frame request_msg;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::GET_BUS_VOLTAGE_AND_CURRENT);
        request_msg.is_extended = false;
        request_msg.is_rtr = true;
        request_msg.dlc = 8;
        sent_messages_pub_.publish(request_msg);
    }

    void ODriveAxis::setAxisRequestedState(ODriveAxisState state) {
        can_msgs::Frame request_msg;
        uint32_t requestedState = (uint32_t)state;
        uint8_t *ptrRequestedState;
        ptrRequestedState = (uint8_t *)&requestedState;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::SET_AXIS_REQUESTED_STATE);
        /*
        ROS_INFO("requesting requestedState : %i ", requestedState);
        ROS_INFO("requesting ID : %i ", request_msg.id);
        */
        //msg = can.Message(arbitration_id=axisID << 5 | 0x07, data=[3, 0, 0, 0, 0, 0, 0, 0], dlc=8, is_extended_id=False)
        request_msg.is_extended = false;
        request_msg.dlc = 8;
        request_msg.data[0] = ptrRequestedState[0];
        request_msg.data[1] = ptrRequestedState[1];
        request_msg.data[2] = ptrRequestedState[2];
        request_msg.data[3] = ptrRequestedState[3];
        /*
        ROS_INFO("requesting state : %i ", request_msg.data[0]);
        ROS_INFO("requesting state : %i ", request_msg.data[1]);
        ROS_INFO("requesting state : %i ", request_msg.data[2]);
        ROS_INFO("requesting state : %i ", request_msg.data[3]);
        ROS_INFO("requesting state : %i ", request_msg.data[4]);
        ROS_INFO("requesting state : %i ", request_msg.data[5]);
        ROS_INFO("requesting state : %i ", request_msg.data[6]);
        ROS_INFO("requesting state LB : %i ", request_msg.data[7]);*/
        sent_messages_pub_.publish(request_msg);
    }
    void ODriveAxis::networkInputPosition(int axis_id_ ,double position) {    	
        can_msgs::Frame request_msg;
        float pos = (float)position;
        uint8_t *ptrPos;
        ptrPos = (uint8_t *)&pos;
        request_msg.id = createCanId(axis_id_, ODriveCommandId::SET_INPUT_POS);
        request_msg.is_extended = false;
        request_msg.dlc = 8;
        request_msg.data[0] = ptrPos[0];
        request_msg.data[1] = ptrPos[1];
        request_msg.data[2] = ptrPos[2];
        request_msg.data[3] = ptrPos[3];
        //request_msg.data[4] = ptrVel[0];
        //request_msg.data[5] = ptrVel[1];
        //request_msg.data[6] = ptrTor[0];
        //request_msg.data[7] = ptrTor[1];
        sent_messages_pub_.publish(request_msg);
    }
    void ODriveAxis::setInputPosition(double position) {
    //Signals		: Input Pos Vel FF Torque FF 
    //Start byte	: 0 4 6
    //Signal Type	: IEEE 754 Float Signed Int Signed Int
    //Bits 		: 32 16 16
    //Factor 		: 1 0.001 0.001
    // Offset		: 0 0 0
    	ROS_INFO("RECEIVED TARGET POSITION REQUEST (%s): %f", axis_name_, position);
    	
        can_msgs::Frame request_msg;
        float pos = (float)position;
        uint8_t *ptrPos;
        ptrPos = (uint8_t *)&pos;
        //int vel = 10;
        //int torq = 2;        
        //uint8_t *ptrVel;
        //uint8_t *ptrTor;
        //ptrVel = (uint8_t *)&vel;
        //ptrTor = (uint8_t *)&torq;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::SET_INPUT_POS);
        request_msg.is_extended = false;
        request_msg.dlc = 8;
        request_msg.data[0] = ptrPos[0];
        request_msg.data[1] = ptrPos[1];
        request_msg.data[2] = ptrPos[2];
        request_msg.data[3] = ptrPos[3];
        //request_msg.data[4] = ptrVel[0];
        //request_msg.data[5] = ptrVel[1];
        //request_msg.data[6] = ptrTor[0];
        //request_msg.data[7] = ptrTor[1];
        sent_messages_pub_.publish(request_msg);
    }
    void ODriveAxis::setInputVelocity(double velocity) {
        can_msgs::Frame request_msg;
        float vel = (float)velocity;
        float torq = 2.5;
        uint8_t *ptrVel;
        uint8_t *ptrTor;
        ptrVel = (uint8_t *)&vel;
        ptrTor = (uint8_t *)&torq;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::SET_INPUT_VELOCITY);
        request_msg.is_extended = false;
        //request_msg.dlc = 8;
        //request_msg.data[0] = ptrVel[0];
        //request_msg.data[1] = ptrVel[1];
        //request_msg.data[2] = ptrVel[2];
        //request_msg.data[3] = ptrVel[3];
        //request_msg.data[4] = ptrTor[0];
        //request_msg.data[5] = ptrTor[1];
        //request_msg.data[6] = ptrTor[2];
        //request_msg.data[7] = ptrTor[3];
        //sent_messages_pub_.publish(request_msg);
    }
    void ODriveAxis::calibrate() {
    	//ROS_INFO("calibrating.............");
    	setAxisRequestedState(ODriveAxisState::FULL_CALIBRATION_SEQUENCE);    	
   // 	ros::spinOnce();
//    	ROS_INFO("MOTOR STATE: %u",axis_motor_state_);
  //  	ROS_INFO("state1 = %i", axis_motor_state_);
//        ros::Duration(0.5).sleep();
	 while (axis_motor_state_ == 1)
	 {
	 	ROS_INFO("setting calibrate");
		//setAxisRequestedState(ODriveAxisState::FULL_CALIBRATION_SEQUENCE);
		//ROS_INFO("%i",axis_motor_state_);
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	 }
	 
        while (axis_motor_state_ != 1)
        {
	      ROS_INFO("waiting calibrate");
	      ros::spinOnce();
	      ros::Duration(0.1).sleep();
        }
        ROS_INFO("Calibration DONE. Engaging...");
        ros::Duration(1.0).sleep();
        engage();
        ROS_INFO("Engaged");        
    }

    void ODriveAxis::engage() {
        ROS_INFO("engaging.............");
        setAxisRequestedState(ODriveAxisState::CLOSED_LOOP_CONTROL);
    }

    void ODriveAxis::disengage() {
        setAxisRequestedState(ODriveAxisState::IDLE);
    }


    uint32_t ODriveAxis::createCanId(uint32_t axis_can_id, uint32_t command) {
        uint32_t can_id = (axis_can_id_ << 5) | command;
        return can_id;
    }

}
