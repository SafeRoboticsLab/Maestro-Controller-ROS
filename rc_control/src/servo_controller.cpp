# include <servo_controller.h>


ServoController::ServoController(ros::NodeHandle node, ros::NodeHandle private_nh):
    _nh(node), _pvt_nh(private_nh), _running(true)
{
    // set dynamic reconfig
    _f = boost::bind(&ServoController::_dynParamCallback, this, _1, _2);
    _dyn_reconfig_server.setCallback(_f);

    // obtain parameters from the launch file
    _ReadParameter();

    // Find if the desired controller is connected
    _device_idx = _SearchController();
    if(_device_idx<0)
    {
        ROS_WARN("The controller is not intialized!");
        _running = false;
        return;
    }  

    // get a copy of the controller setting
    Maestro::Device::ChannelSettings steering_ch_setting = _device_list[_device_idx ].getChannelSettings(_steering_ch);
    _steering_min = steering_ch_setting.minimum;
    _steering_max = steering_ch_setting.maximum;

    Maestro::Device::ChannelSettings throttle_ch_setting = _device_list[_device_idx ].getChannelSettings(_throttle_ch);
    _throttle_min = throttle_ch_setting.minimum;
    _throttle_max = throttle_ch_setting.maximum;

    /* 
    Initialize the controller
    */
    _dynParamUpdate();
    // Allow full speed and acceleration
    _device_list[_device_idx].setSpeed(_steering_ch, 0);
    _device_list[_device_idx].setSpeed(_throttle_ch, 0);
    _device_list[_device_idx].setAcceleration(_steering_ch, 0);
    _device_list[_device_idx].setAcceleration(_throttle_ch, 0);
    natural();

    // set up ros subscriber
    _sub_command = _nh.subscribe(_sub_topic, 1 , &ServoController::_subCallback, this);
    ROS_INFO_STREAM("Controller subscribe to topic: "<<_sub_topic);

}

ServoController::~ServoController()
{
    ROS_INFO("deconstructor");
}

int ServoController::_SearchController()
{
    int index = -1;
    // scan all connected device
    _device_list = Maestro::Device::getConnectedDevices();
    if(_device_list.size() == 0){
        ROS_ERROR("No controller device found");
    }else //if(_device_list.size() == 1)
    {
        index = 0;
        ROS_INFO("Connected to the device: %s",(_device_list[index].getName()).c_str());
    }
    /* Currently ignore that we have multiple servo connected    
    else{
        for(auto device = _device_list.begin(); device != _device_list.end(); device++)
            ROS_INFO("Found device: %s",(device->getName()).c_str());
    }
    */
   return index;
}

void ServoController::_ReadParameter()
{
    ROS_INFO("*** PARAMETER SETTINGS ***");

    _pvt_nh.param<std::string>("ControllerTopic", _sub_topic, "control_input");

    // Channel index for throttle 
    _pvt_nh.param("SteeringChannel", _steering_ch, -1);
    if(_steering_ch<0){
        ROS_ERROR_STREAM("Invalid steering channel "<<_steering_ch);
        _running = false;
    }else
        ROS_INFO_STREAM("Set channel "<<_steering_ch<<" to steering.");

    // Channel index for throttle 
    _pvt_nh.param("ThrottleChannel", _throttle_ch, -1);

    if(_throttle_ch<0){
        ROS_ERROR_STREAM("Invalid throttle channel "<<_throttle_ch);
        _running = false;
    }else
        ROS_INFO_STREAM("Set channel "<<_throttle_ch<<" to throttle.");

    if(_throttle_ch == _steering_ch){
        ROS_ERROR("Steering and Throttle are set to the same channel. Check your setting in the launch file.");
        _running = false;
    }
}

void ServoController::_dynParamCallback(rc_control::calibrationConfig &config, uint32_t level)
{
    DynParam dynamic_params;
    dynamic_params.steering_C = config.steering_C;
    dynamic_params.steering_L = config.steering_L;
    dynamic_params.steering_R = config.steering_R;

    dynamic_params.throttle_N = config.throttle_N;
    dynamic_params.throttle_R = config.throttle_R;
    dynamic_params.throttle_D = config.throttle_D;

    _dynParam.writeFromNonRT(dynamic_params);
}

void ServoController::_dynParamUpdate()
{
    const DynParam dynamic_params = *(_dynParam.readFromRT());
    _steering_C = dynamic_params.steering_C;
    _steering_L = dynamic_params.steering_L;
    _steering_R = dynamic_params.steering_R;

    _throttle_N = dynamic_params.throttle_N;
    _throttle_R = dynamic_params.throttle_R;
    _throttle_D = dynamic_params.throttle_D;
    //ROS_INFO_STREAM(_steering_C<< " "<<_throttle_N);
}

void ServoController::_subCallback(const princeton_racecar_msgs::RC_Control& msg)
{
    if(_running){
        if (_sub_command.getNumPublishers()>1)
        {
            ROS_ERROR_STREAM("Detected " << _sub_command.getNumPublishers()<< " publishers. Only 1 publisher is allowed. Going to brake.");
            return;
        }

        _command_struct.steer = _convertTarget(msg.steer, true);
        _command_struct.throttle = _convertTarget(msg.throttle, false);
        _command_struct.reverse = msg.reverse;
        _command_struct.stamp = ros::Time::now();
        _command.writeFromNonRT(_command_struct);
        ROS_DEBUG_STREAM("Receive control - Steer: "<<_command_struct.steer
                            <<" Throttle: "<<_command_struct.throttle
                            <<" Time: "<<_command_struct.stamp);
    }
}

int ServoController::_convertTarget(double percentage, bool is_steer)
{
    /* 
    convert the percentage input (-100% - 100%) 
    to the intergral input of pluse width (in unit of 0.25us)
    pluse width is in between 1000 and 2000
    So the output is in between 4000 and 8000
    */
    double center = is_steer ? _steering_C : _throttle_N;
    double low = is_steer ? _steering_L : _throttle_R;
    double high = is_steer ? _steering_R : _throttle_D;
    int min_input = is_steer ? _steering_min : _throttle_min;
    int max_input = is_steer ? _steering_max : _throttle_max;
    int input = int(4*(std::max(low, std::min(high, (center+percentage)))*500+1500));
    return std::max(min_input, std::min(input, max_input));
}

void ServoController::brake()
{
    if(_reverse)
    {
        _device_list[_device_idx].setTarget(_steering_ch, _convertTarget(0, true));
        _device_list[_device_idx].setTarget(_throttle_ch, _convertTarget(1, false));   
    }else
    {   
        _device_list[_device_idx].setTarget(_steering_ch, _convertTarget(0, true));
        _device_list[_device_idx].setTarget(_throttle_ch, _convertTarget(-1, false));   
    }
}

void ServoController::natural()
{
    _device_list[_device_idx].setTarget(_steering_ch, _convertTarget(0, true));
    _device_list[_device_idx].setTarget(_throttle_ch, _convertTarget(0, false));
}


void ServoController::update(const ros::Time& time, const ros::Duration& period)
{
    _dynParamUpdate();
    if(!_running)
    {
        ROS_WARN_ONCE_NAMED("ServoController::update", "The controller is not running!");
        return;
    }
    Commands curr_cmd = *(_command.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();
    if (dt > _cmd_vel_timeout){
        natural();
        return;
    }
    _device_list[_device_idx].setTarget(_steering_ch, _convertTarget(curr_cmd.steer, true));
    _device_list[_device_idx].setTarget(_throttle_ch, _convertTarget(curr_cmd.throttle, false));
}