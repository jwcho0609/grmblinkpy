#include "rclcpp/rclcpp.hpp"
#include <mblink/mblink.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trusses_custom_interfaces/msg/spirit_state.hpp"
#include "trusses_custom_interfaces/msg/spirit_lowlevel_emulation_data.hpp"
#include "trusses_custom_interfaces/msg/spirit_set_retry_param.hpp"

// #include "realtime_tools_raw/realtime_helpers.cpp"
#include <string>
#include <chrono>
#include <thread>
#include <sys/time.h>
#include <sys/resource.h>
using std::string;
using std::placeholders::_1; // Create a placeholder for the first argument of the function

constexpr int THREAD_PRIORITY = 99;  // Near highest RT priority
constexpr int LOOP_PERIOD_US = 1000; // desired loop period in microseconds, 1000 is 1 ms, cant really run slower
// constexpr int LOOP_PERIOD_US = 100000; // desired loop period in microseconds, 1000 is 1 ms, cant really run slower, TX2 VERSION
constexpr int cpu_core_high_speed_state = 0; //set the cpu core to run the high speed state info 
constexpr int NICENESS = -20;  //  HIGHEST level NICENESS priority
constexpr int NUM_LATENCY_SAMPLES = 5000;  // Number of samples to track


std::string trim(const std::string &str) {
    auto start = str.find_first_not_of(" \t\n\r");  // find first non-space character
    auto end = str.find_last_not_of(" \t\n\r");     // find last non-space character
    return (start == std::string::npos) ? "" : str.substr(start, end - start + 1);
}
/**
 * @brief Get the aligned start time object
 * 
 * Basically this will take the alignment value and the current time and output
 * a rounded up value of the current time plus the alignment value rounded to the 
 * nearest alignment value and return that 
 * 
 * @param alignment nearest microseconds you want the alignment 
 * @return std::chrono::system_clock::time_point to wait to start loop to maintain the time
 */
std::chrono::system_clock::time_point get_aligned_start_time(std::chrono::microseconds alignment) {
    auto now = std::chrono::system_clock::now(); // curr time
    auto since_epoch = now.time_since_epoch(); //duration since epoch
    auto aligned = ((since_epoch + alignment) / alignment) * alignment; //round up to nearest alignment boundary
    return std::chrono::system_clock::time_point(aligned); //return
}
class MBLinkNode : public rclcpp::Node {
public:
    MBLinkNode(bool sim,bool verbose,int port,const std::string& logname) 
    : Node("mblink_node"), sim_(sim), verbose_(verbose),port_(port),logname_(logname) {
        RCLCPP_INFO(this->get_logger(), "MBLink ROS 2 Node Started");
        // static const rclcpp::QoS qos = rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // set QOS profile to best effort with a queu of 1

        //initialize data
        initRxData();
        //preallocates memoryt for the spirit
        preallocate_memory();
        // Initialize MBLink
        mblink_ = std::make_shared<gr::MBLink>();
        //initialize qos for high speed
        rclcpp::QoS qosHighSpeed(rclcpp::KeepLast(1));
        qosHighSpeed.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qosHighSpeed.durability(rclcpp::DurabilityPolicy::Volatile);

        //Generate spirit state publisher
        spirit_state_low_speed_publisher_ = this->create_publisher<trusses_custom_interfaces::msg::SpiritState>(
            "spirit/state_low_speed",10);
        //generate test speed publisher
        spirit_high_speed_publisher_ = this->create_publisher<trusses_custom_interfaces::msg::SpiritState>(
            "spirit/state_high_speed",qosHighSpeed);

        //generate subscriber to publish Direct lowlevel Emulation Data (toe forces, positions, etc) to spirit
        send_lowlevel_emulation_forces_pos_subscriber_ = this->create_subscription<trusses_custom_interfaces::msg::SpiritLowlevelEmulationData>(
            "spirit/lowlevel_emulation_forces_pos",qosHighSpeed,std::bind(&MBLinkNode::sendLowlevelEmulationForcesPosCallback,this,_1));
        send_lowlevel_emulation_torques_pos_subscriber_ = this->create_subscription<trusses_custom_interfaces::msg::SpiritLowlevelEmulationData>(
            "spirit/lowlevel_emulation_torques_pos",qosHighSpeed,std::bind(&MBLinkNode::sendLowlevelEmulationTorquesPosCallback,this,_1));
        //generate subscriber to publish Direct mblink tx data to the mainboard WHEN RUNNING IN A CUSTOM MODE THAT ALLOWS THIS
        send_custom_mode_parameters_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "spirit/custom_mode_parameters",10,std::bind(&MBLinkNode::sendCustomModeParameters,this,_1));
        
        //generate subscriber to publish self check commands to spirit
        self_check_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "spirit/self_check",10,std::bind(&MBLinkNode::selfCheckCallback,this,_1));
        //generate subscriber to publish custom mode changes to spirit
        custom_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "spirit/custom_mode",10,std::bind(&MBLinkNode::customModeCallback,this,_1));
        //generate subscriber to publish ghost mode changes to spirit
        ghost_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "spirit/ghost_mode",10,std::bind(&MBLinkNode::ghostModeCallback,this,_1));
        //generate subscriber to publish estop to spirit
        estop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "spirit/estop",10,std::bind(&MBLinkNode::sendEstopCallback,this,_1));
        //generate subscriber to turn on and off planners for spirit
        planner_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "spirit/planner_mode",10,std::bind(&MBLinkNode::sendPlannerMessageCallback,this,_1));
        //generate subscriber to publish trot control to spirit 
        send_ghost_trot_control_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "spirit/ghost_trot_control",10,std::bind(&MBLinkNode::sendGhostTrotControlCallback,this,_1));
        //generate subscriber to send setRetry  values for adjusting spirit params
        send_ghost_set_retry_param_subscriber_ = this->create_subscription<trusses_custom_interfaces::msg::SpiritSetRetryParam>(
            "spirit/ghost_set_retry_param",10,std::bind(&MBLinkNode::semdGhostSetRetryParamCallback,this,_1));
        

        //now do mblink setup
        if(sim_){
            std::cout << "RUNNING IN SIM MODE" << std::endl;
        }
        //sim,verbose,port is the input to start 
        mblink_->start(sim_,verbose_, port_); // input 
        std::cout << logname_ << std::endl;
        mblink_->rxContinuousStart(logname_); //continous start 
        //if we are not in sim, set address
        if(!sim_){
            std::cout << "SETTING IP ADDRESS" << std::endl;
            mblink_->setRetry("_UPST_ADDRESS", 105);  //sets upstream address to THIS LAPTOP
        }
        mblink_->setRetry("UPST_LOOP_DELAY", 1); //Set upstream main TX rate sets delay in ms

        
        // Create a timer to periodically send spirit state values, at 20 Hz (50ms)
        timer_state_low_speed_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MBLinkNode::state_low_speed_timer_callback, this));
        
        //now start thread for the higher speed data, I believe this will run its own thread for the publishing of this highspeed data to ensure its at 1kHz
        rt_thread_ = std::thread(&MBLinkNode::run, this); 
        
    }
    ~MBLinkNode()
    {
        running_ = false;
        if (rt_thread_.joinable())
        {
            rt_thread_.join();
        }
    }

private:
    bool sim_;
    bool verbose_;
    int port_;
    std::thread rt_thread_;
    bool running_ = true;
    //initalize current custom mode
    int curr_custom_mode_ = 0;
    //initialize that custom is disabled
    bool custom_enabled_ = false;
    //initialize cusotm mode corresponding to the lowlevel_emulation
    int custom_mode_lowlevel_forces_emulation_ = 5;
    int custom_mode_lowlevel_torques_emulation_ = 6;
    std::chrono::steady_clock::time_point user_data_loop_start_time = std::chrono::steady_clock::now();  // Time before the loop starts for user data
    std::vector<std::chrono::microseconds> latencies_user_data_;
    std::chrono::time_point<std::chrono::system_clock>  time_curr_, 
                                                        time_last_get_call_;
    bool skipped_last_get_call_ = false;
    std::chrono::microseconds total_latency_user_data_ = std::chrono::microseconds(0);
    std::chrono::microseconds max_latency_user_data_ = std::chrono::microseconds(0);
    std::chrono::microseconds previous_compute_time_ = std::chrono::microseconds(0);

    int count_5us_user_data = 0;
    int count_10us_user_data = 0;
    int count_25us_user_data = 0;
    int count_50us_user_data = 0;
    int count_100us_user_data = 0;

    //set time in ms to wait
    int set_mode_waiting_time_ = 1000;
    std::string logname_;
    std::shared_ptr<gr::MBLink> mblink_;
    rclcpp::TimerBase::SharedPtr timer_state_low_speed_,timer_state_high_speed_;
    // Define RxData_t_ as a variable instead of a typedef
    using RxData_t = std::unordered_map<std::string, Eigen::VectorXf>;
    RxData_t rx_data_;  // Declare the variable
    //declare spirit state variable
    trusses_custom_interfaces::msg::SpiritState spirit_state_;
    //declare a sharedpts for the user data return
    trusses_custom_interfaces::msg::SpiritLowlevelEmulationData::SharedPtr lowlevel_emulation_data_forces_msg_, lowlevel_emulation_data_torques_msg_;
    //declare publishers and subscribers
    rclcpp::Publisher<trusses_custom_interfaces::msg::SpiritState>::SharedPtr spirit_state_low_speed_publisher_, spirit_high_speed_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr planner_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ghost_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr custom_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr self_check_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr send_custom_mode_parameters_subscriber_;
    rclcpp::Subscription<trusses_custom_interfaces::msg::SpiritLowlevelEmulationData>::SharedPtr send_lowlevel_emulation_forces_pos_subscriber_, send_lowlevel_emulation_torques_pos_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr send_ghost_trot_control_subscriber_;
    rclcpp::Subscription<trusses_custom_interfaces::msg::SpiritSetRetryParam>::SharedPtr send_ghost_set_retry_param_subscriber_;
    


    //Running high speed comms
    void run()
    {
        // Set FIFO RT scheduling
        struct sched_param param {};
        param.sched_priority = THREAD_PRIORITY;
        if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "STATE_HIGH_SPEED: Failed to set RT scheduling: %s", strerror(errno));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "STATE_HIGH_SPEED: Running with RT FIFO scheduling at priority %d", THREAD_PRIORITY);
        }

        // // Lock memory to avoid page faults
        // if (!realtime_tools::lock_memory().first)
        // {
        //     RCLCPP_WARN(this->get_logger(), "STATE_HIGH_SPEED: Failed to lock memory");
        // }

        // Pin thread to CPU core cpu_core_high_speed_state
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);       // Clear the CPU set
        CPU_SET(cpu_core_high_speed_state, &cpu_set);     // Add core cpu_core_high_speed_state to the set
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpu_set) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "STATE_HIGH_SPEED: Failed to set thread affinity: %s", strerror(errno));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "STATE_HIGH_SPEED: Thread pinned to CPU core %i",cpu_core_high_speed_state);
        }
        if (setpriority(PRIO_PROCESS, 0, NICENESS) == -1) {
            std::cerr << "STATE_HIGH_SPEED:  Failed to set thread niceness!" << std::endl;
        }
        else{
            std::cout << "STATE_HIGH_SPEED: Set thread niceness to "<<NICENESS << std::endl;
    
        }
        auto aligned_start_time = get_aligned_start_time(std::chrono::microseconds(LOOP_PERIOD_US));
        std::this_thread::sleep_until(aligned_start_time);
        auto next_time = aligned_start_time;

        std::vector<std::chrono::microseconds> latencies;
        std::vector<std::chrono::microseconds> compute_times;
        
        std::chrono::microseconds total_latency(0);
        std::chrono::microseconds max_latency(0);
        std::chrono::microseconds total_compute_time(0);
        std::chrono::microseconds max_compute_time(0);
        
        int count_5us = 0;
        int count_10us = 0;
        int count_25us = 0;
        int count_50us = 0;
        int count_100us = 0;
  
        while (running_ && rclcpp::ok())
        {
            auto loop_start_time = std::chrono::steady_clock::now();  // Time before the loop starts
            //get wait time
            next_time += std::chrono::microseconds(LOOP_PERIOD_US);
            //run the callback to publish high speed

            state_high_speed_timer_callback();
            
            auto compute_end = std::chrono::steady_clock::now();  // Time before the loop starts
            
            //wait, we dont really need this as the get is a blocking command in the state_high_speed_timer_callback
            //anyways and it waits for the mainboard so...
            std::this_thread::sleep_until(next_time);
            //SO THE FOLLOWING LINES ARE MOVED UNTIL AFTER THE WAIT FOR THE FOLLOWING REASONS
            /*
             * basically we have two 1 kHz loops running, this one that interacts with 
             mblink and another one that actually runs our code. This one must
             send and recieve mblink data. but first it sends to the other loop which
             also runs at 1 kHz but is offset slightly to ensure it doesnt get stale
             data from previous loop. it then processes the data and sends back forces
             but if the send_user_forces_data_to_robot function was called before the sleep
             this data sent would be from the previous loop and not the current loop. 
             Thus there would be higher delays. So this is moved to after the sleep 
             such that it is populated with the latest data  
             * 
             */
            //now run the send user forces data to the robot function, BUT ONLY IF WE ARE IN CUSTOM MODE AND WE HAVE THE RIGHT MODE ON
            if(custom_enabled_ && (custom_mode_lowlevel_forces_emulation_ == curr_custom_mode_)){
                send_user_forces_data_to_robot();
            }
            //do same for torques
            if(custom_enabled_ && (custom_mode_lowlevel_torques_emulation_ == curr_custom_mode_)){
                send_user_torques_data_to_robot();
            }
            

            // Measure total loop latency (including processing and waiting time)
            auto loop_end_time = std::chrono::steady_clock::now();  // Time after the loop ends
            auto total_loop_time = std::chrono::duration_cast<std::chrono::microseconds>(loop_end_time - loop_start_time);
            auto compute_time = std::chrono::duration_cast<std::chrono::microseconds>(compute_end - loop_start_time);

            // Subtract the expected loop time (LOOP_PERIOD_US) from the actual loop time
            auto actual_latency = std::chrono::microseconds(std::abs(total_loop_time.count() - LOOP_PERIOD_US));

            // Store the loop latency (actual latency beyond the expected loop time)
            latencies.push_back(actual_latency);
            total_latency += actual_latency;
            compute_times.push_back(compute_time);
            total_compute_time += compute_time;

            // Track max loop latency
            if (actual_latency > max_latency) {
                max_latency = actual_latency;
            }
            if (compute_time > max_compute_time) {
                max_compute_time = compute_time;
            }
            
            if(actual_latency > std::chrono::microseconds(100)){
                count_100us++;
              }
              if(actual_latency > std::chrono::microseconds(50)){
                count_50us++;
              }
              if(actual_latency > std::chrono::microseconds(25)){
                count_25us++;
              }
              if(actual_latency > std::chrono::microseconds(10)){
                count_10us++;
              }
              if(actual_latency > std::chrono::microseconds(5)){
                count_5us++;
              }
              //set previous compute time
              previous_compute_time_ = compute_time;
            
            // std::cout << "Next start time: " << std::chrono::duration_cast<std::chrono::microseconds>(next_time.time_since_epoch()).count() << std::endl;

            // Calculate and print average loop latency
            if(latencies.size() == NUM_LATENCY_SAMPLES){
                auto avg_latency = total_latency / static_cast<int>(latencies.size());
                auto avg_compute_time = total_compute_time / static_cast<int>(compute_times.size());
                RCLCPP_WARN(this->get_logger(), "----------------------------------- LATENCY PRINTOUTS -----------------------");
                std::cout << "Average Update Loop Latency for High Speed Data (last " << latencies.size() << " loops): " 
                        << avg_latency.count() << " us, Max Latency: " 
                        << max_latency.count() << " us, Percent over(5us, 10us,25us,50us,100us):"   
                      << std::fixed << std::setprecision(2) << 
                      (100.0 * count_5us) / NUM_LATENCY_SAMPLES << ", " <<
                      (100.0 * count_10us) / NUM_LATENCY_SAMPLES << ", " <<
                      (100.0 * count_25us) / NUM_LATENCY_SAMPLES << ", " <<
                      (100.0 * count_50us) / NUM_LATENCY_SAMPLES << ", " <<
                      (100.0 * count_100us) / NUM_LATENCY_SAMPLES << std::endl; 
                std::cout << "Average Update Loop Compute Time for High Speed Data (last " << latencies.size() << " loops): " 
                      << avg_compute_time.count() << " us, Max Compute Time: " 
                      << max_compute_time.count() << " us"  << std::endl;    
              
                //add some warning prints as needed
                if(max_latency.count() > 100.0){
                    //if we have a greater than 0.1 ms max latency
                    RCLCPP_WARN(this->get_logger(), "MAX LATENCY OF HIGH SPEED DATA EXCEEDED 100 us");

                }       
                if(max_latency.count() > 500.0){
                    //if we have a greater than 0.5 ms max latency
                    RCLCPP_ERROR(this->get_logger(), "MAX LATENCY OF HIGH SPEED DATA EXCEEDED 500 us");

                }
                // Reset the latencies after printing the stats
                latencies.clear();
                compute_times.clear();
                total_latency = std::chrono::microseconds(0);
                max_latency = std::chrono::microseconds(0);
                max_compute_time = std::chrono::microseconds(0);
                total_compute_time = std::chrono::microseconds(0);
                count_5us = 0;
                count_10us = 0;
                count_25us = 0;
                count_50us = 0;
                count_100us = 0;
            }
        
        }
    }
    void preallocate_memory()
    {
        auto reserve_if_vector = [](auto& vec, size_t size)
        {
            if constexpr (std::is_same_v<std::decay_t<decltype(vec)>, std::vector<double>>)
            {
                vec.reserve(size);
            }
        };

        // Preallocate space for known-sized vectors
        for (auto& vec : {spirit_state_.se2twist_des, spirit_state_.joy_twist, spirit_state_.joy_buttons,
                        spirit_state_.debug_leg, spirit_state_.user_custom, spirit_state_.joint_cmd,
                        spirit_state_.joint_position, spirit_state_.joint_velocities, spirit_state_.joint_currents,
                        spirit_state_.joint_temperature, spirit_state_.joint_voltage, spirit_state_.joint_residuals,
                        spirit_state_.joint_status, spirit_state_.phase, spirit_state_.behavior,
                        spirit_state_.contacts, spirit_state_.swing_mode, spirit_state_.mode,
                        spirit_state_.imu_euler, spirit_state_.imu_linear_acceleration, spirit_state_.imu_angular_velocity,
                        spirit_state_.twist_linear, spirit_state_.slope_est, spirit_state_.y,
                        spirit_state_.voltage, spirit_state_.debug_timings, spirit_state_.diagnostics})
        {
            reserve_if_vector(vec, 20);  // Adjust size based on expected max elements
        }
    }
    void semdGhostSetRetryParamCallback(const trusses_custom_interfaces::msg::SpiritSetRetryParam msg){
        const std::string name = msg.name;
        const float value = msg.value;
        // std::cout << "READING PARAM MESSAGE NAME " << name << std::endl;
        // float return_val = mblink_->readParam(name, true);  // Run the setRetry command
        // std::cout << "PARAM MESSAGE NAME " << name <<  "HAS VALUE: " <<return_val << std::endl;

        std::cout << "SENDING SET RETRY MESSAGE NAME " << name << " WITH VALUE: " << value << std::endl;
        mblink_->setRetry(name, value);  // Run the setRetry command

    }
    void sendEnsureModeCommand(const std::string &fieldName, uint32_t valdes, uint32_t timeoutMS){
        //this runs the same ensuremode command that the mblink does but it prints out a success or failure
        bool result = mblink_->ensureMode(fieldName,valdes,timeoutMS);
        if(result){
            std::string print_string = "MODE SWITCH SUCCESS";
            RCLCPP_INFO(this->get_logger(), print_string.c_str());
        }else{
            std::string print_string = "MODE SWITCH UNSUCCESSFUL";
            RCLCPP_INFO(this->get_logger(), print_string.c_str());
        }
    }
    void sendGhostTrotControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        //convert the x_dot, y_dot, and yaw_dot components into an eigen vector
        Eigen::Vector3f control_send = {msg->linear.x,msg->linear.y,msg->angular.z};
        //send the message 
        mblink_->sendControl(control_send);
    }
    void sendPlannerMessageCallback(const std_msgs::msg::Int32::SharedPtr msg){
        std::string planner_en_string = "planner_en";
        std::string print_string = "ENABLING PLANNER MODE: " + std::to_string(msg->data);
        RCLCPP_INFO(this->get_logger(), print_string.c_str());

        //send message
        sendEnsureModeCommand(planner_en_string,msg->data,set_mode_waiting_time_);
    }
    void selfCheckCallback(const std_msgs::msg::Int32::SharedPtr msg){
        //get the checknum value
        int checkNum = msg->data;
        //define print string
        std::string print_string = "Sending Checknum Value: " + std::to_string(checkNum);
        RCLCPP_INFO(this->get_logger(), print_string.c_str());
        //send the checknum value
        mblink_->selfCheck(checkNum,0);
    }
    void sendLowlevelEmulationForcesPosCallback(const trusses_custom_interfaces::msg::SpiritLowlevelEmulationData::SharedPtr msg){
        //i want to switch this callback to be saving user data to a local pointer and then publishing it to the mblink in the high speed
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Received null message in sendLowlevelEmulationForcesPosCallback");
            return;
          }
        lowlevel_emulation_data_forces_msg_ = msg;
    }
    void sendLowlevelEmulationTorquesPosCallback(const trusses_custom_interfaces::msg::SpiritLowlevelEmulationData::SharedPtr msg){
        //i want to switch this callback to be saving user data to a local pointer and then publishing it to the mblink in the high speed
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Received null message in sendLowlevelEmulationTorquesPosCallback");
            return;
          }
        lowlevel_emulation_data_torques_msg_ = msg;
    }
    void sendCustomModeParameters(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Received null message in sendCustomModeParameters");
            return;
          }
        std::cout << "Recieved Custom Mode Parameters:      ";
        for (const auto& value : msg->data) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
        
        //we send direct to the mainboard, gotta check we are in the right mode first though
        if(custom_enabled_ && (custom_mode_lowlevel_forces_emulation_ != curr_custom_mode_) && (custom_mode_lowlevel_torques_emulation_ != curr_custom_mode_)){
            //if custom mode is on AND we are not in the lowlevel emulation mode
            Eigen::Matrix<float,58,1> data_mavlink_send= Eigen::Matrix<float,58,1>::Constant(std::numeric_limits<float>::quiet_NaN());
            //applies the data
            for(int i = 0; i < msg->data.size(); i++){
                data_mavlink_send(i) = msg->data[i];
            }
            std::cout << "Sending Custom Mode Parameter Data to Robot" <<std::endl;
            //send copied data over
            sendUserDataBasic(data_mavlink_send);

        }
    }
    void send_user_forces_data_to_robot(){
        //function to take the private sharedptr and send that info to the robot
        if(lowlevel_emulation_data_forces_msg_){
            //if pointer is not null
            //we need to apply the data now
            std::array<char,58*4> data_mavlink_send; //defines the data_mavlink char array
            //inits empty header
            char header[4];
            //copies header over
            memcpy(data_mavlink_send.data(),header,sizeof(header));
            //copies over the 36 floats to the data mavlink send char array, starting 4 bytes in
            //as first 4 bytes is header
            memcpy(data_mavlink_send.data() + 4,lowlevel_emulation_data_forces_msg_->data_first_thirty_six_spots.data(),lowlevel_emulation_data_forces_msg_->data_first_thirty_six_spots.size()*sizeof(float));
            //copies over remaining char data
            memcpy(data_mavlink_send.data() +4+36*sizeof(float),lowlevel_emulation_data_forces_msg_->remaining_char_data.data(),lowlevel_emulation_data_forces_msg_->remaining_char_data.size());
            

            //and send to the other function for high speed send
            sendUserDataChar(data_mavlink_send);
            //should I nullify the pointer here? 
        }
        

    }
    void send_user_torques_data_to_robot(){
        //function to take the private sharedptr and send that info to the robot
        if(lowlevel_emulation_data_torques_msg_){
            //if pointer is not null
            //we need to apply the data now
            std::array<char,58*4> data_mavlink_send; //defines the data_mavlink char array
            //inits empty header
            char header[4];
            //copies header over
            memcpy(data_mavlink_send.data(),header,sizeof(header));
            //copies over the 36 floats to the data mavlink send char array, starting 4 bytes in
            //as first 4 bytes is header
            memcpy(data_mavlink_send.data() + 4,lowlevel_emulation_data_torques_msg_->data_first_thirty_six_spots.data(),lowlevel_emulation_data_torques_msg_->data_first_thirty_six_spots.size()*sizeof(float));
            //copies over remaining char data
            memcpy(data_mavlink_send.data() +4+36*sizeof(float),lowlevel_emulation_data_torques_msg_->remaining_char_data.data(),lowlevel_emulation_data_torques_msg_->remaining_char_data.size());
            

            //and send to the other function for high speed send
            sendUserDataChar(data_mavlink_send);
            //should I nullify the pointer here? 
        }
        

    }
    void sendUserDataBasic(const Eigen::Matrix<float,58,1> &data_mavlink){
        //publishes data
        mblink_->sendUser(data_mavlink);
    }
    void sendUserDataChar(const std::array<char,58*4> &data_mavlink){
        //this is a high speed data publisher (~1000 Hz)
        //designed to work outside of ros2 overhead and for direct mainboard emulation on the edge compute
        //ideally we can send low level joint data commands with this 
        //BUT THIS SENDS AS CHARS, so we can send more data of lower quality
        //publishes data
        mblink_->sendUserDense(data_mavlink);
    }
    void ghostModeCallback(const std_msgs::msg::Int32::SharedPtr msg){
        //get data
        int ghost_mode = msg->data;
        std::string print_string = "Sending Ghost Mode: " + std::to_string(ghost_mode);
        RCLCPP_INFO(this->get_logger(), print_string.c_str());
        std::string custom_field = "custom";
        std::string ghost_idx = "action";
        std::string gait_idx = "gait";
        //set data

        //first turn off custom mode
        print_string = "DISABLING CUSTOM MODE";
        RCLCPP_INFO(this->get_logger(), print_string.c_str());
        //disable custom mode private variable
        custom_enabled_ = false;
        sendEnsureModeCommand(custom_field,0,set_mode_waiting_time_);
        //now set action
        print_string = "SENDING GHOST MODE COMMAND";
        RCLCPP_INFO(this->get_logger(), print_string.c_str());
        //if we are doing a sit, stand, or walk
        if(ghost_mode < 3){
            //send normally
            sendEnsureModeCommand(ghost_idx,ghost_mode,set_mode_waiting_time_);
            //and make sure gait is set to 0 which is walk
            sendEnsureModeCommand(gait_idx,0,set_mode_waiting_time_);
        }
        else{
            //otherwise, if we want to run, first set action to 2 
            sendEnsureModeCommand(ghost_idx,2,set_mode_waiting_time_);
            //now send a gait command to 1 which is run
            sendEnsureModeCommand(gait_idx,1,set_mode_waiting_time_);
        }
    }
    void customModeCallback(const std_msgs::msg::Int32::SharedPtr msg){
        //get data
        int custom_mode = msg->data;
        std::string print_string = "Sending Custom Mode: " + std::to_string(custom_mode);
        RCLCPP_INFO(this->get_logger(), print_string.c_str());
        //set data
        //first turn on custom mode
        std::string custom_field = "custom";
        std::string custom_idx = "custom_index";

        print_string = "ENABLING CUSTOM MODE";
        custom_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), print_string.c_str());
        sendEnsureModeCommand(custom_field,1,set_mode_waiting_time_);
        //now set custom
        print_string = "SENDING CUSTOM MODE COMMAND";
        RCLCPP_INFO(this->get_logger(), print_string.c_str());
        //saves to private variable
        curr_custom_mode_ = custom_mode;
        sendEnsureModeCommand(custom_idx,custom_mode,set_mode_waiting_time_);
    }
    void sendEstopCallback(const std_msgs::msg::Bool::SharedPtr msg){
        //function to send estop command to spriit
        std::string estop_str = "estop";
        if (msg->data){
            while (true){
                //continaully send 
                RCLCPP_INFO(this->get_logger(), "SENDING ESTOP COMMAND");
                sendEnsureModeCommand(estop_str,1,set_mode_waiting_time_);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }    
        }
    }
    void state_high_speed_timer_callback(){
        //call get command
        //check to make sure our previous compute time isn't higher than our 
        //loop rate. if it is then we need to skip this state high speed timer
        //callback as we dont we could enter a phase offset issue where we can
        //query for the robot state using the get command but it could take 999us
        //to come back from us if there is a 999us offset between this computer's
        //1 kHz clock and the robot's clock. this can help limit how big that offset
        //can become
        if(LOOP_PERIOD_US > previous_compute_time_.count()){

            get();
            time_curr_ = std::chrono::system_clock::now();
            if(skipped_last_get_call_ && (std::chrono::duration<float, std::micro>(time_curr_ - time_last_get_call_).count() > 2*LOOP_PERIOD_US)){
                RCLCPP_ERROR(this->get_logger(), "COMPUTE TIME EXCEEDED LOOP RATE AND IS GREATER THAN 2x LOOP PERIOD, BEHIND");
                std::cout <<  "LAST RAN GET CALL : "<< std::chrono::duration<float, std::milli>(time_curr_ - time_last_get_call_).count() << " ms prior" <<std::endl;
            }
            time_last_get_call_ = std::chrono::system_clock::now();
            skipped_last_get_call_ = false;
        }else{
            skipped_last_get_call_ = true;
            // RCLCPP_ERROR(this->get_logger(), "COMPUTE TIME EXCEEDED LOOP RATE, SKIPPING GET CALL TO RESET");
            // std::cout <<  "COMPUTE TIME: "<<previous_compute_time_.count() << " us" <<std::endl;
            
        }

        //now start assigning variables 
        //first set mainboard time 
        spirit_state_.mainboard_t= rx_data_["t"].value();
        //fills with pointers referring to begin and end of vector, allows for dynamic allocation
        
        //se2twist_des
        spirit_state_.se2twist_des.assign(rx_data_["se2twist_des"].begin(),rx_data_["se2twist_des"].end());
        //joy_twist
        spirit_state_.joy_twist.assign(rx_data_["joy_twist"].begin(),rx_data_["joy_twist"].end());
        //joy_buttons
        spirit_state_.joy_buttons.assign(rx_data_["joy_buttons"].begin(),rx_data_["joy_buttons"].end());

        //debug_legH
        spirit_state_.debug_leg.assign(rx_data_["debug_legH"].begin(),rx_data_["debug_legH"].end());
        //user_custom
        spirit_state_.user_custom.assign(rx_data_["user"].begin(),rx_data_["user"].end());
        

        //joint_cmd
        spirit_state_.joint_cmd.assign(rx_data_["joint_cmd"].begin(),rx_data_["joint_cmd"].end());        
        //pos
        spirit_state_.joint_position.assign(rx_data_["joint_position"].begin(),rx_data_["joint_position"].end());
        //velo
        spirit_state_.joint_velocities.assign(rx_data_["joint_velocity"].begin(),rx_data_["joint_velocity"].end());
        //current
        spirit_state_.joint_currents.assign(rx_data_["joint_current"].begin(),rx_data_["joint_current"].end());
        //joint_temperature
        spirit_state_.joint_temperature.assign(rx_data_["joint_temperature"].begin(),rx_data_["joint_temperature"].end());
        //joint voltage
        spirit_state_.joint_voltage.assign(rx_data_["joint_voltage"].begin(),rx_data_["joint_voltage"].end());
        //joint residuals
        spirit_state_.joint_residuals.assign(rx_data_["joint_residual"].begin(),rx_data_["joint_residual"].end());
        //joint status
        spirit_state_.joint_status.assign(rx_data_["joint_status"].begin(),rx_data_["joint_status"].end());

        //phase
        spirit_state_.phase.assign(rx_data_["phase"].begin(),rx_data_["phase"].end());
        //behavior
        spirit_state_.behavior.assign(rx_data_["behavior"].begin(),rx_data_["behavior"].end());
        //contact
        spirit_state_.contacts.assign(rx_data_["contacts"].begin(),rx_data_["contacts"].end());
        //swing mode
        spirit_state_.swing_mode.assign(rx_data_["swing_mode"].begin(),rx_data_["swing_mode"].end());
        //mode
        spirit_state_.mode.assign(rx_data_["mode"].begin(),rx_data_["mode"].end());


        //imu euler
        spirit_state_.imu_euler.assign(rx_data_["imu_euler"].begin(),rx_data_["imu_euler"].end());
        

        //imu_linear_acceleration
        spirit_state_.imu_linear_acceleration.assign(rx_data_["imu_linear_acceleration"].begin(),rx_data_["imu_linear_acceleration"].end());
        //imu_angular_velocity
        spirit_state_.imu_angular_velocity.assign(rx_data_["imu_angular_velocity"].begin(),rx_data_["imu_angular_velocity"].end());
        
        //twist_linear
        spirit_state_.twist_linear.assign(rx_data_["twist_linear"].begin(),rx_data_["twist_linear"].end());
        //z rel
        spirit_state_.z_rel= rx_data_["z_rel"].value();
        //slope est
        spirit_state_.slope_est.assign(rx_data_["slope_est"].begin(),rx_data_["slope_est"].end());
        //y
        spirit_state_.y.assign(rx_data_["y"].begin(),rx_data_["y"].end());
        
        //now the debug params
        //voltage
        spirit_state_.voltage.assign(rx_data_["voltage"].begin(),rx_data_["voltage"].end());
        //debug_timings
        spirit_state_.debug_timings.assign(rx_data_["debug_timings"].begin(),rx_data_["debug_timings"].end());
        //diagnostics
        spirit_state_.diagnostics.assign(rx_data_["diagnostics"].begin(),rx_data_["diagnostics"].end());
        //version
        spirit_state_.version= rx_data_["version"].value();
        //save time
        auto now = std::chrono::system_clock::now();
        // Example: Duration since epoch as float in seconds
        spirit_state_.msg_send_time = std::chrono::duration<double>(now.time_since_epoch()).count();
        //publish dummy data at high speed
        spirit_high_speed_publisher_->publish(spirit_state_);
    }
    void state_low_speed_timer_callback(){
        //high speed timer covers the get, so we already have the latest data
        //and data is saved to private variable spirit_state_ so we just need to publish
        //at lower rate
        //publish the state
        spirit_state_low_speed_publisher_->publish(spirit_state_);

    }
    void get(){
        rx_data_ = mblink_->get();
        //get time 
        rx_data_["t"] = rx_data_["y"].tail<1>();
        //resize y
        rx_data_["y"] = rx_data_["y"].head(rx_data_["y"].size() - 1);
        
        // RCLCPP_INFO(this->get_logger(), "GETTING DATA");
    }
    void initRxData() {
        // param_value - only when received
        // Commands
        rx_data_["se2twist_des"].resize(3);
        rx_data_["joy_twist"].resize(4);
        rx_data_["joy_buttons"].resize(8);
        // Misc
        rx_data_["debug_legH"].resize(3);
        rx_data_["user"].resize(10);
        // Joints
        rx_data_["joint_cmd"].resize(12);
        rx_data_["joint_position"].resize(12);
        rx_data_["joint_velocity"].resize(12);
        rx_data_["joint_current"].resize(12);
        rx_data_["joint_temperature"].resize(12);
        rx_data_["joint_voltage"].resize(12);
        rx_data_["joint_residual"].resize(8);
        rx_data_["joint_status"].resize(12);
        // Swing/contact states and behavior
        rx_data_["phase"].resize(4);
        rx_data_["behavior"].resize(3);
        rx_data_["contacts"].resize(4);
        rx_data_["swing_mode"].resize(4);
        rx_data_["mode"].resize(2);
        // IMU
        rx_data_["imu_euler"].resize(3);
        rx_data_["imu_linear_acceleration"].resize(3);
        rx_data_["imu_angular_velocity"].resize(3);
        // State estimates
        rx_data_["twist_linear"].resize(3);
        rx_data_["z_rel"].resize(1);
        rx_data_["slope_est"].resize(2);
        rx_data_["y"].resize(21);// last element is time
        // Debugging
        rx_data_["voltage"].resize(2);
        rx_data_["debug_timings"].resize(4);
        rx_data_["diagnostics"].resize(2);
        rx_data_["version"].resize(1); // log version
    
        // ctrldata detect size from data
        for (auto& entry : rx_data_) {
            // Resize to the required size (if not already done)
            // If the resize is already done before, we can skip this
            // Example: entry.second.resize(size);
    
            // Fill the vector with zeros
            std::fill(entry.second.begin(), entry.second.end(), 0);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //get args
    bool sim = false;
    bool verbose = true;
    int port = 0;
    std::string logname = "";
    
    // Simple argument parsing
    int opt;
    //arguments will be like "-s true" for true sim
    while ((opt = getopt(argc, argv, "s:v:p:l:")) != -1) {
        switch (opt) {
        case 's':
            sim = trim(optarg) == "true";
            break;
        case 'v':
            verbose = trim(optarg) == "true";
            break;
        case 'p':
            port = std::stoi(optarg);
            break;
        case 'l':
            logname = trim(optarg);
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " [-s sim] [-v verbose] [-p port] [-l logname]" << std::endl;
            return 1;
        }
    }

    auto node = std::make_shared<MBLinkNode>(sim,verbose,port,logname);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}