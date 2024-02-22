#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <fly_pkg/pos_srv.h>
#include <fly_pkg/pos_msg.h>
#include <tf/tf.h>
#include <math.h>

#define control_hz 100

#define PI 3.1415926f
#define ON 1
#define Land -1
#define NO 0
#define Ready 2  //进入待机模式
#define EEROR 3  //位置失灵

int control_falg = NO; //控制指令

float x_max[2] = {-3.0f,3.0f};  //最小值 最大值
float y_max[2] = {-3.0f,3.0f};  //最小值 最大值
float z_max = 1.0f;              //最大值

typedef struct pose_IMU{
	float roll;
	float pitch;
	float yaw;
}IMU;

float positional_PID (struct P_pid_obj *obj, struct PID_param *pid);
IMU mavros_ENU = {0,0,0};

struct P_pid_obj {
	float output;
	float bias;
	float measure;
	float last_bias;
	float integral;						//积分
	float last_differential;	//上一次微分
	float target;
};
struct PID_param {
	float kp;
	float ki;
	float kd;
	float differential_filterK=0.5;			//滤波系数
	float outputMin;
	float outputMax;
};

struct P_pid_obj x_pid_obj = {0,0,0,0,0,0};
struct PID_param x_pid_param;

struct P_pid_obj y_pid_obj = {0,0,0,0,0,0};
struct PID_param y_pid_param;

struct P_pid_obj z_pid_obj = {0,0,0,0,0,0};
struct PID_param z_pid_param;

struct P_pid_obj yaw_pid_obj = {0,0,0,0,0,0};
struct PID_param yaw_pid_param;

geometry_msgs::Quaternion get_q(float yaw_aim)
{
    tfScalar yaw,pitch,roll;
    tf::Quaternion q;
    geometry_msgs::Quaternion quat;

    roll  = 0;
    pitch = 0;
    yaw   = yaw_aim;

    q.setEulerZYX(yaw,pitch,roll);
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();
    return quat;
}

//返回最小角度
float need2turn(float nowangle,float targetangle)
{			
	float need2Turn;		

	need2Turn=targetangle-nowangle;		//实际所需转的角度
	if(need2Turn>PI)	need2Turn -= 2*PI;
    else if(need2Turn<-PI)	need2Turn += PI;
	
    return need2Turn;		
}

//状态订阅回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

struct POS_RSET{
    float x_bias = 0;
    float y_bias = 0;
    float z_bias = 0;
    float yaw_bias = 0;
}pos_reset;

float last_light_z = 0;   //激光断开保护
//位置信息订阅
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;

    local_pos.pose.position.x-=pos_reset.x_bias;
    local_pos.pose.position.y-=pos_reset.y_bias;
    local_pos.pose.position.z-=pos_reset.z_bias;


    float IMU_q[4] = {0,0,0,0};   //四元数
	//四元数值
	IMU_q[0] = local_pos.pose.orientation.x;
	IMU_q[1] = local_pos.pose.orientation.y;
	IMU_q[2] = local_pos.pose.orientation.z;
	IMU_q[3] = local_pos.pose.orientation.w;

	//四元数解算欧拉角  ---->  (mavros 的ENU坐标)
	mavros_ENU.yaw = atan2(  2* ((IMU_q[0]*IMU_q[1]) + (IMU_q[2]*IMU_q[3])) , 1-2*(IMU_q[1]*IMU_q[1]+IMU_q[2]*IMU_q[2]))-PI/2;
    if(mavros_ENU.yaw>PI){
		mavros_ENU.yaw-=2*PI;
	}
	else if(mavros_ENU.yaw<-PI){
		mavros_ENU.yaw+=2*PI;
	}

    mavros_ENU.yaw -=pos_reset.yaw_bias;
    if(mavros_ENU.yaw>PI){
		mavros_ENU.yaw-=2*PI;
	}
	else if(mavros_ENU.yaw<-PI){
		mavros_ENU.yaw+=2*PI;
	}

}

void debug (int flag)
{
    if(flag==1)
        ROS_INFO("success");
    else
        ROS_INFO("failure");
}

//mian_task_pos订阅
fly_pkg::pos_msg  aim_pos;
bool pos_control_server(fly_pkg::pos_srv::Request& req,fly_pkg::pos_srv::Response& resp) {
    //位置和控制指令
	aim_pos.x = req.x;
    aim_pos.y = req.y ;
    aim_pos.z = req.z;
    aim_pos.control_flag = req.control_flag;
    //pid参数
    x_pid_param.kp = req.x_kp;
    x_pid_param.ki = req.x_ki;
    x_pid_param.kd = req.x_kd;
    x_pid_param.outputMax = req.x_outputMax;
    x_pid_param.outputMin = req.x_outputMin;

    y_pid_param.kp = req.y_kp;
    y_pid_param.ki = req.y_ki;
    y_pid_param.kd = req.y_kd;
    y_pid_param.outputMax = req.y_outputMax;
    y_pid_param.outputMin = req.y_outputMin;

    z_pid_param.kp = req.z_kp;
    z_pid_param.ki = req.z_ki;
    z_pid_param.kd = req.z_kd;
    z_pid_param.outputMax = req.z_outputMax;
    z_pid_param.outputMin = req.z_outputMin;

    yaw_pid_param.kp = req.yaw_kp;
    yaw_pid_param.ki = req.yaw_ki;
    yaw_pid_param.kd = req.yaw_kd;
    yaw_pid_param.outputMax = req.yaw_outputMax;
    yaw_pid_param.outputMin = req.yaw_outputMin;

    //精度信息
    aim_pos.angle_accuracy  = req.angle_accuracy;
    aim_pos.pos_accuracy  = req.pos_accuracy;

    //角度信息
    aim_pos.yaw = req.yaw;

    if(aim_pos.yaw>PI){
		aim_pos.yaw-=2*PI;
	}
	else if(aim_pos.yaw<-PI){
		aim_pos.yaw+=2*PI;
	}

    //ROS_INFO("now_yaw=%f,aim_yaw=%f",mavros_ENU.yaw,aim_pos.yaw);
    if(fabs(local_pos.pose.position.x-aim_pos.x)<aim_pos.pos_accuracy &&
       fabs(local_pos.pose.position.y-aim_pos.y)<aim_pos.pos_accuracy &&
       fabs(local_pos.pose.position.z-aim_pos.z)<aim_pos.pos_accuracy 
       ){
        if(fabs(need2turn(mavros_ENU.yaw,aim_pos.yaw))<aim_pos.angle_accuracy){
            resp.arrive_flag = 1;  //到达
            ROS_INFO("arrive position");
        }
        else{
            ROS_INFO("未到达位置");
            resp.arrive_flag = 0; //未到达
        }
    }
    else{
        resp.arrive_flag = 0; //未到达
    }
    ROS_INFO("x = %f  y = %f  z = %f  yaw = %f",fabs(local_pos.pose.position.x-aim_pos.x),
                                                fabs(local_pos.pose.position.y-aim_pos.y),
                                                fabs(local_pos.pose.position.z-aim_pos.z),
                                                fabs(need2turn(mavros_ENU.yaw,aim_pos.yaw)));
    return true;
}


//位置式PID
//带抗积分饱和
//带微分项低通滤波
float positional_PID (struct P_pid_obj *obj, struct PID_param *pid)
{
	float differential = 0;
	
	obj->bias = obj->target - obj->measure;
	
    //只有误差小 才需要积分项
    if(fabs(obj->bias)<0.3f){
        //抗积分饱和
        if (obj->output >= pid->outputMax)
        {
            if (obj->bias < 0)
                obj->integral += obj->bias/control_hz;
        }
        else if (obj->output <= pid->outputMin)
        {
            if (obj->bias > 0)
                obj->integral += obj->bias/control_hz;
        }
        else
        {
            obj->integral += obj->bias;
        }
    }
    else{
        obj->integral = 0;
    }

	//微分项低通滤波
	differential = (obj->bias - obj->last_bias) * pid->differential_filterK + 
					(1 - pid->differential_filterK) * obj->last_differential;
	
	obj->output = pid->kp * obj->bias + pid->ki * obj->integral + pid->kd * differential;
	
	obj->last_bias = obj->bias;
	obj->last_differential = differential;
	
    //限幅
    if(obj->output>pid->outputMax){
        obj->output = pid->outputMax;
    }
    else if(obj->output<pid->outputMin){
        obj->output = pid->outputMin;
    }
	return obj->output;
}


float get_yaw_speed(void){
    yaw_pid_obj.measure = -need2turn(mavros_ENU.yaw,aim_pos.yaw);
    yaw_pid_obj.target  = 0; 
    return positional_PID(&yaw_pid_obj, &yaw_pid_param);
}

int main (int argc,char** argv)
{
    ros::init(argc,argv,"pos_task");
    ros::NodeHandle nh;

	//话题接收
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>  //接收无人机位置信息
		("mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>			//接收无人机模式状态
		("mavros/state", 10, state_cb);   
    //发布
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>	//发布期望位置
	("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>	//发布期望速度
	("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    //客户端
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>//设置待机
		("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>//设置模式
		("mavros/set_mode");
        
    //服务端
    ros::ServiceServer main_task_pos_server = nh.advertiseService("pos_control",pos_control_server);//服务main_task任务的目标位置
    
    //位置
    geometry_msgs::PoseStamped pose;      //期望位置发布数据
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 0;
    aim_pos.control_flag = 0;     //复位
    aim_pos.yaw = 0.0f;
    ros::Rate pos_rate(control_hz); //100hz

    //速度
    geometry_msgs::Twist vel; 
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    //角度速度
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    while (ros::ok() && !current_state.connected)
    {
        /* wait until px4 connected */
        ROS_INFO_STREAM("wait until px4 connected ");
        ros::spinOnce();
        pos_rate.sleep();
    }

    while (ros::ok() && local_pos.pose.position.x==0 &&local_pos.pose.position.y==0&&local_pos.pose.position.z==0){ //等待位置信息就绪
        ROS_INFO("Not pos_data");
        ros::spinOnce();
        pos_rate.sleep();
    }

    int  need_reset = 0;
    float xbias=0,ybias=0,zbias=0,yawbias=0;
    nh.getParam("need_reset", need_reset);
	for (int i = 500; ros::ok() && i > 0; --i) { //先发送n个位置信息缓冲一下  5s
		local_pos_pub.publish(pose);  
        if(i%25 == 0 && need_reset){
            xbias+= local_pos.pose.position.x;
            ybias+= local_pos.pose.position.y;
            zbias+= local_pos.pose.position.z;
            yawbias+= mavros_ENU.yaw;
        }
		ros::spinOnce();
		pos_rate.sleep();
	}

    if (need_reset){
        pos_reset.x_bias = xbias/20;  
        pos_reset.y_bias = ybias/20;  
        pos_reset.z_bias = zbias/20;  
        pos_reset.yaw_bias = yawbias/20; 
    }
 
    //模式设置参数
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //待机参数
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value=true;

    //姿态参数  采用ENU坐标系
    geometry_msgs::Quaternion q;

    ros::Time last_request = ros::Time::now();

    //偏航角目标
    float yaw_aim = 0;

    //参数复位
    q = get_q(0.0f);
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x=q.x;
    pose.pose.orientation.y=q.y;
    pose.pose.orientation.z=q.z;
    pose.pose.orientation.w=q.w;
    

    float last_z_val = 0;
    int protect = 0;

    float Land_height;  //降落锁电机高度
    nh.getParam("Land_height", Land_height);
    
    while (ros::ok())
    {
        if(control_falg!=EEROR){
            control_falg =  aim_pos.control_flag;
        }

        //正常运行ing 
        switch(control_falg){

            case EEROR:
                vel.linear.z = -0.2f;
                vel.linear.x = 0;
                vel.linear.y = 0;
                ROS_INFO("------ EEROR -----");
                local_vel_pub.publish(vel);  //发送速度信息               
                break;

            case Ready:
                offb_set_mode.request.custom_mode = "OFFBOARD";
                arm_cmd.request.value=true;
                if (current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {//如果模式不是offboard以及时间超过5s
                    if (set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent) { //申请切换为offboard
                        ROS_INFO("Offboard enabled");//成功
                    }
                        last_request = ros::Time::now();
                }
                else{
                    arm_cmd.request.value=true;
                    if (!current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))) { //如果不是待机模式以及时间超过5s
                        if (arming_client.call(arm_cmd) &&   
                                arm_cmd.response.success) {   //申请进入待机模式
                                ROS_INFO("Vehicle armed");    //成功
                        }
                        last_request = ros::Time::now();
                    }
                }
                ROS_INFO("offboard");
                local_pos_pub.publish(pose);  //发送位置信息
                break;

            case NO:
                q = get_q(0.0f);
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = 0;
                pose.pose.orientation.x=q.x;
                pose.pose.orientation.y=q.y;
                pose.pose.orientation.z=q.z;
                pose.pose.orientation.w=q.w;
                local_pos_pub.publish(pose);  //发送位置信息
                ROS_INFO("waitting main_task");
                break;

            case ON:        
                // if(current_state.armed && current_state.mode=="MANUAL"){
                //     control_falg = EEROR;
                // }    
                //PID目标赋值      
                x_pid_obj.measure = local_pos.pose.position.x;
                x_pid_obj.target  = aim_pos.x;
                y_pid_obj.measure = local_pos.pose.position.y;
                y_pid_obj.target  = aim_pos.y;
                z_pid_obj.measure = local_pos.pose.position.z;
                z_pid_obj.target  = aim_pos.z;      

                //ROS_INFO("%f",local_pos.pose.position.z);
                //PID输出速度值
                vel.linear.x = positional_PID(&x_pid_obj, &x_pid_param);
                vel.linear.y = positional_PID(&y_pid_obj, &y_pid_param);
                vel.linear.z = positional_PID(&z_pid_obj, &z_pid_param);
                
                last_z_val = vel.linear.z;

                vel.angular.z  = get_yaw_speed();

                local_vel_pub.publish(vel);  //发送速度信息

                ROS_INFO("  aim_x=%f   pid_x=%f    aim_y=%f pid_y=%f",aim_pos.x, local_pos.pose.position.x, aim_pos.y, local_pos.pose.position.y);
                ROS_INFO("main_task is connect1234----");
                break;

            case Land:
                //PID目标赋值      
                x_pid_obj.measure = local_pos.pose.position.x;
                x_pid_obj.target  = aim_pos.x;
                y_pid_obj.measure = local_pos.pose.position.y;
                y_pid_obj.target  = aim_pos.y;
                z_pid_obj.measure = local_pos.pose.position.z;
                z_pid_obj.target  = aim_pos.z;      

                //PID输出速度值
                vel.linear.x = positional_PID(&x_pid_obj, &x_pid_param);
                vel.linear.y = positional_PID(&y_pid_obj, &y_pid_param);
                vel.linear.z = positional_PID(&z_pid_obj, &z_pid_param);

                vel.angular.z  = get_yaw_speed();

                if(local_pos.pose.position.z < Land_height){
                    ROS_INFO("-------land---------");
                    if(current_state.mode == "OFFBOARD"){
                        offb_set_mode.request.custom_mode = "MANUAL";
                        set_mode_client.call(offb_set_mode);
                    }
                    if(current_state.armed){
                        arm_cmd.request.value=false;
                        arming_client.call(arm_cmd); 
                    }
                }
                else{
                    local_vel_pub.publish(vel);  //发送速度信息
                }

                break;

            default:
                break;
        }     
        //ROS_INFO("x = %f  y = %f  z = %f  yaw = %f",local_pos.pose.position.x,local_pos.pose.position.y,local_pos.pose.position.z,mavros_ENU.yaw);
        //ROS_INFO("x = %f  y = %f  z = %f  yaw = %f",pos_reset.x_bias,pos_reset.y_bias,pos_reset.z_bias,pos_reset.yaw_bias);
        //ROS_INFO("yaw = %f   aim_yaw = %f",mavros_ENU.yaw,aim_pos.yaw);
        ros::spinOnce();
        pos_rate.sleep();
    }
    return 0;
}
