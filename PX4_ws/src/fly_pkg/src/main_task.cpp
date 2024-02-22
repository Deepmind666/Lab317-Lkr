#include <ros/ros.h>
#include <fly_pkg/pos_srv.h>
#include <fly_pkg/vision_srv.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.1415926f

#define ON 1 //飞行模式
#define Land -1 //降落
#define NO 0
#define Ready 2  //进入待机模式
#define EEROR 3  //位置失灵
#define fly_z 0.5
int vision_open = 0;


//位置信息订阅
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
}
//状态订阅回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

//mian_task_pos订阅
fly_pkg::vision_srv::Request vision;
bool vision_data_server(fly_pkg::vision_srv::Request& req,fly_pkg::vision_srv::Response& resp) {
    for(int i = 0;i<req.number;i++){
        vision.x[i] = req.x[i];
    }
    ROS_INFO("%f   %f   %f  %d",vision.x[0],vision.x[1],vision.x[2],req.number);
    resp.need_vision = vision_open;
    return true;
}


//飞行控制  pos_x 为x轴位置 yaw为偏航角 control_flag 为飞行控制指令
void fly_control(fly_pkg::pos_srv::Request& aim_pos,float pos_x,float pos_y,float pos_z,float yaw,short control_flag,
                    float angle_accuracy,float pos_accuracy){
    aim_pos.control_flag = control_flag;
    aim_pos.x   = pos_x;
    aim_pos.y   = pos_y;
    aim_pos.z   = pos_z;
    aim_pos.yaw = yaw;

    aim_pos.angle_accuracy = angle_accuracy;
    aim_pos.pos_accuracy = pos_accuracy;
}

//时间的单位是s 功能为到达目的地，并且保持一定的时间  times 为保持时间 hz为在此期间订阅话题的hz
void fly_control_with_time(fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client,float pos_x,float pos_y,float pos_z,
                            float yaw,short control_flag,float times,float hz,
                            float angle_accuracy,float pos_accuracy){
    ros::Time last_times = ros::Time::now();
    ros::Rate pos_rate(hz);
    fly_control(aim_pos.request, pos_x , pos_y , pos_z , yaw , control_flag,angle_accuracy,pos_accuracy); 
    while(ros::ok()){         
        while(ros::ok()){
            if(pos_client.call(aim_pos))
                break;
        }
        if(control_flag==Land){  //降落模式
            if(!current_state.armed){ //等待模式切换完毕
                break;
            }
        }
        else{
            if(aim_pos.response.arrive_flag){
                ROS_INFO("time question");
                if(ros::Time::now() -last_times> ros::Duration(times))
                    
                    break;
            }
            else{
                last_times = ros::Time::now();
            }
        }

        ros::spinOnce();
        pos_rate.sleep();
    }
}

//飞机进入待机模式 指令
void plane_to_ready(fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client){
    ros::Rate pos_rate(100);
    aim_pos.request.control_flag = Ready;
    while(ros::ok()){
        pos_client.call(aim_pos);
        if(current_state.mode == "OFFBOARD"&&current_state.armed){ //等待模式切换完毕
            break;
        }
        ros::spinOnce();
        pos_rate.sleep();
    }
}

void PID_revise(float x_kp,float x_ki,float x_kd,float x_outputMax,float x_outputMin,
                float y_kp,float y_ki,float y_kd,float y_outputMax,float y_outputMin,
                float z_kp,float z_ki,float z_kd,float z_outputMax,float z_outputMin,
                float yaw_kp,float yaw_ki,float yaw_kd,float yaw_outputMax,float yaw_outputMin,
                fly_pkg::pos_srv::Request& aim_pos){
    aim_pos.x_kp = x_kp;
    aim_pos.x_ki = x_ki;
    aim_pos.x_kd = x_kd;        
    aim_pos.x_outputMax = x_outputMax;
    aim_pos.x_outputMin = x_outputMin;

    aim_pos.y_kp = y_kp;
    aim_pos.y_ki = y_ki;
    aim_pos.y_kd = y_kd;        
    aim_pos.y_outputMax = y_outputMax;
    aim_pos.y_outputMin = y_outputMin;

    aim_pos.z_kp = z_kp;
    aim_pos.z_ki = z_ki;
    aim_pos.z_kd = z_kd;        
    aim_pos.z_outputMax = z_outputMax;
    aim_pos.z_outputMin = z_outputMin;

    aim_pos.yaw_kp = yaw_kp;
    aim_pos.yaw_ki = yaw_ki;
    aim_pos.yaw_kd = yaw_kd;        
    aim_pos.yaw_outputMax = yaw_outputMax;
    aim_pos.yaw_outputMin = yaw_outputMin;
}
void wait_time(float times,float hz,fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client){
    ros::Time last_times = ros::Time::now();
    while(ros::ok()){
        if(ros::Time::now() -last_times> ros::Duration(times)){
            break;
        }
        pos_client.call(aim_pos);
        ros::spinOnce();
        ros::Rate(hz).sleep();
    }
}


int main (int argc,char** argv){
    ros::init(argc,argv,"main_task");
    ros::NodeHandle nh;
    //客户端
    ros::ServiceClient pos_client = nh.serviceClient<fly_pkg::pos_srv>("pos_control");
    //订阅
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>  //接收无人机位置信息
		("mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>			//接收无人机模式状态
		("mavros/state", 10, state_cb);

    //服务端
    ros::ServiceServer vision_task_server = nh.advertiseService("vision_data",vision_data_server);//视觉通道

    fly_pkg::pos_srv aim_pos;

    ros::Rate pos_rate(100); //100hz

    //无用
    aim_pos.request.roll =  0;
    aim_pos.request.pitch = 0;
    
    float x_kp,x_ki,x_kd,x_speed_max,x_speed_min;
    float y_kp,y_ki,y_kd,y_speed_max,y_speed_min;
    float z_kp,z_ki,z_kd,z_speed_max,z_speed_min;
    float yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min;
    
    //PID初始参数读取
    nh.getParam("x_kp", x_kp);nh.getParam("x_ki", x_ki);nh.getParam("x_kd", x_kd);nh.getParam("x_speed_max", x_speed_max);nh.getParam("x_speed_min", x_speed_min);
    nh.getParam("y_kp", y_kp);nh.getParam("y_ki", y_ki);nh.getParam("y_kd", y_kd);nh.getParam("y_speed_max", y_speed_max);nh.getParam("y_speed_min", y_speed_min);
    nh.getParam("z_kp", z_kp);nh.getParam("z_ki", z_ki);nh.getParam("z_kd", z_kd);nh.getParam("z_speed_max", z_speed_max);nh.getParam("z_speed_min", z_speed_min);
    nh.getParam("yaw_kp", yaw_kp);nh.getParam("yaw_ki", yaw_ki);nh.getParam("yaw_kd", yaw_kd);nh.getParam("yaw_speed_max", yaw_speed_max);nh.getParam("yaw_speed_min", yaw_speed_min);
    ROS_INFO("--------------------------------PID-------------------------------------------");
    ROS_INFO("x_kp = %f   x_ki=%f  x_kd=%f x_speed_max=%f x_speed_min=%f ",x_kp,x_ki,x_kd,x_speed_max,x_speed_min);
    ROS_INFO("y_kp = %f   y_ki=%f  y_kd=%f y_speed_max=%f y_speed_min=%f ",y_kp,y_ki,y_kd,y_speed_max,y_speed_min);
    ROS_INFO("z_kp = %f   z_ki=%f  z_kd=%f z_speed_max=%f z_speed_min=%f ",z_kp,z_ki,z_kd,z_speed_max,z_speed_min);
    ROS_INFO("yaw_kp = %f   yaw_ki=%f  yaw_kd=%f yaw_speed_max=%f yaw_speed_min=%f ",yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min);

    ROS_INFO("--------------------------------PID-------------------------------------------");
    ros::service::waitForService("pos_control");  //等待位置服务器开启
    /*-----------进入待机模式-----------------------------------------------------------------------------*/  
    plane_to_ready(aim_pos,pos_client);
    /*-----------PID参数修改-----------------------------------------------------------------------------*/ 
    PID_revise(x_kp,x_ki,x_kd,x_speed_max,x_speed_min,                  //x_kp x_ki x_kd x_speed_max x_speed_min
               y_kp,y_ki,y_kd,y_speed_max,y_speed_min ,                //y_kp y_ki y_kd y_speed_max y_speed_min
               z_kp,z_ki,z_kd,z_speed_max,z_speed_min,                //z_kp z_ki z_kd z_speed_max z_speed_min
               yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min,      //yaw_kp yaw_ki yaw_kd yaw_speed_max yaw_speed_min
               aim_pos.request);    


    //当前角度为0 左边为正 右边为负 180 0 -180
    //正方形测试
    /*-------------------------------------------------------------------------------------------------*/  
    fly_control_with_time(aim_pos,pos_client, 0 , 0 , 1.0 , 0*PI/180 , ON, 1.0f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f);          //角度精度   位置精度 
    //wait_time(1,100,aim_pos,pos_client);
    /*-------------------------------------------------------------------------------------------------*/  
    ROS_INFO("position1");
    fly_control_with_time(aim_pos,pos_client, 0 , 1 , 1.0 , 0*PI/180 , ON,0.25f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f);          //角度精度   位置精度 ff
    /*-------------------------------------------------------------------------------------------------*/ 
    ROS_INFO("position2");
    fly_control_with_time(aim_pos,pos_client, 0 , 1.5 , 1.0 , 0*PI/180 , ON,0.25f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f);          //角度精度   位置精度 ff  
    /*-------------------------------------------------------------------------------------------------*/
    ROS_INFO("position3");
    fly_control_with_time(aim_pos,pos_client, 0 , 1.5 , 0 , 0*PI/180 , Land,1,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f);           //角度精度   位置精度  
    

    ROS_INFO("fly_task is close");
    while(ros::ok()){
        ros::spinOnce();
        pos_rate.sleep();
    }
}
