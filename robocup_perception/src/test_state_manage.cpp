#include<ros/ros.h>
#include<std_msgs/String.h>
#include<std_msgs/Bool.h>
#include<robocup_perception/Classify.h>
#include<robocup_perception/Cameralaunch.h>

int main(int argc,char **argv){
    ros::init(argc, argv, "test_state_manage");
    ros::NodeHandle node;

    bool camera_flag = true;
    bool classify_flag = false;
    int result_number = 0;
    std::string name;

    ros::ServiceClient camera_client = node.serviceClient<robocup_perception::Cameralaunch>("camera_call");
    ros::ServiceClient classify_client = node.serviceClient<robocup_perception::Classify>("classify_call");

    robocup_perception::Cameralaunch image_srv;
    robocup_perception::Classify classify_srv;

    ros::Rate loop_rate(1);

    while(ros::ok()){
        image_srv.request.flag = camera_flag;

        if(camera_client.call(image_srv)){
            name = image_srv.response.name;
            classify_flag = image_srv.response.result;
            classify_srv.request.image_name = name;
            classify_srv.request.flag = classify_flag;
            if(classify_client.call(classify_srv)){
                result_number = classify_srv.response.result_num;
                camera_flag = false;
                classify_flag = false;
                std::cout<<result_number<<std::endl;
            }else{
                ROS_ERROR("Faild to call service classify_client");
            }
        }else{
            ROS_ERROR("Faild to call service camera_client");
            camera_flag = true;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
