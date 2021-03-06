#include "ros/ros.h"
#include <stdio.h>
#include <time.h>

// msg
#include "fallback_decision/aes_data.h"
#include "fallback_decision/Chassis.h"
#include "fallback_decision/Sensor.h"
#include "fallback_decision/Traffic.h"
#include "fallback_decision/Line.h"
#include "fallback_decision/AEB.h"

// bev image header
#include "BEV_image.h"
#include "MWCUSOLVERUtils.hpp"
#include "BEV_image_data.h"
#include "BEV_image_initialize.h"
#include "BEV_image_terminate.h"
#include "BEV_image_types.h"
#include "MWCudaDimUtility.hpp"
#include "MWLaunchParametersUtilities.hpp"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

// predict header
#include "Decision_Predict_data.h"
#include "Decision_Predict.h"
#include "Decision_Predict_initialize.h"
#include "Decision_Predict_internal_types.h"
#include "Decision_Predict_terminate.h"
#include "DeepLearningNetwork.h"
#include "predict.h"
#include "rtwtypes.h"

#include "MWCNNLayer.hpp"
#include "MWCNNLayerImpl.hpp"
//#include "MWCudaDimUtility.hpp"
#include "MWCustomLayerForCuDNN.hpp"
#include "MWElementwiseAffineLayer.hpp"
#include "MWFCLayer.hpp"
#include "MWFCLayerImpl.hpp"
#include "MWFusedConvReLULayer.hpp"
#include "MWFusedConvReLULayerImpl.hpp"
#include "MWInputLayerImpl.hpp"
#include "MWKernelHeaders.hpp"
#include "MWMaxPoolingLayer.hpp"
#include "MWMaxPoolingLayerImpl.hpp"
#include "MWOutputLayer.hpp"
#include "MWOutputLayerImpl.hpp"
#include "MWSoftmaxLayer.hpp"
#include "MWSoftmaxLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "MWTensorBase.hpp"

// dicision header
#include "P_result.h"


double chassis_[12];
double sensor_[11];
double traffic_[10];
double line_[11];
double aeb_[2];
unsigned char outBEV[367464];
float outPredict[7];
double outResult;

void msgCopy(const fallback_decision::aes_data::ConstPtr& msg);

void msgCallback(const fallback_decision::aes_data::ConstPtr& msg) {
//    printf("in bev_image %lf\n", msg->chassis.timestamp);
    const fallback_decision::aes_data::ConstPtr& msg_tmp=msg;
    clock_t start_c, end_c, half_c;
    start_c = clock();
    end_c = half_c = 0;

    msgCopy(msg_tmp); // function
    
//    printf("in bev_image %lf\n", chassis_[0]);

    BEV_image(chassis_, sensor_, traffic_, line_, aeb_, outBEV);
    
//    printf("in bev_image outBEV %d\n", outBEV[0]);
    half_c = clock();

    Decision_Predict(outBEV, outPredict);

    outResult = P_result(outPredict);
 
    end_c = clock();

    printf("timestamp: %lf, Result: %lf, ",chassis_[0], outResult);
    printf("CycleTime: %lf (bev: %lf, predictL %lf)\n",(double)(end_c-start_c)/CLOCKS_PER_SEC,(double)(half_c-start_c)/CLOCKS_PER_SEC,(double)(end_c-half_c)/CLOCKS_PER_SEC);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "bev_sub_node");
    ros::NodeHandle nh;

    ros::Subscriber bev_sub = nh.subscribe("AES_data_pub", 1, msgCallback);

    ros::spin();

    return 0;
}

void msgCopy(const fallback_decision::aes_data::ConstPtr& msg)
{
    chassis_[0] = msg->chassis.timestamp;
    chassis_[1] = msg->chassis.invehicle_yaw;
    chassis_[2] = msg->chassis.invehicle_gloposy;   
    chassis_[3] = msg->chassis.invehicle_gloposx;   
    chassis_[4] = msg->chassis.invehicle_velocity;  
    chassis_[5] = msg->chassis.invehicle_gloaccx;   
    chassis_[6] = msg->chassis.invehicle_glovelx;   
    chassis_[7] = msg->chassis.invehicle_glovely;   
    chassis_[8] = msg->chassis.invehicle_cg2rearbumper; 
    chassis_[9] = msg->chassis.invehicle_cg2frontbumper;
    chassis_[10] = msg->chassis.invehicle_length;
    chassis_[11] = msg->chassis.invehicle_width;

    sensor_[0] = msg->sensor.timestamp;
    sensor_[1] = msg->sensor.camera_dtct;
    sensor_[2] = msg->sensor.camera_relposy;
    sensor_[3] = msg->sensor.camera_relposx;
    sensor_[4] = msg->sensor.camera_relvely;
    sensor_[5] = msg->sensor.camera_relvelx;
    sensor_[6] = msg->sensor.frontradar_dtct;
    sensor_[7] = msg->sensor.frontradar_relposy;
    sensor_[8] = msg->sensor.frontradar_relposx;
    sensor_[9] = msg->sensor.frontradar_relvely;
    sensor_[10] = msg->sensor.frontradar_relvelx;

    traffic_[0] = msg->traffic.timestamp;
    traffic_[1] = msg->traffic.traffic_rv_gloposy;
    traffic_[2] = msg->traffic.traffic_rv_gloposx;
    traffic_[3] = msg->traffic.traffic_rv_headingangle;
    traffic_[4] = msg->traffic.traffic_rv_glovelx;
    traffic_[5] = msg->traffic.traffic_rv_glovely;
    traffic_[6] = msg->traffic.traffic_rv_cg2rearbumper;
    traffic_[7] = msg->traffic.traffic_rv_cg2frontbumper;
    traffic_[8] = msg->traffic.traffic_rv_length;
    traffic_[9] = msg->traffic.traffic_rv_width;

    line_[0] = msg->line.timestamp;
    line_[1] = msg->line.linepoly_a_l;
    line_[2] = msg->line.linepoly_a_r;
    line_[3] = msg->line.linepoly_b_l;
    line_[4] = msg->line.linepoly_b_r;
    line_[5] = msg->line.linepoly_c_l;
    line_[6] = msg->line.linepoly_c_r;
    line_[7] = msg->line.linepoly_d_l;
    line_[8] = msg->line.linepoly_d_r;
    line_[9] = msg->line.sensor_line_front_nline_left;
    line_[10] = msg->line.sensor_line_front_nline_right;

    aeb_[0] = msg->aeb.timestamp;
    aeb_[1] = msg->aeb.linepoly_a_l;
} 
