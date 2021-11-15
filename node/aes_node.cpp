#include "ros/ros.h"
#include <boost/bind.hpp>
#include <stdio.h>
#include <string.h>
#include <time.h>
//#include <omp.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*** message header ***/
// chassis
#include "chassis_msg/LOG_BYTE0.h"
#include "chassis_msg/LOG_BYTE1.h"
// track
#include "autoware_msgs/DetectedObjectArray.h"
// lane
#include "mobileye_avante_msg/ME_Right_Lane_A.h"
#include "mobileye_avante_msg/ME_Right_Lane_B.h"
#include "mobileye_avante_msg/ME_Left_Lane_A.h"
#include "mobileye_avante_msg/ME_Left_Lane_B.h"

#include "chassis_msg/LOG_BYTE2.h"

#include "fallback_decision/AES_decision.h" 

/*** bev image header ***/
#include "coder_bounded_array.h"
#include "CTRV_MODEL.h"
#include "CV_MODEL.h"
#include "det.h"
#include "find.h"
#include "HONDA.h"
#include "I_lat.h"
#include "inpolygon.h"
#include "Interacting.h"
#include "inv.h"
#include "isequal.h"
#include "linspace.h"
#include "meshgrid.h"
#include "minOrMax.h"
#include "Mixing.h"
#include "mrdivide_helper.h"
#include "norm.h"
#include "RSS_model.h"
#include "rtwtypes.h"
#include "sqrt.h"
#include "sqrtm.h"
#include "TLC.h"
#include "xdhseqr.h"
#include "xdlanv2.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xzlarf.h"
#include "BEV_image.h"
#include "BEV_image_data.h"
#include "BEV_image_initialize.h"
#include "BEV_image_terminate.h"
#include "BEV_image_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

/*** predict header ***/
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

/*** dicision header ***/
#include "P_result.h"

/*** quat2eul header ***/
#include "quat2eul_aes_data.h"
#include "quat2eul_aes.h"
#include "quat2eul_aes_initialize.h"
#include "quat2eul_aes_terminate.h"
#include "quat2eul_aes_types.h"
#include "rt_defines.h"

//magick
#include "Magick++.h"
#include "matlab_array2magick.h"

using namespace message_filters;
using namespace chassis_msg;
using namespace autoware_msgs;
using namespace mobileye_avante_msg;
using namespace Magick;


double chassis_[12] = {0, 0, 0, 0, 0, 0, 0, 2.19, 2.46, 4.85, 1.82};
double track_[289];
double line_[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1};
double aeb_ = 1;
unsigned char outBEV[275598];
unsigned char img[275598];
float outPredict[7];
double outResult;
//unsigned char pix [] = {200,200,200,100,100,100,0,0,0,255,0,0,0,255,0,0,0,255};
static long int idx;
static int pub_flag;




void AES_Publish(float result, ros::Publisher AES_pub)
{
    fallback_decision::AES_decision AES_msg;
    static int index = 0;

    AES_msg.header.stamp = ros::Time::now();
    AES_msg.header.seq = ++index;

    switch((int)result){
    case 2:
        AES_msg.result.data="ESL";
        break;
    case 3:
        AES_msg.result.data="ESR";
        break;
    case 4:
        AES_msg.result.data="ESS";
        break;
    case 5:
        AES_msg.result.data="ELCL";
        break;
    case 6:
        AES_msg.result.data="ELCR";
        break;
    case 7:
        AES_msg.result.data="CM";
        break;
    case 8:
        AES_msg.result.data="Safe";
        break;
    }
    AES_pub.publish(AES_msg);
}

void AES_Decision(void)
{

    clock_t start_c, end_c, half_c;
    start_c = clock();
    end_c = half_c = 0;

    
//    printf("in bev_image %lf\n", chassis_[0]);

    BEV_image(chassis_+1, track_+1, line_+1, aeb_, outBEV, img);
    
//    printf("in bev_image outBEV %d\n", outBEV[0]);
    half_c = clock();
    
    Decision_Predict(outBEV, outPredict);
//    printf("in predict outPredict %f\n", outPredict[0]);

    outResult = P_result(outPredict);
 
    char file_name[100] = "./src/fallback_decision/bev_image/";
    char str[20] = {};

    Image image;
    image.read(366,251,"RGB",CharPixel,img);
//    printf("AES check2\n");
    sprintf(str, "%06ld.jpg", idx++);
    strcat(file_name, str);
    image.write(file_name);

    pub_flag = 1;
//    AES_Publish(outResult);

    end_c = clock();

    printf("Result: %lf, ", outResult);
    printf("CycleTime: %lf (bev: %lf, predictL %lf)\n",(double)(end_c-start_c)/CLOCKS_PER_SEC,(double)(half_c-start_c)/CLOCKS_PER_SEC,(double)(end_c-half_c)/CLOCKS_PER_SEC);

//    #pragma omp parallel for
//   for(int i=0; i<=10;++i) {
//       printf("i %d\n", i);
//    }

}

void AESCb(const LOG_BYTE0ConstPtr& byte0,
           const LOG_BYTE1ConstPtr& byte1,
           const ME_Left_Lane_AConstPtr& leftA,
           const ME_Left_Lane_BConstPtr& leftB,
           const ME_Right_Lane_AConstPtr& rightA,
           const ME_Right_Lane_BConstPtr& rightB)
           //const DetectedObjectArrayConstPtr& objectarr)

{
    //printf("sync success\n");
// Chassis
//    chassis_[0] = byte0->header.stamp.sec + byte0->header.stamp.nsec/1000000000.;
//    printf("chassis time : %.3lf\n", chassis_[0]);
    //printf("chassis time : %d.%d\n", byte0->header.stamp.sec, byte0->header.stamp.nsec);
    chassis_[4] = ( byte0->WHL_SPD_RL + byte0->WHL_SPD_RR ) * 0.5;
    chassis_[6] = chassis_[4];
    chassis_[5] = byte1->LONG_ACCEL;

// Lane
    line_[3] = leftA->LaneMarkModelA_C2_Lh_ME;
    line_[7] = leftA->LaneMarkPosition_C0_Lh_ME;
    line_[1] = leftB->LaneMarkModelDerivA_C3_Lh_ME;
    line_[5] = leftB->LaneMarkHeadingAngle_C1_Lh_ME;

    line_[4] = rightA->LaneMarkModelA_C2_Rh_ME;
    line_[8] = rightA->LaneMarkPosition_C0_Rh_ME;
    line_[2] = rightB->LaneMarkModelDerivA_C3_Rh_ME;
    line_[6] = rightB->LaneMarkHeadingAngle_C1_Rh_ME;

// Track
/*
    memset(track_, 0, sizeof(track_));
    for(int i = 0; i<1; ++i) {
        printf("%lf\n", objectarr->objects[i].pose.position.x);

    } 
*/
    AES_Decision();

        
}

void objectCb(const DetectedObjectArrayConstPtr& objectarr)
{
// Track
    memset(track_, 0, sizeof(track_));
    if(objectarr->objects.size()) {
        printf("object size : %d\n", objectarr->objects.size());
        for(int i = 0; i<objectarr->objects.size(); ++i) {
//            printf("%lf\n", objectarr->objects[i].pose.position.x);
            double a = 0;
            double b = 0;
            //printf("object time : %d.%d\n", objectarr->header.stamp.sec, objectarr->header.stamp.nsec);
            track_[1+(9*i)] = objectarr->objects[i].pose.position.x; 
            track_[2+(9*i)] = objectarr->objects[i].pose.position.y; 
            track_[3+(9*i)] = objectarr->objects[i].velocity.linear.x; 
            track_[4+(9*i)] = objectarr->objects[i].velocity.linear.y; 
     
            track_[5+(9*i)] = 3; // yaw 
            quat2eul_aes(objectarr->objects[i].pose.orientation.w,
                         0, 0, objectarr->objects[i].pose.orientation.z,
                         &track_[5+(9*i)], &a, &b); // yaw
//            track_[4+(9*i)] = ; // yaw 
            track_[8+(9*i)] = objectarr->objects[i].dimensions.x; // length
            track_[9+(9*i)] = objectarr->objects[i].dimensions.y; // width
            track_[6+(9*i)] = track_[8+(9*i)] * 0.5;
            track_[7+(9*i)] = track_[6+(9*i)];
/*
            for(int j=0; j<9;++j) {
                printf("%lf\n", track_[j+(9*i)]);
            }
            printf("\n");
*/
        } 
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle nh;
    

    InitializeMagick(*argv);

    ros::Subscriber object_sub = nh.subscribe("/detection/lidar_objects_test", 1, objectCb);
    ros::Publisher AES_pub = nh.advertise<fallback_decision::AES_decision>("AES_Decision", 1);

    message_filters::Subscriber<LOG_BYTE0> byte0_sub(nh, "LOG_BYTE0", 1);
    message_filters::Subscriber<LOG_BYTE1> byte1_sub(nh, "LOG_BYTE1", 1);
    message_filters::Subscriber<ME_Left_Lane_A> leftA_sub(nh, "ME_Left_Lane_A", 1);
    message_filters::Subscriber<ME_Left_Lane_B> leftB_sub(nh, "ME_Left_Lane_B", 1);
    message_filters::Subscriber<ME_Right_Lane_A> rightA_sub(nh, "ME_Right_Lane_A", 1);
    message_filters::Subscriber<ME_Right_Lane_B> rightB_sub(nh, "ME_Right_Lane_B", 1);
    //message_filters::Subscriber<DetectedObjectArray> object_sub(nh, "/detection/lidar_objects_test", 1);

    //typedef sync_policies::ApproximateTime<LOG_BYTE0, LOG_BYTE1> SyncChassis;
    typedef sync_policies::ApproximateTime<LOG_BYTE0, LOG_BYTE1,
            ME_Left_Lane_A, ME_Left_Lane_B, ME_Right_Lane_A, ME_Right_Lane_B> SyncAES;
            //DetectedObjectArray> SyncAES;
    Synchronizer<SyncAES> sync(SyncAES(10), 
                          byte0_sub, byte1_sub,
                          leftA_sub, leftB_sub, rightA_sub, rightB_sub);
                          //object_sub);
    //sync.registerCallback(boost::bind(&chassisCb, _1, _2));
    sync.registerCallback(boost::bind(&AESCb, _1, _2, _3, _4, _5, _6));

    while(1) {
    
        ros::spinOnce();
        if(pub_flag) {
//            printf("AES check, pub_flag\n");
            AES_Publish(outResult, AES_pub);
            pub_flag = 0;
        }
 
    }

    printf("\n");


    ros::spin();

    return 0;
}

