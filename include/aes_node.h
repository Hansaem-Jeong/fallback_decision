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

class AES{

public:
    AES(ros::NodeHandle& nh);
    ~AES();
    void init();
    void AES_Publish(float result);
    void AES_Decision();
    void AESCb(const LOG_BYTE0ConstPtr& byte0,
           const LOG_BYTE1ConstPtr& byte1,
           const ME_Left_Lane_AConstPtr& leftA,
           const ME_Left_Lane_BConstPtr& leftB,
           const ME_Right_Lane_AConstPtr& rightA,
           const ME_Right_Lane_BConstPtr& rightB);
    void objectCb(const DetectedObjectArrayConstPtr& objectarr);
private:
    ros::NodeHandle nh_;
    
    ros::Publisher AES_pub;
    ros::Subscriber object_sub;

    message_filters::Subscriber<LOG_BYTE0> byte0_sub;
    message_filters::Subscriber<LOG_BYTE1> byte1_sub;
    message_filters::Subscriber<ME_Left_Lane_A> leftA_sub;
    message_filters::Subscriber<ME_Left_Lane_B> leftB_sub;
    message_filters::Subscriber<ME_Right_Lane_A> rightA_sub;
    message_filters::Subscriber<ME_Right_Lane_B> rightB_sub;


};

 
