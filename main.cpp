#include <rapp/cloud/service_controller/service_controller.hpp>
#include <rapp/cloud/vision/object_detection/object_detection.hpp>
#include <rapp/cloud/vision/hazard_detection/hazard_detection.hpp>

#include <rapp-robots-api/vision/vision.hpp>
#include <rapp-robots-api/communication/communication.hpp>
#include <rapp-robots-api/navigation/navigation.hpp>

#include <opencv2/opencv.hpp>

//#define DEBUG

#ifdef DEBUG
#define SHOW_DBG_PIC(x) cv::imshow("Out", x); cv::waitKey(-1);
#else
#define SHOW_DBG_PIC(x) ;
#endif

namespace rr = rapp::robot;

void status_cb(int status) {
    if (status != 0)
        std::cout << "Error!\n";
}

void callback(std::vector<std::string> names, std::vector<rapp::object::point> centers, std::vector<float> scores, int result) {
    for (size_t i = 0; i < names.size(); ++i) {
        std::cout << names[i] << "(" << centers[i].x << "," << centers[i].y << ")\n";
    }
}

int main(int argc, char * argv[]) {
    rr::communication com(argc, argv);
    rr::vision vis(argc, argv);
    rr::navigation nav(argc, argv);
    
    // look straight ahead
    nav.take_predefined_posture("Stand", 0.3);
    nav.move_joint({"HeadYaw","HeadPitch"}, {0.0f, 0.0f}, 0.5);

    // load camera info for default/main/front-facing camera
    rr::vision::camera_info cam = vis.load_camera_info(0);

    com.text_to_speech("Give me a second...");
    
    vis.set_camera_param(0, 11, 1);
    vis.set_camera_param(0, 12, 1);
    
    // take picture of scene with object
    auto picture = vis.capture_image(0, rapp::robot::vision::vga4, "png");

//  rapp::cloud::platform_info info = {"155.207.19.229", "9001", "rapp_token"}; 
    rapp::cloud::platform_info info = {"192.168.18.186", "9001", "rapp_token"}; 
    rapp::cloud::service_controller ctrl(info);

    ctrl.make_call<rapp::cloud::object_detection_clear_models>(status_cb);

    float cx = cam.K[2];
    float cy = cam.K[5];
    float fx = cam.K[0];
    float fy = cam.K[4];

    // try to recognize learned object
    std::vector<std::string> models = {"mint", "cranberry"};
    ctrl.make_call<rapp::cloud::object_detection_load_models>(models, status_cb);
    ctrl.make_call<rapp::cloud::object_detection_find_objects>(picture, 10, [&](std::vector<std::string> names, std::vector<rapp::object::point> centers, std::vector<float> scores, int result) {
        for (size_t i = 0; i < names.size(); ++i) {
            com.text_to_speech(names[i]);
	    std::cout << names[i] << "(" << centers[i].x << "," << centers[i].y << ")\n";
        
            float u = centers[i].x;
            float v = centers[i].y;

            float a1 = -atan2(u-cx, fx);
            float a2 = atan2(v-cy, fy);
            
	    float cam_z = 0.5;
            float cam_x = (u-cx)/fx*cam_z;
	    float cam_y = (v-cy)/fy*cam_z;

            std::cout << "In cam: " << cam_x << "," << cam_y << "," << cam_z << "\n";

            nav.move_joint({"HeadYaw", "HeadPitch"}, {a1, a2}, 0.1);
        }
    });

    // crouch and turn off all the motors
    nav.rest("Crouch");

    return 0;
}
