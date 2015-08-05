#include "double_arc.h"
int main(){
	Motor *motor = Motor::get_instance();
    //cv::VideoCapture capture1(1);
    //Transform trans1(1);
    Location location(motor);
    //location.add_camera(&capture1, &trans1);
    location.set_current_location(cv::Point2d(0, 0), cv::Point2d(0, 1));
    
    //cv::Point2d dest_loc = cv::Point2d(-126.75,1640.75);
    //cv::Point2d dest_dir = cv::Point2d(-0.707,0.707);
	cv::Point2d dest_loc = cv::Point2d(-600,2000);
    cv::Point2d dest_dir = cv::Point2d(0,1);
    
    generateArcPath(motor,location,dest_loc,dest_dir,0.6);

	Motor::destroy_instance();
	return 0;
}