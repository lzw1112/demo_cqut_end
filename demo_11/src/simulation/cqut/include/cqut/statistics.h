#include "cqut_msg/msg/state.hpp"

#include <opencv2/opencv.hpp>

namespace cqut
{

bool ccw(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C);
bool intersect(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C, const cv::Point2f &D);
cv::Point2f to_point(const cqut_msg::msg::State &state);

class LapStaticstic
{
public:
    LapStaticstic() = default;
    LapStaticstic(const std::string mission);

    bool request_stop();
    bool is_mission_finnished();
    void update_state(const cqut_msg::msg::State &state);
    
private:
    std::string mission_;
    cqut_msg::msg::State last_state_;

    cv::Point2f start_A_;
    cv::Point2f start_B_;
    cv::Point2f end_A_;
    cv::Point2f end_B_;

    int lap_count_;
    double vel_avg_;

    std::string rosbag_name_;
    bool state_received_;

    double starting_time_;
    double res_go_time_;
    double current_time_;
    std::vector<double> lap_time_;

    std::string report_file_name_;
}; 

} // namespace cqut
