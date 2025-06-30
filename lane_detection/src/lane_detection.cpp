#include <rclcpp/rclcpp.hpp>정
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LaneDetectionNode : public rclcpp::Node {
public:
    LaneDetectionNode() : Node("lane_detection_node") {
        RCLCPP_INFO(this->get_logger(), "Lane Detection Node has started.");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/raw_image", 10,
            std::bind(&LaneDetectionNode::imageCallback, this, std::placeholders::_1)
        );

        // 원근 변환을 위한 포인트 설정 (입력 이미지에 따라 조정 필요)
        src_points = {cv::Point2f(100, 480), cv::Point2f(540, 480), cv::Point2f(420, 300), cv::Point2f(220, 300)};
        dst_points = {cv::Point2f(150, 480), cv::Point2f(490, 480), cv::Point2f(490, 0), cv::Point2f(150, 0)};
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Image received with encoding: %s", msg->encoding.c_str());

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Converted frame is empty.");
            return;
        }

        // Bird's Eye View 변환 적용
        cv::Mat bev_image = applyBirdEyeView(frame);

        // BEV 이미지에서 차선 검출
        cv::Mat lane_image = detectLaneWithSlidingWindow(bev_image);

        // 원본 이미지에 차선 다시 매핑
        cv::Mat result = mapLaneBackToOriginal(frame, lane_image);

        // 결과 표시
        cv::imshow("Original Image", frame);
        cv::imshow("Lane Detection (Bird's Eye View)", bev_image);
        cv::imshow("Lane Detection on Original Image", result);
        cv::waitKey(1);
    }

    cv::Mat applyBirdEyeView(const cv::Mat& frame) {
        // 원근 변환 행렬 계산
        cv::Mat transform_matrix = cv::getPerspectiveTransform(src_points, dst_points);

        // Bird's Eye View 이미지 생성
        cv::Mat bev_image;
        cv::warpPerspective(frame, bev_image, transform_matrix, cv::Size(640, 720));  // 변환 후 크기 조
        return bev_image;
    }

    cv::Mat detectLaneWithSlidingWindow(const cv::Mat& frame) {
        cv::Mat gray, edges, result;

        // 그레이스케일 변환 및 Canny 에지 검출
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
        cv::Canny(gray, edges, 50, 150);

        // 슬라이딩 윈도우 파라미터 설정
        int window_height = edges.rows / 10;
        int margin = 50;
        int min_pix = 50;

        // 결과 이미지 초기화
        result = frame.clone();
        std::vector<cv::Point> left_lane_points, right_lane_points;

        int midpoint = edges.cols / 2;
        int leftx_base = midpoint / 2;
        int rightx_base = midpoint + midpoint / 2;

        // 슬라이딩 윈도우 탐색
        for (int window = 0; window < 10; window++) {
            int win_y_low = edges.rows - (window + 1) * window_height;
            int win_y_high = edges.rows - window * window_height;

            int win_xleft_low = std::max(leftx_base - margin, 0);
            int win_xleft_high = std::min(leftx_base + margin, edges.cols);
            if (win_xleft_high > win_xleft_low) {
                cv::Mat left_region = edges(cv::Range(win_y_low, win_y_high), cv::Range(win_xleft_low, win_xleft_high));
                if (cv::countNonZero(left_region) > min_pix) {
                    cv::Moments left_m = cv::moments(left_region, true);
                    int left_x = static_cast<int>(left_m.m10 / left_m.m00) + win_xleft_low;
                    left_lane_points.push_back(cv::Point(left_x, (win_y_low + win_y_high) / 2));
                    leftx_base = left_x;
                }
            }

            int win_xright_low = std::max(rightx_base - margin, 0);
            int win_xright_high = std::min(rightx_base + margin, edges.cols);
            if (win_xright_high > win_xright_low) {
                cv::Mat right_region = edges(cv::Range(win_y_low, win_y_high), cv::Range(win_xright_low, win_xright_high));
                if (cv::countNonZero(right_region) > min_pix) {
                    cv::Moments right_m = cv::moments(right_region, true);
                    int right_x = static_cast<int>(right_m.m10 / right_m.m00) + win_xright_low;
                    right_lane_points.push_back(cv::Point(right_x, (win_y_low + win_y_high) / 2));
                    rightx_base = right_x;
                }
            }
        }

        // 차선 그리기
        if (left_lane_points.size() > 1) {
            cv::polylines(result, left_lane_points, false, cv::Scalar(0, 255, 0), 8, cv::LINE_AA);
        }
        if (right_lane_points.size() > 1) {
            cv::polylines(result, right_lane_points, false, cv::Scalar(0, 255, 0), 8, cv::LINE_AA);
        }

        return result;
    }

    cv::Mat mapLaneBackToOriginal(const cv::Mat& original, const cv::Mat& lane_image) {
        // 원근 변환의 역변환 행렬 계산
        cv::Mat inverse_transform = cv::getPerspectiveTransform(dst_points, src_points);

        // 검출된 차선 이미지를 원본 이미지로 매핑
        cv::Mat warped_lane_image;
        cv::warpPerspective(lane_image, warped_lane_image, inverse_transform, original.size());
        
        // 원본 이미지 위에 차선 덮어쓰기
        cv::Mat result = original.clone();
        cv::addWeighted(result, 1, warped_lane_image, 0.3, 0, result);
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::vector<cv::Point2f> src_points, dst_points;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneDetectionNode>());
    rclcpp::shutdown();
    return 0;
}

