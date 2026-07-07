#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// 계약(docs/interfaces.md) 형식으로 인지 결과를 추출한다.
//
// 주의: "일단 픽셀 유지" — 필드/구조는 racer_msgs 계약을 미러링하되,
//       값 단위는 아직 픽셀이다(미터/ base_link 변환은 후속 작업).
//       heading_error 만 각도라서 rad 로 계산된다(픽셀 x/y 스케일 동일 가정).
// ---------------------------------------------------------------------------

// racer_msgs/LaneStatus 미러 (interfaces.md §4.3)
struct LaneStatus
{
    bool  lane_detected  = false;  // 유효 차선 검출 여부
    float confidence     = 0.0f;   // 0.0~1.0 검출 신뢰도
    int   num_points     = 0;      // lane_path 점 개수(0이면 미검출)
    float lateral_offset = 0.0f;   // 차량중심 대비 차선중심 횡오차, +좌측 [지금은 px]
    float heading_error  = 0.0f;   // 차량 전방 대비 차선 접선 오차 [rad]
    bool  stop_line      = false;  // 정지선 검출 여부
    float stop_line_dist = -1.0f;  // 정지선까지 거리, 미검출 -1.0 [지금은 px]
};

// /perception/lane_path 미러 (interfaces.md §4.2, nav_msgs/Path):
//   차선 중심선 점열, "가까운→먼" 순서.
//   계약상 base_link 미터 좌표(+x 전방, +y 좌측)여야 하지만,
//   지금은 이미지 픽셀 좌표(x=열, y=행)를 담는다. 미터 변환은 후속.
struct PerceptionResult
{
    LaneStatus               status;      // = racer_msgs/LaneStatus
    std::vector<cv::Point2f> lane_path;   // = nav_msgs/Path (near→far), 지금은 px

    // --- 계약 외 부가(미션 M4 체커보드/디버그) ---
    bool    checkerboard_detected = false;
    cv::Mat debug_image;
};

class LanePerception
{
public:
    LanePerception()
        : M_computed_(false),
          last_leftx_(80),
          last_rightx_(240)
    {
    }

    PerceptionResult process(const cv::Mat& frame)
    {
        PerceptionResult result;

        if (frame.empty()) {
            return result;
        }

        const int height = frame.rows;
        const int width = frame.cols;
        const int midpoint = width / 2;

        if (!M_computed_) {
            std::vector<cv::Point2f> src = {
                {float(width) * 0.2f, float(height) * 0.4f},
                {float(width) * 0.8f, float(height) * 0.4f},
                {float(width), float(height)},
                {0.0f, float(height)}
            };

            std::vector<cv::Point2f> dst = {
                {0.0f, 0.0f},
                {float(width), 0.0f},
                {float(width), float(height)},
                {0.0f, float(height)}
            };

            M_ = cv::getPerspectiveTransform(src, dst);
            M_computed_ = true;
        }

        cv::Mat bev;
        cv::warpPerspective(frame, bev, M_, cv::Size(width, height));

        cv::Mat hls;
        cv::cvtColor(bev, hls, cv::COLOR_BGR2HLS);

        cv::Mat yellow_mask;
        cv::Mat white_mask;
        cv::Mat lane_mask;

        cv::inRange(hls, cv::Scalar(15, 80, 70), cv::Scalar(35, 255, 255), yellow_mask);
        cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 70), white_mask);

        if (cv::countNonZero(yellow_mask) > yellow_lane_pixel_threshold_) {
            lane_mask = yellow_mask;
        } else {
            cv::bitwise_or(yellow_mask, white_mask, lane_mask);
        }

        cv::Mat masked;
        cv::Mat gray;
        cv::Mat blur;
        cv::Mat edges;

        cv::bitwise_and(bev, bev, masked, lane_mask);
        cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
        cv::Canny(blur, edges, 50, 150);

        cv::Mat debug_image = bev.clone();

        // --- 차선: 중심선 점열(near→far) + 신뢰도 산출 ---
        std::vector<cv::Point2f> centerline;
        float confidence = 0.0f;
        bool lane_ok = detect_lanes(edges, debug_image, centerline, confidence);

        result.status.lane_detected = lane_ok;
        result.lane_path = centerline;
        result.status.num_points = static_cast<int>(centerline.size());
        result.status.confidence = confidence;

        // lateral_offset(+좌측), heading_error(rad) — 중심선에서 계산
        if (!centerline.empty()) {
            const cv::Point2f& p_near = centerline.front();  // 창0, 바닥(가까움)
            const cv::Point2f& p_far  = centerline.back();   // 마지막 창, 위(멂)

            // 차량중심(midpoint) 기준, 차선중심이 왼쪽이면 +.
            result.status.lateral_offset = midpoint - p_near.x;

            // 전방(위=y감소) 대비 중심선 접선 기울기. 왼쪽으로 휘면 +(CCW).
            float dx = p_near.x - p_far.x;
            float dy = p_near.y - p_far.y;  // near 가 아래라 dy>0
            if (dy > 1e-3f) {
                result.status.heading_error = std::atan2(dx, dy);
            }
        }

        // --- 정지선: 검출 여부 + 거리(px) ---
        float stop_dist = -1.0f;
        result.status.stop_line = detect_stopline(bev, debug_image, stop_dist);
        result.status.stop_line_dist = stop_dist;

        // --- 체커보드(계약 외, 미션 M4) ---
        result.checkerboard_detected = detect_checkerboard(bev, debug_image);

        // --- 디버그 오버레이 ---
        cv::line(debug_image,
                 cv::Point(midpoint, 0),
                 cv::Point(midpoint, height),
                 cv::Scalar(255, 255, 0),
                 1);

        if (!centerline.empty()) {
            cv::circle(debug_image,
                       cv::Point(static_cast<int>(centerline.front().x), height - 40),
                       5,
                       cv::Scalar(0, 255, 0),
                       -1);
        }

        result.debug_image = debug_image;
        return result;
    }

private:
    // 중심선 점열(centerline, near→far)과 confidence 를 채운다.
    bool detect_lanes(const cv::Mat& edges,
                      cv::Mat& debug_image,
                      std::vector<cv::Point2f>& centerline,
                      float& confidence)
    {
        const int height = edges.rows;
        const int width = edges.cols;
        const int midpoint = width / 2;

        cv::Mat roi_hist = edges.rowRange(static_cast<int>(height * 0.6f), height);
        cv::Mat hist_mat;
        cv::reduce(roi_hist, hist_mat, 0, cv::REDUCE_SUM, CV_32S);

        double minVal;
        double maxVal;
        cv::Point minLoc;
        cv::Point maxLocLeft;
        cv::Point maxLocRight;

        cv::minMaxLoc(hist_mat.colRange(0, midpoint), &minVal, &maxVal, &minLoc, &maxLocLeft);
        int current_leftx = maxLocLeft.x;

        cv::minMaxLoc(hist_mat.colRange(midpoint, width), &minVal, &maxVal, &minLoc, &maxLocRight);
        int current_rightx = maxLocRight.x + midpoint;

        int leftx = static_cast<int>(last_leftx_ * 0.8f + current_leftx * 0.2f);
        int rightx = static_cast<int>(last_rightx_ * 0.8f + current_rightx * 0.2f);

        const int nwindows = 9;
        const int margin = 20;
        const int minpix = 5;
        const int window_height = height / nwindows;

        std::vector<cv::Point> nonzero_points;
        cv::findNonZero(edges, nonzero_points);

        if (nonzero_points.empty()) {
            return false;
        }

        std::vector<int> nonzerox;
        std::vector<int> nonzeroy;

        nonzerox.reserve(nonzero_points.size());
        nonzeroy.reserve(nonzero_points.size());

        for (const auto& point : nonzero_points) {
            nonzerox.push_back(point.x);
            nonzeroy.push_back(point.y);
        }

        int valid_windows = 0;

        // 창 0 = 바닥(가까움) → 창 nwindows-1 = 위(멂). 순서대로 push → near→far.
        for (int window = 0; window < nwindows; ++window) {
            int y_low = height - (window + 1) * window_height;
            int y_high = height - window * window_height;

            int lx_low = leftx - margin;
            int lx_high = leftx + margin;
            int rx_low = rightx - margin;
            int rx_high = rightx + margin;

            cv::rectangle(debug_image,
                          cv::Point(lx_low, y_low),
                          cv::Point(lx_high, y_high),
                          cv::Scalar(255, 0, 0),
                          2);

            cv::rectangle(debug_image,
                          cv::Point(rx_low, y_low),
                          cv::Point(rx_high, y_high),
                          cv::Scalar(0, 0, 255),
                          2);

            std::vector<int> good_left_inds;
            std::vector<int> good_right_inds;

            for (size_t i = 0; i < nonzeroy.size(); ++i) {
                int y = nonzeroy[i];
                int x = nonzerox[i];

                if (y >= y_low && y < y_high) {
                    if (x >= lx_low && x < lx_high) {
                        good_left_inds.push_back(i);
                    }

                    if (x >= rx_low && x < rx_high) {
                        good_right_inds.push_back(i);
                    }
                }
            }

            bool updated = false;

            if (good_left_inds.size() > minpix) {
                long long sumx = 0;
                for (int index : good_left_inds) {
                    sumx += nonzerox[index];
                }
                leftx = static_cast<int>(sumx / good_left_inds.size());
                updated = true;
            }

            if (good_right_inds.size() > minpix) {
                long long sumx = 0;
                for (int index : good_right_inds) {
                    sumx += nonzerox[index];
                }
                rightx = static_cast<int>(sumx / good_right_inds.size());
                updated = true;
            }

            if (updated) {
                ++valid_windows;
            }

            float cx = (leftx + rightx) / 2.0f;
            float cy = (y_low + y_high) / 2.0f;
            centerline.emplace_back(cx, cy);
        }

        last_leftx_ = leftx;
        last_rightx_ = rightx;

        confidence = static_cast<float>(valid_windows) / nwindows;

        return true;
    }

    // 정지선 검출 여부 + 차량(바닥)으로부터의 거리[px] (미검출 -1.0)
    bool detect_stopline(const cv::Mat& bev, cv::Mat& debug_image, float& stop_line_dist)
    {
        stop_line_dist = -1.0f;

        cv::Rect roi_rect(0, bev.rows - 80, bev.cols, 80);
        cv::Mat roi = bev(roi_rect);

        cv::Mat hls;
        cv::Mat white_mask;
        cv::Mat edges;

        cv::cvtColor(roi, hls, cv::COLOR_BGR2HLS);
        cv::inRange(hls, cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 255), white_mask);
        cv::Canny(white_mask, edges, 100, 200);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 40, 40, 5);

        double total_horizontal_length = 0.0;

        for (const auto& line : lines) {
            double angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI;

            if (std::abs(angle) < 10 || std::abs(angle) > 170) {
                double length = std::hypot(line[2] - line[0], line[3] - line[1]);
                total_horizontal_length += length;

                // 바닥으로부터의 거리[px] = 이미지 높이 - 선의 절대 y. 가장 가까운 값 유지.
                float y_abs = (line[1] + line[3]) * 0.5f + roi_rect.y;
                float dist = bev.rows - y_abs;
                if (dist >= 0.0f && (stop_line_dist < 0.0f || dist < stop_line_dist)) {
                    stop_line_dist = dist;
                }

                cv::line(debug_image,
                         cv::Point(line[0], line[1] + roi_rect.y),
                         cv::Point(line[2], line[3] + roi_rect.y),
                         cv::Scalar(0, 255, 255),
                         2);
            }
        }

        bool detected = total_horizontal_length > 150.0;
        if (!detected) {
            stop_line_dist = -1.0f;
        }
        return detected;
    }

    bool detect_checkerboard(const cv::Mat& bev, cv::Mat& debug_image)
    {
        cv::Rect roi_rect(0, bev.rows * 0.6, bev.cols, bev.rows * 0.4);
        cv::Mat roi = bev(roi_rect);

        cv::Mat hls;
        cv::Mat white_mask;

        cv::cvtColor(roi, hls, cv::COLOR_BGR2HLS);
        cv::inRange(hls, cv::Scalar(0, 160, 0), cv::Scalar(180, 255, 255), white_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int square_like_contours = 0;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);

            const double min_check_area = 5.0;
            const double max_check_area = 200.0;

            if (area < min_check_area || area > max_check_area) {
                continue;
            }

            cv::Rect bbox = cv::boundingRect(contour);
            float aspect_ratio = static_cast<float>(bbox.width) / bbox.height;

            const float min_aspect_ratio = 0.3f;
            const float max_aspect_ratio = 3.0f;

            if (aspect_ratio > min_aspect_ratio && aspect_ratio < max_aspect_ratio) {
                square_like_contours++;

                cv::Rect full_bbox(
                    bbox.x + roi_rect.x,
                    bbox.y + roi_rect.y,
                    bbox.width,
                    bbox.height
                );

                cv::rectangle(debug_image, full_bbox, cv::Scalar(0, 255, 0), 1);
            }
        }

        return square_like_contours > 8;
    }

private:
    cv::Mat M_;
    bool M_computed_;

    float last_leftx_;
    float last_rightx_;

    const int yellow_lane_pixel_threshold_ = 30;
};

int main()
{
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "카메라 열기 실패" << std::endl;
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    LanePerception perception;

    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            std::cerr << "프레임 수신 실패" << std::endl;
            continue;
        }

        PerceptionResult result = perception.process(frame);
        const LaneStatus& s = result.status;

        // 계약(LaneStatus + lane_path) 형식 그대로 출력
        std::cout
            << "lane_detected=" << s.lane_detected
            << " confidence=" << s.confidence
            << " num_points=" << s.num_points
            << " lateral_offset=" << s.lateral_offset
            << " heading_error=" << s.heading_error
            << " stop_line=" << s.stop_line
            << " stop_line_dist=" << s.stop_line_dist
            << " checkerboard=" << result.checkerboard_detected
            << std::endl;

        cv::imshow("Lane Perception", result.debug_image);

        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    return 0;
}
