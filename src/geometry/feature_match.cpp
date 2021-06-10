
#include "my_slam/geometry/feature_match.h"
#include "my_slam/basics/opencv_funcs.h"
#include "my_slam/basics/config.h"

namespace my_slam
{
namespace geometry
{

void calcKeyPoints(
    const cv::Mat &image,
    vector<cv::KeyPoint> &keypoints)
{
    // -- Set arguments
    static const int num_keypoints = basics::Config::get<int>("number_of_keypoints_to_extract");
    static const double scale_factor = basics::Config::get<double>("scale_factor");
    static const int level_pyramid = basics::Config::get<int>("level_pyramid");
    static const int score_threshold = basics::Config::get<int>("score_threshold");

    // -- Create ORB
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_keypoints, scale_factor, level_pyramid,
                                                  31, 0, 2, cv::ORB::HARRIS_SCORE, 31, score_threshold);
    // Default arguments of ORB:
    //           int        nlevels = 8,
    //           int        edgeThreshold = 31,
    //           int        firstLevel = 0,
    //           int        WTA_K = 2,
    //           ORB::ScoreType  scoreType = ORB::HARRIS_SCORE,
    //           int        patchSize = 31,
    //           int        fastThreshold = 20

    // compute
    orb->detect(image, keypoints);
    selectUniformKptsByGrid(keypoints, image.rows, image.cols);
}

void calcDescriptors(
    const cv::Mat &image,
    vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    static const int num_keypoints = basics::Config::get<int>("number_of_keypoints_to_extract");
    static const double scale_factor = basics::Config::get<double>("scale_factor");
    static const int level_pyramid = basics::Config::get<int>("level_pyramid");
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_keypoints, scale_factor, level_pyramid);

    // compute
    orb->compute(image, keypoints, descriptors);
}

void selectUniformKptsByGrid(
    vector<cv::KeyPoint> &keypoints,
    int image_rows, int image_cols)
{
    // -- Set argumetns
    static const int max_num_keypoints = basics::Config::get<int>("max_number_of_keypoints");
    static const int kpts_uniform_selection_grid_size = basics::Config::get<int>("kpts_uniform_selection_grid_size");
    static const int kpts_uniform_selection_max_pts_per_grid = basics::Config::get<int>("kpts_uniform_selection_max_pts_per_grid");
    static const int rows = image_rows / kpts_uniform_selection_grid_size, cols = image_cols / kpts_uniform_selection_grid_size;

    // Create an empty grid
    static vector<vector<int>> grid(rows, vector<int>(cols, 0));
    for (auto &row : grid) // clear grid
        std::fill(row.begin(), row.end(), 0);

    // Insert keypoints to grid. If not full, insert this cv::KeyPoint to result
    vector<cv::KeyPoint> tmp_keypoints;
    int cnt = 0;
    for (auto &kpt : keypoints)
    {
        int row = ((int)kpt.pt.y) / kpts_uniform_selection_grid_size, col = ((int)kpt.pt.x) / kpts_uniform_selection_grid_size;
        if (grid[row][col] < kpts_uniform_selection_max_pts_per_grid)
        {
            tmp_keypoints.push_back(kpt);
            grid[row][col]++;
            cnt++;
            if (cnt > max_num_keypoints)
                break;
        }
    }

    // return
    keypoints = tmp_keypoints;
}

vector<cv::DMatch> matchByRadiusAndBruteForce(
    const vector<cv::KeyPoint> &keypoints_1,
    const vector<cv::KeyPoint> &keypoints_2,
    const cv::Mat1b &descriptors_1,
    const cv::Mat1b &descriptors_2,
    float max_matching_pixel_dist)
{
    int N1 = keypoints_1.size(), N2 = keypoints_2.size();
    assert(N1 == descriptors_1.rows && N2 == descriptors_2.rows);
    vector<cv::DMatch> matches;
    float r2 = max_matching_pixel_dist * max_matching_pixel_dist;
    for (int i = 0; i < N1; i++)
    {
        const cv::KeyPoint &kpt1 = keypoints_1[i];
        bool is_matched = false;
        float x = kpt1.pt.x, y = kpt1.pt.y;
        double min_feature_dist = 99999999.0, target_idx = 0;
        for (int j = 0; j < N2; j++)
        {
            if ((x - x2) * (x - x2) + (y - y2) * (y - y2) <= r2)
            {
        
                //double feature_dist = cv::norm(descriptors_1.row(i), descriptors_2.row(j));
                cv::Mat diff;
                cv::absdiff(descriptors_1.row(i), descriptors_2.row(j), diff);
                double feature_dist = cv::sum(diff)[0] / descriptors_1.cols;
                if (feature_dist < min_feature_dist)
                {
                    min_feature_dist = feature_dist;
                    target_idx = j;
                    is_matched = true;
                }
            }
        }
        if (is_matched)
        matches.push_back(cv::DMatch(i, target_idx, static_cast<float>(min_feature_dist)));
        
    }
    return matches;
}

void matchFeatures(
    const cv::Mat1b &descriptors_1, const cv::Mat1b &descriptors_2,
    vector<cv::DMatch> &matches,
    int method_index,
    bool is_print_res,
    // Below are optional arguments for feature_matching_method_index==3
    const vector<cv::KeyPoint> &keypoints_1,
    const vector<cv::KeyPoint> &keypoints_2,
    float max_matching_pixel_dist)
{
    // -- Set arguments
    static const double xiang_gao_method_match_ratio = basics::Config::get<int>("xiang_gao_method_match_ratio");
    static const double lowe_method_dist_ratio = basics::Config::get<int>("lowe_method_dist_ratio");
    static const double method_3_feature_dist_threshold = basics::Config::get<int>("method_3_feature_dist_threshold");
    static cv::FlannBasedMathcer matcher_flann(new cv::flann::LshIndexParams(5, 10, 2));
    static cv::Ptr<cv::DescriptorMatcher> matcher_bf = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // -- Debug: see descriptors_1's content:
    //    Result: It's 8UC1, the value ranges from 0 to 255. It's not binary!
    // basics::print_MatProperty(descriptors_1);
    // for (int i = 0; i < 32; i++)
    //     std::cout << int(descriptors_1.at<unsigned char>(0, i)) << std::endl;

    // Start matching
    matches.clear();
    double min_dis = 9999999, max_dis = 0, distance_threshold = -1;
    
}

}
}