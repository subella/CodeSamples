#include <target_pose_estimating/keypoint_detector.hpp>
#include <target_pose_estimating/reproject_keypoints.hpp>
#include <target_pose_estimating/pose_estimator.hpp>
#include <gtest/gtest.h>
 
TEST(KeypointDetector, DetectKeypoints) {
    sdrone::KeypointDetector keypoint_detector("./data/test_model.pt");
    cv::Mat input_img = cv::imread("./data/input_img.png");
    torch::jit::script::Module container = torch::jit::load("./data/keypoints.pt");
    torch::Tensor keypoints_answer = container.attr("keypoints").toTensor();
    torch::Tensor keypoints_cpp;
    keypoint_detector.DetectKeypoints(input_img, keypoints_cpp);
    bool allclose = torch::allclose(keypoints_cpp, keypoints_answer);
    EXPECT_TRUE(allclose);
}

TEST(ReprojectKeypoints, ReprojectKeypoints) {
    Eigen::Matrix3d camera_intrinsics;
    camera_intrinsics << 629.1040649414062, 0.0,              637.203369140625, 
                         0.0,               628.583251953125, 380.56463623046875, 
                         0.0,               0.0,              1.0;

    sdrone::ReprojectKeypoints reproject_keypoints(camera_intrinsics);
    cv::Mat depth_img = cv::imread("./data/test_depth_img.png", cv::IMREAD_UNCHANGED);

    Eigen::MatrixX2i px_py_mat(13, 2);
    px_py_mat << 665, 514, 
                 642, 523,
                 639, 501,
                 613, 509,
                 656, 576,
                 655, 561,
                 626, 576,
                 577, 597,
                 574, 616,
                 540, 618,
                 521, 583,
                 655, 587,
                 646, 591;
    Eigen::MatrixX3d keypoints_3D_answer(13, 3);
    keypoints_3D_answer << 34.59644147,   166.21488006,  783.,
                           5.8099016,     172.66725904,  762.,
                           2.31324367,    155.19447002,  810.,
                           -30.50889797,  162.02983957,  793.,
                           23.21552664,   241.58021579,  777.,
                           21.81070376,   221.31621394,  771.,
                           -13.05359485,  227.89999765,  733.,
                           -65.8395332,   236.89388766,  688.,
                           -69.01992376,  257.31531091,  687.,
                           -105.6853837,  258.36798596,  684.,
                           -125.23505834, 218.3500375,   678.,
                           23.59290134,   273.89704204,  834.,
                           11.45190609,   274.18255639,  819.;

    Eigen::MatrixX3d keypoints_3D(13, 3);
    reproject_keypoints.reprojectKeypoints(px_py_mat, depth_img, keypoints_3D);

    bool allclose;
    if ((keypoints_3D - keypoints_3D_answer).norm() > 0.01)
    {
        allclose = false;
    }
    else
    {
        allclose = true;
    }

    EXPECT_TRUE(allclose);
}

TEST(PoseEstimator, SolveTransformation) {
    sdrone::TeaserParams params;
    params.cbar2 = 1;
    params.noise_bound = 5;
    params.estimate_scaling = false;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_gnc_factor = 1.4;
    params.rotation_max_iterations = 100;
    params.rotation_cost_threshold = 1e-6;

    sdrone::PoseEstimator pose_estimator("./data/test_cad_frame.csv", params);
    Eigen::Matrix3Xd keypoints_3D (3, 13);
    keypoints_3D.row(0) << 33.2724982, 4.9671052, 1.25381468, -31.26401346, 21.82148281,
                           20.08924026, -14.09728151, -66.51182183, -69.04602762, -105.7634699,
                           -126.35688104, 21.54286407, 9.59531259;

    keypoints_3D.row(1) << 164.79990612, 168.29259231, 154.91091854, 159.63966852, 239.96312074,
                           219.90923897, 224.50828476, 232.98404034, 253.64538866, 254.1953732,
                           216.87030347, 268.9356993,  270.24513745;

    keypoints_3D.row(2) << 780.78155908, 747.98341003, 813.90712514, 785.87050527, 774.19316156,
                           771.4787762,  724.36339773, 680.92319297, 679.28279643, 676.00235736,
                           676.99352248, 822.48078072, 809.36966347;

    Eigen::Matrix3d R_answer;
    R_answer << 0.125562, -0.673051,  -0.72886,
                0.992011, 0.0942214, 0.0838889,
                0.0122127, -0.73357,  0.679504;

    Eigen::Vector3d t_answer;
    t_answer << -206.811, -457.354, 1065.48;

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    pose_estimator.solveTransformation(keypoints_3D, R, t);

    bool allclose;
    if ((R-R_answer).norm() > 0.01 || (t - t_answer).norm() > 0.01)
    {
        allclose = false;
    }
    else
    {
        allclose = true;
    }

    EXPECT_TRUE(allclose);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}