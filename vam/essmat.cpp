#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\features2d.hpp>

#include "vam.h"

using namespace cv;
using namespace std;

//#define ORB_FEATURES 256
#define MATCH_DISTANCE 20

const int IMAGE_DOWNSAMPLE = 2;
const double FOCAL_LENGTH = 655.899779 / IMAGE_DOWNSAMPLE;
const double CX = 589.928903 / IMAGE_DOWNSAMPLE;
const double CY = 614.458172 / IMAGE_DOWNSAMPLE;

static void
get_keypoints(void *pixels, Mat & frame, vector<KeyPoint> & keypoints, Mat & descriptors)
{
    static auto orb = ORB::create();

    Mat tmp = Mat(1200, 1200, CV_8UC4, pixels);
    resize(tmp, frame, cv::Size(), 0.5, 0.5);


    orb->detectAndCompute(_InputArray(frame), cv::noArray(), keypoints, descriptors);
}

void
filter_matches(vector<DMatch> & all_matches, vector<DMatch> & close_matches,
               vector<KeyPoint> & keypoints1, vector<KeyPoint> & keypoints2,
               vector<Point2f> & points1, vector<Point2f> & points2)
{
    for (auto match = all_matches.begin(); match != all_matches.end(); ++match)
    {
        if (match->distance > MATCH_DISTANCE)
        {
            /* distance to large, ignore */
            continue;
        }

        points1.push_back(keypoints1[match->queryIdx].pt);
        points2.push_back(keypoints2[match->trainIdx].pt);
        close_matches.push_back(*match);
    }
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

#define R_TO_D 57.2957795

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Vec3f(x * R_TO_D, y * R_TO_D, z * R_TO_D);



}

void
show_rot_angles(Mat & R1, Mat & R2)
{
    static Vec3f last_rot = Vec3f(0, 0, 0);

    auto rot_angls1 = rotationMatrixToEulerAngles(R1);
    auto rot_angls2 = rotationMatrixToEulerAngles(R2);

    auto diff1 = norm(last_rot - rot_angls1);
    auto diff2 = norm(last_rot - rot_angls2);

    if (diff1 < diff2)
    {
        last_rot = rot_angls1;
    }
    else
    {
        last_rot = rot_angls2;
    }

    cout << last_rot << endl;
//    cout << rot_angls1 << diff1 << endl;
//    cout << rot_angls2 << diff2 << endl;
}

static void 
open_debug_console()
{
    AllocConsole();
    FILE* fp;

    freopen_s(&fp, "CONOUT$", "w", stdout);
    printf("Debugging Window:\n");
}


static bool haz_reference_frame = false;
static vector<KeyPoint> ref_keypoints;
static Mat ref_frame, ref_descriptors;

extern "C" __declspec(dllexport) vam_handle *
vam_init(int width, int height)
{
    open_debug_console();
    haz_reference_frame = false;
    return NULL;
}

void
dump_homo_coord(Mat & coord)
{
    cout << coord.rowRange(0, 3) / coord.at<float>(3) << endl;
}

void
triangulate(Mat & E, Mat & mask, vector<Point2f> & points1, vector<Point2f> & points2)
{
    Mat K = Mat::eye(3, 3, CV_64F);

    K.at<double>(0, 0) = FOCAL_LENGTH;
    K.at<double>(1, 1) = FOCAL_LENGTH;
    K.at<double>(0, 2) = CX;
    K.at<double>(1, 2) = CY;

    Mat local_R, local_t;

    recoverPose(E, points1, points2, local_R, local_t, FOCAL_LENGTH, Point2d(CX, CY), mask);

    Mat T = Mat::eye(4, 4, CV_64F);
    local_R.copyTo(T(Range(0, 3), Range(0, 3)));
    local_t.copyTo(T(Range(0, 3), Range(3, 4)));

    // make projection matrix
    Mat R = T(Range(0, 3), Range(0, 3));
    Mat t = T(Range(0, 3), Range(3, 4));

    Mat P(3, 4, CV_64F);

    P(Range(0, 3), Range(0, 3)) = R.t();
    P(Range(0, 3), Range(3, 4)) = -R.t()*t;
    P = K*P;
    
    Mat points4D;
//    Mat ref_P = K * Mat::eye(4, 4, CV_64F);
    triangulatePoints(Mat::eye(3, 4, CV_64FC1), P, points1, points2, points4D);

    dump_homo_coord(points4D.col(0));
    dump_homo_coord(points4D.col(1));
    dump_homo_coord(points4D.col(2));
}

extern "C" __declspec(dllexport) void
vam_process_frame(vam_handle *vah, void *pixels)
{
    Mat frame;

    if (!haz_reference_frame)
    {
        get_keypoints(pixels, ref_frame, ref_keypoints, ref_descriptors);
        haz_reference_frame = true;
        return;
    }

    static auto matcher = BFMatcher(NORM_HAMMING, true);
    vector<KeyPoint> keypoints;
    Mat descriptors;

    get_keypoints(pixels, frame, keypoints, descriptors);

    vector<DMatch> matches = vector<DMatch>();
    matcher.match(ref_descriptors, descriptors, matches);

    vector<DMatch> close_matches = vector<DMatch>();
    vector<Point2f> points1;
    vector<Point2f> points2;
    filter_matches(matches, close_matches,
                   ref_keypoints, keypoints,
                   points1, points2);

    if (close_matches.size() < 8)
    {
        printf("lost track\n");
        return;
    }

    Mat mask;
    auto essMat = findEssentialMat(points1, points2, FOCAL_LENGTH, Point2d(CX, CY),
                                   RANSAC, 0.999, 1.0, mask);
    triangulate(essMat, mask, points1, points2);

    Mat R1, R2, t;
    decomposeEssentialMat(essMat, R1, R2, t);
    show_rot_angles(R1, R2);

    Mat tmp1, tmp2;
    cvtColor(ref_frame, tmp1, CV_BGRA2RGB);
    cvtColor(frame, tmp2, CV_BGRA2RGB);

    vector<DMatch> disp_matches = vector<DMatch>();

    /* debug dump and visualzation */
    for (int i = 0; i < close_matches.size() && i < 3; i += 1)
    {
        //printf("match %i ", i);
        //cout << points1[i] << " -> "
        //    << points2[i] << endl;

        disp_matches.push_back(close_matches[i]);
        cout << ref_keypoints[close_matches[i].queryIdx].pt << endl;
    }

    Mat img_matches;
    Mat tmp;
    drawMatches(tmp1, ref_keypoints, tmp2, keypoints, disp_matches, img_matches,
                Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


    cv::resize(img_matches, tmp, cv::Size(), 1.2, 1.2);
    // drawing the results
    namedWindow("matches", 1);
    imshow("matches", tmp);
    waitKey(0);
}


//void 
//process_two_frames(void *pixels1, void *pixels2)
//{
//    static auto matcher = BFMatcher(NORM_HAMMING, true);
//    vector<KeyPoint> keypoints1, keypoints2;
//    Mat frame1, descriptors1, frame2, descriptors2;
//
//    vector<DMatch> matches = vector<DMatch>();
//
//    get_keypoints(pixels1, frame1, keypoints1, descriptors1);
//    get_keypoints(pixels2, frame2, keypoints2, descriptors2);
//    matcher.match(descriptors1, descriptors2, matches);
//
//    vector<DMatch> close_matches = vector<DMatch>();
//    vector<Point2f> points1;
//    vector<Point2f> points2;
//    filter_matches(matches, close_matches, 
//                   keypoints1, keypoints2,
//                   points1, points2);
//
//    Mat tmp1, tmp2;
//    cvtColor(frame1, tmp1, CV_BGRA2RGB);
//    cvtColor(frame2, tmp2, CV_BGRA2RGB);
//
////    printf("matches %zi\n", close_matches.size());
//    if (close_matches.size() < 8)
//    {
//        printf("lost track\n");
//        return;
//    }
//
//
//    Mat mask;
//    auto essMat = findEssentialMat(points1, points2, 655.899779 * .5, Point2d(589.928903 * .5, 614.458172 * .5),
//                                   RANSAC, 0.999, 1.0, mask);
//
//    triangulate(essMat, mask, points1, points2);
//    //cout << essMat << endl;
//
//    Mat R1, R2, t;
//    decomposeEssentialMat(essMat, R1, R2, t);
//    //cout << "R1" << endl << R1 << endl;
//    //cout << "R2" << endl << R2 << endl;
//    //cout << "t" << endl << t << endl;
//
//    show_rot_angles(R1, R2);
//
//    vector<DMatch> disp_matches;
//
//    /* debug dump and visualzation */
//    for (int i = 0; i < close_matches.size(); i += 1)
//    {
//        //printf("match %i ", i);
//        //cout << points1[i] << " -> "
//        //    << points2[i] << endl;
//
//        disp_matches.push_back(close_matches[i]);
//    }
//
//    Mat img_matches;
//    Mat tmp;
//    drawMatches(tmp1, keypoints1, tmp2, keypoints2, disp_matches, img_matches,
//        Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//
//
//    cv::resize(img_matches, tmp, cv::Size(), 1.2, 1.2);
//    // drawing the results
//    namedWindow("matches", 1);
//    imshow("matches", tmp);
//    waitKey(25);
//
//    ///* debug dump and visualzation */
//    //for (int i = 0; i < close_matches.size(); i += 1)
//    //{
//    //    
//    //    printf("match %i ", i);
//    //    cout << points1[i] << " -> " 
//    //         << points2[i] << endl;
//
//    //    vector<DMatch> disp_matches = vector<DMatch>();
//    //    disp_matches.push_back(close_matches[i]);
//
//    //    Mat img_matches;
//    //    Mat tmp;
//    //    drawMatches(tmp1, keypoints1, tmp2, keypoints2, disp_matches, img_matches,
//    //                Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//
//
//    //    cv::resize(img_matches, tmp, cv::Size(), 1.2, 1.2);
//    //    imshow("matches", tmp);
//    //    waitKey(0);
//    //    break;
//    //}
//}
