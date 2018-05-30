#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\features2d.hpp>

#include "vam.h"

using namespace cv;
using namespace std;

//#define ORB_FEATURES 256
#define MATCH_DISTANCE 20

static void
get_keypoints(void *pixels, Mat & frame, vector<KeyPoint> & keypoints, Mat & descriptors)
{
    static auto orb = ORB::create();

    Mat tmp = Mat(1200, 1200, CV_8UC4, pixels);
    cv::resize(tmp, frame, cv::Size(), 0.5, 0.5);

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

    auto essMat = findEssentialMat(points1, points2, 655.899779 * .5, Point2d(589.928903 * .5, 614.458172 * .5));

    Mat R1, R2, t;
    decomposeEssentialMat(essMat, R1, R2, t);
    show_rot_angles(R1, R2);

    Mat tmp1, tmp2;
    cvtColor(ref_frame, tmp1, CV_BGRA2RGB);
    cvtColor(frame, tmp2, CV_BGRA2RGB);

    vector<DMatch> disp_matches = vector<DMatch>();

    /* debug dump and visualzation */
    for (int i = 0; i < close_matches.size(); i += 1)
    {
        //printf("match %i ", i);
        //cout << points1[i] << " -> "
        //    << points2[i] << endl;

        disp_matches.push_back(close_matches[i]);
    }

    Mat img_matches;
    Mat tmp;
    drawMatches(tmp1, ref_keypoints, tmp2, keypoints, disp_matches, img_matches,
                Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


    cv::resize(img_matches, tmp, cv::Size(), 1.2, 1.2);
    // drawing the results
    namedWindow("matches", 1);
    imshow("matches", tmp);
    waitKey(25);
}
