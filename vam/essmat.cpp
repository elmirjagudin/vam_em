#include "stdafx.h"

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\features2d.hpp>

#include "vam.h"

#define DOWNSAMPLE 0.25

using namespace cv;
using namespace std;

//#define ORB_FEATURES 256
#define MATCH_DISTANCE 20

static void
get_keypoints(void *pixels, Mat & frame, vector<KeyPoint> & keypoints, Mat & descriptors)
{
    static auto orb = ORB::create();

    Mat tmp = Mat(1200, 1200, CV_8UC4, pixels);
    cv::resize(tmp, frame, cv::Size(), DOWNSAMPLE, DOWNSAMPLE);

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
Vec3d rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));

    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    double x, y, z;
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
    return Vec3d(x * R_TO_D, y * R_TO_D, z * R_TO_D);
}

static void 
open_debug_console()
{
    AllocConsole();
    FILE* fp;

    freopen_s(&fp, "CONOUT$", "w", stdout);
    printf("Debugging Window:\n");
}


class KeyFrame
{
    enum
    {
        LEFT_NEIGHBOUR = 0,
        RIGHT_NEIGHBOUR = 1,
    };

    vector<KeyFrame *> neighbour_frames = vector<KeyFrame *>(2);
public:
    vector<KeyPoint> keypoints;
    Mat descriptors;
    Mat frame;
    Mat R;

    KeyFrame(Mat & frame,
             vector<KeyPoint> & keypoints,
             Mat & descriptors,
             KeyFrame *left_neighbour = nullptr,
             KeyFrame *right_neighbour = nullptr,
             Mat R = Mat::eye(3, 3, CV_64F))
    {
        this->keypoints = keypoints;
        descriptors.copyTo(this->descriptors);
        frame.copyTo(this->frame);

        neighbour_frames[LEFT_NEIGHBOUR] = left_neighbour;
        neighbour_frames[RIGHT_NEIGHBOUR] = right_neighbour;

        R.copyTo(this->R);
    }

    

    KeyFrame *next_frame(Mat & R, 
                         Mat & frame,
                         vector<KeyPoint> & keypoints,
                         Mat & descriptors,
                         bool & frame_changed)
    {
        //    cout << "R " << R << endl;
        auto rotAngles = rotationMatrixToEulerAngles(R);
        //cout << "R " << rotAngles << endl;
        Mat globalR = R * this->R ;
        cout << "global R " <<  rotationMatrixToEulerAngles(globalR) << endl;

        if (abs(rotAngles[1]) < 2.5)
        {
            /* this frame is still good */
            frame_changed = false;
            return this;
        }

        /* we are switching frames */
        frame_changed = true;

        int neighbour;
        KeyFrame *left = nullptr;
        KeyFrame *right = nullptr;

        if (rotAngles[1] > 0)
        {
            neighbour = RIGHT_NEIGHBOUR;
            left = this;
        }
        else
        {
            neighbour = LEFT_NEIGHBOUR;
            right = this;
        }

        if (neighbour_frames[neighbour] == nullptr)
        {
//            printf("making new frame\n");
            neighbour_frames[neighbour] = new KeyFrame(
                frame, keypoints, descriptors, left, right,
                R * this->R);
        }

//        printf("switching frame %i\n", neighbour);
        
        return neighbour_frames[neighbour];
    }
};

static KeyFrame *curKF = nullptr;
//static bool haz_reference_frame = false;
//static vector<KeyPoint> ref_keypoints;
//static Mat ref_frame, ref_descriptors;

extern "C" __declspec(dllexport) vam_handle *
vam_init(int width, int height)
{
    open_debug_console();
    return NULL;
}

bool
filter_rot_matrix(Mat & R)
{
    auto rSum = sum(R)[0];
    //cout << rSum << " " << abs(rSum - 3) << endl;

    return abs(rSum - 3) < 0.5;
}

extern "C" __declspec(dllexport) void
vam_process_frame(vam_handle *vah, void *pixels)
{
    Mat frame;
    vector<KeyPoint> keypoints;
    Mat descriptors;

    get_keypoints(pixels, frame, keypoints, descriptors);

    if (curKF == nullptr)
    {
        curKF = new KeyFrame(frame, keypoints, descriptors);
        return;
    }

    static auto matcher = BFMatcher(NORM_HAMMING, true);


    vector<DMatch> matches = vector<DMatch>();
    matcher.match(curKF->descriptors, descriptors, matches);

    vector<DMatch> close_matches = vector<DMatch>();
    vector<Point2f> points1;
    vector<Point2f> points2;
    filter_matches(matches, close_matches,
                   curKF->keypoints, keypoints,
                   points1, points2);

    if (close_matches.size() < 8)
    {
        printf("lost track\n");
        return;
    }

    auto essMat = findEssentialMat(points2, points1, 655.899779 * DOWNSAMPLE, Point2d(589.928903 * DOWNSAMPLE, 614.458172 * DOWNSAMPLE), RANSAC, 0.8);

    Mat R, t;
    recoverPose(essMat, points2, points1, R, t, 655.899779 * DOWNSAMPLE, Point2d(589.928903 * DOWNSAMPLE, 614.458172 * DOWNSAMPLE));

    if (!filter_rot_matrix(R))
    {
//        printf("bogus essential matrix, skipping\n");
//        cout << "R " << rotationMatrixToEulerAngles(R) << endl;
        return;
    }

    bool frame_changed = false;
    curKF = curKF->next_frame(R, frame, keypoints, descriptors, frame_changed);

    if (frame_changed)
    {
        return;
    }

    Mat tmp1, tmp2;
    cvtColor(curKF->frame, tmp1, CV_BGRA2RGB);
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
    drawMatches(tmp1, curKF->keypoints, tmp2, keypoints, disp_matches, img_matches,
                Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::resize(img_matches, tmp, cv::Size(), 2.4, 2.4);
    // drawing the results
    namedWindow("matches", 1);
    imshow("matches", tmp);
    waitKey(25);
}
