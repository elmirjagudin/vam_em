#include "stdafx.h"
#include "vam.h"

#include <stdio.h>
#include <algorithm>

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\features2d.hpp>

#include "angles.h"

#define FEATURES 32*8
#define MIN_MATCH_DISTANCE 16
#define NUM_TRACK_POINTS 16
#define MAX_INIT_POINTS 256*4

using namespace std;
using namespace cv;

enum vam_mode { vam_aquire, vam_tracking };

struct key_point_s
{
    int descriptor_idx;
    int matches;
    key_point_s(int descriptor_idx)
    {
        this->descriptor_idx = descriptor_idx;
        this->matches = 0;
    }
};

typedef struct key_point_s key_point;

struct vam_handle_s
{
    vam_mode mode;
    int width;
    int height;
    Ptr<ORB> orb;
    BFMatcher matcher;
    Mat descriptors;
    Mat track_descriptors;
    vector<key_point> key_points;
};

static void
show_keypoints(Mat img, vector<KeyPoint> & keypoints)
{
    Mat tmp;
    /* drawKeypoint can draw on BGRA images, convert to supported RGB */
    cvtColor(img, tmp, CV_BGRA2RGB);
    drawKeypoints(tmp, keypoints, tmp, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    namedWindow("win");
    imshow("win", tmp);

    waitKey(1);
}

static void open_debug_console()
{
    AllocConsole();
    FILE* fp;

    freopen_s(&fp, "CONOUT$", "w", stdout);
    printf("Debugging Window:\n");
}

//vam_handle *
//vam_init(int width, int height)
//{
//    open_debug_console();
//
//    vam_handle *vah = new vam_handle_s();
//    vah->mode = vam_aquire;
//    vah->width = width;
//    vah->height = height;
//
//    vah->orb = cv::ORB::create(FEATURES);
//    vah->matcher = BFMatcher(NORM_HAMMING, true);
//
//    return vah;
//}

static bool
vam_add_new_keypoints(vam_handle *vah,
                      Mat & descriptors,
                      vector<bool> & matched_descriptors)
{
    for (int i = 0; i < descriptors.rows; i += 1)
    {
        if (vah->descriptors.rows >= MAX_INIT_POINTS)
        {
            printf("reached peak points, we are done here\n");
            return false;
        }

        if (matched_descriptors[i])
        {
            /* matches to existing descriptor, not new point */
            continue;
        }
        vah->key_points.push_back(key_point(vah->descriptors.rows));
        vah->descriptors.push_back(descriptors.row(i));
    }

    printf("tracking %zi %i points\n", vah->key_points.size(), vah->descriptors.rows);
    return true;
}

static void
vam_tally_matches(vam_handle *vah, vector<DMatch> & matches, vector<bool> & matched_descriptors)
{
    for (auto match = matches.begin(); match != matches.end(); ++match)
    {
        if (match->distance > MIN_MATCH_DISTANCE)
        {
            /* distance to large, ignore */
            continue;
        }

        vah->key_points[match->queryIdx].matches += 1;
        matched_descriptors[match->trainIdx] = true;
    }
}

bool
kp_matches(key_point l, key_point r)
{
    return l.matches > r.matches;
}

static void
pick_top_points(vam_handle *vah)
{
    sort(vah->key_points.begin(),
         vah->key_points.end(),
         kp_matches);

    vah->track_descriptors =
        Mat(0, vah->descriptors.cols, vah->descriptors.type());

    for (auto i = vah->key_points.begin();
         i != vah->key_points.begin() + NUM_TRACK_POINTS;
         i++)
    {
        auto desc_row = (*i).descriptor_idx;
        vah->track_descriptors.push_back(vah->descriptors.row(desc_row));
    }
}

static vam_mode
vam_aquire_points(vam_handle *vah, Mat & descriptors)
{
    vector<DMatch> matches = vector<DMatch>();

    vah->matcher.match(vah->descriptors, descriptors, matches);
    vector<bool> matched_descriptors = vector<bool>(descriptors.rows);

    vam_tally_matches(vah, matches, matched_descriptors);
    if (!vam_add_new_keypoints(vah, descriptors, matched_descriptors))
    {
        pick_top_points(vah);
        return vam_tracking;
    }

    return vam_aquire;
}

#define M_PI 3.14159265359

static void
vam_track_points(vam_handle *vah, Mat & frame, vector<KeyPoint> & keypoints, Mat & descriptors)
{
    vector<KeyPoint> matched_keypoints;
    vector<DMatch> matches = vector<DMatch>();
    vah->matcher.match(vah->track_descriptors, descriptors, matches);

    printf("tracking...\n");

    for (auto match = matches.begin(); match != matches.end(); ++match)
    {
        if (match->distance > MIN_MATCH_DISTANCE)
        {
            /* distance to large, ignore */
            continue;
        }

        auto kp = keypoints[match->trainIdx];
        matched_keypoints.push_back(kp);
    }

    for (auto i = matched_keypoints.begin(); i != matched_keypoints.end(); ++i)
    {
        auto kp = *i;

        printf("x %f y %f\n", kp.pt.x, kp.pt.y);

        auto vect = pixel_to_vect(kp.pt.x, kp.pt.y);
//        cout << vect << endl;
        cout << vect_angle(vect, X_AXIS) * (360 / (M_PI * 2)) << " " 
             << vect_angle(vect, Y_AXIS) * (360 / (M_PI * 2)) << endl;
    }

    show_keypoints(frame, matched_keypoints);
}

//void
//vam_process_frame(vam_handle *vah, void *pixels)
//{
//    vector<KeyPoint> keypoints;
//    Mat descriptors;
//
//    Mat tmp = Mat(vah->width, vah->height, CV_8UC4, pixels);
//    Mat frame;
//    cv::resize(tmp, frame, cv::Size(), 0.5, 0.5);
//
//    vah->orb->detectAndCompute(_InputArray(frame), cv::noArray(), keypoints, descriptors);
//
//    switch (vah->mode)
//    {
//    case vam_aquire:
//        vah->mode = vam_aquire_points(vah, descriptors);
//        break;
//    case vam_tracking:
//        vam_track_points(vah, frame, keypoints, descriptors);
//        break;
//    default:
//        assert(false);
//    }
//}
