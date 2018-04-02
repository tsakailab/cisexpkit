/*
  Plane detection

  Input:  color and depth images
  Output: color and depth image (window)

  Compile me with:
  $ g++ -std=c++11 Plane.cpp -lrealsense -lopencv_core -lopencv_imgproc -lopencv_highgui -o Plane
  and run
  $ ./Plane

  Written by tsakai@cis.nagasaki-u.ac.jp, 2017
*/

// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// camera parameter set
rs::intrinsics camera_param;
float scale;

// inline functions
inline float J_TO_U(int j){ return -(j - camera_param.ppx);}
inline float I_TO_V(int i){ return -(i - camera_param.ppy);}

// (j, i) of three points, left, right and bottom, that determine the plane
Point posL(180, 380), posR(460, 380), posB(320, 440);

// Font faces
int face[] = {FONT_HERSHEY_SIMPLEX, FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX, FONT_HERSHEY_COMPLEX, FONT_HERSHEY_TRIPLEX, FONT_HERSHEY_COMPLEX_SMALL, FONT_HERSHEY_SCRIPT_SIMPLEX, FONT_HERSHEY_SCRIPT_COMPLEX, FONT_ITALIC};

// prototye declaration
int ComputePlane(const Mat& Depth, Mat&, Mat&);
int ColorPlane(const Mat&, const Mat&, const Mat&, const float&, Mat&, Mat&, Point&, float&, bool maskc=false);
void Zuv_to_XY(float, float, float, float *, float *);

int main(void)
{
    // Create a context object. This object owns the handles to all connected realsense devices
    rs::context ctx;

    // Access the first available RealSense device
    rs::device * dev = ctx.get_device(0);

    // Configure BGR and depth streams to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);

    // Get depth camera parameters
    camera_param = dev->get_stream_intrinsics(rs::stream::color);
    scale = dev->get_depth_scale();

    // Parameters
    Mat vecN, vecP, vecC; 
    Point posC;
    float height;

    // Start streaming
    dev->start();

    // Camera warmup - Dropped several first frames to let auto-exposure stabilize
    //for(int i = 0; i < 30; i++)

    char key;
    bool mask_nodepth = false;
    bool show_dist = true;
    bool show_3points = false;
    while((key = waitKey(1)) != 'q'){
        dev->wait_for_frames();

        // Creating OpenCV Matrix from color and depth images
        Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color));
        Mat depth(Size(640, 480), CV_16UC1, (void*)dev->get_frame_data(rs::stream::depth_aligned_to_color));

        // Estimate plane parameters
        if(ComputePlane(depth, vecN, vecP) == 0){

            if(key == 'd') show_dist = !show_dist;

            if(key == 'm') mask_nodepth = !mask_nodepth;
            // detect and color the plane
            ColorPlane(vecN, vecP, depth, eps0.02, color, vecC, posC, height, mask_nodepth);

            // Write the height by
            // putText(image, text, position(bottom left),
            //              font face, text scale, text color, thickness, type);
            if(show_dist){
                char cnumUser[13];
                sprintf(cnumUser, "%d [mm]", (int) (height*1000));
                putText(color, cnumUser, Point(50,50),
                                face[0], 1.2, Scalar(0,0,200), 2, CV_AA);

                circle(color, posC, 4, Scalar(0,255,0), -1);
           }
        }

        // Plot three points that determine the plane
        if(show_3points){
            circle(color, posL, 2, Scalar(255,128,128), -1);
            circle(color, posR, 2, Scalar(255,128,128), -1);
            circle(color, posB, 2, Scalar(255,128,128), -1);
        }

        Mat gray_depth;
        depth.convertTo(gray_depth, CV_8UC1, 255*scale, 0);
        applyColorMap(gray_depth, gray_depth, COLORMAP_OCEAN);

        // Display in a GUI
        imshow("BGR Image", color);
        imshow("Depth Image", gray_depth);
    }

    return 0;
}


int ComputePlane(const Mat& Depth, Mat& vecNormal, Mat& vecPoint)
{
    Point3f Pl, Pr, Pb;
    int i, j;

    j = posL.x; i = posL.y;
    Pl.z = scale * Depth.at<unsigned short>(i, j);
    if( Pl.z == 0. ) return -1;
    Zuv_to_XY(Pl.z, J_TO_U(j), I_TO_V(i), &(Pl.x), &(Pl.y));

    j = posR.x; i = posR.y;
    Pr.z = scale * Depth.at<unsigned short>(i, j);
    if( Pr.z == 0. ) return -1;
    Zuv_to_XY(Pr.z, J_TO_U(j), I_TO_V(i), &(Pr.x), &(Pr.y));

    j = posB.x; i = posB.y;
    Pb.z = scale * Depth.at<unsigned short>(i, j);
    if( Pb.z == 0. ) return -1;
    Zuv_to_XY(Pb.z, J_TO_U(j), I_TO_V(i), &(Pb.x), &(Pb.y));

    // a point on the plane
    vecPoint = Mat(Pb);

    Mat vecA, vecB;
    vecA = Mat(Pl) - Mat(Pb);
    vecB = Mat(Pr) - Mat(Pb);
    // normal vector
    vecNormal = vecB.cross(vecA);
    vecNormal = vecNormal / norm(vecNormal);

    return 0;
}


int ColorPlane(const Mat& vecN, const Mat& vecP, const Mat& Depth, const double& eps_mm, Mat& bgrImage,
                    Mat& vecC, Point& posC, float& height, bool maskc=false)
{
    Point3f P;
    float maxheight, h;

    maxheight = 0.0;
    posC = Point(0,0);

    // for all depth pixels
    for(int i = 0; i != Depth.rows; i ++){
        for(int j = 0; j != Depth.cols; j ++){

            // get the depth value in meters
            P.z = scale * Depth.at<unsigned short>(i, j);

            if( P.z != 0. ){      // if depth is available
                Zuv_to_XY(P.z, J_TO_U(j), I_TO_V(i), &(P.x), &(P.y)); // compute P.x and P.y from P.z, i and j

                // compute the distance between P and the plane
//              h = fabs(vecN.dot(Mat(P) - vecP));
                h = vecN.dot(Mat(P) - vecP);

                // record it if currently maximum
                if(h > maxheight){
                    maxheight = h;
                    vecC = Mat(P);
                    posC = Point(j, i);
                }

                // modify the pixel color if the distance is less than eps_mm
                if( fabs(h) < eps_mm ){
                    bgrImage.at<Vec3b>(i, j) += Vec3b(0,0,127);
                }
            }
            else if(maskc){
                bgrImage.at<Vec3b>(i, j) = Vec3b(0,0,0);
            }
        }
    }

    return 0;
}


/*
    Inverse perspective transformation
    computes X and Y from Z, u and v
*/
// COMPLETE THIS FUNCTION ON YOUR OWN. GOOD LUCK!!
void Zuv_to_XY(float Z, float u, float v, float *X, float *Y)
{
//  *X = // (available variables: Z, u, v, camera_param.fx)
//  *Y = // (available variables: Z, u, v, camera_param.fy)

    *X = Z * u / camera_param.fx;
    *Y = Z * v / camera_param.fy;
}

