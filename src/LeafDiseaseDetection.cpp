#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;
using namespace std;

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}

int main()
{
    string image_path = samples::findFile("/home/gabiyuri/skyrats_ws2/src/indoor22/images/leaf-spot-disease.jpg");
    Mat img = imread( image_path, IMREAD_COLOR );
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }

    // imgGreen: not green --> black
    Mat imgHSV;
    cvtColor( img, imgHSV, COLOR_BGR2HSV );
    
    Mat greenMask;
    inRange( imgHSV, Scalar(0, 0, 0), Scalar(87, 255, 255), greenMask );

    Mat imgGreen = Mat::zeros( img.size(), img.type() );

    img.copyTo( imgGreen, greenMask );

    // imgCropped: get a specific area

    Mat imgCropped = img( Range(600, 950), Range(1000, 1500) );
    Mat greenCropped = imgGreen( Range(600, 950), Range(1000, 1500) );

    Mat grayCropped;
    cvtColor( greenCropped, grayCropped, COLOR_RGB2GRAY );

    Mat threshGreen = Mat::zeros( grayCropped.size(), grayCropped.type() );
    threshold( grayCropped, threshGreen, 110, 255, THRESH_BINARY );

    // find the biggest leaf contours in that area
    vector< vector<Point> > contoursLeaf;
    vector<Vec4i> hierarchyLeaf;
    findContours( threshGreen, contoursLeaf, hierarchyLeaf, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

    Mat copyCropped = imgCropped.clone();

    /* draw all contours
    int idx = 0; 
    for( ; idx >= 0; idx = hierarchyLeaf[idx][0] )
    {
        drawContours( copyCropped, contoursLeaf, idx, Scalar(255, 0, 200), 2, LINE_AA, hierarchyLeaf );
    }
    */

    sort(contoursLeaf.begin(), contoursLeaf.end(), compareContourAreas);
    vector< vector<Point> > largestContours;
    largestContours.push_back(contoursLeaf[contoursLeaf.size()-1]);

    drawContours( copyCropped, largestContours, 0, Scalar(255, 0, 200), 2, LINE_AA );

    Mat leafMask = Mat::zeros( copyCropped.size(), copyCropped.type() );
    //leafMask.setTo(cv::Scalar(255, 255, 255));

    fillPoly(leafMask, largestContours, Scalar(255, 255, 255));

    //Mat cropped = img( Range(600, 950), Range(1000, 1500) );
    Mat biggestLeaf;
    bitwise_and(leafMask, imgCropped, biggestLeaf);

    Mat leafHSV;
    cvtColor(biggestLeaf, leafHSV, COLOR_BGR2HSV);

    Mat spotMask;
    inRange( leafHSV, Scalar(107, 0, 0), Scalar(179, 255, 255), spotMask );

    Mat brown = Mat::zeros( imgCropped.size(), imgCropped.type() );

    imgCropped.copyTo( brown, spotMask );

    Mat grayLeaf;
    cvtColor( brown, grayLeaf, COLOR_RGB2GRAY );

    Mat threshBrown = Mat::zeros( grayLeaf.size(), grayLeaf.type() );
    threshold( grayLeaf, threshBrown, 40, 255, THRESH_BINARY );
    
    vector< vector<Point> > contoursBrown;
    vector<Vec4i> hierarchyBrown;
    findContours( threshBrown, contoursBrown, hierarchyBrown, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

    Mat croppedBrown = imgCropped.clone();
    int idx = 0; 
    for( ; idx >= 0; idx = hierarchyBrown[idx][0] )
    {
        drawContours( croppedBrown, contoursBrown, idx, Scalar(0, 0, 255), 4, LINE_AA, hierarchyBrown );
    }

    imshow("Display window", croppedBrown);

    int k = waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        imwrite("leaf-spot-disease.jpeg", img);
    }
    return 0;
}
