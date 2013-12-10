/*

#include "opencv2/opencv.hpp"

using namespace cv;
	int i;
	bool b;
	int n;
	std::stringstream res; //gabung string
	
int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
	
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);

    //Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        //cvtColor(frame, edges, CV_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        imshow("edges", frame);
		if(i >= 100 )
		{			
			res <<  n << " image.jpg";	//gabung string
			b = imwrite(res.str(), frame, compression_params); 
			n++;
			i = 0;
		}
        if(waitKey(30) >= 0) break;
		i++;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

*/

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
bool b;
int i = 1;
std::stringstream res; //gabung string
int main( int argc, char** argv )
{
  Mat image;
  image = imread( argv[1], 1 );
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);

  if( argc != 2 || !image.data )
    {
      printf( "No image data \n" );
      return -1;
    }

  namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
  imshow( "Display Image", image );
//filename = "
	res <<  i << "name.jpg";	//gabung string
	b = imwrite(res.str(), image, compression_params); 

  waitKey(0);

  return 0;
}


