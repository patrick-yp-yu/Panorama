// CSCE643 Project2
// Yuan-Peng Yu
//
// Q1. Compose panorama using the H matrix that is computed by DLT

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    // read image
    Mat img00 = imread("image00.jpg", CV_LOAD_IMAGE_COLOR);
    Mat img01 = imread("image01.jpg", CV_LOAD_IMAGE_COLOR);

    vector<Point2f> pts_img00;	//source points
    vector<Point2f> pts_img01;	//destination points


	// Input 20 featured-points pairs
    pts_img01.push_back(Point2f( 1970,717 ));
    pts_img00.push_back(Point2f( 374,737 ));
    pts_img01.push_back(Point2f( 1954,680 ));
    pts_img00.push_back(Point2f( 364,701 ));
    pts_img01.push_back(Point2f( 1956,444 ));
    pts_img00.push_back(Point2f( 389,476 ));
    pts_img01.push_back(Point2f( 1867,466 ));
    pts_img00.push_back(Point2f( 303,480 ));
    pts_img01.push_back(Point2f( 1859,575 ));
    pts_img00.push_back(Point2f( 284,588 ));
    pts_img01.push_back(Point2f( 1855,484 ));
    pts_img00.push_back(Point2f( 287,498 ));
    pts_img01.push_back(Point2f( 1849,636 ));
    pts_img00.push_back(Point2f( 267,646 ));
    pts_img01.push_back(Point2f( 1951,1412 ));
    pts_img00.push_back(Point2f( 252,1416 ));
    pts_img01.push_back(Point2f( 1813,1363 ));
    pts_img00.push_back(Point2f( 125,1380 ));
    pts_img01.push_back(Point2f( 1672,1140 ));
    pts_img00.push_back(Point2f( 30,1152 ));

    pts_img01.push_back(Point2f( 1986,1088 ));
    pts_img00.push_back(Point2f( 357,1096 ));
    pts_img01.push_back(Point2f( 1824,1088 ));
    pts_img00.push_back(Point2f( 200,1094 ));
    pts_img01.push_back(Point2f( 1970,757 ));
    pts_img00.push_back(Point2f( 369,777 ));
    pts_img01.push_back(Point2f( 1745,562 ));
    pts_img00.push_back(Point2f( 172,559 ));
    pts_img01.push_back(Point2f( 1805,550 ));
    pts_img00.push_back(Point2f( 232,556 ));
    pts_img01.push_back(Point2f( 1780,601 ));
    pts_img00.push_back(Point2f( 203,604 ));
    pts_img01.push_back(Point2f( 1794,429 ));
    pts_img00.push_back(Point2f( 236,432 ));
    pts_img01.push_back(Point2f( 1672,752 ));
    pts_img00.push_back(Point2f( 72,745 ));
    pts_img01.push_back(Point2f( 1665,508 ));
    pts_img00.push_back(Point2f( 93,491 ));
    pts_img01.push_back(Point2f( 1592,523 ));
    pts_img00.push_back(Point2f( 11,495 ));


    // Find H matrix by OpenCV findHomography
    Mat H_verify = findHomography(pts_img00, pts_img01);	// srcPoints, dstPoints, img01 = H * img00


// ------------------------------------------------
// The Direct Linear Transformation algorithm (DLT)
// ------------------------------------------------
    vector<Point2f> viewSrc, viewDst;
    viewSrc = pts_img00;
    viewDst = pts_img01;

    int num = viewSrc.size();
    cout << " The number of points pair = " << num << endl;
    Mat A( 2*num, 9, CV_32F);	// (2*n) x 9 matrix
	Mat h_vector(9,1, CV_32F);	// 9x1 vector

	// Build matrix A, A*h =0
	// page 90. Choose wi' =1, and let h33 !=1
    for (int p=0; p<num; ++p){	//  points
			A.at<float>(2*p,0) = 0;
			A.at<float>(2*p,1) = 0;
			A.at<float>(2*p,2) = 0;
			A.at<float>(2*p,3) = - viewSrc.at(p).x;	// -xw' = -x
			A.at<float>(2*p,4) = - viewSrc.at(p).y;	// -yw' = -y
			A.at<float>(2*p,5) = -1;	// -ww' = -1
			A.at<float>(2*p,6) = viewSrc.at(p).x * viewDst.at(p).y;		// xy'
			A.at<float>(2*p,7) = viewSrc.at(p).y * viewDst.at(p).y; 	// yy'
			A.at<float>(2*p,8) = viewDst.at(p).y;	// y'

			A.at<float>(2*p+1,0) = viewSrc.at(p).x;	// xw'=x
			A.at<float>(2*p+1,1) = viewSrc.at(p).y;	// yw'=y
			A.at<float>(2*p+1,2) = 1; 	// ww'=1
			A.at<float>(2*p+1,3) = 0;
			A.at<float>(2*p+1,4) = 0;
			A.at<float>(2*p+1,5) = 0;
			A.at<float>(2*p+1,6) = - viewSrc.at(p).x * viewDst.at(p).x ;	// -xx'
			A.at<float>(2*p+1,7) = - viewSrc.at(p).y * viewDst.at(p).x;  	// -yx'
			A.at<float>(2*p+1,8) = - viewDst.at(p).x;	// -wx'=-x'
    }
	cout << "A = " << endl << A << endl << endl;

	// use SVD::compute to gain homography matrix H
	Mat D;	// singular values in diagonal matrix
	Mat U;	// left singular vectors
	Mat V, Vt;	// V = right singular vector, Vt= transopsed matrix of V

	SVD::compute(A, D, U, Vt, SVD::FULL_UV);
	V = Vt.t();
	h_vector = V.col(8);	// the last column = h
	cout << "h = " << endl << h_vector << endl << endl;

	Mat out; // Verify A*h = 0
	out = A * h_vector;
	cout << "Verify A*h = 0 " << endl << out << endl << endl;

	// put h vector in the matrix form
	Mat H_matrix(3,3, CV_32F), H_div_h33(3,3, CV_32F);	// 3x3 matrix
	int k=0;
	for(int i=0; i<3; ++i){
		for(int j=0; j<3; ++j){
			H_matrix.at<float>(i,j) = h_vector.at<float>(k,0);
			H_div_h33.at<float>(i,j) = h_vector.at<float>(k,0) / h_vector.at<float>(8,0); // divided by h33
			++k;
		}
	}
	cout << "Homography matrix H = " << endl << H_matrix << endl << endl;
	cout << "Verify H by dividing h33" << endl << H_div_h33 << endl << endl;
	cout << "Verify by OpenCV findHomography() = " << endl << H_verify << endl << endl;
// ------------------------------------------------
// The end of DLT
// ------------------------------------------------

	// // Show selected points
	// for(int i=0; i<4; ++i){
	// 	circle( img1st, Point( x_1st[i], y_1st[i] ), 10, Scalar( 0, 255, 0 ), 3, 8 );
	// }

	// 1. Verify H by findHomography
	Mat img_findH;
	warpPerspective(img00, img_findH, H_verify, Size(img00.cols + img01.cols, img00.rows) );
	// imgfindH = h * (img00)
	// warpPerspective( src, dst, transformation matrix, size of the o/p)
	Mat canvas_findH(img_findH, Rect(0,0, img01.cols, img01.rows));
	img01.copyTo(canvas_findH);
	namedWindow("Image computed by findHomography()", WINDOW_NORMAL);
    imshow("Image computed by findHomography()", img_findH);
    imwrite( "result_by_findHomography.jpg", img_findH );

 	// 2. Generate panorama by H that is computed by DLT. Source = img00
	Mat panorama;
	warpPerspective(img00, panorama, H_matrix, Size(img00.cols + img01.cols, img00.rows) );
	//// img01 = h * (img00), warpPerspective( src, dst, transformation matrix, size of the o/p)
	Mat canvas_DLT(panorama, Rect(0,0, img01.cols, img01.rows));
	img01.copyTo(canvas_DLT);
	namedWindow("Image computed by DLT.", WINDOW_NORMAL);
    imshow("Image computed by DLT.", panorama);
    imwrite( "result_by_DLT.jpg", panorama );

 // 	//3. Generate panorama by H that is computed by DLT. Source = img01
	// Mat panorama2;
	// cout << "H^-1 = " << endl << H_matrix.inv() << endl << endl;
	// warpPerspective(img01, panorama2, H_matrix.inv(), Size(img00.cols + img01.cols, img00.rows)); //Size(img00.cols + img01.cols, img00.rows) );
	// //// img-1panorama = h^-1 * (img01), warpPerspective( src, dst, transformation matrix, size of the o/p)
	// Mat canvas_DLT2(panorama2, Rect(img01.cols, 0, img00.cols, img00.rows));	// Rect(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
	// img00.copyTo(canvas_DLT2);
	// namedWindow("Image computed by DLT. Perspective 2", WINDOW_NORMAL);
 //    imshow("Image computed by DLT. Perspective 2", panorama2);
 //    imwrite( "Q1_result_DLT_p2.jpg", panorama2 );

    waitKey(0);
    return 0;
}
