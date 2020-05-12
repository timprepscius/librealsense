// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.


#include <iostream>

#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// --------------

struct ChessboardCalibrationCam {

	template<typename T>
	using vector = std::vector<T>;
	
	using Point3f = cv::Point3f;
	using Point2f = cv::Point2f;
	using Mat = cv::Mat;
	using Size = cv::Size;
	using TermCriteria = cv::TermCriteria;
//	using Error = cv::Error;
	
	enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

	static bool findCalibrationPoints (Mat &view, const Size &boardSize, vector<Point2f> &pointbuf, Pattern pattern = CHESSBOARD)
	{
		bool found;
		switch( pattern )
		{
			case CHESSBOARD:
				found = findChessboardCorners( view, boardSize, pointbuf,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				found = findCirclesGrid( view, boardSize, pointbuf );
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				found = findCirclesGrid( view, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID );
				break;
			default:
				return fprintf( stderr, "Unknown pattern type\n" ), -1;
		}
		
		// improve the found corners' coordinate accuracy
		 if( pattern == CHESSBOARD && found) cornerSubPix( view, pointbuf, Size(11,11),
			 Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));

		return found;
	}
	
	static Mat drawPattern (Mat &viewGray, const Size &boardSize, vector<Point2f> &pointbuf, bool found, Pattern pattern = CHESSBOARD)
	{
		cv::Mat view;
		cv::cvtColor(viewGray, view, cv::COLOR_GRAY2BGR);
		drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
		
		return view;
	}

	std::string lensName;
	vector<vector<Point2f> > imagePoints;
	int flags =  cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT;
	Size boardSize;
	Pattern pattern = CHESSBOARD;
    bool isCalibrated = false;
    bool found = false;
	
	ChessboardCalibrationCam(
		const std::string &lensName,
		cv::Size boardSize)
	{
		this->lensName = lensName;
		this->boardSize = boardSize;
	}
	
	void drawContour(Mat &grey, const vector<Point2f> &contour)
	{
		vector<cv::Point> contouri;
		for (auto &p : contour)
			contouri.push_back({ (int)p.x, (int)p.y });

		vector<vector<cv::Point>> hull;
		hull.resize(1);
		cv::convexHull(contouri, hull[0]);
		
		drawContours(grey, hull, 0, 255, -1);
	}
	
    static cv::Mat toGrayStdNormalize(const cv::Mat &mat, double m=2.0)
    {
        cv::Scalar meanS, stddevS;
        cv::meanStdDev(mat, meanS, stddevS);

        auto mean = meanS[0];
        auto stddev = stddevS[0];
        auto stdC = 0.0;

        cv::Mat u;
        auto spanH = (m * stddev);
        auto span = 2.0 * spanH;
        auto lowerBound = mean - spanH;
        auto maxValue = 255.0;
        auto alpha = maxValue / span;
        auto beta = -(alpha * lowerBound) + stdC;
        
        mat.convertTo(u, CV_8UC1, alpha, beta);
        
        return u;
    }

	void run (Mat &viewGray_)
	{
		cv::Scalar white(255, 255, 255, 255);
		cv::Scalar red(0, 0, 255, 255);
		cv::Scalar blue(255, 0, 0, 255);
		cv::Scalar green(0, 255, 0, 255);
		Mat viewGray = toGrayStdNormalize(viewGray_);

		vector<Point2f> pointbuf;
		found = findCalibrationPoints(viewGray, boardSize, pointbuf);
		auto debugCalibration = drawPattern(viewGray, boardSize, pointbuf, found);
		
		cv::putText(debugCalibration, std::to_string(imagePoints.size()), cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 2.0, white, 2);
		cv::imshow("calibration-" + lensName, debugCalibration);
		
		if (found)
		{
			imagePoints.push_back(pointbuf);
		}
		
		cv::Mat debugImagePointsArea;
		cv::cvtColor(viewGray, debugImagePointsArea, cv::COLOR_GRAY2BGR);
		debugImagePointsArea = cv::Scalar(0,0,0);
		
		auto width = debugImagePointsArea.size().width;
		auto height = debugImagePointsArea.size().height;
		auto marginW = width * 20/100;
		auto marginH = height * 20/100;
		cv::Rect usefulArea(marginW, marginH, width-2*marginW, height-2*marginH);

		cv::Mat pointsAreaThresh = cv::Mat::zeros(viewGray.size(), CV_8U);
		for (auto i=0; i<imagePoints.size(); ++i)
		{
			auto &c = imagePoints[i];
			vector<cv::Point> contour;
			for (auto &p : c)
				contour.push_back({ (int)p.x, (int)p.y });
				
			vector<vector<cv::Point>> hull;
			hull.resize(1);
			
			cv::convexHull(contour, hull[0]);
			auto color = i == imagePoints.size() - 1 ? red : white;
			drawContours(debugImagePointsArea, hull, 0, color, -1);
			drawContours(pointsAreaThresh, hull, 0, 255, -1);
		}
		cv::rectangle(debugImagePointsArea, usefulArea, blue);
		
		auto nonZero = cv::countNonZero(pointsAreaThresh(usefulArea));
		auto zeros = usefulArea.area() - nonZero;

        isCalibrated = zeros <= 0;
		
		cv::putText(debugImagePointsArea, std::to_string(zeros), cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 2.0, green, 2);
		cv::imshow("imageArea-" + lensName, debugImagePointsArea);
	}
} ;