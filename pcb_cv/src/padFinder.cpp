/*
 * padFinder.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: johan
 */
#include "../include/pcb_cv/padFinder.h"
using namespace std;
using namespace cv;

// Pad Finder
#define MIN_CONTOUR_AREA 100
#define MIN_PAD_AREA 70
#define DILATE_ITERATIONS 3

// Chip Finder
#define ERODE_ITERATIONS_CHIP_FINDER 3
#define MIN_CONTOUR_AREA_CHIP 300
#define ERROR_PERCENT_CHIP 20.0

// SMD Finder
#define ERODE_ITERATIONS_SMD_FINDER 1
#define MIN_AREA_SMD_FINDER 300
#define ERROR_PERCENT_SMALLSMD 20.0

// SMD Tape
#define ERODE_ITERATIONS_TAPE_FINDER 1
#define MIN_AREA_TAPE_FINDER 300
#define ERROR_PERCENT_SMDTAPE 20.0

padFinder::padFinder() {
	foundVia = false;
	partWidth_ = 0.0;
	partHeight_ = 0.0;
}

padFinder::~padFinder() {

}

void padFinder::setSize(float width, float height) {
	partWidth_ = width;
	partHeight_ = height;
}

bool padFinder::nearestPart(std::vector<smdPart>* list, smdPart* partDst,
		int width, int height) {
	int smdId = -1;
	float centerX, centerY, shortestDistance;
	centerX = width / 2 - 1;
	centerY = height / 2 - 1;
	shortestDistance = std::numeric_limits<float>::infinity();

	for (int i = 0; i < list->size(); i++) {
		float distance = sqrt(
				std::pow(std::fabs(centerX - (*list)[i].x), 2)
						+ std::pow(std::fabs(centerY - (*list)[i].y), 2));
		if (distance < shortestDistance) {
			shortestDistance = distance;
			smdId = i;
		}
	}

	if (smdId != -1) {
		partDst->x = (*list)[smdId].x;
		partDst->y = (*list)[smdId].y;
		partDst->rot = (*list)[smdId].rot;
		return true;
	} else {
		return false;
	}

}

double padFinder::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2)
			/ sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

bool padFinder::isBorderTouched(cv::RotatedRect pad) {
	cv::Point2f vertices2f[4];
	cv::Point vertices[4];
	pad.points(vertices2f);
	for (int i = 0; i < 4; ++i) {
		vertices[i] = vertices2f[i];

		if (vertices[i].x < 5 || vertices[i].x > 635 || vertices[i].y > 475
				|| vertices[i].y < 5) {
			return true;
		}
	}
	return false;

}

// default color is white
void padFinder::drawRotatedRect(cv::Mat& image, cv::RotatedRect rRect,
		cv::Scalar color) {

	cv::Point2f vertices2f[4];
	cv::Point vertices[4];
	rRect.points(vertices2f);
	for (int i = 0; i < 4; ++i) {
		vertices[i] = vertices2f[i];
	}

	cv::fillConvexPoly(image, vertices, 4, color);
}

/**
 * Helper function to display text in the center of a contour
 */
void padFinder::setLabel(cv::Mat& im, const std::string label,
		std::vector<cv::Point>& contour) {
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;
	cv::Size text = cv::getTextSize(label, fontface, scale, thickness,
			&baseline);
	cv::Rect r = cv::boundingRect(contour);
	cv::Point pt(r.x + ((r.width - text.width) / 2),
			r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline),
			pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255),
			CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

smdPart padFinder::findChip(cv::Mat* input) {
	cv::Mat gray;
	cv::Mat final = input->clone();
	std::vector<smdPart> smdObjects;
	cv::cvtColor(*input, gray, CV_BGR2GRAY);
	cv::threshold(gray, gray, 255, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	cv::erode(gray, gray, cv::Mat(), cv::Point(-1, -1),
	ERODE_ITERATIONS_CHIP_FINDER);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contoursSorted;
	cv::findContours(gray.clone(), contours, CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_NONE);

	for (int i = 0; i < contours.size(); i++) {
		cv::RotatedRect rect = minAreaRect(contours[i]);
		float rectConvertedArea = rect.size.width / PIXEL_TO_MM
				* rect.size.height / PIXEL_TO_MM;

		if (rectConvertedArea
				> ((partWidth_ * partHeight_) / 100.0)
						* (100.0 - ERROR_PERCENT_CHIP)
				&& rectConvertedArea
						< ((partWidth_ * partHeight_) / 100.0)
								* (100.0 + ERROR_PERCENT_CHIP)) {
			contoursSorted.push_back(contours[i]);

			smdPart smd;
			smd.x = rect.center.x;
			smd.y = rect.center.y;
			smd.rot = rect.angle;
			smdObjects.push_back(smd);
			cv::drawContours(final, contours, i, CV_RGB(0, 255, 0), 2);
		} else
			break;
	}

	if (contoursSorted.size() > 1) {
		ROS_INFO("More than one chip found!");
	}

	smdPart smdFinal;

	if (nearestPart(&smdObjects, &smdFinal, input->cols, input->rows)) {
		circle(final, Point2f(smdFinal.x, smdFinal.y), 5, CV_RGB(0, 0, 255), 3);
		smdFinal.x = smdFinal.x - (input->cols / 2 - 1);
		smdFinal.y = (input->rows / 2 - 1) - smdFinal.y;
	}

	/*
	 cv::imshow("grey", gray);
	 cv::imshow("input", *input);
	 cv::imshow("final", final);
	 cv::waitKey(0);
	 */
	*input = final.clone();
	return smdFinal;
}

smdPart padFinder::findSmallSMD(cv::Mat* input) {
	cv::Mat gray;
	std::vector<smdPart> smdObjects;
	cv::Mat final = input->clone();
	cv::cvtColor(*input, gray, CV_BGR2GRAY);
	cv::threshold(gray, gray, 255, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	cv::erode(gray, gray, cv::Mat(), cv::Point(-1, -1),
	ERODE_ITERATIONS_SMD_FINDER);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contoursSorted;
	vector<Vec4i> hierarchy;
	cv::findContours(gray.clone(), contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); i++) {
		//ROS_INFO("Index %d",i);

		if (hierarchy[i][2] != -1) {
			int closedContour = hierarchy[i][2];
			for (int i = 0; i < contours.size(); i++) {
				//ROS_INFO("Parent %d",hierarchy[i][3]);
				if (hierarchy[i][3] == closedContour - 1) {
					cv::RotatedRect rect = minAreaRect(contours[i]);
					float rectConvertedArea = rect.size.width / PIXEL_TO_MM
							* rect.size.height / PIXEL_TO_MM;

					if (rectConvertedArea
							> ((partWidth_ * partHeight_) / 100.0)
									* (100.0 - ERROR_PERCENT_SMALLSMD)
							&& rectConvertedArea
									< ((partWidth_ * partHeight_) / 100.0)
											* (100.0 + ERROR_PERCENT_SMALLSMD)) {
						cv::drawContours(final, contours, i, CV_RGB(0, 255, 0),
								2);
						smdPart smd;
						smd.x = rect.center.x;
						smd.y = rect.center.y;
						smd.rot = rect.angle;
						//ROS_INFO("Angle : %f Width: %f Height: %f",rect.angle,rect.size.width,rect.size.height);
						if (rect.size.height < rect.size.width) {
							smd.rot = std::fabs(smd.rot);
						} else {
							smd.rot = -(90.0 + smd.rot);
						}
						smdObjects.push_back(smd);
					}
				}
			}
		}
	}
	smdPart smdFinal;

	if (nearestPart(&smdObjects, &smdFinal, input->cols, input->rows)) {
		circle(final, Point2f(smdFinal.x, smdFinal.y), 5, CV_RGB(0, 0, 255), 3);
		smdFinal.x = smdFinal.x - (input->cols / 2 - 1);
		smdFinal.y = (input->rows / 2 - 1) - smdFinal.y;
	}
	//cv::imshow("grey", gray);
	//cv::imshow("input", *input);
	//cv::imshow("final", final);
	//cv::waitKey(0);
	*input = final.clone();
	return smdFinal;
}

smdPart padFinder::findSMDTape(cv::Mat* input) {
	std::vector<smdPart> smdObjects;
	cv::Mat gray;
	cv::Mat final = input->clone();
	cv::cvtColor(*input, gray, CV_BGR2GRAY);
	cv::threshold(gray, gray, 255, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	cv::erode(gray, gray, cv::Mat(), cv::Point(-1, -1),
	ERODE_ITERATIONS_TAPE_FINDER);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contoursSorted;
	vector<Vec4i> hierarchy;
	vector<int> circlesIndex;

	cv::findContours(gray.clone(), contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;
	for (int i = 0; i < contours.size(); i++) {
		cv::approxPolyDP(cv::Mat(contours[i]), approx,
				cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
		// Skip small or non-convex objects
		if (std::fabs(cv::contourArea(contours[i])) < MIN_AREA_TAPE_FINDER
				|| !cv::isContourConvex(approx)) {
			continue;
			ROS_INFO("contour too small");
		}

		if (approx.size() >= 4 && approx.size() <= 6) {
		} else {
			// Detect and label circles
			double area = cv::contourArea(contours[i]);
			cv::Rect r = cv::boundingRect(contours[i]);
			int radius = r.width / 2;
			if (std::abs(1 - ((double) r.width / r.height)) <= 0.3
					&& std::abs(1 - (area / (CV_PI * std::pow(radius, 2))))
							<= 0.3) {
				setLabel(final, "HOLE", contours[i]);
				circlesIndex.push_back(i);
			}
		}
	}

	for (int i = 0; i < contours.size(); i++) {
		// Sort out circles
		bool found = false;
		for (int j = 0; j <= circlesIndex.size(); j++) {
			if (circlesIndex[j] == i) {
				found = true;
			}
		}

		if (!found) {
			cv::RotatedRect rect = minAreaRect(contours[i]);
			float rectConvertedArea = rect.size.width / PIXEL_TO_MM
					* rect.size.height / PIXEL_TO_MM;
			//ROS_INFO("Tape size : %f Des size: %f",rectConvertedArea,(partWidth_ * partHeight_));
			if (!isBorderTouched(rect)
					&& rectConvertedArea
							> ((partWidth_ * partHeight_) / 100.0)
									* (100.0 - ERROR_PERCENT_SMDTAPE)
					&& rectConvertedArea
							< ((partWidth_ * partHeight_) / 100.0)
									* (100.0 + ERROR_PERCENT_SMDTAPE)) {
				smdPart smd;
				smd.x = rect.center.x;
				smd.y = rect.center.y;
				smd.rot = rect.angle;
				if (rect.size.height < rect.size.width) {
					smd.rot = std::fabs(smd.rot);
				} else {
					smd.rot = -(90.0 + smd.rot);
				}
				//ROS_INFO("Angle : %f Width: %f Height: %f",rect.angle,rect.size.width,rect.size.height);
				smdObjects.push_back(smd);
				cv::drawContours(final, contours, i, CV_RGB(0, 255, 0), 2);
			}
		}
	}

	smdPart smdFinal;

	if (nearestPart(&smdObjects, &smdFinal, input->cols, input->rows)) {
		circle(final, Point2f(smdFinal.x, smdFinal.y), 5, CV_RGB(0, 0, 255), 3);
		smdFinal.x = smdFinal.x - (input->cols / 2 - 1);
		smdFinal.y = (input->rows / 2 - 1) - smdFinal.y;
	}

	/*
	 cv::imshow("grey", gray);
	 cv::imshow("input", *input);
	 cv::imshow("final", final);
	 cv::waitKey(0);
	 */
	*input = final.clone();
	return smdFinal;
}

//std::vector<cv::Point2f >
cv::Point2f padFinder::findPads(cv::Mat* input, bool startSelect,
		cv::Point2f selectPad) {
	foundVia = false;
	cv::Mat gray;
	cv::Point2f outputPosition;
	outputPosition.x = 0.0;
	outputPosition.y = 0.0;
	cv::cvtColor(*input, gray, CV_BGR2GRAY);

	cv::threshold(gray, gray, 255, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	/*
	 cv::Mat bw2 = gray.clone();
	 //cv::Canny(gray, bw2, 0, 70, 3);
	 //cv::dilate(bw2, bw2, cv::Mat(), cv::Point(-1, -1), 3);

	 vector<cv::Vec4i> lines;
	 // detect lines
	 cv::HoughLinesP(bw2, lines, 1, CV_PI / 180, 150, 0, 3);
	 //cv::HoughLinesP(bw, lines, 1, CV_PI/180, 50, 50, 10 );
	 cv::Mat colLines;
	 cv::cvtColor(gray, colLines, CV_GRAY2BGR);
	 // draw lines

	 for (size_t i = 0; i < lines.size(); i++) {
	 cv::Vec4i l = lines[i];
	 double length = sqrt(
	 ((l[0] - l[2]) * (l[0] - l[2]))
	 + ((l[1] - l[3]) * (l[1] - l[3])));
	 double offset = length * 1.2;
	 double alpha = atan2((double) l[3] - l[1], (double) l[2] - l[0]);
	 double x2 = l[0] + cos(alpha) * offset;
	 double y2 = l[1] + sin(alpha) * offset;

	 double x1 = l[0]
	 - sin(abs(90.0 / (2.0 * CV_PI)) - alpha)
	 * ((offset - length) / 2.0);
	 double y1 = l[1]
	 - cos(abs(90.0 / (2.0 * CV_PI)) - alpha)
	 * ((offset - length) / 2.0);
	 //cv::line( gray, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255,255,255), 3, CV_AA);
	 }
	 */
	cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1), DILATE_ITERATIONS);

	cv::Mat bw;
	cv::threshold(gray, bw, 255, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

	cv::Mat final = input->clone();
	repeat = false;
	std::vector<std::vector<cv::Point> > contours;
	while (1) {
		//cv::Mat dst = input->clone();
		foundVia = false;
		// Find contours
		contours.clear();
		vector<Vec4i> hierarchy;
		vector<int> circlesIndex;
		cv::findContours(bw.clone(), contours, hierarchy, CV_RETR_TREE,
				CV_CHAIN_APPROX_SIMPLE);
		std::vector<cv::Point> approx;

		for (int i = 0; i < contours.size(); i++) {
			if (std::fabs(cv::contourArea(contours[i])) > MIN_CONTOUR_AREA) {
				//cv::drawContours(dst, contours, i, CV_RGB(0, 0, 255), 5);
			}
			//ROS_INFO("Contour");
			// Approximate contour with accuracy proportional
			// to the contour perimeter
			cv::approxPolyDP(cv::Mat(contours[i]), approx,
					cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
			// Skip small or non-convex objects
			if (std::fabs(cv::contourArea(contours[i])) < 50
					|| !cv::isContourConvex(approx)) {
				continue;
				ROS_INFO("Zu klein");
			}

			if (approx.size() >= 4 && approx.size() <= 6) {

				//ROS_INFO("Found contour ");
				// Number of vertices of polygonal curve
				int vtc = approx.size();
				// Get the cosines of all corners
				std::vector<double> cos;
				for (int j = 2; j < vtc + 1; j++)
					cos.push_back(
							angle(approx[j % vtc], approx[j - 2],
									approx[j - 1]));
				// Sort ascending the cosine values
				std::sort(cos.begin(), cos.end());
				// Get the lowest and the highest cosine
				double mincos = cos.front();
				double maxcos = cos.back();
				// Use the degrees obtained above and the number of vertices
				// to determine the shape of the contour
				if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3) {
					//setLabel(dst, "RECT", contours[i]);
					//ROS_INFO("Rect found!");
				} else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27) {
					//ROS_INFO("PENTA FOUND");
					//setLabel(dst, "PENTA", contours[i]);
				} else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45) {
					//ROS_INFO("HEXA FOUND");
					//setLabel(dst, "HEXA", contours[i]);
				}

			} else {
				// Detect and label circles
				double area = cv::contourArea(contours[i]);
				cv::Rect r = cv::boundingRect(contours[i]);
				int radius = r.width / 2;
				if (std::abs(1 - ((double) r.width / r.height)) <= 0.3
						&& std::abs(1 - (area / (CV_PI * std::pow(radius, 2))))
								<= 0.3) {
					//ROS_INFO("Found circle");
					circlesIndex.push_back(i);
					//setLabel(dst, "CIR", contours[i]);
				}

			}
		}

		for (int i = 0; i < contours.size(); i++) {
			for (int j = 0; j < circlesIndex.size(); j++) {
				if (hierarchy[i][2] == circlesIndex[j]) {
					RotatedRect viaPad = minAreaRect(contours[i]);
					RotatedRect via = minAreaRect(contours[circlesIndex[j]]);
					via.size.height = via.size.height * 1.6;
					via.size.width = via.size.width * 1.6;
					via.angle = viaPad.angle;

					Point2f vertices[4];
					Point2f verticesVia[4];
					viaPad.points(vertices);
					via.points(verticesVia);
					for (int k = 0; k < 4; k++) {
						//line(dst, vertices[k], vertices[(k + 1) % 4],
						//		Scalar(0, 255, 0));
						//line(dst, verticesVia[k], verticesVia[(k + 1) % 4],
						//		Scalar(0, 0, 255));
						drawRotatedRect(bw, via, CV_RGB(0, 0, 0));
					}
					foundVia = true;
					break;
				}
			}
		}

		if (foundVia) {
			repeat = true;
			ROS_INFO("Repeat...");
		} else
			repeat = false;

		if (!repeat) {
			//final = dst;
			break;
		}
	}

	// Draw pads rectangles

	vector<Point2f> padPoints;
	vector<RotatedRect> padRects;

	unsigned int padCounter = 0;
	for (int i = 0; i < contours.size(); i++) {
		double area = cv::contourArea(contours[i]);
		//ROS_INFO("Area: %f", area);
		if (area > MIN_PAD_AREA) {

			RotatedRect pad = minAreaRect(contours[i]);
			if (!isBorderTouched(pad)) {
				// Calculate moments of image
				Moments mu;
				mu = moments(contours[i], false);
				//Calculate mass center
				Point2f mc;
				mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
				padPoints.push_back(mc);
				padRects.push_back(pad);
				//cv::Rect::contains

				if (startSelect
						&& cv::pointPolygonTest(contours[i], selectPad,
								false)==1.0) {
					drawRotatedRect(final, pad, CV_RGB(0, 0, 255));

					outputPosition.x = mc.x;
					outputPosition.y = mc.y;
				} else {
					drawRotatedRect(final, pad, CV_RGB(255, 0, 0));
				}
				padCounter++;
				//cv::circle(final,mc,3,CV_RGB(0,0,255),3);
			}
		}
	}

	cv::Mat out;
	cv::cvtColor(bw, out, CV_GRAY2BGR);

	*input = final.clone();
	return outputPosition;
	//cv::imshow("src", input);
	//cv::imshow("bw", bw);
	//cv::imshow("final", final);
	//cv::imshow("grey", gray);
	//return final;
	//cv::waitKey(0);
	//ROS_INFO("Found %d pads..", padCounter);
}

//void checkFit
