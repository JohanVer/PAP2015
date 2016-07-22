/*
 * GerberPadParser.hpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */

#ifndef PAP_GUI_SRC_GERBERPADPARSER_HPP_
#define PAP_GUI_SRC_GERBERPADPARSER_HPP_

#include "DispenserPlanner.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <QPoint>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <QString>
#include <QStandardItemModel>
#include <QTableWidget>
#include <QtAlgorithms>
#include <cmath>
#include "PadView.hpp"
#include <QGraphicsRectItem>
#include <QGraphicsObject>
#include <QRectF>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pap_common/CommonDataClasses.hpp>

//!
//! \brief The GerberPadParser class implements a virtual PCB viewer based on gerber information,
//! including some calibration and manipulation functionalities
//!
class GerberPadParser {
public:
	GerberPadParser();
	virtual ~GerberPadParser();
    
    //!
    //! \brief createPadFromView creates a new padInformation entry based on
    //! the QRect pad argument and stores it in the global padInformationArray
    //! \param pad
    //!
    void createPadFromView(QRectF pad);
    
    //!
    //! \brief parseFloat converts a substring to float
    //! \param line - complete string
    //! \param start index of float in string
    //! \param end index of float in string
    //! \return 
    //!
	float parseFloat(std::string line, std::size_t start, std::size_t end);

    //!
    //! \brief loadFile reads a gerber file of bottom or top pcb layer
    //! \param fileName
    //! \param bottomLayer if true, function assumes bottomLayer loaded, otherwise toplayer
    //!
	void loadFile(std::string fileName,bool bottomLayer);

    //!
    //! \brief parseShapes parses given file and stores all shapes in shapeInformationArray
    //! \param fileName
    //!
	void parseShapes(std::string fileName);

    //!
    //! \brief searchShape searchs for shapeIndex in shapeInformationArray
    //! \param shapeIndex - index to be searched for
    //! \param pad - pointer to padInformation if index found
    //! \return true if shape found, otherwise false
    //!
	bool searchShape(int shapeIndex, PadInformation* pad);

    //!
    //! \brief setSize sets globla PCB size
    //! \param height of PCB
    //! \param width of PCB
    //!
	void setSize(float height, float width);

    //!
    //! \brief renderImage clears passed scene and redraws all pads and dispensed
    //! shapes in new scene with given size, corresponds to pcbSize
    //! \param scene to be clears and redrawed
    //! \param width of new scene
    //! \param height of new scene
    //! \param dispensed_set stores all dots/lines dispensed so far
    //! \return pcbSize
    //!
    QRectF renderImage(QGraphicsScene* scene, int width, int height, std::set<size_t> dispensed_set);

    //!
    //! \brief setTable writes all padInformation available in given table
    //! \param table where padInformation is written to
    //!
	void setTable(QTableWidget* table);

    //!
    //! \brief searchId
    //! \param position searched for
    //! \return pad id containing given position
    //!
    int searchId(QPointF position);

    //!
    //! \brief calibratePads computes coordinate transformations based on two point correspondences
    //! \param local1 point
    //! \param local2 point
    //! \param global1 point
    //! \param global2 point
    //! \param simulationMode, if true, coordinate transformation is simulated
    //! \return
    //!
    float calibratePads(QPointF local1, QPointF local2, QPointF global1, QPointF global2,bool simulationMode);

    //!
    //! \brief setTransformation sets gobal transTransformIntoRobot
    //! \param tf
    //!
    void setTransformation(tf::Transform tf);

    //!
    //! \brief rotatePads transforms all pads on pcb
    //!
	void rotatePads(void);

    //!
    //! \brief transformComponent transforms given component into robot coordinate system
    //! \param componentInformation of component to be transformed
    //!
	void transformComponent(componentEntry *componentInformation);

    //!
    //! \brief getMarkerList
    //! \return pointer to marker array
    //!
	visualization_msgs::MarkerArray* getMarkerList(void);

    //!
    //! \brief transformDispenserInfo transforms given dispense information into robot coordinate system
    //! \param info dispenseInfo to be transformed
    //!
	void transformDispenserInfo(dispenseInfo *info);

    //!
    //! \brief deleteEntry removes a pad entry from global padInformationArray
    //! \param index of entry to be removed
    //!
	void deleteEntry(int index);

    //!
    //! \brief setDispenserInfo updates dispenser variable diameter and perc
    //! \param diameter of currently used nozzle
    //! \param perc of currently used edge distance
    //!
    void setDispenserInfo(double diameter, double perc);

    //!
    //! \brief resets all global variables
    //!
    void reset();

    //!
    //! \brief strToFloat
    //! \param in
    //! \return 
    //!
    float strToFloat(const std::string &in);

    //!
    //! \brief deleteBackground
    //!
    void deleteBackground();

    //!
    //! \brief setBackGround
    //! \param background
    //!
    void setBackGround(QImage background){
        background_ = background;
    }

	std::vector<PadInformation> padInformationArray_,padInformationArrayPrint_;
	double pixelConversionFactor;
	float height_, width_,outerRectRot_;
	int heightPixel_;

private:
	std::vector<ShapeInformation> shapeInformationArray_;
	std::vector<QRectF> printedRects;
	QRectF pcbSize;

	tf::Transform transformIntoGlobalPoint_, transTransformIntoRobot_,rotation_;
	float differenceAngle_;
	visualization_msgs::MarkerArray markerArray;
	bool bottomLayer_;

    QImage background_;
    QGraphicsPixmapItem*  background_item_;

    double current_nozzle_diameter_;
    double current_perc_edge_dist_;
    unsigned int pad_counter_;
};

#endif /* PAP_GUI_SRC_GERBERPADPARSER_HPP_ */
