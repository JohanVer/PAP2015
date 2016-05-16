/*
 * GerberPadParser.cpp
 *
 *  Created on: Jul 5, 2015
 *      Author: johan
 */

#define CHANGE_SHAPE 54
#define ENDOFFILE_GERBER 02
#include <pap_gui/GerberPadParser.hpp>

GerberPadParser::GerberPadParser() {
    height_ = 0.0;
    width_ = 0.0;
    outerRectRot_ = 0.0;
    pcbSize.setX(0.0);
    pcbSize.setY(0.0);
    differenceAngle_ = 0.0;

    // Initial Transform for simulation
    transTransformIntoRobot_.setOrigin(tf::Vector3(300.0, 150.0, 0.0));
    transTransformIntoRobot_.setRotation(tf::Quaternion(0, 0, 0, 1));

    transformIntoGlobalPoint_.setOrigin(tf::Vector3(0, 0, 0));
    transformIntoGlobalPoint_.setRotation(tf::Quaternion(0, 0, 0, 1));

    rotation_.setOrigin(tf::Vector3(0, 0, 0));
    rotation_.setRotation(tf::Quaternion(0, 0, 0, 1));
}

GerberPadParser::~GerberPadParser() {
    if(background_item_){
        delete background_item_;
        background_item_ = NULL;
    }
}

float GerberPadParser::strToFloat(const std::string &in){
    QString qs = QString(in.c_str());
    return qs.toFloat();
}

void GerberPadParser::parseShapes(std::string fileName){
    std::vector <std::vector <std::string> > data;
    std::ifstream infile(fileName.c_str());

    while (infile)
    {
        std::string s;
        if (!getline( infile, s )) break;
        std::istringstream ss( s );
        std::vector <std::string> record;

        while (ss)
        {
            std::string s;
            if (!getline( ss, s, ' ' )) break;
            if(s.size() && s != "x")
                record.push_back( s );
        }

        data.push_back( record );
    }
    if (!infile.eof())
    {
        std::cerr << "Fooey!\n";
    }


    shapeInformationArray_.clear();

    for(size_t i = 0; i < data.size(); i++){
        int dcode = 0;
        if(((data.at(i)).at(0)).substr(0,1) == "D"){
            dcode =  std::stoi(((data.at(i)).at(0)).substr(1,((data.at(i)).at(0)).size()));
            std::cout << "DCode: " << dcode << std::endl;
            std::string shape_str = ((data.at(i)).at(1));

            ShapeInformation shape;
            if(shape_str == "rectangle" || shape_str =="oval"){
                double width = strToFloat(data.at(i).at(2));
                double height = strToFloat((data.at(i)).at(3));
                double rot = 0;
                if(4 < data.at(i).size()){
                    rot = strToFloat((data.at(i)).at(4));
                }
                shape.shapeIndex = dcode;
                shape.shapeStr = shape_str;
                shape.padDimensions.setY(width * 25.4);
                shape.padDimensions.setX(height * 25.4);
                shape.rotation = rot;
                shapeInformationArray_.push_back(shape);

                std::cout << "Detected rectangle with: " << width << " / " << height << " / " << rot << std::endl;
            }
            else if(shape_str == "square"){
                double lenght = strToFloat((data.at(i)).at(2));
                double rot = 0;
                if(3 < data.at(i).size()){
                    rot = strToFloat((data.at(i)).at(3));
                }
                shape.shapeIndex = dcode;
                shape.shapeStr = shape_str;
                shape.padDimensions.setY(lenght * 25.4);
                shape.padDimensions.setX(lenght * 25.4);
                shape.rotation = rot;
                shapeInformationArray_.push_back(shape);

                std::cout << "Detected square with: " << lenght << " / " << rot << std::endl;
            }/*
            else if(shape_str == "roundrect"){
                double width = strToFloat((data.at(i)).at(2));
                double height = strToFloat((data.at(i)).at(3));
                double rot = 0;
                if(5 < data.at(i).size()){
                    rot = strToFloat((data.at(i)).at(5));
                }
                shape.shapeIndex = dcode;
                shape.shapeStr = shape_str;
                shape.padDimensions.setY(width * 25.4);
                shape.padDimensions.setX(height * 25.4);
                shape.rotation = rot;
                shapeInformationArray_.push_back(shape);

                std::cout << "Detected roundrect with: " << width << " / " << height << " / " << rot << std::endl;
            }
            */
        }

        std::cerr << std::endl;
    }
}

void GerberPadParser::parseShapes2(std::string fileName) {
    std::ifstream infile;
    shapeInformationArray_.clear();
    infile.open(fileName.c_str(), std::ios::in);
    std::string line;
    while (std::getline(infile, line)) {
        if (line[0] == '%') {
            continue;
        }

        // D-Codes
        if (line[0] == 'D') {
            std::size_t emptyFound = line.find(" ");
            if (emptyFound != std::string::npos) {
                QString* dCodeIndexStr = new QString(
                            line.substr(1, emptyFound - 1).c_str());
                int dCode = dCodeIndexStr->toInt();
                //ROS_INFO("D-Code : %d ", dCode);
                std::size_t wordFound = line.find_first_not_of(" ", emptyFound);
                if (wordFound != std::string::npos) {
                    std::size_t emptyFound2 = line.find(" ", wordFound);
                    if (emptyFound2 != std::string::npos) {

                        // String
                        std::string shapeStr = line.substr(wordFound,
                                                           emptyFound2 - wordFound);
                        //ROS_INFO("Shapestr : %s", shapeStr.c_str());

                        if (shapeStr == "square") {
                            size_t xFound = line.find("x", emptyFound2 + 1);
                            if (xFound != std::string::npos) {

                                // Arguments
                                QString* firstArgStr = new QString(
                                            line.substr(emptyFound2,
                                                        xFound - emptyFound2).c_str());
                                float firstArg = firstArgStr->toFloat();
                                QString* secondArgStr =
                                        new QString(
                                            line.substr(xFound + 1,
                                                        line.size()
                                                        - (xFound + 1)).c_str());
                                float secondArg = secondArgStr->toFloat();
                                ROS_INFO(
                                            "D-Code: %d Square X: %f, Rotation: %f",
                                            dCode, firstArg, secondArg);
                                ShapeInformation shape;
                                shape.shapeIndex = dCode;
                                shape.shapeStr = shapeStr;
                                shape.padDimensions.setX(firstArg * 25.4);
                                shape.padDimensions.setY(firstArg * 25.4);
                                shape.rotation = secondArg;
                                shapeInformationArray_.push_back(shape);
                            } else {
                                // Arguments
                                QString* firstArgStr = new QString(
                                            line.substr(emptyFound2,
                                                        xFound - emptyFound2).c_str());
                                float firstArg = firstArgStr->toFloat();

                                ROS_INFO("D-Code: %d Square X: %f, Rotation: 0",
                                         dCode, firstArg);
                                ShapeInformation shape;
                                shape.shapeIndex = dCode;
                                shape.shapeStr = shapeStr;
                                shape.padDimensions.setX(firstArg * 25.4);
                                shape.padDimensions.setY(firstArg * 25.4);
                                shape.rotation = 0.0;
                                shapeInformationArray_.push_back(shape);
                            }
                        }

                        else if (shapeStr == "draw") {
                            QString* drawArgStr = new QString(
                                        line.substr(emptyFound2,
                                                    line.size() - emptyFound2).c_str());
                            float drawArg = drawArgStr->toFloat();
                            //ROS_INFO("Arg : %f", drawArg);
                            ShapeInformation shape;
                            shape.shapeIndex = dCode;
                            shape.shapeStr = shapeStr;
                            shape.padDimensions.setX(drawArg);
                            shapeInformationArray_.push_back(shape);
                        }

                        else if (shapeStr == "rectangle"
                                 || shapeStr == "oval") {
                            size_t xFound = line.find("x", emptyFound2 + 1);
                            float secondArg = 0.0;
                            float thirdArg = 0.0;

                            if (xFound != std::string::npos) {

                                // Arguments
                                QString* firstArgStr = new QString(
                                            line.substr(emptyFound2,
                                                        xFound - emptyFound2).c_str());
                                float firstArg = firstArgStr->toFloat();

                                size_t xFound2 = line.find("x", xFound + 1);
                                if (xFound2 != std::string::npos) {

                                    QString* secondArgStr =
                                            new QString(
                                                line.substr(xFound + 1,
                                                            xFound2
                                                            - (xFound
                                                               + 1)).c_str());
                                    secondArg = secondArgStr->toFloat();

                                    QString* thirdArgStr =
                                            new QString(
                                                line.substr(xFound2 + 1,
                                                            line.size()
                                                            - (xFound2
                                                               + 1)).c_str());
                                    thirdArg = thirdArgStr->toFloat();

                                } else {
                                    QString* secondArgStr =
                                            new QString(
                                                line.substr(xFound + 1,
                                                            line.size()
                                                            - (xFound
                                                               + 1)).c_str());
                                    secondArg = secondArgStr->toFloat();
                                }
                                ROS_INFO(
                                            "D-Code: %d Rectangle X: %f, Y: %f, Rotation %f",
                                            dCode, firstArg, secondArg, thirdArg);
                                ShapeInformation shape;
                                shape.shapeIndex = dCode;
                                shape.shapeStr = shapeStr;
                                shape.padDimensions.setY(firstArg * 25.4);
                                shape.padDimensions.setX(secondArg * 25.4);
                                shape.rotation = thirdArg;
                                shapeInformationArray_.push_back(shape);
                            }
                        }
                    }
                }
            }
        }
    }
}

float GerberPadParser::parseFloat(std::string line, std::size_t start,
                                  std::size_t end) {
    int numberLength = end - start - 1;
    std::string floatStr;
    if (numberLength <= 5) {
        int zeros = 5 - numberLength;
        floatStr = "0.";
        for (int i = 0; i < zeros; i++) {
            floatStr = floatStr + "0";
        }
        floatStr = floatStr + line.substr(start + 1, numberLength);
    } else {
        floatStr = line.substr(start + 1, (numberLength - 5)) + "."
                + line.substr(start + 1 + (numberLength - 5), 5);
    }
    QString* qtstring = new QString(floatStr.c_str());

    return qtstring->toFloat();
}

bool GerberPadParser::searchShape(int shapeIndex, PadInformation* pad) {
    for (int i = 0; i < shapeInformationArray_.size(); i++) {
        if (shapeInformationArray_[i].shapeIndex == shapeIndex) {
            pad->rect.setWidth(shapeInformationArray_[i].padDimensions.x());
            pad->rect.setHeight(shapeInformationArray_[i].padDimensions.y());
            pad->rotation = shapeInformationArray_[i].rotation;
            pad->shapeStr = shapeInformationArray_[i].shapeStr;
            return true;
        }
    }
    ROS_ERROR("Required Shape not found %d", shapeIndex);
    return false;
}

void GerberPadParser::setTable(QTableWidget* table) {

    table->setRowCount(padInformationArray_.size());
    QStringList vLabels;
    for (int i = 1; i <= padInformationArray_.size(); i++) {
        vLabels << QString::number(i);
    }
    table->setVerticalHeaderLabels(vLabels);
    for (std::size_t i = 0; i < padInformationArray_.size(); i++) {
        PadInformation pad;
        pad = padInformationArray_[i];
        table->setItem(i, 0,
                       new QTableWidgetItem(QString::number(pad.rect.x())));

        table->setItem(i, 1,
                       new QTableWidgetItem(QString::number(pad.rect.y())));

        table->setItem(i, 2,
                       new QTableWidgetItem(QString::number(pad.rect.width())));

        table->setItem(i, 3,
                       new QTableWidgetItem(QString::number(pad.rect.height())));

        table->setItem(i, 4,
                       new QTableWidgetItem(QString::number(pad.rotation)));

        table->setItem(i, 5,
                       new QTableWidgetItem(QString::fromStdString(pad.shapeStr)));
    }
}

void GerberPadParser::loadFile(std::string fileName, bool bottomLayer) {

    std::ifstream infile;
    infile.open(fileName.c_str(), std::ios::in);
    padInformationArray_.clear();
    padInformationArrayPrint_.clear();

    // Outlines are part of the first entry!

    PadInformation outerRect;
    outerRect.rect.setY(width_ / 2 + pcbSize.x());
    outerRect.rect.setX(height_ / 2 + pcbSize.y());
    outerRect.rotation = outerRectRot_;
    outerRect.rect.setHeight(width_);
    outerRect.rect.setWidth(height_);
    padInformationArray_.push_back(outerRect);
    padInformationArrayPrint_.push_back(outerRect);

    float xParsed, yParsed = 0.0;
    std::string line;
    int gCode, dCodeShape = 0;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        if (line[0] == '%') {
            continue;
        }

        if (line[0] == 'G') {
            // G-code
            QString* gCodeIndexStr = new QString(line.substr(1, 2).c_str());
            gCode = gCodeIndexStr->toInt();
            // D-Code
            std::size_t dFound = line.find("D");
            if (dFound != std::string::npos) {
                std::size_t endFound = line.find("*");
                if (endFound != std::string::npos) {
                    QString* dCodeIndexStr =
                            new QString(
                                line.substr(dFound + 1,
                                            endFound - (dFound + 1)).c_str());
                    if (gCode == CHANGE_SHAPE) {
                        dCodeShape = dCodeIndexStr->toInt();
                    }
                    ROS_INFO("D-Code %d ", dCodeShape);
                }
            }
            continue;
        }

        // Misc codes
        if (line[0] == 'M') {
            QString* mCodeIndexStr = new QString(line.substr(1, 2).c_str());
            int mCodeIndex = mCodeIndexStr->toInt();
            if (mCodeIndex == ENDOFFILE_GERBER) {
                ROS_INFO("End of file reached");
                break;
            }
            continue;
        }

        std::size_t xFound = line.find("X");
        if (xFound != std::string::npos) {
            std::size_t yFound = line.find("Y");
            if (yFound != std::string::npos) {
                xParsed = parseFloat(line, xFound, yFound);
            }
            std::size_t dFound = line.find("D");
            if (dFound != std::string::npos) {
                yParsed = parseFloat(line, yFound, dFound);
                PadInformation newPad;

                bottomLayer_ = bottomLayer;
                newPad.rect.setX((yParsed * 25.4));
                if (bottomLayer) {
                    ROS_INFO("Bottom layer detected");
                    newPad.rect.setY((xParsed * 25.4));

                } else {
                    ROS_INFO("Top layer detected");
                    newPad.rect.setY((xParsed * 25.4));
                }
                //newPad.rect.setY(yParsed * 25.4);
                // Convert d-code using tabular out of whl file
                if (searchShape(dCodeShape, &newPad)) {
                    padInformationArray_.push_back(newPad);
                    padInformationArrayPrint_.push_back(newPad);
                    ROS_INFO(
                                "Shape: %d , X-pos: %f , Y-pos: %f, X-Size: %f, Y-Size: %f, Rotation %f, Type: %s  ",
                                dCodeShape, newPad.rect.x(), newPad.rect.y(),
                                newPad.rect.width(), newPad.rect.height(),
                                newPad.rotation, newPad.shapeStr.c_str());
                }
            }
        }
    }
    ROS_INFO("Parsed %d pads...", (int )padInformationArray_.size());
}

void GerberPadParser::setSize(float height, float width) {
    width_ = width;
    height_ = height;
}

void GerberPadParser::deleteBackground(){
    if(!background_.isNull()){
        background_ = QImage();
    }
}

void GerberPadParser::createPadFromView(QRectF pad){
    QRectF tf;
    tf.setTopLeft(pad.topLeft());
    tf.setWidth(pad.width());
    tf.setHeight(pad.height());

    PadInformation newPad;

    newPad.rect.setX(((double)-tf.center().y()) / pixelConversionFactor);
    newPad.rect.setY((double)tf.center().x() / pixelConversionFactor);
    newPad.rect.setHeight((double)tf.width() / pixelConversionFactor);
    newPad.rect.setWidth((double)tf.height() / pixelConversionFactor);
    newPad.rotation = 0;

    padInformationArray_.push_back(newPad);
    padInformationArrayPrint_.push_back(newPad);

}

QRectF GerberPadParser::renderImage(QGraphicsScene* scene, int width,
                                    int height) {
    //gen_data_.clear();
    printedRects.clear();
    qDeleteAll(scene->items());

    if(!background_.isNull()){
        std::cerr << "Inserting background...\n";
        background_item_ = new QGraphicsPixmapItem(QPixmap::fromImage(background_));
        background_item_->setPos(-width,0);
        scene->addItem(background_item_);
    }

    std::cerr << "Pixel conv factor : " << pixelConversionFactor << std::endl;

    // Draw Pads
    for (std::size_t i = 0; i < padInformationArrayPrint_.size(); i++) {
        QRectF pad;
        PadInformation padInfo;

        padInfo.rect.setX(padInformationArrayPrint_[i].rect.y());
        padInfo.rect.setY(-padInformationArrayPrint_[i].rect.x());
        padInfo.rect.setWidth(padInformationArrayPrint_[i].rect.height());
        padInfo.rect.setHeight(padInformationArrayPrint_[i].rect.width());
        padInfo.rotation = padInformationArrayPrint_[i].rotation;

        double upperCornerPadX =
                ((padInfo.rect.x() - padInfo.rect.width() / 2.0)
                 * pixelConversionFactor);
        double upperCornerPadY = ((padInfo.rect.y() - padInfo.rect.height() / 2.0)
                                  * pixelConversionFactor);

        pad.setX(upperCornerPadX);
        pad.setY(upperCornerPadY);

        pad.setWidth(
                    (padInfo.rect.width() * pixelConversionFactor));
        pad.setHeight(
                    (padInfo.rect.height() * pixelConversionFactor));

        printedRects.push_back(pad);
        //std::shared_ptr<QGraphicsRectItem> rect = std::shared_ptr<QGraphicsRectItem>(new QGraphicsRectItem(pad.x(), pad.y(), pad.width(), pad.height()));
        QGraphicsRectItem* rect =  new QGraphicsRectItem(pad.x(), pad.y(), pad.width(), pad.height());
        //gen_data_.push_back(rect);
        rect->setPen(QPen(Qt::red, 1, Qt::SolidLine));
        if (i != 0) {
            rect->setBrush(Qt::red);
        } else {
            rect->setBrush(Qt::green);
        }
        rect->setTransformOriginPoint(padInfo.rect.x() * pixelConversionFactor,
                                      (padInfo.rect.y() * pixelConversionFactor));
        rect->setRotation(padInfo.rotation);

        scene->addItem(rect);
    }

    return pcbSize;
}

int GerberPadParser::searchId(QPointF position, int height) {
    QPointF convPoint;
    convPoint.setX(position.x());
    convPoint.setY(position.y());
    // First rect is outline
    for (std::size_t i = 0; i < printedRects.size(); i++) {
        if (printedRects[i].contains(convPoint)) {
            return i;
        }
    }
    return -1;
}

void GerberPadParser::deleteEntry(int index) {
    if (index < padInformationArray_.size())
        padInformationArray_.erase(padInformationArray_.begin() + index);
    if (index < printedRects.size())
        printedRects.erase(printedRects.begin() + index);
    if (index < padInformationArrayPrint_.size())
        padInformationArrayPrint_.erase(
                    padInformationArrayPrint_.begin() + index);
}

visualization_msgs::MarkerArray* GerberPadParser::getMarkerList(void) {
    markerArray.markers.clear();

    float xCameraOffset = 69.45;
    float yCameraOffset = 31.66;

    for (std::size_t i = 0; i < padInformationArray_.size(); i++) {
        PadInformation padInfo;
        padInfo = padInformationArray_[i];
        // Create Marker

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.id = i + 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = ((padInfo.rect.y() + yCameraOffset) * 0.001)
                - 0.11097;
        marker.pose.position.y = ((padInfo.rect.x() + xCameraOffset) * 0.001)
                + 0.14813;
        if (i != 0) {
            marker.pose.position.z = 0.061;
        } else {
            marker.pose.position.z = 0.06;
        }
        tf::Quaternion rotQuat;
        rotQuat.setEuler(0.0, 0.0, -padInfo.rotation * (M_PI / 180.0));
        marker.pose.orientation.w = rotQuat.getW();
        marker.pose.orientation.x = rotQuat.getX();
        marker.pose.orientation.y = rotQuat.getY();
        marker.pose.orientation.z = rotQuat.getZ();

        marker.scale.y = padInfo.rect.width() * 0.001;
        marker.scale.x = padInfo.rect.height() * 0.001;
        marker.scale.z = 1 * 0.001;
        if (i != 0) {
            marker.scale.z = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.scale.z = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        markerArray.markers.push_back(marker);

    }
    return &markerArray;
}

float GerberPadParser::calibratePads(QPointF local1, QPointF local2,
                                     QPointF global1, QPointF global2, bool simulationMode) {

    ROS_INFO("In calibration routine!");
    // Local = Robot reference coordinate system
    // Global = PCB coordinate system
    float fixedAngle = atan2(global2.y() - global1.y(),
                             global2.x() - global1.x());
    float cvAngle = atan2(local2.y() - local1.y(), local2.x() - local1.x());

    if (simulationMode) {
        differenceAngle_ = 0.0;
    } else {
        differenceAngle_ = fixedAngle - cvAngle;
    }
    ROS_INFO("Calibration : Difference Angle %f",
             differenceAngle_*(180.0/M_PI));

    // This calculates the translation of the pads in the global pad frame
    transformIntoGlobalPoint_.setOrigin(
                tf::Vector3(-global1.x(), -global1.y(), 0.0));
    transformIntoGlobalPoint_.setRotation(tf::Quaternion(0, 0, 0, 1));

    // This rotates the pads in the global pad frame
    tf::Quaternion rotQuat;
    rotQuat.setEuler(0.0, 0.0, -differenceAngle_);
    rotation_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    rotation_.setRotation(rotQuat);

    // This transforms the pads into the robot frame
    transTransformIntoRobot_.setOrigin(
                tf::Vector3(local1.x() - global1.x(), local1.y() - global1.y(),
                            0.0));
    transTransformIntoRobot_.setRotation(tf::Quaternion(0, 0, 0, 1));
}

void GerberPadParser::setTransformation(tf::Transform tf){
    transTransformIntoRobot_ = tf;
}

void GerberPadParser::rotatePads(void) {
    // The postitions of the pad are stored in it's own reference coordinate system
    // 1. Rotate the pad positions around the global1 point about the angle which is
    // calculated from the local positions
    // 2. Transform all points of the global pad coordinate system into the local robot
    // coordinate system

    tf::Transform backTransform = transformIntoGlobalPoint_;
    for (size_t i = 0; i < padInformationArray_.size(); i++) {

        tf::Point pointToTransform;
        // This point should be transformed
        pointToTransform.setX(padInformationArray_[i].rect.x());
        pointToTransform.setY(padInformationArray_[i].rect.y());
        pointToTransform.setZ(0.0);

        float width, height = 0.0;
        width = padInformationArray_[i].rect.width();
        height = padInformationArray_[i].rect.height();

        pointToTransform = transformIntoGlobalPoint_ * pointToTransform;
        pointToTransform = rotation_ * pointToTransform;
        pointToTransform = backTransform.inverse() * pointToTransform;
        pointToTransform = transTransformIntoRobot_ * pointToTransform;

        padInformationArray_[i].rect.setX(pointToTransform.x());
        padInformationArray_[i].rect.setY(pointToTransform.y());
        padInformationArray_[i].rect.setWidth(width);
        padInformationArray_[i].rect.setHeight(height);
        padInformationArray_[i].rotation = padInformationArray_[i].rotation
                - (differenceAngle_ * (180.0 / M_PI));
    }

    ROS_INFO("Calibration: Transformed %d pads...",
             (int )padInformationArray_.size());
}

void GerberPadParser::transformDispenserInfo(dispenseInfo *info) {
    tf::Transform backTransform = transformIntoGlobalPoint_;
    tf::Point pointToTransform, pointToTransform2;
    // This point should be transformed
    pointToTransform.setX(info->xPos);
    pointToTransform.setY(info->yPos);
    pointToTransform.setZ(0.0);

    pointToTransform2.setX(info->xPos2);
    pointToTransform2.setY(info->yPos2);
    pointToTransform2.setZ(0.0);

    pointToTransform = transformIntoGlobalPoint_ * pointToTransform;
    pointToTransform = rotation_ * pointToTransform;
    pointToTransform = backTransform.inverse() * pointToTransform;
    pointToTransform = transTransformIntoRobot_ * pointToTransform;

    pointToTransform2 = transformIntoGlobalPoint_ * pointToTransform2;
    pointToTransform2 = rotation_ * pointToTransform2;
    pointToTransform2 = backTransform.inverse() * pointToTransform2;
    pointToTransform2 = transTransformIntoRobot_ * pointToTransform2;

    info->xPos = pointToTransform.x();
    info->yPos = pointToTransform.y();
    info->xPos2 = pointToTransform2.x();
    info->yPos2 = pointToTransform2.y();
}

void GerberPadParser::transformComponent(componentEntry *componentInformation) {

    tf::Point pointToTransform;
    // This point should be transformed
    if (bottomLayer_) {
        pointToTransform.setX(height_ - componentInformation->posY);
    } else {
        pointToTransform.setX(componentInformation->posY);
    }
    pointToTransform.setY(componentInformation->posX);
    pointToTransform.setZ(0.0);

    pointToTransform = transformIntoGlobalPoint_ * pointToTransform;
    pointToTransform = rotation_ * pointToTransform;
    pointToTransform = transformIntoGlobalPoint_.inverse() * pointToTransform;
    pointToTransform = transTransformIntoRobot_ * pointToTransform;
    componentInformation->posX = pointToTransform.x();
    componentInformation->posY = pointToTransform.y();

}

