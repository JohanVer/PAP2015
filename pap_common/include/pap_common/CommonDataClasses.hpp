/*
 * CommonDataClasses.hpp
 *
 *  Created on: 30.07.2015
 *      Author: johan
 */

#ifndef COMMONDATACLASSES_HPP_
#define COMMONDATACLASSES_HPP_

#include <string>
#include <QRectF>
#include <QString>
#include "pap_common/Task.h"
#include "pap_common/Status.h"
#include <pap_common/task_message_def.h>
#include <pap_common/status_message_def.h>
#include <pap_common/vision_message_def.h>

namespace dispenser_types {
enum DispenserType{
    DOT_DISPENSE,
    LINE_DISPENSE,
    NOT_DISPENSE
};

enum DispenserTask{
    INIT,
    DISPENSING
};
}

enum TIP {
    LEFT_TIP = 1,
    RIGHT_TIP = 2
};

class databaseEntry {
public:
    QString package;
    float length, width, height;
    int pins;
    databaseEntry(){
        length = 0.0;
        width = 0.0;
        height = 0.0;
        pins = 0;
    }
};

class PartEntry{
public:
    std::string package, value;
    int count, slot;
    PartEntry(){
        count = 0;
        slot = 0;
    }
};

class dispenseInfos{
public:
    dispenseInfos(){
        xPos = 0.0;
        yPos = 0.0;
        xPos2 = 0.0;
        yPos2 = 0.0;
        type = Point;
        rotation = 0.0;
        velocity = 0.0;
        time = 0.0;
    }

    enum dispensePadType {
        Point, Long
    };

    int type;
    float xPos, yPos, xPos2, yPos2;
    float rotation;
    float velocity;
    float time;
};

class ComponentPlacerData {
public:
    float destX;
    float destY;
    float rotation;
    float length, width, height;
    int box;
    float tapeX, tapeY, tapeRot;
    bool isWaiting;
    std::string name;
    pap_vision::VISION finderType;


    ComponentPlacerData() {
        destX = 0.0;
        destY = 0.0;
        rotation = 0.0;
        length = 0.0;
        width = 0.0;
        height = 0.0;
        box = 0;
        tapeX = 0.0;
        tapeY = 0.0;
        tapeRot = 0.0;
        name = "-";
        isWaiting = false;
    }
private:
};

class Offset {
public:

//    Offset() {
//        x = 0.0;
//        y = 0.0;
//        z = 0.0;
//    }

    Offset operator+(const Offset& b)
    {
        Offset out;
        out.x = this->x + b.x;
        out.y = this->y + b.y;
        out.z = this->z + b.z;
        out.rot = this->rot + b.rot;
        return out;
    }

    double x;
    double y;
    double z;
    double rot;

private:
};





class componentEntry {
public:
    std::string name, package, side, value;
    float posX, posY, length, width, height;
    int box, rotation, pins, index;
};

class dispenseInfo{
public:
    dispenseInfo(){
        xPos = 0.0;
        yPos = 0.0;
        xPos2 = 0.0;
        yPos2 = 0.0;
        type = Point;
        rotation = 0.0;
        velocity = 0.0;
        time = 0.0;
    }

    enum dispensePadType {
        Point, Long
    };

    int type;
    float xPos, yPos, xPos2, yPos2;
    float rotation;
    float velocity;
    float time;
};

class PadInformation {
public:
    PadInformation() {
        rotation = 0.0;
        id = 0;
    }
    QRectF rect;
    std::string shapeStr;
    float rotation;
    bool dispensed;
    unsigned int id;
};

class ShapeInformation {
public:
    ShapeInformation() {
        shapeIndex = 0;
        shapeStr = "";
        rotation = 0.0;
    }
    int shapeIndex;
    std::string shapeStr;
    QPointF padDimensions;
    float rotation;
};

#endif /* COMMONDATACLASSES_HPP_ */
