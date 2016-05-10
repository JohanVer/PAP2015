#ifndef STITCHWAYPOINTMAKER_H
#define STITCHWAYPOINTMAKER_H
#include <QVector3D>
#include <QVector2D>
#include <vector>

namespace stitch_waypoint_maker{

unsigned int calcMinImgNumber(unsigned int p_fov, unsigned int px_to_go, unsigned int min_p_overlap);

double calcOverlap(unsigned int num_img, unsigned int p_fov, unsigned int px_to_go);

std::vector<QVector3D> generateWaypoints(const QVector3D &init_point, unsigned int min_p_overlap, const QVector2D &pcb_size, double px_conv, double py_conv, double camera_pcb_height);
}


#endif // STITCHWAYPOINTMAKER_H
