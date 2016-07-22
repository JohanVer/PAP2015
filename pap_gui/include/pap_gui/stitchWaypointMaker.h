#ifndef STITCHWAYPOINTMAKER_H
#define STITCHWAYPOINTMAKER_H
#include <QVector3D>
#include <QVector2D>
#include <vector>

namespace stitch_waypoint_maker{

//!
//! \brief calcMinImgNumber computes minimum number of images to be taken
//! \param p_fov - field of view of camera
//! \param px_to_go - pixels to go
//! \param min_p_overlap - minimum image overlap
//! \return number of images
//!
unsigned int calcMinImgNumber(unsigned int p_fov, unsigned int px_to_go, unsigned int min_p_overlap);

//!
//! \brief calcOverlap computes overlap between images
//! \param num_img - number of images taken in this stitching process
//! \param p_fov - field of view of camera
//! \param px_to_go - pixels to go
//! \return image overlap in px
//!
double calcOverlap(unsigned int num_img, unsigned int p_fov, unsigned int px_to_go);

//!
//! \brief generateWaypoints which need to be traversed to be able to stitch entire PCB
//! \param init_point - starting coordinate
//! \param min_p_overlap - minimal image overlap
//! \param pcb_size to be stitched
//! \param px_conv
//! \param py_conv
//! \param camera_pcb_height
//! \return vector of 3D points needed
//!
std::vector<QVector3D> generateWaypoints(const QVector3D &init_point, unsigned int min_p_overlap, const QVector2D &pcb_size, double px_conv, double py_conv, double camera_pcb_height);
}

#endif // STITCHWAYPOINTMAKER_H
