#include <pap_gui/stitchWaypointMaker.h>

namespace stitch_waypoint_maker{
	
	unsigned int calcMinImgNumber(unsigned int p_fov, unsigned int px_to_go, unsigned int min_p_overlap){
		return std::ceil((double)(px_to_go - min_p_overlap) / (double)(p_fov - min_p_overlap));
	}
	
	double calcOverlap(unsigned int num_img, unsigned int p_fov, unsigned int px_to_go){
        return ((double)(num_img * p_fov) - (double)px_to_go) / ((double)num_img - 1.0);
	}
	
    std::vector<QVector3D> generateWaypoints(const QVector3D &init_point, unsigned int min_p_overlap, const QVector2D &pcb_size, double px_conv, double py_conv, double camera_pcb_height){
		const unsigned int px_fov = 640;
        const unsigned int py_fov = 480;
		
        const unsigned int px_size = std::ceil(pcb_size.x() * px_conv);
        const unsigned int py_size = std::ceil(pcb_size.y() * py_conv);
		
		unsigned int images_x = calcMinImgNumber(px_fov, px_size, min_p_overlap);
		unsigned int images_y = calcMinImgNumber(py_fov, py_size, min_p_overlap);
		
        double px_overlap = calcOverlap(images_x, px_fov, px_size);
        double py_overlap = calcOverlap(images_y, py_fov, py_size);
		
		double step_x_mm = ((double)px_fov - px_overlap) / px_conv;
		double step_y_mm = ((double)py_fov - py_overlap) / py_conv;
		
		const double x_fov = px_fov / px_conv;
		const double y_fov = py_fov / py_conv;
		
        std::vector<QVector3D> waypoints;
		
		// Generate waypoints
		
		for(size_t n_y = 0; n_y < images_y; n_y++){
			for(size_t n_x = 0; n_x < images_x; n_x++){
				
                double x_pos = (x_fov/2.0) + step_x_mm * n_x;
                double y_pos = (y_fov/2.0) + step_y_mm * n_y;
				
                QVector3D p = init_point - QVector3D(x_pos, y_pos, camera_pcb_height);
				waypoints.push_back(p);
			}
			if(n_y + 1 < images_y){
				n_y++;
			for(size_t n_x = images_x-1; n_x >= 0;  n_x--){
				
				double x_pos = (x_fov/2.0) + step_x_mm * n_x;
				double y_pos = (y_fov/2.0) + step_y_mm * n_y;
				
                QVector3D p = init_point - QVector3D(x_pos, y_pos, camera_pcb_height);
				waypoints.push_back(p);
			}
			}
		}
		
		return waypoints;
		
		
	}
	
}
