/*
 * DispenserPlanner.hpp
 *
 *  Created on: 24.07.2015
 *      Author: johan
 */

#ifndef PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_
#define PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_

#include <pap_common/CommonDataClasses.hpp>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace DispenserPlanner{

//!
//! \brief isPadCompatibleToNozzle checks if selected pad is dispensable with current nozzle
//! \param pad - selected pad reference
//! \param nozzle_diameter of currently used nozzle
//! \param percentage_edge_dist - minimal distance to pad edge
//! \return
//!
bool isPadCompatibleToNozzle(const PadInformation &pad, double nozzle_diameter, double percentage_edge_dist);

//!
//! \brief The DOT_ALIGN enum
//!
enum DOT_ALIGN{
    CENTER,
    LEFT,
    RIGHT
};

//!
//! \brief The PLANNER_SELECT enum
//!
enum PLANNER_SELECT{
    LINE_PLANNER,
    DOT_PLANNER
};


//!
//! \brief The DotPlanner class implements planning functionalities for
//! dispensing a single pad with dots
//!
class DotPlanner{
public:
    DotPlanner();
    virtual ~DotPlanner();

    //!
    //! \brief planDispensing plans dot dispensing for one given pad
    //! \param padInfo
    //! \param nozzleDiameter of currently used tip
    //! \param percentage_edge_dist minimal distance to pad edge
    //! \param wait_time time to be dispensed for on dot
    //! \param alpha
    //! \param alignment of dots: left, center, right
    //! \return list of all dispensing tasks to be performed for this pad, including: Start/Stop positions , Velocity/Time
    //!
    std::vector<dispenseInfo> planDispensing(PadInformation padInfo, float nozzleDiameter, double percentage_edge_dist = 0.1, double wait_time = 1, double alpha =0.5, enum DOT_ALIGN alignment = DOT_ALIGN::RIGHT);
};

//!
//! \brief The DispenserPlanner class implements planning functionalities for
//! dispensing a single pad with a line/multiple lines
//!
class DispenserPlanner {
public:
	DispenserPlanner();
	virtual ~DispenserPlanner();

    //!
    //! \brief planDispensing plans line dispensing if one given pad
    //! \param padInfo
    //! \param nozzleDiameter of currently used nozzle in mm
    //! \param percentage_edge_dist percentage of pad size used as distances to pad edge
    //! \param velocity for dispending a line
    //! \param wait_time
    //! \return list of all dispense actions needed to dispense given pad, including: Start/Stop positions , Velocity/Time
    //!
    std::vector<dispenseInfo> planDispensing(PadInformation padInfo, float nozzleDiameter, double percentage_edge_dist = 0.1, double velocity = 1, double wait_time = 1);

};

}
#endif /* PAP2015_PAP_GUI_SRC_DISPENSERPLANNER_HPP_ */
