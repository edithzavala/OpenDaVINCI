/**
 * irus - Distance data generator (part of simulation environment)
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef MONITORADAPTATION_H_
#define MONITORADAPTATION_H_

#include <string>
#include "opendavinci/odcore/opendavinci.h"

#include "opendavinci/odcore/data/SerializableData.h"

namespace irus {

    using namespace std;

    /**
     * This class can be used to produce some objects detected by
     * point providing sensors.
     */
class MonitorAdaptation: public odcore::data::SerializableData {
  public:
    MonitorAdaptation();

    /**
     * Constructor.
     *
     * @param position Position.
     * @param rotation Rotation.
     * @param velocity Velocity.
     * @param acceleration Acceleration.
     */
    MonitorAdaptation(const int32_t &vehicleId, const string &monitorName,
            const string &action);

    /**
     * Copy constructor.
     *
     * @param obj Reference to an object of this class.
     */
    MonitorAdaptation(const MonitorAdaptation &obj);

    virtual ~MonitorAdaptation();

    /**
     * Assignment operator.
     *
     * @param obj Reference to an object of this class.
     * @return Reference to this instance.
     */
    MonitorAdaptation& operator=(const MonitorAdaptation &obj);

    virtual ostream& operator<<(ostream &out) const;
    virtual istream& operator>>(istream &in);

    /**
     * This method returns the message id.
     *
     * @return Message id.
     */
    static int32_t ID();
    virtual int32_t getID() const;
    virtual const string getShortName() const;
    virtual const string getLongName() const;
    virtual const string toString() const;

    int32_t getVehicleId() const;
    string getMonitorName() const;
    string getAction() const;
    void setVehicleId(const int32_t &vehicleId);
    void setMonitorName(const string &monitorName);
    void setAction(const string &action);

  private:
    int32_t m_vehicleId;
    string m_monitorName;
    string m_action;
};

}

#endif
