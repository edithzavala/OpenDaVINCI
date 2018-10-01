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

#ifndef TRAFFICDATA_H_
#define TRAFFICDATA_H_

#include <string>
#include "opendavinci/odcore/opendavinci.h"

#include "opendavinci/odcore/data/SerializableData.h"

namespace automotive {
namespace miniature {

using namespace std;

/**
 * This class can be used to produce some objects detected by
 * point providing sensors.
 */
class TrafficData: public odcore::data::SerializableData {
public:
	TrafficData();

	/**
	 * Constructor.
	 *
	 * @param double trafficFactor
	 */
	TrafficData(const double &trafficFactor);

	/**
	 * Copy constructor.
	 *
	 * @param obj Reference to an object of this class.
	 */
	TrafficData(const TrafficData &obj);

	virtual ~TrafficData();

	/**
	 * Assignment operator.
	 *
	 * @param obj Reference to an object of this class.
	 * @return Reference to this instance.
	 */
	TrafficData& operator=(const TrafficData &obj);

	virtual ostream& operator<<(ostream &out) const;
	virtual istream& operator>>(istream &in);

	/**
	 * This method returns the message id.
	 *
	 * @return Message id.
	 */
	static int32_t ID();
	virtual int32_t getID() const;
	const virtual string getShortName() const;
	const virtual string getLongName() const;
	const virtual string toString() const;

	double getTrafficFactor() const;
	void setTrafficFactor(const double &trafficFactor);

private:
	double m_trafficFactor;
};

}
}

#endif
