/**
 * OpenDLV - Simulation environment
 * Copyright (C) 2008 - 2015 Christian Berger, Bernhard Rumpe
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

#include <sstream>
#include <string>
#include <iostream>

#include "TrafficData.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

namespace automotive {
namespace miniature {
using namespace std;

TrafficData::TrafficData() :
		m_trafficFactor() {
}

TrafficData::TrafficData(const double &trafficFactor) :
		m_trafficFactor(trafficFactor) {
}

TrafficData::TrafficData(const TrafficData &obj) :
		m_trafficFactor(obj.m_trafficFactor) {
}

TrafficData::~TrafficData() {
}

TrafficData& TrafficData::operator=(const TrafficData &obj) {
	setTrafficFactor(obj.getTrafficFactor());
	return (*this);
}

double TrafficData::getTrafficFactor() const {
	return m_trafficFactor;
}

void TrafficData::setTrafficFactor(const double &trafficFactor) {
	m_trafficFactor = trafficFactor;
}

ostream& TrafficData::operator<<(ostream &out) const {
//  std::cout << "Try out" << std::endl;
	odcore::serialization::SerializationFactory& sf =
			odcore::serialization::SerializationFactory::getInstance();

	std::shared_ptr < odcore::serialization::Serializer > s =
			sf.getQueryableNetstringsSerializer(out);
	s->write(1, m_trafficFactor);
	return out;
}

istream& TrafficData::operator>>(istream &in) {
//  std::cout << "Try in" << std::endl;
	odcore::serialization::SerializationFactory& sf =
			odcore::serialization::SerializationFactory::getInstance();

	std::shared_ptr < odcore::serialization::Deserializer > d =
			sf.getQueryableNetstringsDeserializer(in);

	d->read(1, m_trafficFactor);

	return in;
}

int32_t TrafficData::ID() {
	return 800;
}

int32_t TrafficData::getID() const {
	return TrafficData::ID();
}

const string TrafficData::getShortName() const {
	return "TrafficData";
}

const string TrafficData::getLongName() const {
	return "ksamserver.TrafficData";
}

const string TrafficData::toString() const {
	stringstream sstr;
	sstr << "TrafficData";
	return sstr.str();
}
}

}
