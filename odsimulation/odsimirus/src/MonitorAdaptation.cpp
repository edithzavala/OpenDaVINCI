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

#include "MonitorAdaptation.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

namespace irus {
using namespace std;

MonitorAdaptation::MonitorAdaptation() :
        m_vehicleId(), m_monitorName(), m_action() {
}

MonitorAdaptation::MonitorAdaptation(const int32_t &vehicleId,
        const string &monitorName, const string &action) :
        m_vehicleId(vehicleId), m_monitorName(monitorName), m_action(action) {
}

MonitorAdaptation::MonitorAdaptation(const MonitorAdaptation &obj) :
        m_vehicleId(obj.m_vehicleId), m_monitorName(obj.m_monitorName), m_action(
                obj.m_action) {
}

MonitorAdaptation::~MonitorAdaptation() {
}

MonitorAdaptation& MonitorAdaptation::operator=(const MonitorAdaptation &obj) {
  setVehicleId(obj.getVehicleId());
  setMonitorName(obj.getMonitorName());
  setAction(obj.getAction());
  return (*this);
}

int32_t MonitorAdaptation::getVehicleId() const {
  return m_vehicleId;
}

string MonitorAdaptation::getMonitorName() const {
  return m_monitorName;
}

string MonitorAdaptation::getAction() const {
  return m_action;
}

void MonitorAdaptation::setVehicleId(const int32_t &vehicleId) {
  m_vehicleId = vehicleId;
}

void MonitorAdaptation::setMonitorName(const string &monitorName) {
  m_monitorName = monitorName;
}

void MonitorAdaptation::setAction(const string &action) {
  m_action = action;
}

ostream& MonitorAdaptation::operator<<(ostream &out) const {
//  std::cout << "Try out" << std::endl;
  odcore::serialization::SerializationFactory& sf =
          odcore::serialization::SerializationFactory::getInstance();

  std::shared_ptr < odcore::serialization::Serializer > s =
          sf.getQueryableNetstringsSerializer(out);
  s->write(1, m_vehicleId);

  s->write(2, m_monitorName);

  s->write(3, m_action);
  return out;
}

istream& MonitorAdaptation::operator>>(istream &in) {
//  std::cout << "Try in" << std::endl;
  odcore::serialization::SerializationFactory& sf =
          odcore::serialization::SerializationFactory::getInstance();

  std::shared_ptr<odcore::serialization::Deserializer> d =
          sf.getQueryableNetstringsDeserializer(in);

  d->read(1, m_vehicleId);

  d->read(2, m_monitorName);

  d->read(3, m_action);

  return in;
}

int32_t MonitorAdaptation::ID() {
  return 190;
}

int32_t MonitorAdaptation::getID() const {
  return MonitorAdaptation::ID();
}

const string MonitorAdaptation::getShortName() const {
  return "MonitorAdaptation";
}

const string MonitorAdaptation::getLongName() const {
  return "irus.MonitorAdaptation";
}

const string MonitorAdaptation::toString() const {
  stringstream sstr;
  sstr << "MonitorAdaptation";
  return sstr.str();
}

}
