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

#include "V2vRequest.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

namespace irus {
using namespace std;

V2vRequest::V2vRequest() :
  m_size(), m_data() {
}

  V2vRequest::V2vRequest(const int32_t &size, const string &data) :
  m_size(size), m_data(data) {
}

V2vRequest::V2vRequest(const V2vRequest &obj) :
  m_size(obj.m_size), m_data(obj.m_data) {
}

V2vRequest::~V2vRequest() {
}

V2vRequest& V2vRequest::operator=(const V2vRequest &obj) {
    setSize(obj.getSize());
  setData(obj.getData());
  return (*this);
}

  int32_t V2vRequest::getSize() const {
    return m_size;
}

  string V2vRequest::getData() const {
    return m_data;
}

  void V2vRequest::setSize(const int32_t &size) {
    m_size = size;
}

  void V2vRequest::setData(const string &data) {
    m_data = data;
}

ostream& V2vRequest::operator<<(ostream &out) const {
//  std::cout << "Try out" << std::endl;
  odcore::serialization::SerializationFactory& sf =
          odcore::serialization::SerializationFactory::getInstance();

  std::shared_ptr < odcore::serialization::Serializer > s =
          sf.getQueryableNetstringsSerializer(out);
    s->write(1, m_size);

    s->write(2, m_data);
  return out;
}

istream& V2vRequest::operator>>(istream &in) {
//  std::cout << "Try in" << std::endl;
  odcore::serialization::SerializationFactory& sf =
          odcore::serialization::SerializationFactory::getInstance();

  std::shared_ptr<odcore::serialization::Deserializer> d =
          sf.getQueryableNetstringsDeserializer(in);

    d->read(1, m_size);

    d->read(2, m_data);

  return in;
}

int32_t V2vRequest::ID() {
  return 172;
}

int32_t V2vRequest::getID() const {
  return V2vRequest::ID();
}

const string V2vRequest::getShortName() const {
  return "V2vRequest";
}

const string V2vRequest::getLongName() const {
  return "irus.V2vRequest";
}

const string V2vRequest::toString() const {
  stringstream sstr;
  sstr << "V2vRequest";
  return sstr.str();
}

}
