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

#include "Voice.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"

namespace automotive {
namespace miniature {
using namespace std;

Voice::Voice() :
        m_type(), m_size(), m_data() {
}

Voice::Voice(const string &type, const int32_t &size, const string &data) :
        m_type(type), m_size(size), m_data(data) {
}

Voice::Voice(const Voice &obj) :
        m_type(obj.m_type), m_size(obj.m_size), m_data(obj.m_data) {
}

Voice::~Voice() {
}

Voice& Voice::operator=(const Voice &obj) {
  setType(obj.getType());
  setSize(obj.getSize());
  setData(obj.getData());
  return (*this);
}

string Voice::getType() const {
  return m_type;
}

int32_t Voice::getSize() const {
    return m_size;
}

string Voice::getData() const {
    return m_data;
}

void Voice::setType(const string &type) {
  m_type = type;
}

void Voice::setSize(const int32_t &size) {
    m_size = size;
}

void Voice::setData(const string &data) {
    m_data = data;
}

ostream& Voice::operator<<(ostream &out) const {
//  std::cout << "Try out" << std::endl;
  odcore::serialization::SerializationFactory& sf =
          odcore::serialization::SerializationFactory::getInstance();

  std::shared_ptr < odcore::serialization::Serializer > s =
          sf.getQueryableNetstringsSerializer(out);
  s->write(1, m_type);

  s->write(2, m_size);

  s->write(3, m_data);
  return out;
}

istream& Voice::operator>>(istream &in) {
//  std::cout << "Try in" << std::endl;
  odcore::serialization::SerializationFactory& sf =
          odcore::serialization::SerializationFactory::getInstance();

  std::shared_ptr<odcore::serialization::Deserializer> d =
          sf.getQueryableNetstringsDeserializer(in);

  d->read(1, m_type);

  d->read(2, m_size);

  d->read(3, m_data);

  return in;
}

int32_t Voice::ID() {
  return 175;
}

int32_t Voice::getID() const {
  return Voice::ID();
}

const string Voice::getShortName() const {
  return "Voice";
}

const string Voice::getLongName() const {
  return "v2vcam.Voice";
}

const string Voice::toString() const {
  stringstream sstr;
  sstr << "Voice";
  return sstr.str();
}
}

}
