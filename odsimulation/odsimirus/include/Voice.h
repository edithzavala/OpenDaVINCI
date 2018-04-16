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

#ifndef VOICE_H
#define VOICE_H

#include <string>
#include "opendavinci/odcore/opendavinci.h"

#include "opendavinci/odcore/data/SerializableData.h"

namespace irus {

    using namespace std;

    /**
     * This class can be used to produce some objects detected by
     * point providing sensors.
     */
class Voice: public odcore::data::SerializableData {
  public:
    Voice();

    /**
     * Constructor.
     *
     * @param position Position.
     * @param rotation Rotation.
     * @param velocity Velocity.
     * @param acceleration Acceleration.
     */
    Voice(const string &type, const int32_t &size, const string &data);

    /**
     * Copy constructor.
     *
     * @param obj Reference to an object of this class.
     */
    Voice(const Voice &obj);

    virtual ~Voice();

    /**
     * Assignment operator.
     *
     * @param obj Reference to an object of this class.
     * @return Reference to this instance.
     */
    Voice& operator=(const Voice &obj);

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

    string getType() const;
    int32_t getSize() const;
    string getData() const;
    void setType(const string &type);
    void setSize(const int32_t &size);
    void setData(const string &data);


  private:
    string m_type;
    int32_t m_size;
    string m_data;
};
}

#endif
