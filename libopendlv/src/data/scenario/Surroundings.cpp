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

#include <memory>
#include <istream>
#include <string>
#include <vector>

#include "opendavinci/odcore/opendavinci.h"
#include "opendavinci/odcore/serialization/Deserializer.h"
#include "opendavinci/odcore/serialization/Serializable.h"
#include "opendavinci/odcore/serialization/SerializationFactory.h"
#include "opendavinci/odcore/serialization/Serializer.h"
#include "opendavinci/odcore/data/SerializableData.h"
#include "opendlv/data/scenario/ComplexModel.h"
#include "opendlv/data/scenario/Cylinder.h"
#include "opendlv/data/scenario/Polygon.h"
#include "opendlv/data/scenario/ScenarioVisitor.h"
#include "opendlv/data/scenario/Shape.h"
#include "opendlv/data/scenario/Surroundings.h"

namespace opendlv {
    namespace data {
        namespace scenario {

            using namespace std;
            using namespace odcore::base;
            using namespace scenario;

            Surroundings::Surroundings() :
                    m_listOfShapes() {}

            Surroundings::Surroundings(const Surroundings &obj) :
                    SerializableData(),
                    m_listOfShapes() {
                // Clean up.
                cleanUp();

                // Create deep copy.
                deepCopy(obj);
            }

            Surroundings::~Surroundings() {
                // Clean up.
                cleanUp();
            }

            Surroundings& Surroundings::operator=(const Surroundings &obj) {
                // Clean up.
                cleanUp();

                // Create deep copy.
                deepCopy(obj);

                return (*this);
            }

            void Surroundings::deepCopy(const Surroundings &obj) {
                const vector<Shape*>& list = obj.getListOfShapes();
                vector<Shape*>::const_iterator it = list.begin();
                while (it != list.end()) {
                    Shape *s = (*it++);
                    switch (s->getType()) {
                    case Shape::COMPLEXMODEL: {
                            ComplexModel *cm = new ComplexModel(dynamic_cast<ComplexModel&>(*s));
                            m_listOfShapes.push_back(cm);
                            break;
                        }
                    case Shape::CYLINDER: {
                            Cylinder *c = new Cylinder(dynamic_cast<Cylinder&>(*s));
                            m_listOfShapes.push_back(c);
                            break;
                        }
                    case Shape::POLYGON: {
                            Polygon *p = new Polygon(dynamic_cast<Polygon&>(*s));
                            m_listOfShapes.push_back(p);
                            break;
                        }
                    case Shape::UNDEFINED:
                        break;
                    }
                }
            }

            void Surroundings::cleanUp() {
                vector<Shape*>::iterator it = m_listOfShapes.begin();
                while (it != m_listOfShapes.end()) {
                    Shape *s = (*it++);
                    OPENDAVINCI_CORE_DELETE_POINTER(s);
                }
                m_listOfShapes.clear();
            }

            void Surroundings::accept(ScenarioVisitor &visitor) {
                visitor.visit(*this);

                vector<Shape*>::iterator it = m_listOfShapes.begin();
                while (it != m_listOfShapes.end()) {
                    Shape *s = (*it++);
                    s->accept(visitor);
                }
            }

            const vector<Shape*>& Surroundings::getListOfShapes() const {
                return m_listOfShapes;
            }

            void Surroundings::addShape(Shape *s) {
                if (s != NULL) {
                    m_listOfShapes.push_back(s);
                }
            }

            int32_t Surroundings::getID() const {
                return 835;
            }

            const string Surroundings::getShortName() const {
                return "Surroundings";
            }

            const string Surroundings::getLongName() const {
                return "hesperia.data.scenario.Surroundings";
            }

            const string Surroundings::toString() const {
                return "";
            }

            ostream& Surroundings::operator<<(ostream &out) const {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();

                std::shared_ptr<odcore::serialization::Serializer> s = sf.getQueryableNetstringsSerializer(out);

                // Do not serialize UNKNOWN TYPES.
                uint32_t numberOfShapes = 0;
                vector<Shape*>::const_iterator it = m_listOfShapes.begin();
                while (it != m_listOfShapes.end()) {
                    Shape *shape = (*it++);
                    if (shape != NULL) {
                        numberOfShapes++;
                    }
                }
                s->write(1, numberOfShapes);

                // Write shapes to stringstream.
                stringstream sstr;
                for (uint32_t i = 0; i < m_listOfShapes.size(); i++) {
                    Shape *shape = m_listOfShapes.at(i);
                    if (shape != NULL) {
                        // Write data to stringstream.
                        sstr << static_cast<uint32_t>(shape->getType())
                        << *shape
                        << endl;
                    }
                }

                // Write shapes.
                if (numberOfShapes > 0) {
                    s->write(2, sstr.str());
                }

                return out;
            }

            istream& Surroundings::operator>>(istream &in) {
                odcore::serialization::SerializationFactory& sf=odcore::serialization::SerializationFactory::getInstance();

                std::shared_ptr<odcore::serialization::Deserializer> d = sf.getQueryableNetstringsDeserializer(in);

                // Clean up.
                cleanUp();

                uint32_t numberOfShapes = 0;
                d->read(1, numberOfShapes);

                if (numberOfShapes > 0) {
                    string str;
                    // Read shapes into stringstream.
                    d->read(2, str);

                    stringstream sstr(str);

                    // Read shapes from stringstream.
                    for (uint32_t i = 0; i < numberOfShapes; i++) {
                        uint32_t type = 0;
                        sstr >> type;
                        enum Shape::SHAPETYPE shapeType = static_cast<enum Shape::SHAPETYPE>(type);

                        switch (shapeType) {
                        case Shape::COMPLEXMODEL: {
                                ComplexModel *cm = new ComplexModel();

                                sstr >> *cm;

                                m_listOfShapes.push_back(cm);
                                break;
                            }
                        case Shape::CYLINDER: {
                                Cylinder *c = new Cylinder();

                                // Read cylinder.
                                sstr >> *c;

                                m_listOfShapes.push_back(c);
                                break;
                            }
                        case Shape::POLYGON: {
                                Polygon *p = new Polygon();

                                // Read polygon.
                                sstr >> *p;

                                m_listOfShapes.push_back(p);
                                break;
                            }
                        case Shape::UNDEFINED:
                            break;
                        }
                    }
                }

                return in;
            }

        }
    }
} // opendlv::data::scenario
