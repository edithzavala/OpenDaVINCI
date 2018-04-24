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

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>

#include "IRUS.h"
#include "MonitorAdaptation.h"
#include "V2vRequest.h"
#include "Voice.h"
#include "buffer.hpp"
#include "opendavinci/odcore/opendavinci.h"
#include "opendavinci/odcore/base/Thread.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendlv/data/environment/EgoState.h"
#include "opendlv/vehiclecontext/model/IRUS.h"

namespace odcore { namespace base { class KeyValueDataStore; } }

namespace irus {

    using namespace std;
    using namespace odcore::base;
    using namespace odcore::data;
    using namespace opendlv::data::environment;

    IRUS::IRUS(const int32_t &argc, char **argv) :
        TimeTriggeredConferenceClientModule(argc, argv, "odsimirus"), m_areSensorAlternatives(), m_KeyValueAdhocDataStore(), m_minFaultyIterations(
                200) {
}

    IRUS::~IRUS() {}

void IRUS::setUp() {
}

void IRUS::tearDown() {
  //check if it is required!!!! (Right now IRUS not using v2v also sends this requests to stop)
  //use this code for eliminating v2v when IRUS sensor is up again
  if (m_areSensorAlternatives["FrontRight"] || m_areSensorAlternatives["Rear"]
          || m_areSensorAlternatives["RearRight"]
          || m_areSensorAlternatives["FrontCenter"]) {
    std::string request("0");
    V2vRequest nextMessage(request.size(), request);
    odcore::data::Container cReq(nextMessage);
    getConference().send(cReq);
  }
}

void IRUS::nextContainer(Container &c) {
  if (c.getSenderStamp() == getIdentifier()
          || c.getDataType() == MonitorAdaptation::ID()
          || c.getDataType() == Voice::ID()) {
    // Store data using a plain map.
    m_KeyValueAdhocDataStore[c.getDataType()] = c;
  }
}


odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode IRUS::body() {
  stringstream sstrConfiguration;
  getKeyValueConfiguration().writeTo(sstrConfiguration);

  // Use libodsimulation's odsimirus implementation.
  string config = sstrConfiguration.str();

  opendlv::vehiclecontext::model::IRUS irus(config);
  irus.setup();

//  // Use the most recent EgoState available.
//  KeyValueDataStore &kvs = getKeyValueDataStore();

  std::cout << config << std::endl;
  string lastAdaptedMonitor;
  uint32_t acummFaultyIterations = 0;

  m_areSensorAlternatives["FrontRight"] = false;
  m_areSensorAlternatives["Rear"] = false;
  m_areSensorAlternatives["RearRight"] = false;
  m_areSensorAlternatives["FrontCenter"] = false;


        while (getModuleStateAndWaitForRemainingTimeInTimeslice()
          == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    /*Process adaptation messages if exists*/
    Container c2 = m_KeyValueAdhocDataStore[MonitorAdaptation::ID()];

    if (c2.getDataType() == MonitorAdaptation::ID()) {
    MonitorAdaptation ma = c2.getData<MonitorAdaptation>();
      m_KeyValueAdhocDataStore.erase(MonitorAdaptation::ID());

    if ((unsigned) ma.getVehicleId() == getIdentifier()) {
      if (lastAdaptedMonitor.compare(ma.getMonitorName()) != 0) {
        lastAdaptedMonitor = ma.getMonitorName();
        if (ma.getAction() == "add") {
          m_areSensorAlternatives[lastAdaptedMonitor.substr(
                  lastAdaptedMonitor.find("_") + 1, std::string::npos)] = true;
        std::string request("1");
        V2vRequest nextMessage(request.size(), request);
        odcore::data::Container cReq(nextMessage);
        getConference().send(cReq);
        } else { // in case of remove
          m_areSensorAlternatives[lastAdaptedMonitor.substr(
                  lastAdaptedMonitor.find("_") + 1, std::string::npos)] = false;
          std::string request("0");
          V2vRequest nextMessage(request.size(), request);
          odcore::data::Container cReq(nextMessage);
          getConference().send(cReq);
        }
      }
    }
    }

    /**/

    // Get current EgoState.
    Container c =
            m_KeyValueAdhocDataStore[opendlv::data::environment::EgoState::ID()];

    if (c.getDataType() == opendlv::data::environment::EgoState::ID()) {

      EgoState es = c.getData<EgoState>();
      m_KeyValueAdhocDataStore.erase(
              opendlv::data::environment::EgoState::ID());
      // Calculate result and propagate it.
      vector<Container> toBeSent = irus.calculate(es);
      if (toBeSent.size() > 0) {
        vector<Container>::iterator it = toBeSent.begin();
        while (it != toBeSent.end()) {
          it->setSenderStamp(getIdentifier());

          if (it->getDataType()
                  == automotive::miniature::SensorBoardData::ID()) {

            automotive::miniature::SensorBoardData sbd = it->getData<
                    automotive::miniature::SensorBoardData>();

            int numSensors = getKeyValueConfiguration().getValue<double>(
                    "odsimirus.numberOfSensors");
            for (int i = 0; i < numSensors; i++) {
              double faultModelSkip = 0;
              try {
                stringstream faultModelSkipStr;
                faultModelSkipStr << "odsimirus.sensor" << i
                        << ".faultModel.skip";
                faultModelSkip = getKeyValueConfiguration().getValue<double>(
                        faultModelSkipStr.str());
                if (faultModelSkip > 0) {
                  if (acummFaultyIterations > m_minFaultyIterations) {
                  string name;
                  name = getKeyValueConfiguration().getValue<string>(
                          "odsimirus.sensor" + std::to_string(i) + ".name");
                  if (m_areSensorAlternatives.at(
                          name.substr(name.find("_") + 1, std::string::npos))) {
                    std::cout << "There is an alternative for " << name
                            << "-------------------: " << lastAdaptedMonitor
                            << std::endl;

                    // Get data from vehicles around if exists
                    Container c2Voice = m_KeyValueAdhocDataStore[Voice::ID()];
                    std::cout << "Try to get last data using v2v " << std::endl;

                    if (c2Voice.getDataType() == Voice::ID()) {
                      std::cout << "Vehicle " << c2Voice.getSenderStamp()
                              << " answered" << std::endl;
                      Voice voice = c2Voice.getData<Voice>();
//                      m_KeyValueAdhocDataStore.erase(Voice::ID());
                      sbd.putTo_MapOfDistances(i,
                              GetMeasurementFromV2v(voice, es, i));
                    } else {
                      std::cout << "No vehicles around" << std::endl;
                      sbd.putTo_MapOfDistances(i, -2);
                    }

                  } else {
                    sbd.putTo_MapOfDistances(i, -2);
                  }
                  } else {
                    acummFaultyIterations++;
                  }
                }
//                        else {
//                        sbd.putTo_MapOfDistances(i,sbd.getValueForKey_MapOfDistances(i));
//              }
              } catch (const odcore::exceptions::ValueForKeyNotFoundException &e) {
              }
//              std::cout << sbd.getValueForKey_MapOfDistances(i) << std::endl;
            }

            Container data = Container(sbd);
            getConference().send(data);
          } else {
            getConference().send(*it);
          }
          it++;
          Thread::usleepFor(50);
        }
      }
    }
        }

        irus.tearDown();

        return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
    }

double IRUS::GetMeasurementFromV2v(Voice const &voice, EgoState const &ego,
      uint32_t const &sensorKey) {
  std::string dataString = voice.getData();
  std::vector<unsigned char> data(dataString.begin(), dataString.end());
  std::shared_ptr<Buffer const> buffer(new Buffer(data));
  std::shared_ptr<Buffer::Iterator> inIterator = buffer->GetIterator();
  //Long and little endian reverser
  inIterator->ItReversed();

  //unsigned char messageId = inIterator->ReadByte();
  //int32_t stationId = inIterator->ReadInteger();
  //int32_t generationDeltaTime = inIterator->ReadInteger();
  inIterator->ReadByte();
  inIterator->ReadInteger();
  inIterator->ReadInteger();
  int32_t latitude = inIterator->ReadInteger();
  int32_t longitude = inIterator->ReadInteger();

  //Assume the other vehicle is behind us in a straight street
  double xOffset = ego.getPosition().getX() - longitude;
  double yOffset = ego.getPosition().getY() - latitude;

  if (sensorKey == 1) {
    double distance = std::sqrt((xOffset * xOffset) + (yOffset * yOffset));
    std::cout << "Distance to other vehicle : " << std::to_string(distance)
            << std::endl;
    if (distance > 39) {
      std::cout << "Vehicles around are far" << std::endl;
      return -1;
    } else {
      return distance;
    }

  }
  return -1;
}
} // irus
