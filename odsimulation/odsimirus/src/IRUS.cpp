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

#include "IRUS.h"
#include "MonitorAdaptation.h"
#include "V2vRequest.h"
#include "Voice.h"
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
        TimeTriggeredConferenceClientModule(argc, argv, "odsimirus") {}

    IRUS::~IRUS() {}

    void IRUS::setUp() {}

    void IRUS::tearDown() {}

    odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode IRUS::body() {
        stringstream sstrConfiguration;
        getKeyValueConfiguration().writeTo(sstrConfiguration);

        // Use libodsimulation's odsimirus implementation.
        string config = sstrConfiguration.str();

        opendlv::vehiclecontext::model::IRUS irus(config);
        irus.setup();

        // Use the most recent EgoState available.
        KeyValueDataStore &kvs = getKeyValueDataStore();

  std::cout << config << std::endl;
  string lastAdaptedMonitor;
  std::map<string, bool> areSensorAlternatives;
  areSensorAlternatives["FrontRight"] = false;
  areSensorAlternatives["Rear"] = false;
  areSensorAlternatives["RearRight"] = false;
  areSensorAlternatives["FrontCenter"] = false;


        while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
            // Get current EgoState.
    Container c = kvs.get(opendlv::data::environment::EgoState::ID());
    std::cout << "Vehicle egostate id: " << c.getSenderStamp() << std::endl;

    Container c2 = kvs.get(MonitorAdaptation::ID());
    MonitorAdaptation ma = c2.getData<MonitorAdaptation>();
    if ((unsigned) ma.getVehicleId() == getIdentifier()) {
      if (lastAdaptedMonitor.compare(ma.getMonitorName()) != 0) {
        lastAdaptedMonitor = ma.getMonitorName();
      areSensorAlternatives[lastAdaptedMonitor.substr(
              lastAdaptedMonitor.find("_") + 1, std::string::npos)] = true;
        std::string request("request");
        V2vRequest nextMessage(request.size(), request);
        odcore::data::Container cReq(nextMessage);
        getConference().send(cReq);
//        std::cout << "Adaptation received--------------------------"
//                << ma.getAction() << " monitor variable:"
//                << lastAdaptedMonitor.substr(lastAdaptedMonitor.find("_") + 1,
//                        std::string::npos) << std::endl;
      }
    }

    if (c.getSenderStamp() == getIdentifier()) {

            EgoState es = c.getData<EgoState>();

            // Calculate result and propagate it.
            vector<Container> toBeSent = irus.calculate(es);
            if (toBeSent.size() > 0) {
                vector<Container>::iterator it = toBeSent.begin();
                while(it != toBeSent.end()) {
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
              faultModelSkip = getKeyValueConfiguration().getValue<double>(faultModelSkipStr.str());
                      if (faultModelSkip > 0) {
                  string name;
                  name = getKeyValueConfiguration().getValue<string>(
                          "odsimirus.sensor" + std::to_string(i) + ".name");
                if(areSensorAlternatives.at(name.substr(
                                        name.find("_") + 1, std::string::npos))) {
                    std::cout
                            << "There is an alternative for " << name
                            << "-------------------: "
                            << lastAdaptedMonitor << std::endl;

                    Container c2Voice = kvs.get(Voice::ID());
                    std::cout << "Get last data from v2v "
                            << c2Voice.getDataType() << std::endl;
//                    std::shared_ptr < Buffer > outBuffer(new Buffer());
//                    // Reverser for big and little endian specification of V2V.
//                    outBuffer->Reversed();
//                    outBuffer->AppendInteger(1); //stationId
//                    std::vector<unsigned char> bytes = outBuffer->GetBytes();
//                    std::string bytesString(bytes.begin(), bytes.end());

                    sbd.putTo_MapOfDistances(i, -1);
                  } else {
                    sbd.putTo_MapOfDistances(i, -2);
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

} // irus
