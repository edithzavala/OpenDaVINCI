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
  std::cout << config << std::endl;
        opendlv::vehiclecontext::model::IRUS irus(config);
        irus.setup();

        // Use the most recent EgoState available.
        KeyValueDataStore &kvs = getKeyValueDataStore();

        while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
            // Get current EgoState.
    Container c = kvs.get(opendlv::data::environment::EgoState::ID());
    std::cout << "Vehicle egostate id: " << c.getSenderStamp() << std::endl;

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
              faultModelSkipStr << "odsimirus.sensor" << i << ".faultModel.skip";
              faultModelSkip = getKeyValueConfiguration().getValue<double>(faultModelSkipStr.str());
                      if (faultModelSkip > 0) {
                        sbd.putTo_MapOfDistances(i, -2);
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
