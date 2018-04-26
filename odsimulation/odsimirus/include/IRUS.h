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

#ifndef IRUS_H_
#define IRUS_H_

#include "MonitorAdaptation.h"
#include "Voice.h"

#include "opendavinci/odcore/opendavinci.h"
#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "opendavinci/generated/odcore/data/dmcp/ModuleExitCodeMessage.h"
#include "opendlv/data/environment/EgoState.h"
#include "opendavinci/odcore/data/Container.h"

namespace irus {

    using namespace std;

    /**
     * This class can be used to produce some objects detected by
     * point providing sensors.
     */
    class IRUS : public odcore::base::module::TimeTriggeredConferenceClientModule {
        private:
            /**
             * "Forbidden" copy constructor. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the copy constructor.
             *
             * @param obj Reference to an object of this class.
             */
            IRUS(const IRUS &/*obj*/);

            /**
             * "Forbidden" assignment operator. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the assignment operator.
             *
             * @param obj Reference to an object of this class.
             * @return Reference to this instance.
             */
            IRUS& operator=(const IRUS &/*obj*/);

    double GetMeasurementFromV2v(Voice const &,
            opendlv::data::environment::EgoState const &,
            uint32_t const &);
    std::map<string, bool> m_areSensorAlternatives;
    std::map<string, bool> m_areSensorsActive;
    std::map<uint32_t, odcore::data::Container> m_KeyValueAdhocDataStore;
    uint32_t m_minFaultyIterations;
    string m_lastAdaptedMonitorAdded;
    string m_lastAdaptedMonitorRemoved;

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            IRUS(const int32_t &argc, char **argv);

            virtual ~IRUS();

            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

        private:
            virtual void setUp();

            virtual void tearDown();
    virtual void nextContainer(odcore::data::Container &c);
    };

} // irus

#endif /*IRUS_H_*/
