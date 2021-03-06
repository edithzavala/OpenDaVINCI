/**
 * OpenDaVINCI - Portable middleware for distributed components.
 * Copyright (C) 2015 Christian Berger
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <iostream>

#include "opendavinci/odcore/serialization/ProtoSerializerVisitor.h"

namespace odcore {
    namespace serialization {

class Serializable;

        ProtoSerializerVisitor::ProtoSerializerVisitor() :
            ProtoSerializer() {}

        ProtoSerializerVisitor::~ProtoSerializerVisitor() {}

        void ProtoSerializerVisitor::beginVisit(const int32_t &/*id*/, const string &/*shortName*/, const string &/*longName*/) {}

        void ProtoSerializerVisitor::endVisit() {}

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, Serializable &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, bool &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, char &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, unsigned char &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, int8_t &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, int16_t &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, uint16_t &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, int32_t &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, uint32_t &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, int64_t &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, uint64_t &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, float &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, double &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, string &v) {
            write(id, longName, shortName, v);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &id, const string &longName, const string &shortName, void *data, const uint32_t &size) {
            write(id, longName, shortName, data, size);
        }

        void ProtoSerializerVisitor::visit(const uint32_t &/*id*/, const string &/*longName*/, const string &/*shortName*/, void */*data*/, const uint32_t &/*count*/, const odcore::TYPE_ &/*t*/) {
            std::cerr << "core::serialization::ProtoSerializerVisitor::visit not implemented." << std::endl;
        }

    }
} // odcore::serialization

