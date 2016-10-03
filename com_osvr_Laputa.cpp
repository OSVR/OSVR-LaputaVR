/** @file
    @brief Implementation

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef __ANDROID__
// Define this for verbose output during polling.
#define OSVR_MULTISERVER_VERBOSE
#endif

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/Util/UniquePtr.h>
#include <osvr/Util/StringLiteralFileToString.h>
#include <osvr/VRPNServer/VRPNDeviceRegistration.h>

#include "com_osvr_Laputa_json.h"
#include "com_osvr_IMU_Combiner_json.h"

// Library/third-party includes
#include <vendor/hidapi/hidapi/hidapi.h>
#include "VRPNMultiserver.h"
#include "vrpn_Connection.h"
#include "vrpn_Laputa.h"
#include "vrpn_Tracker_IMU.h"
#include <boost/noncopyable.hpp>

#ifdef OSVR_LAPUTA_VERBOSE
#include <boost/format.hpp>
#endif

// Standard includes
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#ifdef OSVR_LAPUTA_VERBOSE
#include <iostream>
#endif

class VRPNHardwareDetect : boost::noncopyable {
  public:
    VRPNHardwareDetect(VRPNMultiserverData &data) : m_data(data) {}
    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
        bool gotDevice;
#ifdef OSVR_LAPUTA_VERBOSE
        bool first = true;
#endif
        do {
            gotDevice = false;
            struct hid_device_info *enumData = hid_enumerate(0, 0);
            for (struct hid_device_info *dev = enumData; dev != nullptr;
                 dev = dev->next) {

                if (m_isPathHandled(dev->path)) {
                    continue;
                }

#ifdef OSVR_LAPUTA_VERBOSE
                if (first) {
                    std::cout << "[OSVR Laputa] HID Enumeration: "
                              << boost::format("0x%04x") % dev->vendor_id << ":"
                              << boost::format("0x%04x") % dev->product_id
                              << std::endl;
                }
#endif

                if (gotDevice) {
                    continue;
                }

                // Laputa VR
                if (dev->vendor_id == 0x2633 && dev->product_id == 0x0006) {
                    gotDevice = true;
                    m_handlePath(dev->path);

                    /// Decorated name for Hydra
                    std::string name;

                    // vrpn_Laputa
                    osvr::vrpnserver::VRPNDeviceRegistration reg(ctx);
                    name =
                        reg.useDecoratedName(m_data.getName("LaputaHeroRaw"));
                    reg.registerDevice(
                        new vrpn_Laputa(name.c_str(), reg.getVRPNConnection()));
                    reg.setDeviceDescriptor(
                        osvr::util::makeString(com_osvr_Laputa_json));

                    std::string localName = "*" + name;

                    // I think we also need vrpn_Magnetometer here

                    // vrpn_Tracker_IMU
                    vrpn_Tracker_IMU_Params params;
                    params.d_acceleration.name = localName.c_str();
                    params.d_rotational_vel.name = localName.c_str();
                    int accel_offset = 2;
                    int rotat_offest = 5;
                    for (size_t i = 0; i < 3; i++) {
                        params.d_acceleration.channels[i] = i + accel_offset;
                        params.d_acceleration.offsets[i] = 0;
                        params.d_acceleration.scales[i] = -1.0;
                        params.d_rotational_vel.channels[i] = i + rotat_offest;
                        params.d_rotational_vel.offsets[i] = 0;
                        params.d_rotational_vel.scales[i] = 1.0;
                    }

                    osvr::vrpnserver::VRPNDeviceRegistration reg2(ctx);
                    reg2.registerDevice(new vrpn_IMU_SimpleCombiner(
                        reg2.useDecoratedName(
                                m_data.getName("LaputaHeroTracker"))
                            .c_str(),
                        reg2.getVRPNConnection(), &params, 400.0));
                    reg2.setDeviceDescriptor(
                        osvr::util::makeString(com_osvr_IMU_Combiner_json));
                }
            }
            hid_free_enumeration(enumData);

#ifdef OSVR_LAPUTA_VERBOSE
            first = false;
#endif

        } while (gotDevice);
        return OSVR_RETURN_SUCCESS;
    }

  private:
    bool m_isPathHandled(const char *path) {
        return std::find(begin(m_handledPaths), end(m_handledPaths),
                         std::string(path)) != end(m_handledPaths);
    }
    void m_handlePath(const char *path) {
        m_handledPaths.push_back(std::string(path));
    }
    VRPNMultiserverData &m_data;
    std::vector<std::string> m_handledPaths;
};

OSVR_PLUGIN(com_osvr_Laputa) {
    osvr::pluginkit::PluginContext context(ctx);

    VRPNMultiserverData &data =
        *context.registerObjectForDeletion(new VRPNMultiserverData);
    context.registerHardwareDetectCallback(new VRPNHardwareDetect(data));

    return OSVR_RETURN_SUCCESS;
}
