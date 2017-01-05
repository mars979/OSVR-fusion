#include "stdafx.h"
#include <iostream>

// Anonymous namespace to avoid symbol collision
namespace je_nourish_fusion {

class FusionDevice {
  public:
    FusionDevice(OSVR_PluginRegContext ctx, Json::Value config) {
        // osvrPose3SetIdentity(&m_state);
        /// zero out previous position and orientation
        osvrVec3Zero(&m_prevPosition);
        osvrQuatSetIdentity(&m_prevOrientation);

        m_useTimestamp = config.isMember("timestamp");
        m_usePositionTimestamp =
            m_useTimestamp &&
            config["timestamp"].asString().compare("position") == 0;

        if ((m_useOffset = config.isMember("offsetFromRotationCenter"))) {
            osvrVec3Zero(&m_offset);

            if (config["offsetFromRotationCenter"].isMember("x")) {
                osvrVec3SetX(
                    &m_offset,
                    config["offsetFromRotationCenter"]["x"].asDouble());
            }
            if (config["offsetFromRotationCenter"].isMember("y")) {
                osvrVec3SetY(
                    &m_offset,
                    config["offsetFromRotationCenter"]["y"].asDouble());
            }
            if (config["offsetFromRotationCenter"].isMember("z")) {
                osvrVec3SetZ(
                    &m_offset,
                    config["offsetFromRotationCenter"]["z"].asDouble());
            }
        }

        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        osvrDeviceTrackerConfigure(opts, &m_tracker);

        OSVR_DeviceToken token;
        osvrAnalysisSyncInit(ctx, config["name"].asCString(), opts, &token,
                             &m_ctx);

        m_dev = osvr::pluginkit::DeviceToken(token);

        m_dev.sendJsonDescriptor(je_nourish_fusion_json);
        m_dev.registerUpdateCallback(this);

        /*m_positionReader = PositionReaderFactory::getReader(m_ctx,
        config["position"]);
        m_orientationReader = OrientationReaderFactory::getReader(m_ctx,
        config["orientation"]);*/

        if (OSVR_RETURN_FAILURE ==
            osvrClientGetInterface(m_ctx, config["position"].asCString(),
                                   &m_positionIface)) {
            throw std::runtime_error(
                "Could not get client position interface for analysis plugin!");
        }
        if (OSVR_RETURN_FAILURE ==
            osvrClientGetInterface(m_ctx, config["orientation"].asCString(),
                                   &m_orientIface)) {
            throw std::runtime_error("Could not get client orientation "
                                     "interface for analysis plugin!");
        }

        /*if (m_positionReader == NULL) {
                std::cout << "Fusion Device: Position Reader not created" <<
        std::endl;
        }
        if (m_orientationReader == NULL) {
                std::cout << "Fusion Device: Orientation Reader not created" <<
        std::endl;
        }*/

        osvrRegisterOrientationCallback(m_orientIface, &myOrientationCallback,
                                        this);
        /*osvrRegisterPoseCallback(m_positionIface, &myPositionCallback,
         * this);*/
    }

    ~FusionDevice() {
        if (m_positionIface) {
            osvrClientFreeInterface(m_ctx, m_positionIface);
        }
        if (m_orientIface) {
            osvrClientFreeInterface(m_ctx, m_orientIface);
        }
    }

    OSVR_ReturnCode update() {
        osvrClientUpdate(m_ctx);
        /*


        OSVR_TimeValue timeValuePosition;
        OSVR_TimeValue timeValueOrientation;

        OSVR_ReturnCode ret =
            m_positionReader->update(&m_state.translation, &timeValuePosition);
        m_orientationReader->update(&m_state.rotation, &timeValueOrientation);

        if (ret != OSVR_RETURN_SUCCESS) {
            // use previous position instead of sending nothing, and re-use
            // orientation timestamp
            timeValuePosition.microseconds = timeValueOrientation.microseconds;
            timeValuePosition.seconds = timeValueOrientation.seconds;
            osvrVec3SetX(&m_state.translation, osvrVec3GetX(&m_prevPosition));
            osvrVec3SetY(&m_state.translation, osvrVec3GetY(&m_prevPosition));
            osvrVec3SetZ(&m_state.translation, osvrVec3GetZ(&m_prevPosition));
        } else {
            // record previous position in case next report is not available
            osvrVec3SetX(&m_prevPosition, osvrVec3GetX(&m_state.translation));
            osvrVec3SetY(&m_prevPosition, osvrVec3GetY(&m_state.translation));
            osvrVec3SetZ(&m_prevPosition, osvrVec3GetZ(&m_state.translation));
        }

        if (m_useOffset) {
            Eigen::Quaterniond rotation =
                osvr::util::fromQuat(m_state.rotation);
            Eigen::Map<Eigen::Vector3d> translation =
                osvr::util::vecMap(m_state.translation);

            translation +=
                rotation._transformVector(osvr::util::vecMap(m_offset));
        }

        if (m_useTimestamp) {
            OSVR_TimeValue timeValue = m_usePositionTimestamp
                                           ? timeValuePosition
                                           : timeValueOrientation;
            osvrDeviceTrackerSendPoseTimestamped(*m_dev, m_tracker, &m_state, 0,
                                                 &timeValue);
        } else {
            osvrDeviceTrackerSendPose(*m_dev, m_tracker, &m_state, 0);
        }
        */
        return OSVR_RETURN_SUCCESS;
    }

    static void myOrientationCallback(void *userdata,
                                      const OSVR_TimeValue *timestamp,
                                      const OSVR_OrientationReport *report) {
        auto &self = *static_cast<FusionDevice *>(userdata);
        self.handleOrientation(*timestamp, *report);
    }

    void handleOrientation(OSVR_TimeValue const &timestamp,
                           OSVR_OrientationReport const &report) {

        OSVR_PoseState state;
        /// cache orientation for position callback
        osvrQuatSetW(&m_prevOrientation, osvrQuatGetW(&report.rotation));
        osvrQuatSetX(&m_prevOrientation, osvrQuatGetX(&report.rotation));
        osvrQuatSetY(&m_prevOrientation, osvrQuatGetY(&report.rotation));
        osvrQuatSetZ(&m_prevOrientation, osvrQuatGetZ(&report.rotation));

        /// get position report
        OSVR_TimeValue orientTimestamp;
        OSVR_PoseState poseState;
        OSVR_ReturnCode ret =
            osvrGetPoseState(m_positionIface, &orientTimestamp, &poseState);
        if (ret != OSVR_RETURN_SUCCESS) {
            /*std::cout << "Could not get pose state... Using previously cached
               "
                         "position"
                      << std::endl;*/
            osvrVec3SetX(&state.translation, osvrVec3GetX(&m_prevPosition));
            osvrVec3SetY(&state.translation, osvrVec3GetY(&m_prevPosition));
            osvrVec3SetZ(&state.translation, osvrVec3GetZ(&m_prevPosition));

        } else {
            /// cache translation
            osvrVec3SetX(&m_prevPosition, osvrVec3GetX(&poseState.translation));
            osvrVec3SetY(&m_prevPosition, osvrVec3GetY(&poseState.translation));
            osvrVec3SetZ(&m_prevPosition, osvrVec3GetZ(&poseState.translation));

            osvrVec3SetX(&state.translation,
                         osvrVec3GetX(&poseState.translation));
            osvrVec3SetY(&state.translation,
                         osvrVec3GetY(&poseState.translation));
            osvrVec3SetZ(&state.translation,
                         osvrVec3GetZ(&poseState.translation));
        }
        osvrQuatSetW(&state.rotation, osvrQuatGetW(&report.rotation));
        osvrQuatSetX(&state.rotation, osvrQuatGetX(&report.rotation));
        osvrQuatSetY(&state.rotation, osvrQuatGetY(&report.rotation));
        osvrQuatSetZ(&state.rotation, osvrQuatGetZ(&report.rotation));

        OSVR_TimeValue timeValue;
        osvrTimeValueGetNow(&timeValue);

        osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &state, 0,
                                             &timeValue);
    }

    static void myPositionCallback(void *userdata,
                                   const OSVR_TimeValue *timestamp,
                                   const OSVR_PoseReport *report) {

        auto &self = *static_cast<FusionDevice *>(userdata);
        self.handlePosition(*timestamp, *report);
    }

    void handlePosition(OSVR_TimeValue const &timestamp,
                        OSVR_PoseReport const &report) {

        OSVR_PoseState state;
        /// cache translation for orientation callback
        osvrVec3SetX(&m_prevPosition, osvrVec3GetX(&report.pose.translation));
        osvrVec3SetY(&m_prevPosition, osvrVec3GetY(&report.pose.translation));
        osvrVec3SetZ(&m_prevPosition, osvrVec3GetZ(&report.pose.translation));

        /// get orientation report
        OSVR_TimeValue orientTimestamp;
        OSVR_OrientationState orientState;
        OSVR_ReturnCode ret = osvrGetOrientationState(
            m_orientIface, &orientTimestamp, &orientState);
        if (ret != OSVR_RETURN_SUCCESS) {
            std::cout
                << "Could not get orientation state... Using previously cached"
                << std::endl;
            osvrQuatSetW(&state.rotation, osvrQuatGetW(&m_prevOrientation));
            osvrQuatSetX(&state.rotation, osvrQuatGetX(&m_prevOrientation));
            osvrQuatSetY(&state.rotation, osvrQuatGetY(&m_prevOrientation));
            osvrQuatSetZ(&state.rotation, osvrQuatGetZ(&m_prevOrientation));
        } else {

            osvrQuatSetW(&state.rotation, osvrQuatGetW(&orientState));
            osvrQuatSetX(&state.rotation, osvrQuatGetX(&orientState));
            osvrQuatSetY(&state.rotation, osvrQuatGetY(&orientState));
            osvrQuatSetZ(&state.rotation, osvrQuatGetZ(&orientState));
        }

        osvrVec3SetX(&state.translation,
                     osvrVec3GetX(&report.pose.translation));
        osvrVec3SetY(&state.translation,
                     osvrVec3GetY(&report.pose.translation));
        osvrVec3SetZ(&state.translation,
                     osvrVec3GetZ(&report.pose.translation));

        OSVR_TimeValue timeValue;
        osvrTimeValueGetNow(&timeValue);

        osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &state, 0,
                                             &timeValue);
    }

  private:
    /*IPositionReader* m_positionReader;
    IOrientationReader* m_orientationReader;*/

    OSVR_ClientInterface m_positionIface = nullptr;
    OSVR_ClientInterface m_orientIface = nullptr;

    OSVR_ClientContext m_ctx;
    OSVR_ClientInterface m_position;
    OSVR_ClientInterface m_orientation;

    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
    // OSVR_PoseState m_state;
    OSVR_PositionState m_prevPosition;
    OSVR_OrientationState m_prevOrientation;
    bool m_useOffset;
    OSVR_Vec3 m_offset;

    bool m_useTimestamp;
    bool m_usePositionTimestamp;
};

class FusionDeviceConstructor {
  public:
    FusionDeviceConstructor() {}
    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {
        Json::Value root;
        if (params) {
            Json::Reader r;
            if (!r.parse(params, root)) {
                std::cerr << "Could not parse parameters!" << std::endl;
            }
        }

        if (!root.isMember("name") || !root.isMember("position") ||
            !root.isMember("orientation")) {
            std::cerr << "Warning: got configuration, but no trackers specified"
                      << std::endl;
            return OSVR_RETURN_FAILURE;
        }

        osvr::pluginkit::registerObjectForDeletion(ctx,
                                                   new FusionDevice(ctx, root));

        return OSVR_RETURN_SUCCESS;
    }
};
} // namespace

OSVR_PLUGIN(je_nourish_fusion) {

    osvr::pluginkit::registerDriverInstantiationCallback(
        ctx, "FusionDevice", new je_nourish_fusion::FusionDeviceConstructor);

    return OSVR_RETURN_SUCCESS;
}
