#include "TrackerDevice.hpp"
#include <Windows.h>

ExampleDriver::TrackerDevice::TrackerDevice(std::string serial):
    serial_(serial)
{
}

std::string ExampleDriver::TrackerDevice::GetSerial()
{
    return this->serial_;
}

void ExampleDriver::TrackerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;
    if (this->device_index_ < 3)
        return;

    // Check if this device was asked to be identified
    auto events = GetDriver()->GetOpenVREvents();
    for (auto event : events) {
        // Note here, event.trackedDeviceIndex does not necissarily equal this->device_index_, not sure why, but the component handle will match so we can just use that instead
        //if (event.trackedDeviceIndex == this->device_index_) {
        if (event.eventType == vr::EVREventType::VREvent_Input_HapticVibration) {
            if (event.data.hapticVibration.componentHandle == this->haptic_component_) {
                this->did_vibrate_ = true;
            }
        }
        //}
    }

    // Check if we need to keep vibrating
    if (this->did_vibrate_) {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count()/1000.f);
        if (this->vibrate_anim_state_ > 1.0f) {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }

    // Setup pose for this frame
    auto pose = IVRDevice::MakeDefaultPose();

    // Find a HMD
    vr::TrackedDevicePose_t hmd_pose[10];
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, hmd_pose, 10);

    GetDriver()->Log("***hmd_pose:");
    std::string str;
    for (int i = 0; i < 3; i++)
    {
        str += std::to_string(hmd_pose[0].mDeviceToAbsoluteTracking.m[i][3]);
        str += ",";
    }
    GetDriver()->Log(str);

    // WIP values for test, atached to controlers
    if (serial_.find("_Hip") != -1) {
        pose.vecPosition[0] = hmd_pose[0].mDeviceToAbsoluteTracking.m[0][3]; //right
        pose.vecPosition[1] = hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3] - 0.5; //top
        pose.vecPosition[2] = hmd_pose[0].mDeviceToAbsoluteTracking.m[2][3] + 0.1; //back
    }else if (serial_.find("_LegR") != -1) {
        pose.vecPosition[0] = hmd_pose[1].mDeviceToAbsoluteTracking.m[0][3] + 0.1; //right
        pose.vecPosition[1] = hmd_pose[1].mDeviceToAbsoluteTracking.m[1][3] - 1.0; //top
        pose.vecPosition[2] = hmd_pose[1].mDeviceToAbsoluteTracking.m[2][3] - 0.1; //back
    }else if (serial_.find("_LegL") != -1) {
        pose.vecPosition[0] = hmd_pose[2].mDeviceToAbsoluteTracking.m[0][3] - 0.1; //right
        pose.vecPosition[1] = hmd_pose[2].mDeviceToAbsoluteTracking.m[1][3] - 1.0; //top
        pose.vecPosition[2] = hmd_pose[2].mDeviceToAbsoluteTracking.m[2][3] - 0.1; //back
    }
    else {
        pose.vecPosition[0] = 0.0; //right
        pose.vecPosition[1] = 0.0; //top
        pose.vecPosition[2] = 0.0; //back
    }
    
    vr::HmdQuaternion_t q;
    q.w = sqrt(fmax(0, 1 + hmd_pose[0].mDeviceToAbsoluteTracking.m[0][0] + hmd_pose[0].mDeviceToAbsoluteTracking.m[1][1] + hmd_pose[0].mDeviceToAbsoluteTracking.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + hmd_pose[0].mDeviceToAbsoluteTracking.m[0][0] - hmd_pose[0].mDeviceToAbsoluteTracking.m[1][1] - hmd_pose[0].mDeviceToAbsoluteTracking.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - hmd_pose[0].mDeviceToAbsoluteTracking.m[0][0] + hmd_pose[0].mDeviceToAbsoluteTracking.m[1][1] - hmd_pose[0].mDeviceToAbsoluteTracking.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - hmd_pose[0].mDeviceToAbsoluteTracking.m[0][0] - hmd_pose[0].mDeviceToAbsoluteTracking.m[1][1] + hmd_pose[0].mDeviceToAbsoluteTracking.m[2][2])) / 2;
    q.x = copysign(q.x, hmd_pose[0].mDeviceToAbsoluteTracking.m[2][1] - hmd_pose[0].mDeviceToAbsoluteTracking.m[1][2]);
    q.y = copysign(q.y, hmd_pose[0].mDeviceToAbsoluteTracking.m[0][2] - hmd_pose[0].mDeviceToAbsoluteTracking.m[2][0]);
    q.z = copysign(q.z, hmd_pose[0].mDeviceToAbsoluteTracking.m[1][0] - hmd_pose[0].mDeviceToAbsoluteTracking.m[0][1]);
    pose.qRotation.w = q.w;
    pose.qRotation.x = q.x;
    pose.qRotation.y = q.y;
    pose.qRotation.z = q.z;
    GetDriver()->Log("***hmd_angle:"+ std::to_string(q.w)+","+ std::to_string(q.x)+","+ std::to_string(q.y)+","+ std::to_string(q.z));

    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.deviceIsConnected = true;

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

DeviceType ExampleDriver::TrackerDevice::GetDeviceType()
{
    return DeviceType::TRACKER;
}

vr::TrackedDeviceIndex_t ExampleDriver::TrackerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

vr::EVRInitError ExampleDriver::TrackerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("[SlimeVR] Activating tracker " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);
    /*
    // Setup inputs and outputs
    GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &this->haptic_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &this->system_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/touch", &this->system_touch_component_);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);

    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "Vive Tracker Pro MV");

    // Opt out of hand selection
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);

    // Set up a render model path
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "vr_controller_05_wireless_b");
    //GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "{example}example_controller");

    // Set controller profile
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{example}/input/example_tracker_bindings.json");

    // Set the icon
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, "{example}/icons/tracker_ready.png");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{example}/icons/tracker_not_ready.png");

    if (serial_.find("_Hip") != -1) {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "vive_tracker_waist");
    }
    else if (serial_.find("_LegL") != -1) {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "vive_tracker_left_foot");
    }
    else if (serial_.find("_LegR") != -1) {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "vive_tracker_right_foot");
    }
    else {
        GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "vive_tracker");
    }

    return vr::EVRInitError::VRInitError_None;*/

    // Normally a vive tracker emulator would (logically) always set the tracking system to "lighthouse" but in order to do space calibration
    // with existing tools such as OpenVR Space calibrator and be able to calibrate to/from ALVR HMD (and the proxy tracker) space to/from
    // a native HMD/tracked device which is already using "lighthouse" as the tracking system the proxy tracker needs to be in a different
    // tracking system to treat them differently and prevent those tools doing the same space transform to the proxy tracker.
    vr::VRProperties()->SetStringProperty(props, vr::Prop_TrackingSystemName_String, "lighthouse");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "Vive Tracker Pro MV");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_SerialNumber_String, "ALVR HMD Tracker Proxy"); // Changed
    vr::VRProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_WillDriftInYaw_Bool, false);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "HTC");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_TrackingFirmwareVersion_String, "1541800000 RUNNER-WATCHMAN$runner-watchman@runner-watchman 2018-01-01 FPGA 512(2.56/0/0) BL 0 VRC 1541800000 Radio 1518800000"); // Changed
    vr::VRProperties()->SetStringProperty(props, vr::Prop_HardwareRevision_String, "product 128 rev 2.5.6 lot 2000/0/0 0"); // Changed
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ConnectedWirelessDongle_String, "D0000BE000"); // Changed
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceIsWireless_Bool, true);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceIsCharging_Bool, false);
    vr::VRProperties()->SetFloatProperty(props, vr::Prop_DeviceBatteryPercentage_Float, 1.f); // Always charged

    vr::HmdMatrix34_t l_transform = { -1.f, 0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f };
    vr::VRProperties()->SetProperty(props, vr::Prop_StatusDisplayTransform_Matrix34, &l_transform, sizeof(vr::HmdMatrix34_t), vr::k_unHmdMatrix34PropertyTag);

    vr::VRProperties()->SetBoolProperty(props, vr::Prop_Firmware_UpdateAvailable_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_Firmware_ManualUpdate_Bool, false);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_Firmware_ManualUpdateURL_String, "https://developer.valvesoftware.com/wiki/SteamVR/HowTo_Update_Firmware");
    vr::VRProperties()->SetUint64Property(props, vr::Prop_HardwareRevision_Uint64, 2214720000); // Changed
    vr::VRProperties()->SetUint64Property(props, vr::Prop_FirmwareVersion_Uint64, 1541800000); // Changed
    vr::VRProperties()->SetUint64Property(props, vr::Prop_FPGAVersion_Uint64, 512); // Changed
    vr::VRProperties()->SetUint64Property(props, vr::Prop_VRCVersion_Uint64, 1514800000); // Changed
    vr::VRProperties()->SetUint64Property(props, vr::Prop_RadioVersion_Uint64, 1518800000); // Changed
    vr::VRProperties()->SetUint64Property(props, vr::Prop_DongleVersion_Uint64, 8933539758); // Changed, based on vr::Prop_ConnectedWirelessDongle_String above
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceCanPowerOff_Bool, true);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_Firmware_ProgrammingTarget_String, "ALVR HMD Tracker Proxy");
    vr::VRProperties()->SetInt32Property(props, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_Firmware_ForceUpdateRequired_Bool, false);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ResourceRoot_String, "htc");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_RegisteredDeviceType_String, "ALVR/tracker/hmd_proxy");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_Identifiable_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_Firmware_RemindUpdate_Bool, false);
    vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "vive_tracker_waist");
    vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerHandSelectionPriority_Int32, -1);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.gif");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.gif");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.png");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.png");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_error.png");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.png");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.png");
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_HasDisplayComponent_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_HasCameraComponent_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_HasDriverDirectModeComponent_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_HasVirtualDisplayComponent_Bool, false);
    return vr::EVRInitError::VRInitError_None;
}

void ExampleDriver::TrackerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void ExampleDriver::TrackerDevice::EnterStandby()
{
}

void* ExampleDriver::TrackerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    if (strcmp(vr::IVRDriverInput_Version, pchComponentNameAndVersion) == 0)
    {
        return this;
    }
    return NULL;
    //return nullptr;
}

void ExampleDriver::TrackerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t ExampleDriver::TrackerDevice::GetPose()
{
    return last_pose_;
}
