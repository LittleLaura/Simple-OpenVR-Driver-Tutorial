#include "ControllerDevice.hpp"
#include <Windows.h>

ExampleDriver::ControllerDevice::ControllerDevice(std::string serial, ControllerDevice::Handedness handedness):
    serial_(serial),
    handedness_(handedness)
{
}

std::string ExampleDriver::ControllerDevice::GetSerial()
{
    return this->serial_;
}

void ExampleDriver::ControllerDevice::Update()
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
    /*
    // Find a HMD
    auto devices = GetDriver()->GetDevices();
    auto hmd = std::find_if(devices.begin(), devices.end(), [](const std::shared_ptr<IVRDevice>& device_ptr) {return device_ptr->GetDeviceType() == DeviceType::HMD; });
    if (hmd != devices.end()) {
        // Found a HMD
        vr::DriverPose_t hmd_pose = (*hmd)->GetPose();

        // Here we setup some transforms so our controllers are offset from the headset by a small amount so we can see them
        linalg::vec<float, 3> hmd_position{ (float)hmd_pose.vecPosition[0], (float)hmd_pose.vecPosition[1], (float)hmd_pose.vecPosition[2] };
        linalg::vec<float, 4> hmd_rotation{ (float)hmd_pose.qRotation.x, (float)hmd_pose.qRotation.y, (float)hmd_pose.qRotation.z, (float)hmd_pose.qRotation.w };

        // Do shaking animation if haptic vibration was requested
        float controller_y = -0.2f + 0.01f * std::sinf(8 * 3.1415f * vibrate_anim_state_);

        // Left hand controller on the left, right hand controller on the right, any other handedness sticks to the middle
        float controller_x = this->handedness_ == Handedness::LEFT ? -0.2f : (this->handedness_ == Handedness::RIGHT ? 0.2f : 0.f);

        linalg::vec<float, 3> hmd_pose_offset = { controller_x, controller_y, -0.5f };

        hmd_pose_offset = linalg::qrot(hmd_rotation, hmd_pose_offset);

        linalg::vec<float, 3> final_pose = hmd_pose_offset + hmd_position;

        pose.vecPosition[0] = final_pose.x;
        pose.vecPosition[1] = final_pose.y;
        pose.vecPosition[2] = final_pose.z;

        pose.qRotation.w = hmd_rotation.w;
        pose.qRotation.x = hmd_rotation.x;
        pose.qRotation.y = hmd_rotation.y;
        pose.qRotation.z = hmd_rotation.z;
    }
    */
    vr::TrackedDevicePose_t hmd_pose[3];
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, hmd_pose, 3);

    GetDriver()->Log("***hmd_pose:");
    std::string str;
    for (int i = 0; i < 3; i++)
    {
        str += std::to_string(hmd_pose[0].mDeviceToAbsoluteTracking.m[i][3]);
        str += ",";
    }
    GetDriver()->Log(str);

    if (serial_.find("_Hip") != -1) {
        pose.vecPosition[0] = hmd_pose[0].mDeviceToAbsoluteTracking.m[0][3]; //right
        pose.vecPosition[1] = hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3] - 0.5; //top
        pose.vecPosition[2] = hmd_pose[0].mDeviceToAbsoluteTracking.m[2][3] + 0.1; //back
    }
    else if (serial_.find("_Right") != -1) {
        pose.vecPosition[0] = hmd_pose[1].mDeviceToAbsoluteTracking.m[0][3] + 0.1; //right
        pose.vecPosition[1] = hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3] - 1.0; //top
        pose.vecPosition[2] = hmd_pose[1].mDeviceToAbsoluteTracking.m[2][3] - 0.1; //back
    }
    else if (serial_.find("_Left") != -1) {
        pose.vecPosition[0] = hmd_pose[2].mDeviceToAbsoluteTracking.m[0][3] - 0.1; //right
        pose.vecPosition[1] = hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3] - 1.0; //top
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
    GetDriver()->Log("***hmd_angle:" + std::to_string(q.w) + "," + std::to_string(q.x) + "," + std::to_string(q.y) + "," + std::to_string(q.z));
    

    // Check if we need to press any buttons (I am only hooking up the A button here but the process is the same for the others)
    // You will still need to go into the games button bindings and hook up each one (ie. a to left click, b to right click, etc.) for them to work properly
    if (GetAsyncKeyState(0x45 /* E */) != 0) {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, true, 0);
    }
    else {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, false, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, false, 0);
    }

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

DeviceType ExampleDriver::ControllerDevice::GetDeviceType()
{
    //return DeviceType::CONTROLLER; 
    return DeviceType::TRACKER;
}

ExampleDriver::ControllerDevice::Handedness ExampleDriver::ControllerDevice::GetHandedness()
{
    return this->handedness_;
}

vr::TrackedDeviceIndex_t ExampleDriver::ControllerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

vr::EVRInitError ExampleDriver::ControllerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating controller " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Setup inputs and outputs
    GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &this->haptic_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/click", &this->a_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/touch", &this->a_button_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/click", &this->b_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/touch", &this->b_button_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trigger/click", &this->trigger_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trigger/touch", &this->trigger_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trigger/value", &this->trigger_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/grip/touch", &this->grip_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/grip/value", &this->grip_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/grip/force", &this->grip_force_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &this->system_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/touch", &this->system_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/click", &this->trackpad_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/touch", &this->trackpad_touch_component_); 
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/x", &this->trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/y", &this->trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/click", &this->joystick_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/touch", &this->joystick_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/x", &this->joystick_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/y", &this->joystick_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);
    
    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "Vive Tracker PVT");
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_HardwareRevision_Uint64, 1);

    // Set up a render model path
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "vr_tracker_vive_1_0");

    // Give SteamVR a hint at what hand this controller is for
   
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);
    

    // Set controller profile
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{example}/input/example_controller_bindings.json");

    // Change the icon depending on which handedness this controller is using (ANY uses right)
    std::string controller_handedness_str = this->handedness_ == Handedness::LEFT ? "left" : "right";
    std::string controller_ready_file = "{example}/icons/controller_ready_" + controller_handedness_str + ".png";
    std::string controller_not_ready_file = "{example}/icons/controller_not_ready_" + controller_handedness_str + ".png";

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, controller_ready_file.c_str());

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, controller_not_ready_file.c_str());

    return vr::EVRInitError::VRInitError_None;
}

void ExampleDriver::ControllerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void ExampleDriver::ControllerDevice::EnterStandby()
{
}

void* ExampleDriver::ControllerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void ExampleDriver::ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t ExampleDriver::ControllerDevice::GetPose()
{
    return last_pose_;
}
