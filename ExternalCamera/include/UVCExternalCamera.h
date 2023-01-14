/*===============================================================================
Copyright (c) 2018 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/

#ifndef _UVC_EXTERNAL_CAMERA_H_
#define _UVC_EXTERNAL_CAMERA_H_

#include <VuforiaEngine/Driver/Driver.h>
#include <libuvc/libuvc.h>

#include <vector>

/// UVCExternalCamera that implements the VuforiaDriver::ExternalCamera base class.
/**
 * The documentation of the overridden public methods can be found in VuforiaEngine/Driver/Driver.h header.
 */
class UVCExternalCamera final : public VuforiaDriver::ExternalCamera
{
public:
    UVCExternalCamera(VuforiaDriver::PlatformData* platformData);
    ~UVCExternalCamera();

    bool open() override;
    bool close() override;
    bool start(VuforiaDriver::CameraMode cameraMode, VuforiaDriver::CameraCallback* cb) override;
    bool stop() override;

    uint32_t getNumSupportedCameraModes() override;
    bool getSupportedCameraMode(uint32_t index, VuforiaDriver::CameraMode* out) override;

    bool supportsExposureMode(VuforiaDriver::ExposureMode parameter) override;
    VuforiaDriver::ExposureMode getExposureMode() override;
    bool setExposureMode(VuforiaDriver::ExposureMode mode) override;

    bool supportsExposureValue() override;
    uint64_t getExposureValueMin() override;
    uint64_t getExposureValueMax() override;
    uint64_t getExposureValue() override;
    bool setExposureValue(uint64_t exposureTime) override;

    bool supportsFocusMode(VuforiaDriver::FocusMode parameter) override;
    VuforiaDriver::FocusMode getFocusMode() override;
    bool setFocusMode(VuforiaDriver::FocusMode mode) override;

    bool supportsFocusValue() override;
    float getFocusValueMin() override;
    float getFocusValueMax() override;
    float getFocusValue() override;
    bool setFocusValue(float value) override;

    /// Used by the UVC C-callback to get hold of the VuforiaDriver-callback.
    VuforiaDriver::CameraCallback* getCallback();

    /// Used by the UVC C-callback to get hold of the per frame intrinsics.
    VuforiaDriver::CameraIntrinsics getCameraIntrinsics();

private:
    void getSupportedCameraModes();

    // JNI methods
    int getNumDevices();
    bool useDevice(int index);
    int getVendorId();
    int getProductId();
    int getFileDescriptor();
    char* getUSBFS();
    int getBusNumber();
    int getDeviceNumber();
    VuforiaDriver::CameraIntrinsics getCalibrationValue(int vid, int pid, int width, int height);

    uvc_context_t*                              mContext{ nullptr };
    uvc_device_t*                               mDevice{ nullptr };
    uvc_device_handle_t*                        mDeviceHandle{ nullptr };
    uvc_stream_ctrl_t                           mStreamControl;

    JavaVM*                                     mJavaVM{ nullptr };
    jint                                        mJniVersion{ 0 };
    jobject                                     mActivity{ nullptr };
    jclass                                      mUSBControllerClass{ nullptr };
    jobject                                     mUSBControllerObj{ nullptr };
    jclass                                      mCalibrationControllerClass{ nullptr };
    jobject                                     mCalibrationControllerObj{ nullptr };

    VuforiaDriver::CameraCallback*              mCallback{ nullptr };
    std::vector<VuforiaDriver::CameraMode>      mSupportedCameraModes;
    VuforiaDriver::CameraIntrinsics             mCameraIntrinsics;
};

#endif // _UVC_EXTERNAL_CAMERA_H_
