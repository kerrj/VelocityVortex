package org.firstinspires.ftc.teamcode.CameraStuff;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.os.Handler;
import android.os.HandlerThread;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.Type;
import android.util.Size;
import android.widget.Toast;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.InstallCallbackInterface;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import java.util.Arrays;

/**
 * Created by Justin on 10/13/2016.
 */
public class FTCCamera {
    /*
    This class sets up the camera for renderscript/opencv interaction and delivers Allocations in an interface each frame
     */
    public interface AllocationListener {
        public void onAllocationAvailable(Allocation inAlloc,Allocation outAlloc);
    }

    private CameraManager mCameraManager;
    private CameraDevice mCameraDevice;
    private String mCameraId;
    private Context context;
    private Handler mHandler;
    private Allocation mAllocationIn,mAllocationOut;
    private RenderScript mRS;
    private AllocationListener allocationListener;

    private Allocation.OnBufferAvailableListener listener = new Allocation.OnBufferAvailableListener() {
            @Override
            public void onBufferAvailable(Allocation a) {
                mAllocationIn.ioReceive();//mAllocationIn now has the current camera frame, is ready for processing
                allocationListener.onAllocationAvailable(mAllocationIn,mAllocationOut);
            }
        };


    public FTCCamera(AllocationListener listener) {
        this.allocationListener=listener;
        mRS = FtcRobotControllerActivity.getRenderScript();
        context = FtcRobotControllerActivity.getActivity().getBaseContext();
        mAllocationIn = FtcRobotControllerActivity.getCameraAllocation();
        mAllocationOut=FtcRobotControllerActivity.getmAllocationOut();
    }
    public void startCamera(){
        startHandler();
        setupCamera();
    }


    private void startHandler() {
        HandlerThread mHandlerThread = new HandlerThread("CameraThread");
        mHandlerThread.start();
        mHandler = new Handler(mHandlerThread.getLooper());

    }


    private CameraDevice.StateCallback callback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice cameraDevice) {
            mCameraDevice = cameraDevice;
            startPreview();
        }

        @Override
        public void onDisconnected(CameraDevice cameraDevice) {
            cameraDevice.close();
            mCameraDevice = null;
        }

        @Override
        public void onError(CameraDevice cameraDevice, int i) {

        }
    };

    public void stopCamera(){
        mCameraDevice.close();
    }

    private void setupCamera() {
        mCameraManager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);

        try {
            for (String id : mCameraManager.getCameraIdList()) {
                CameraCharacteristics mCameraCharacteristics = mCameraManager.getCameraCharacteristics(id);
                if (mCameraCharacteristics.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK) {
                    mCameraId = id;
                }
            }
            CameraCharacteristics mCameraCharacteristics = mCameraManager.getCameraCharacteristics(mCameraId);
            StreamConfigurationMap mStreamConfigurationMap = mCameraCharacteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size[] sizes = mStreamConfigurationMap.getOutputSizes(ImageFormat.YUV_420_888);

            mCameraManager.openCamera(mCameraId, callback, mHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }



    private void startPreview(){
        mAllocationIn.setOnBufferAvailableListener(listener);
        try {
            final CaptureRequest.Builder mCaptureRequestBuilder=mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            mCaptureRequestBuilder.addTarget(mAllocationIn.getSurface());
            mCameraDevice.createCaptureSession(Arrays.asList(mAllocationIn.getSurface()), new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(CameraCaptureSession cameraCaptureSession) {
                    try {
                        mCaptureRequestBuilder.set(CaptureRequest.FLASH_MODE,CaptureRequest.FLASH_MODE_TORCH);
                        CaptureRequest mCaptureRequest = mCaptureRequestBuilder.build();
                        cameraCaptureSession.setRepeatingRequest(mCaptureRequest, null, mHandler);
                    } catch (CameraAccessException e) {
                        e.printStackTrace();
                    }
                }

                @Override
                public void onConfigureFailed(CameraCaptureSession cameraCaptureSession) {
                }
            }, null);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }


}
