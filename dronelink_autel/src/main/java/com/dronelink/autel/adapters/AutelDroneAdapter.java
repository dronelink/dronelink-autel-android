package com.dronelink.autel.adapters;

import com.autel.common.CallbackWithTwoParams;
import com.autel.common.camera.CameraProduct;
import com.autel.common.error.AutelError;
import com.autel.sdk.camera.AutelBaseCamera;
import com.autel.sdk.camera.AutelCameraManager;
import com.dronelink.autel.AutelDroneSession;
import com.dronelink.core.adapters.CameraAdapter;
import com.dronelink.core.adapters.DroneAdapter;
import com.dronelink.core.adapters.GimbalAdapter;
import com.dronelink.core.adapters.RemoteControllerAdapter;
import com.dronelink.core.command.Command;
import com.dronelink.core.kernel.command.drone.RemoteControllerSticksDroneCommand;
import com.dronelink.core.kernel.command.drone.VelocityDroneCommand;

import com.autel.sdk.product.BaseProduct;

import java.lang.ref.WeakReference;
import java.util.Collection;
import java.util.ArrayList;

public class AutelDroneAdapter extends DroneAdapter {
    public BaseProduct _drone;
    private AutelBaseCamera _currentCamera;
    private AutelCameraManager _autelCameraManager;
    public WeakReference<AutelDroneSession> _session;


    public AutelDroneAdapter(BaseProduct drone)
    {
        _drone = drone;
        _autelCameraManager = drone.getCameraManager();
        initCameraListener();
    }

    private void initCameraListener() {
        if (null == _autelCameraManager) {
            return;
        }

        _autelCameraManager.setCameraChangeListener(new CallbackWithTwoParams<CameraProduct, AutelBaseCamera>() {
            @Override
            public void onSuccess(CameraProduct cameraProduct, AutelBaseCamera autelBaseCamera) {
                if (_currentCamera == autelBaseCamera) {
                    return;
                }

                _currentCamera = autelBaseCamera;
            }

            @Override
            public void onFailure(AutelError autelError) {

            }
        });
    }

    @Override
    public Collection<RemoteControllerAdapter> getRemoteControllers() {
        return null;
    }

    @Override
    public Collection<CameraAdapter> getCameras() {
        CameraAdapter camera = getCamera(0);
        if (camera != null) {
            ArrayList<CameraAdapter> cameras = new ArrayList<CameraAdapter>();
            cameras.add(camera);
            return cameras;
        }

        return null;
    }

    @Override
    public Collection<GimbalAdapter> getGimbals() {
        GimbalAdapter gimbal = getGimbal(0);
        if (gimbal != null) {
            ArrayList<GimbalAdapter> gimbals = new ArrayList<GimbalAdapter>();
            gimbals.add(gimbal);
            return gimbals;
        }

        return null;
    }

    @Override
    public RemoteControllerAdapter getRemoteController(int channel) {
        if (channel == 0) {
            return new AutelRemoteControllerAdapter(_drone.getRemoteController());
        }
        return null;
    }

    @Override
    public Integer getCameraChannel(Integer videoFeedChannel) {
        return null;
    }

    @Override
    public CameraAdapter getCamera(int channel) {
        if (channel == 0) {
            return new AutelCameraAdapter(_currentCamera);
        }
        return null;
    }

    @Override
    public GimbalAdapter getGimbal(int channel) {
        if (channel == 0) {
            return new AutelGimbalAdapter(_drone.getGimbal());
        }
        return null;
    }

    @Override
    public void sendVelocityCommand(VelocityDroneCommand velocityCommand) {
        if (velocityCommand == null) {
            sendResetVelocityCommand();
            return;
        }

        WeakReference<AutelDroneSession> session = _session;

        //TODO: pick back up here

    }

    @Override
    public void sendRemoteControllerSticksCommand(RemoteControllerSticksDroneCommand remoteControllerSticksDroneCommand) {

    }

    @Override
    public void startGoHome(Command.Finisher finisher) {

    }

    @Override
    public void startLanding(Command.Finisher finisher) {

    }
}
