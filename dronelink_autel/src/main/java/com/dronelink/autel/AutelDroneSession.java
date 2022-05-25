package com.dronelink.autel;

import android.location.Location;

import com.autel.common.CallbackWithNoParam;
import com.autel.common.CallbackWithOneParam;
import com.autel.common.error.AutelError;
import com.autel.common.flycontroller.FlyControllerVersionInfo;
import com.autel.sdk.flycontroller.AutelFlyController;
import com.autel.sdk.product.BaseProduct;
import com.dronelink.autel.adapters.AutelDroneAdapter;
import com.dronelink.core.CameraFile;
import com.dronelink.core.DatedValue;
import com.dronelink.core.DroneSessionManager;
import com.dronelink.core.adapters.RemoteControllerStateAdapter;
import com.dronelink.core.command.CommandQueue;
import com.dronelink.core.command.MultiChannelCommandQueue;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Date;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class AutelDroneSession {
    public DroneSessionManager _manager;
    public AutelDroneAdapter _adapter;

    private Date _opened = new Date();
    private boolean _closed = false;
    private UUID _id = UUID.randomUUID();
    private String _serialNumber;
    private String _firmwarePackageVersion;
    private boolean _initialized = false;
    private boolean _located = false;
    private Location _lastKnownGroundLocation;

    private CommandQueue _droneCommands = new CommandQueue();
    private MultiChannelCommandQueue _remoteControllerCommands = new MultiChannelCommandQueue();
    private MultiChannelCommandQueue _cameraCommands = new MultiChannelCommandQueue();
    private MultiChannelCommandQueue _gimbalCommands = new MultiChannelCommandQueue();

    private ExecutorService _mainControllerSerialQueue = Executors.newSingleThreadExecutor();
    private DatedValue<AUTELMCSystemState> _mainControllerState;

    private ExecutorService _batterySerialQueue = Executors.newSingleThreadExecutor();
    private DatedValue<AUTELBatteryState> _batteryState;

    private ExecutorService _remoteControllerSerialQueue = Executors.newSingleThreadExecutor();
    private DatedValue<AUTELRCState> _remoteControllerRCState;
    private DatedValue<RemoteControllerStateAdapter> _remoteControllerStateAdapter;

    private ExecutorService _cameraSerialQueue = Executors.newSingleThreadExecutor();
    private Dictionary<Integer, DatedValue<AUTELCameraSystemBaseState>> _cameraStates = new Dictionary<Integer, DatedValue<AUTELCameraSystemBaseState>>();
    private Dictionary<Integer, DatedValue<AUTELCameraSDCardState>> _cameraStorageStates = new Dictionary<Integer, DatedValue<AUTELCameraSDCardState>>();
    private Dictionary<String, DatedValue<AUTELCameraExposureParameters>> _cameraExposureParameters = new Dictionary<String, DatedValue<AUTELCameraExposureParameters>>();

    private ExecutorService _gimbalSerialQueue = Executors.newSingleThreadExecutor();
    private Dictionary<Integer, DatedValue<GimbalStateAdapter>> _gimbalStates = new Dictionary<Integer, DatedValue<GimbalStateAdapter>>();

    private DatedValue<CameraFile> _mostRecentCameraFile;
    public DatedValue<CameraFile> getMostRecentCameraFile() {
        return _mostRecentCameraFile;
    }

    private double _maxHorizontalVelocity = 15.0;
    public double getMaxHorizontalVelocity() {
        return _maxHorizontalVelocity;
    }

    private double _maxAscentVelocity = 5.0;
    public double getMaxAscentVelocity() {
        return _maxAscentVelocity;
    }

    private double _maxDescentVelocity = 5.0;
    public double getMaxDescentVelocity() {
        return _maxDescentVelocity;
    }

    public AutelDroneSession(DroneSessionManager manager, BaseProduct drone) {
        this._manager = manager;
        _adapter = new AutelDroneAdapter(drone);
        _adapter._session = new WeakReference<AutelDroneSession>(this);
        initDrone();
    }

    private void initDrone() {
        initMainController();
        initDroneDetails(0);
    }

    private void initMainController() {
        AutelFlyController flyController = _adapter._drone.getFlyController();
        if (null == flyController) {
            // TODO: log error
            return;
        }

        flyController.isBeginnerModeEnable(new CallbackWithOneParam<Boolean>() {
            @Override
            public void onSuccess(Boolean isEnabled) {
                if (isEnabled) {
                    flyController.setBeginnerModeEnable(false, new CallbackWithNoParam() {
                        @Override
                        public void onSuccess() {
                            // do nothing
                        }

                        @Override
                        public void onFailure(AutelError autelError) {
                            // TODO: log error
                        }
                    });
                }
            }

            @Override
            public void onFailure(AutelError autelError) {
                // TODO: log error
            }
        });

        flyController.setMaxHorizontalSpeed(15, new CallbackWithNoParam() {
            @Override
            public void onSuccess() {
                flyController.getMaxHorizontalSpeed(new CallbackWithOneParam<Float>() {
                    @Override
                    public void onSuccess(Float aFloat) {
                        // TODO: log success
                        _maxHorizontalVelocity = aFloat;
                    }

                    @Override
                    public void onFailure(AutelError autelError) {
                        // TODO: log error
                    }
                });
            }

            @Override
            public void onFailure(AutelError autelError) {
                // TODO: log error
            }
        });
    }

    private void initDroneDetails(int attempt) {
        if (attempt >= 3) {
            return;
        }

        AutelFlyController flyController = _adapter._drone.getFlyController();
        if (null == flyController) {
            // TODO: log error
            return;
        }

        flyController.getSerialNumber(new CallbackWithOneParam<String>() {
            @Override
            public void onSuccess(String serialNumber) {
                _serialNumber = serialNumber;
                // TODO: log serial number
            }

            @Override
            public void onFailure(AutelError autelError) {
                // TODO: log error
            }
        });

        flyController.getVersionInfo(new CallbackWithOneParam<FlyControllerVersionInfo>() {
            @Override
            public void onSuccess(FlyControllerVersionInfo flyControllerVersionInfo) {
                _firmwarePackageVersion = flyControllerVersionInfo.getFlyControllerVersion();
                // TODO: log firmwarePackageVersion
                // TODO: log additional info
            }

            @Override
            public void onFailure(AutelError autelError) {
                // TODO: log error
            }
        });
    }

    public DatedValue<AUTELMCSystemState> getMainControllerState() {
        return _mainControllerState;
    }

    public DatedValue<AUTELBatteryState> getBatteryState() {
        return _batteryState;
    }

    public DatedValue<AUTELRCState> getRemoteControllerRCState() {
        return _remoteControllerRCState;
    }

    private void execute() {
        // TODO: pick back up here
    }
}
