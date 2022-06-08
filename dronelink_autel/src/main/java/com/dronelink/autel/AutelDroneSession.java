package com.dronelink.autel;

import android.content.Context;
import android.graphics.Point;
import android.graphics.PointF;
import android.location.Location;
import android.os.AsyncTask;
import android.os.Handler;
import android.util.Log;
import android.util.SparseArray;

import com.autel.common.CallbackWithNoParam;
import com.autel.common.CallbackWithOneParam;
import com.autel.common.battery.BatteryState;
import com.autel.common.error.AutelError;
import com.autel.common.flycontroller.FlyControllerInfo;
import com.autel.common.flycontroller.FlyControllerVersionInfo;
import com.autel.common.flycontroller.evo.EvoAttitudeInfo;
import com.autel.common.flycontroller.evo.EvoFlyControllerInfo;
import com.autel.common.flycontroller.evo.EvoGpsInfo;
import com.autel.common.mission.AutelCoordinate3D;
import com.autel.common.remotecontroller.RemoteControllerInfo;
import com.autel.internal.sdk.flycontroller.AutelAttitudeInfoInternal;
import com.autel.sdk.battery.AutelBattery;
import com.autel.sdk.flycontroller.AutelFlyController;
import com.autel.sdk.flycontroller.Evo2FlyController;
import com.autel.sdk.product.BaseProduct;
import com.autel.sdk.product.Evo2Aircraft;
import com.dronelink.autel.adapters.AutelDroneAdapter;
import com.dronelink.autel.adapters.AutelRemoteControllerStateAdapter;
import com.dronelink.core.CameraFile;
import com.dronelink.core.DatedValue;
import com.dronelink.core.DroneControlSession;
import com.dronelink.core.DroneSession;
import com.dronelink.core.DroneSessionManager;
import com.dronelink.core.Dronelink;
import com.dronelink.core.Executor;
import com.dronelink.core.MissionExecutor;
import com.dronelink.core.adapters.CameraStateAdapter;
import com.dronelink.core.adapters.DroneAdapter;
import com.dronelink.core.adapters.DroneStateAdapter;
import com.dronelink.core.adapters.GimbalStateAdapter;
import com.dronelink.core.adapters.RemoteControllerStateAdapter;
import com.dronelink.core.command.CommandConfig;
import com.dronelink.core.command.CommandError;
import com.dronelink.core.command.CommandQueue;
import com.dronelink.core.command.MultiChannelCommandQueue;
import com.dronelink.core.kernel.command.Command;
import com.dronelink.core.kernel.command.camera.AEBCountCameraCommand;
import com.dronelink.core.kernel.command.camera.ApertureCameraCommand;
import com.dronelink.core.kernel.command.camera.AutoExposureLockCameraCommand;
import com.dronelink.core.kernel.command.camera.AutoLockGimbalCameraCommand;
import com.dronelink.core.kernel.command.camera.CameraCommand;
import com.dronelink.core.kernel.command.camera.ColorCameraCommand;
import com.dronelink.core.kernel.command.camera.ContrastCameraCommand;
import com.dronelink.core.kernel.command.camera.DisplayModeCameraCommand;
import com.dronelink.core.kernel.command.camera.ExposureCompensationCameraCommand;
import com.dronelink.core.kernel.command.camera.ExposureCompensationStepCameraCommand;
import com.dronelink.core.kernel.command.camera.ExposureModeCameraCommand;
import com.dronelink.core.kernel.command.camera.FileIndexModeCameraCommand;
import com.dronelink.core.kernel.command.camera.FocusCameraCommand;
import com.dronelink.core.kernel.command.camera.FocusDistanceCameraCommand;
import com.dronelink.core.kernel.command.camera.FocusModeCameraCommand;
import com.dronelink.core.kernel.command.camera.FocusRingCameraCommand;
import com.dronelink.core.kernel.command.camera.ISOCameraCommand;
import com.dronelink.core.kernel.command.camera.MechanicalShutterCameraCommand;
import com.dronelink.core.kernel.command.camera.MeteringModeCameraCommand;
import com.dronelink.core.kernel.command.camera.ModeCameraCommand;
import com.dronelink.core.kernel.command.camera.PhotoAspectRatioCameraCommand;
import com.dronelink.core.kernel.command.camera.PhotoFileFormatCameraCommand;
import com.dronelink.core.kernel.command.camera.PhotoIntervalCameraCommand;
import com.dronelink.core.kernel.command.camera.PhotoModeCameraCommand;
import com.dronelink.core.kernel.command.camera.SaturationCameraCommand;
import com.dronelink.core.kernel.command.camera.SharpnessCameraCommand;
import com.dronelink.core.kernel.command.camera.ShutterSpeedCameraCommand;
import com.dronelink.core.kernel.command.camera.SpotMeteringTargetCameraCommand;
import com.dronelink.core.kernel.command.camera.StartCaptureCameraCommand;
import com.dronelink.core.kernel.command.camera.StopCaptureCameraCommand;
import com.dronelink.core.kernel.command.camera.StorageLocationCameraCommand;
import com.dronelink.core.kernel.command.camera.VideoCaptionCameraCommand;
import com.dronelink.core.kernel.command.camera.VideoFileCompressionStandardCameraCommand;
import com.dronelink.core.kernel.command.camera.VideoFileFormatCameraCommand;
import com.dronelink.core.kernel.command.camera.VideoModeCameraCommand;
import com.dronelink.core.kernel.command.camera.VideoResolutionFrameRateCameraCommand;
import com.dronelink.core.kernel.command.camera.VideoStandardCameraCommand;
import com.dronelink.core.kernel.command.camera.VideoStreamSourceCameraCommand;
import com.dronelink.core.kernel.command.camera.WhiteBalanceCustomCameraCommand;
import com.dronelink.core.kernel.command.camera.WhiteBalancePresetCameraCommand;
import com.dronelink.core.kernel.command.drone.AccessoryDroneCommand;
import com.dronelink.core.kernel.command.drone.BeaconDroneCommand;
import com.dronelink.core.kernel.command.drone.CollisionAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.ConnectionFailSafeBehaviorDroneCommand;
import com.dronelink.core.kernel.command.drone.DroneCommand;
import com.dronelink.core.kernel.command.drone.FlightAssistantDroneCommand;
import com.dronelink.core.kernel.command.drone.HomeLocationDroneCommand;
import com.dronelink.core.kernel.command.drone.LandingGearAutomaticMovementDroneCommand;
import com.dronelink.core.kernel.command.drone.LandingGearDeployDroneCommand;
import com.dronelink.core.kernel.command.drone.LandingGearDroneCommand;
import com.dronelink.core.kernel.command.drone.LandingGearRetractDroneCommand;
import com.dronelink.core.kernel.command.drone.LandingProtectionDroneCommand;
import com.dronelink.core.kernel.command.drone.LightbridgeChannelDroneCommand;
import com.dronelink.core.kernel.command.drone.LightbridgeChannelSelectionModeDroneCommand;
import com.dronelink.core.kernel.command.drone.LightbridgeDroneCommand;
import com.dronelink.core.kernel.command.drone.LightbridgeFrequencyBandDroneCommand;
import com.dronelink.core.kernel.command.drone.LowBatteryWarningThresholdDroneCommand;
import com.dronelink.core.kernel.command.drone.MaxAltitudeDroneCommand;
import com.dronelink.core.kernel.command.drone.MaxDistanceDroneCommand;
import com.dronelink.core.kernel.command.drone.MaxDistanceLimitationDroneCommand;
import com.dronelink.core.kernel.command.drone.OcuSyncChannelDroneCommand;
import com.dronelink.core.kernel.command.drone.OcuSyncChannelSelectionModeDroneCommand;
import com.dronelink.core.kernel.command.drone.OcuSyncDroneCommand;
import com.dronelink.core.kernel.command.drone.OcuSyncFrequencyBandDroneCommand;
import com.dronelink.core.kernel.command.drone.OcuSyncVideoFeedSourcesDroneCommand;
import com.dronelink.core.kernel.command.drone.PrecisionLandingDroneCommand;
import com.dronelink.core.kernel.command.drone.ReturnHomeAltitudeDroneCommand;
import com.dronelink.core.kernel.command.drone.ReturnHomeObstacleAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.ReturnHomeRemoteObstacleAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.SeriousLowBatteryWarningThresholdDroneCommand;
import com.dronelink.core.kernel.command.drone.SmartReturnHomeDroneCommand;
import com.dronelink.core.kernel.command.drone.SpotlightBrightnessDroneCommand;
import com.dronelink.core.kernel.command.drone.SpotlightDroneCommand;
import com.dronelink.core.kernel.command.drone.UpwardsAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.VisionAssistedPositioningDroneCommand;
import com.dronelink.core.kernel.command.gimbal.GimbalCommand;
import com.dronelink.core.kernel.command.gimbal.ModeGimbalCommand;
import com.dronelink.core.kernel.command.remotecontroller.RemoteControllerCommand;
import com.dronelink.core.kernel.command.remotecontroller.TargetGimbalChannelRemoteControllerCommand;
import com.dronelink.core.kernel.core.CameraFocusCalibration;
import com.dronelink.core.kernel.core.GeoCoordinate;
import com.dronelink.core.kernel.core.Message;
import com.dronelink.core.kernel.core.Orientation3;
import com.dronelink.core.kernel.core.enums.CameraMode;
import com.dronelink.core.kernel.core.enums.DroneLightbridgeFrequencyBand;
import com.dronelink.core.kernel.core.enums.DroneOcuSyncFrequencyBand;
import com.dronelink.core.kernel.core.enums.ExecutionEngine;

import org.json.JSONException;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class AutelDroneSession implements DroneSession {
    private static final String TAG = AutelDroneSession.class.getCanonicalName();

    private final Context _context;
    private final DroneSessionManager _manager;
    private final AutelDroneAdapter _adapter;
    private final Evo2FlyController _flyController;

    private Date _opened = new Date();
    private boolean _closed = false;
    private UUID _id = UUID.randomUUID();
    private String _serialNumber;
    private String _firmwarePackageVersion;
    private String _model;
    private boolean _initialized = false;
    private boolean _located = false;

    private final List<Listener> listeners = new LinkedList<>();
    private final ExecutorService listenerExecutor = Executors.newSingleThreadExecutor();
    private final CommandQueue _droneCommands = new CommandQueue();
    private final MultiChannelCommandQueue _remoteControllerCommands = new MultiChannelCommandQueue();
    private final MultiChannelCommandQueue _cameraCommands = new MultiChannelCommandQueue();
    private final MultiChannelCommandQueue _gimbalCommands = new MultiChannelCommandQueue();

    private final ExecutorService _mainControllerSerialQueue = Executors.newSingleThreadExecutor();
    private DatedValue<EvoFlyControllerInfo> _mainControllerState;

    private final ExecutorService _batterySerialQueue = Executors.newSingleThreadExecutor();
    private DatedValue<BatteryState> _batteryState;

    private final ExecutorService _remoteControllerSerialQueue = Executors.newSingleThreadExecutor();
    private DatedValue<RemoteControllerInfo> _remoteControllerRCState;
    private DatedValue<RemoteControllerStateAdapter> _remoteControllerStateAdapter;

//    private final ExecutorService _cameraSerialQueue = Executors.newSingleThreadExecutor();
//    private final SparseArray<DatedValue<com.autel.common.camera.base.BaseStateInfo>> _cameraStates = new SparseArray<>();
//    private final SparseArray<DatedValue<com.autel.common.camera.base.SdCardInternal>> _cameraStorageStates = new SparseArray<>();
//    private final SparseArray<DatedValue<com.autel.common.camera.XT706.XT706CameraInfo>> _cameraExposureParameters = new SparseArray<>();

    private final ExecutorService _gimbalSerialQueue = Executors.newSingleThreadExecutor();
    private final SparseArray<DatedValue<GimbalStateAdapter>> _gimbalStates = new SparseArray<>();

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

    // extended variables
    private String _mode;
    private boolean _isFlying = false;
    private boolean isHomeValid = false;
    private AutelLocation homeLocation;
    private AutelLocation _lastKnownGroundLocation;
    private AutelLocation _takeOffLocation;
    private Double _takeOffAltitude = null;
    private double _course = 0;
    private double _horizontalSpeed = 0;
    private double _verticalSpeed = 0;
    private double _altitude = 0;
    private Double _ultrasonicAltitude = null;
    private Orientation3 _orientation = null;

    public AutelDroneSession(final Context context, DroneSessionManager manager, BaseProduct drone) throws Exception {
        switch (drone.getType()) {
            case EVO_2:
                _flyController = ((Evo2Aircraft) drone).getFlyController();

                if (null == _flyController) {
                    Log.e(TAG, "Main controller unavailable");
                    throw new Exception("Main controller unavailable");
                }
                break;

            default:
                throw new Exception("Unsupported drone type: We only support the EVO 2");
        }

        _context = context;
        _manager = manager;
        _adapter = new AutelDroneAdapter(drone);
        _adapter._session = new WeakReference<AutelDroneSession>(this);

        initDrone();
    }

    private void initDrone() {
        initMainController();
        initDroneDetails(0);

        new Thread() {
            @Override
            public void run() {
                execute();
            }
        }.start();
    }

    private void initMainController() {
        _flyController.isBeginnerModeEnable(new CallbackWithOneParam<Boolean>() {
            @Override
            public void onSuccess(Boolean isEnabled) {
                if (isEnabled) {
                    _flyController.setBeginnerModeEnable(false, new CallbackWithNoParam() {
                        @Override
                        public void onSuccess() {
                            Log.d(TAG, "Main controller beginner mode disabled");
                        }

                        @Override
                        public void onFailure(AutelError autelError) {
                            Log.e(TAG, "Unable to set main controller beginner mode");
                        }
                    });
                }
            }

            @Override
            public void onFailure(AutelError autelError) {
                Log.e(TAG, "Unable to query main controller beginner mode");
            }
        });

        _flyController.setMaxHorizontalSpeed(15, new CallbackWithNoParam() {
            @Override
            public void onSuccess() {
                _flyController.getMaxHorizontalSpeed(new CallbackWithOneParam<Float>() {
                    @Override
                    public void onSuccess(Float aFloat) {
                        Log.d(TAG, "Main controller max horizontal velocity: " + aFloat);
                        _maxHorizontalVelocity = aFloat;
                    }

                    @Override
                    public void onFailure(AutelError autelError) {
                        Log.d(TAG, "Main controller max horizontal velocity unknown");
                    }
                });
            }

            @Override
            public void onFailure(AutelError autelError) {
                Log.e(TAG, "Unable to set main controller max horizontal velocity");
            }
        });

        _flyController.setFlyControllerInfoListener(new CallbackWithOneParam<EvoFlyControllerInfo>() {
            @Override
            public void onSuccess(EvoFlyControllerInfo flyControllerInfo) {
                _mainControllerSerialQueue.submit(new Callable<Object>() {
                    @Override
                    public Object call() throws Exception {
                        _mainControllerState = new DatedValue<EvoFlyControllerInfo>(flyControllerInfo);
                        return null;
                    }
                });
            }

            @Override
            public void onFailure(AutelError autelError) {

            }
        });
    }

    private void initDroneDetails(int attempt) {
        if (attempt >= 3) {
            return;
        }

        if (null == _flyController) {
            Log.e(TAG, "Main controller unavailable");
            return;
        }

        _flyController.getSerialNumber(new CallbackWithOneParam<String>() {
            @Override
            public void onSuccess(String serialNumber) {
                _serialNumber = serialNumber;
                Log.i(TAG, "Serial number: " + _serialNumber);
            }

            @Override
            public void onFailure(AutelError autelError) {
                Log.e(TAG, "Unable to retrieve serial number");
            }
        });

        _flyController.getVersionInfo(new CallbackWithOneParam<FlyControllerVersionInfo>() {
            @Override
            public void onSuccess(FlyControllerVersionInfo flyControllerVersionInfo) {
                _firmwarePackageVersion = flyControllerVersionInfo.getFlyControllerVersion();
                Log.i(TAG, "Firmware package version: " + _firmwarePackageVersion);
            }

            @Override
            public void onFailure(AutelError autelError) {
                Log.e(TAG, "Unable to retrieve firmware package version");
            }
        });

        _model = _adapter._drone.getType().getDescription();
    }

    public class AutelLocation {
        public AutelCoordinate3D coord3D;
        public EvoAttitudeInfo attitude;
    }

    private AutelLocation getAircraftLocation()  {
        DatedValue<EvoFlyControllerInfo> mainControllerState = getMainControllerState();
        if (null != mainControllerState && null != mainControllerState.value) {
            EvoGpsInfo gpsInfo = mainControllerState.value.getGpsInfo();
            if (null != gpsInfo) {
                AutelCoordinate3D coord3D = new AutelCoordinate3D(gpsInfo.getLatitude(), gpsInfo.getLongitude(), gpsInfo.getAltitude());
                if (null != coord3D) {
                    AutelLocation result = new AutelLocation();
                    result.coord3D = coord3D;
                    result.attitude = _mainControllerState.value.getAttitudeInfo();
                    return result;
                }
            }
        }
        return null;
    }

    private Double getBatteryPercent() {
        try
        {
            return _batterySerialQueue.submit(new Callable<Double>() {
                @Override
                public Double call() throws Exception {
                    if (null != _batteryState.value) {
                        int remainingPercent = _batteryState.value.getRemainingPercent();
                        return (double) remainingPercent / 100.0;
                    }
                    return null;
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    private Double getLowBatteryThreshold() {
        // TODO: fixme
        return null;
    }

    private Double getFlightTimeRemaining() {
        // TODO: fixme
        return null;
    }

    private Double getObstacleDistance() {
        // TODO: fixme
        return null;
    }

    public Orientation3 getOrientation() {
        return _orientation != null ? _orientation : new Orientation3();
    }

    public Integer getGpsSatellitesCount() {
        DatedValue<EvoFlyControllerInfo> mainControllerState = getMainControllerState();
        if (null != mainControllerState && null != mainControllerState.value) {
            EvoGpsInfo gpsInfo = mainControllerState.value.getGpsInfo();
            if (null != gpsInfo) {
                return gpsInfo.getSatellitesVisible();
            }
        }
        return null;
    }

    public Double getGpsSignalStrength() {
        DatedValue<EvoFlyControllerInfo> mainControllerState = getMainControllerState();
        if (null != mainControllerState && null != mainControllerState.value) {
            EvoGpsInfo gpsInfo = mainControllerState.value.getGpsInfo();
            if (null != gpsInfo) {
                return (double)gpsInfo.getGpsLevel();
            }
        }
        return null;
    }

    public Double getDownlinkSignalStrength() {
        DatedValue<RemoteControllerInfo> remoteControllerState = getRemoteControllerRCState();
        if (null != remoteControllerState && null != remoteControllerState.value) {
            return (double)remoteControllerState.value.getDSPPercentage();
        }
        return null;
    }

    public Double getUplinkSignalStrength() {
        DatedValue<RemoteControllerInfo> remoteControllerState = getRemoteControllerRCState();
        if (null != remoteControllerState && null != remoteControllerState.value) {
            return (double)remoteControllerState.value.getControllerSignalPercentage();
        }
        return null;
    }

    public DroneLightbridgeFrequencyBand getLightbridgeFrequencyBand() {
        return null;
    }

    public DroneOcuSyncFrequencyBand getOcuSyncFrequencyBand() {
        return null;
    }

    public DatedValue<EvoFlyControllerInfo> getMainControllerState() {
        try {
            AutelDroneSession self = this;
            return _mainControllerSerialQueue.submit(new Callable<DatedValue<EvoFlyControllerInfo>>() {
                @Override
                public DatedValue<EvoFlyControllerInfo> call() throws Exception {
                    return self._mainControllerState;
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    ; public DatedValue<BatteryState> getBatteryState() {
        try {
            AutelDroneSession self = this;
            return _batterySerialQueue.submit(new Callable<DatedValue<BatteryState>>() {
                @Override
                public DatedValue<BatteryState> call() throws Exception {
                    return self._batteryState;
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    public DatedValue<RemoteControllerInfo> getRemoteControllerRCState() {
        try {
            AutelDroneSession self = this;
            return _remoteControllerSerialQueue.submit(new Callable<DatedValue<RemoteControllerInfo>>() {
                @Override
                public DatedValue<RemoteControllerInfo> call() throws Exception {
                    return self._remoteControllerRCState;
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    private void execute() {
        while (!_closed) {
            if (!_initialized && _serialNumber != null && _firmwarePackageVersion != null) {
                _initialized = true;
                onInitialized();
            }

            AutelLocation location = getAircraftLocation();
            if (null != location) {
                _located = true;
                onLocated();

                if (!_isFlying) {

                }
            }
        }
    }

    public AutelDroneAdapter getAdapter() { return _adapter; }

    private void getLocation() {
    }

    @Override
    public DroneSessionManager getManager() {
        return _manager;
    }

    @Override
    public DroneAdapter getDrone() {
        return _adapter;
    }

    @Override
    public DatedValue<DroneStateAdapter> getState() {
        return null;
    }

    @Override
    public Date getOpened() {
        return _opened;
    }

    @Override
    public boolean isClosed() {
        return _closed;
    }

    @Override
    public String getId() {
        return null;
    }

    @Override
    public String getManufacturer() {
        return "Autel";
    }

    @Override
    public String getSerialNumber() {
        return _serialNumber;
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public String getModel() {
        return _model;
    }

    @Override
    public String getFirmwarePackageVersion() {
        return _firmwarePackageVersion;
    }

    @Override
    public boolean isInitialized() {
        return _initialized;
    }

    @Override
    public boolean isLocated() {
        return _located;
    }

    @Override
    public boolean isTelemetryDelayed() {
        return System.currentTimeMillis() - getState().date.getTime() > 2000;
    }

    @Override
    public Message getDisengageReason() {
//        if (_closed) {
//            return new Message(_context.getString(R.string.MissionDisengageReason_drone_disconnected_title));
//        }
//
//        if (_adapter._drone.getFlightController() == null) {
//            return new Message(context.getString(R.string.MissionDisengageReason_drone_control_unavailable_title));
//        }
//
//        final DatedValue<FlightControllerState> flightControllerState = state.flightControllerState;
//        if (flightControllerState == null || flightControllerState.value == null) {
//            return new Message(context.getString(R.string.MissionDisengageReason_telemetry_unavailable_title));
//        }
//
//        if (isTelemetryDelayed()) {
//            return new Message(context.getString(R.string.MissionDisengageReason_telemetry_delayed_title));
//        }
//
//        if (flightControllerState.value.hasReachedMaxFlightHeight()) {
//            return new Message(context.getString(R.string.MissionDisengageReason_drone_max_altitude_title), context.getString(R.string.MissionDisengageReason_drone_max_altitude_details));
//        }
//
//        if (flightControllerState.value.hasReachedMaxFlightRadius()) {
//            return new Message(context.getString(R.string.MissionDisengageReason_drone_max_distance_title), context.getString(R.string.MissionDisengageReason_drone_max_distance_details));
//        }

        return null;
    }

    @Override
    public void identify(String s) {

    }

    @Override
    public void addListener(Listener listener) {
        final DroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                listeners.add(listener);

                if (_initialized) {
                    listener.onInitialized(self);
                }

                if (_located) {
                    listener.onLocated(self);
                }
            }
        });
    }

    @Override
    public void removeListener(Listener listener) {
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                listeners.remove(listener);
            }
        });
    }

    private void onInitialized() {
        for (final Listener listener : listeners) {
            listener.onInitialized(this);
        }
    }

    private void onLocated() {
        final AutelDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onLocated(self);
                }
            }
        });
    }

    private void onMotorsChanged(final boolean value) {
        final AutelDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onMotorsChanged(self, value);
                }
            }
        });
    }

    private void onCommandExecuted(final com.dronelink.core.kernel.command.Command command) {
        final AutelDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onCommandExecuted(self, command);
                }
            }
        });
    }

    private void onCommandFinished(final com.dronelink.core.kernel.command.Command command, final CommandError error) {
        final AutelDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onCommandFinished(self, command, error);
                }
            }
        });
    }

//    private void onCameraFileGenerated(final DJICameraFile file) {
//        for (final Listener listener : listeners) {
//            listener.onCameraFileGenerated(this, file);
//        }
//    }

    @Override
    public void addCommand(Command command) throws Dronelink.UnregisteredException, CommandTypeUnhandledException {
        com.dronelink.core.command.Command.Executor executor = null;

        if (command instanceof DroneCommand) {
            executor = new com.dronelink.core.command.Command.Executor() {
                @Override
                public CommandError execute(final com.dronelink.core.command.Command.Finisher finished) {
                    onCommandExecuted(command);
                    return executeDroneCommand((DroneCommand)command, finished);
                }
            };
        }
        else if (command instanceof RemoteControllerCommand) {
            executor = new com.dronelink.core.command.Command.Executor() {
                @Override
                public CommandError execute(final com.dronelink.core.command.Command.Finisher finished) {
                    onCommandExecuted(command);
                    return executeRemoteControllerCommand((RemoteControllerCommand) command, finished);
                }
            };
        }
//        else if (command instanceof CameraCommand) {
//            executor = new com.dronelink.core.command.Command.Executor() {
//                @Override
//                public CommandError execute(final com.dronelink.core.command.Command.Finisher finished) {
//                    onCommandExecuted(command);
//                    return executeCameraCommand((CameraCommand)command, finished);
//                }
//            };
//        }
//        else if (command instanceof GimbalCommand) {
//            executor = new com.dronelink.core.command.Command.Executor() {
//                @Override
//                public CommandError execute(final com.dronelink.core.command.Command.Finisher finished) {
//                    onCommandExecuted(command);
//                    return executeGimbalCommand((GimbalCommand)command, finished);
//                }
//            };
//        }

        if (executor != null) {
            final com.dronelink.core.command.Command c = new com.dronelink.core.command.Command(
                    command,
                    executor,
                    new com.dronelink.core.command.Command.Finisher() {
                        @Override
                        public void execute(final CommandError error) {
                            onCommandFinished(command, error);
                        }
                    },
                    command.getConfig());

            if (c.config.retriesEnabled == null) {
                //disable retries when the DJI SDK reports that the product does not support the feature
                c.config.retriesEnabled = new CommandConfig.RetriesEnabled() {
                    @Override
                    public boolean execute(final CommandError error) {
                        if (error != null && error.code == DJIError.COMMAND_NOT_SUPPORTED_BY_HARDWARE.getErrorCode()) {
                            return false;
                        }
                        return true;
                    }
                };

                if (c.config.finishDelayMillis == null) {
                    //adding a 1.5 second delay after camera and gimbal mode commands
                    if (command instanceof ModeCameraCommand || command instanceof ModeGimbalCommand) {
                        c.config.finishDelayMillis = 1500.0;
                    }
                }
            }

            if (command instanceof DroneCommand) {
                _droneCommands.addCommand(c);
            }
            else if (command instanceof RemoteControllerCommand) {
                _remoteControllerCommands.addCommand(((RemoteControllerCommand)command).channel, c);
            }
//            else if (command instanceof CameraCommand) {
//                _cameraCommands.addCommand(((CameraCommand)command).channel, c);
//            }
//            else if (command instanceof GimbalCommand) {
//                _gimbalCommands.addCommand(((GimbalCommand)command).channel, c);
//            }
            return;
        }

        throw new CommandTypeUnhandledException();
    }

    @Override
    public void removeCommands() {
        _droneCommands.removeAll();
        _remoteControllerCommands.removeAll();
        _cameraCommands.removeAll();
        _gimbalCommands.removeAll();
    }

    @Override
    public DroneControlSession createControlSession(Context context, ExecutionEngine executionEngine, Executor executor) throws UnsupportedExecutionEngineException, UnsupportedDroneDJIExecutionEngineException {
        switch (executionEngine) {
            case DRONELINK_KERNEL:
                return new AutelVirtualStickSession(context, this);

            case DJI:
                switch (_adapter._drone.getType()) {
                    // we only support the EVO 2
                    case EVO_2:
                        break;

                    default:
                        throw new UnsupportedDroneDJIExecutionEngineException();

                }

//                if (executor instanceof MissionExecutor) {
//                    try {
//                        return new AutelWaypointMissionSession(context, this, (MissionExecutor)executor);
//                    } catch (final JSONException e) {
//                        throw new UnsupportedExecutionEngineException(executionEngine);
//                    }
//                }
                break;

            default:
                break;
        }

        throw new UnsupportedExecutionEngineException(executionEngine);
    }

    @Override
    public DatedValue<RemoteControllerStateAdapter> getRemoteControllerState(int i) {
        try {
            return _remoteControllerSerialQueue.submit(new Callable<DatedValue<RemoteControllerStateAdapter>>() {
                @Override
                public DatedValue<RemoteControllerStateAdapter> call() {
                    if (_remoteControllerRCState == null) {
                        return null;
                    }

                    final RemoteControllerStateAdapter remoteControllerStateAdapter = new AutelRemoteControllerStateAdapter(_remoteControllerRCState.value);
                    return new DatedValue<>(remoteControllerStateAdapter, _remoteControllerRCState.date);
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    @Override
    public DatedValue<CameraStateAdapter> getCameraState(int channel) {
        return getCameraState(channel, null);
    }

    @Override
    public DatedValue<CameraStateAdapter> getCameraState(int channel, Integer lensIndex) {
        return null;
    }

    @Override
    public DatedValue<GimbalStateAdapter> getGimbalState(int channel) {
//        try {
//            return _gimbalSerialQueue.submit(new Callable<DatedValue<GimbalStateAdapter>>() {
//                @Override
//                public DatedValue<GimbalStateAdapter> call() {
//                    final DatedValue<GimbalState> gimbalState = gimbalStates.get(channel);
//                    if (gimbalState == null) {
//                        return null;
//                    }
//
//                    final GimbalStateAdapter gimbalStateAdapter = new DJIGimbalStateAdapter(gimbalState.value);
//                    return new DatedValue<>(gimbalStateAdapter, gimbalState.date);
//                }
//            }).get();
//        }
//        catch (final ExecutionException | InterruptedException e) {
//            return null;
//        }
        return null;
    }

    @Override
    public void resetPayloads() {
//        sendResetGimbalCommands();
//        sendResetCameraCommands();
    }

    @Override
    public void close() {
        _closed = true;
    }

    private CallbackWithNoParam createCompletionCallback(final com.dronelink.core.command.Command.Finisher finished) {
        return new CallbackWithNoParam() {
            @Override
            public void onSuccess() {
                finished.execute(null);
            }

            @Override
            public void onFailure(AutelError autelError) {
                finished.execute(DronelinkAutel.createCommandError(autelError));
            }
        };
    }

    private <V> CallbackWithOneParam<V> createCompletionCallbackWith(final com.dronelink.core.command.Command.FinisherWith<V> success, final com.dronelink.core.command.Command.Finisher error) {
        return new CallbackWithOneParam<V>() {
            @Override
            public void onSuccess(final V value) {
                success.execute(value);
            }

            @Override
            public void onFailure(final AutelError djiError) {
                error.execute(DronelinkAutel.createCommandError(djiError));
            }
        };
    }

    private CommandError executeDroneCommand(final DroneCommand command, final com.dronelink.core.command.Command.Finisher finished) {
        if (command instanceof FlightAssistantDroneCommand) {
            return executeFlightAssistantDroneCommand((FlightAssistantDroneCommand) command, finished);
        }

        if (command instanceof LandingGearDroneCommand) {
            return executeLandingGearDroneCommand((LandingGearDroneCommand) command, finished);
        }

        if (command instanceof LightbridgeDroneCommand) {
            return executeLightbridgeDroneCommand((LightbridgeDroneCommand) command, finished);
        }

        if (command instanceof OcuSyncDroneCommand) {
            return executeOcuSyncDroneCommand((OcuSyncDroneCommand) command, finished);
        }

        if (command instanceof AccessoryDroneCommand) {
            return executeAccessoryDroneCommand((AccessoryDroneCommand) command, finished);
        }

        final Evo2FlyController flyController = (Evo2FlyController)_adapter._drone.getFlyController();
        if (flyController == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_control_unavailable_title));
        }

        // TODO: what should this be in the Autel SDK?
        if (command instanceof ConnectionFailSafeBehaviorDroneCommand) {
            flyController.getConnectionFailSafeBehavior(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<ConnectionFailSafeBehavior>() {
                @Override
                public void execute(final ConnectionFailSafeBehavior current) {
                    final ConnectionFailSafeBehavior target = DronelinkDJI.getDroneConnectionFailSafeBehavior(((ConnectionFailSafeBehaviorDroneCommand) command).connectionFailSafeBehavior);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setConnectionFailSafeBehavior(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof HomeLocationDroneCommand) {
            final GeoCoordinate coordinate = ((HomeLocationDroneCommand) command).coordinate;
            flyController.setLocationAsHomePoint(DronelinkAutel.getCoordinate(coordinate), createCompletionCallback(finished));
            return null;
        }

        if (command instanceof LowBatteryWarningThresholdDroneCommand) {
            AutelBattery battery = _adapter._drone.getBattery();
            battery.getLowBatteryNotifyThreshold(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Float>() {
                @Override
                public void execute(final Float current) {
                    final Float target = (float)(((LowBatteryWarningThresholdDroneCommand) command).lowBatteryWarningThreshold * 100);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            battery.setLowBatteryNotifyThreshold(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof MaxAltitudeDroneCommand) {
            flyController.getMaxHeight(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Float() {
                @Override
                public void execute(final Float current) {
                    final Float target = (float)(((MaxAltitudeDroneCommand) command).maxAltitude);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flyController.setMaxHeight(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof MaxDistanceDroneCommand) {
            flyController.getMaxRange(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Float>() {
                @Override
                public void execute(final Float current) {
                    final Float target = (float)(((MaxDistanceDroneCommand) command).maxDistance);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flyController.setMaxRange(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        // TODO: where is this in the Autel SDK?
//        if (command instanceof MaxDistanceLimitationDroneCommand) {
//            flightController.getMaxFlightRadiusLimitationEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((MaxDistanceLimitationDroneCommand) command).enabled;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            flightController.setMaxFlightRadiusLimitationEnabled(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }

        // TODO: this doesn't seem to be part of the Autel SDK
//        if (command instanceof ReturnHomeAltitudeDroneCommand) {
//            flightController.getGoHomeHeightInMeters(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Integer>() {
//                @Override
//                public void execute(final Integer current) {
//                    final Integer target = (int)(((ReturnHomeAltitudeDroneCommand) command).returnHomeAltitude);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            flightController.setGoHomeHeightInMeters(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }

        if (command instanceof SeriousLowBatteryWarningThresholdDroneCommand) {
            AutelBattery battery = _adapter._drone.getBattery();
            battery.getCriticalBatteryNotifyThreshold(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Float>() {
                @Override
                public void execute(final Float current) {
                    final Float target = (float)(((SeriousLowBatteryWarningThresholdDroneCommand) command).seriousLowBatteryWarningThreshold * 100);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            battery.setCriticalBatteryNotifyThreshold(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        // TODO: what is this in the Autel SDK?
//        if (command instanceof SmartReturnHomeDroneCommand) {
//            flightController.getSmartReturnToHomeEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((SmartReturnHomeDroneCommand) command).enabled;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            flightController.setSmartReturnToHomeEnabled(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeFlightAssistantDroneCommand(final FlightAssistantDroneCommand command, final com.dronelink.core.command.Command.Finisher finished) {
        final FlightController flightController = adapter.getDrone().getFlightController();
        if (flightController == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_flight_assistant_unavailable_title));
        }

        final FlightAssistant flightAssistant = flightController.getFlightAssistant();
        if (flightAssistant == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_flight_assistant_unavailable_title));
        }

        if (command instanceof CollisionAvoidanceDroneCommand) {
//            flightAssistant.getCollisionAvoidanceEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((CollisionAvoidanceDroneCommand) command).enabled;
//                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            flightAssistant.setCollisionAvoidanceEnabled(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
            //skipping conditional execution for now because it seems like the DJI SDK always returns true for getCollisionAvoidanceEnabled
            flightAssistant.setCollisionAvoidanceEnabled(((CollisionAvoidanceDroneCommand) command).enabled, createCompletionCallback(finished));
            return null;
        }

        if (command instanceof LandingProtectionDroneCommand) {
            flightAssistant.getLandingProtectionEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((LandingProtectionDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setLandingProtectionEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof PrecisionLandingDroneCommand) {
            flightAssistant.getPrecisionLandingEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((PrecisionLandingDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setPrecisionLandingEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof ReturnHomeObstacleAvoidanceDroneCommand) {
            flightAssistant.getRTHObstacleAvoidanceEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((ReturnHomeObstacleAvoidanceDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setRTHObstacleAvoidanceEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof ReturnHomeRemoteObstacleAvoidanceDroneCommand) {
            flightAssistant.getRTHRemoteObstacleAvoidanceEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((ReturnHomeRemoteObstacleAvoidanceDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setRTHRemoteObstacleAvoidanceEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof UpwardsAvoidanceDroneCommand) {
            flightAssistant.getUpwardVisionObstacleAvoidanceEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((UpwardsAvoidanceDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setUpwardVisionObstacleAvoidanceEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof VisionAssistedPositioningDroneCommand) {
            flightAssistant.getVisionAssistedPositioningEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((VisionAssistedPositioningDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setVisionAssistedPositioningEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeLandingGearDroneCommand(final LandingGearDroneCommand command, final com.dronelink.core.command.Command.Finisher finished) {
        final LandingGear landingGear = adapter.getDrone().getFlightController().getLandingGear();
        if (landingGear == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_landing_gear_unavailable_title));
        }

        if (command instanceof LandingGearAutomaticMovementDroneCommand) {
            landingGear.getAutomaticMovementEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((LandingGearAutomaticMovementDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            landingGear.setAutomaticMovementEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof LandingGearDeployDroneCommand) {
            com.dronelink.core.command.Command.conditionallyExecute(!(landingGear.getState() == LandingGearState.DEPLOYED || landingGear.getState() == LandingGearState.DEPLOYING), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                @Override
                public void execute() {
                    landingGear.deploy(createCompletionCallback(finished));
                }
            });
            return null;
        }

        if (command instanceof LandingGearRetractDroneCommand) {
            com.dronelink.core.command.Command.conditionallyExecute(!(landingGear.getState() == LandingGearState.RETRACTED || landingGear.getState() == LandingGearState.RETRACTING), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                @Override
                public void execute() {
                    landingGear.retract(createCompletionCallback(finished));
                }
            });
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeLightbridgeDroneCommand(final LightbridgeDroneCommand command, final com.dronelink.core.command.Command.Finisher finished) {
        final AirLink airLink = adapter.getDrone().getAirLink();
        if (airLink == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_lightbridge_unavailable_title));
        }

        final LightbridgeLink link = airLink.getLightbridgeLink();
        if (link == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_lightbridge_unavailable_title));
        }

        if (command instanceof LightbridgeChannelDroneCommand) {
            link.getChannelNumber(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final Integer target = ((LightbridgeChannelDroneCommand) command).lightbridgeChannel;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            link.setChannelNumber(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof LightbridgeChannelSelectionModeDroneCommand) {
            link.getChannelSelectionMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<ChannelSelectionMode>() {
                @Override
                public void execute(final ChannelSelectionMode current) {
                    final ChannelSelectionMode target = DronelinkDJI.getLightbridgeChannelSelectionMode(((LightbridgeChannelSelectionModeDroneCommand) command).lightbridgeChannelSelectionMode);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            link.setChannelSelectionMode(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof LightbridgeFrequencyBandDroneCommand) {
            link.getFrequencyBand(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<LightbridgeFrequencyBand>() {
                @Override
                public void execute(final LightbridgeFrequencyBand current) {
                    final LightbridgeFrequencyBand target = DronelinkDJI.getLightbridgeFrequencyBand(((LightbridgeFrequencyBandDroneCommand) command).lightbridgeFrequencyBand);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            link.setFrequencyBand(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeOcuSyncDroneCommand(final OcuSyncDroneCommand command, final com.dronelink.core.command.Command.Finisher finished) {
        final AirLink airLink = adapter.getDrone().getAirLink();
        if (airLink == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_ocusync_unavailable_title));
        }

        final OcuSyncLink link = airLink.getOcuSyncLink();
        if (link == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_ocusync_unavailable_title));
        }

        if (command instanceof OcuSyncChannelDroneCommand) {
            link.getChannelNumber(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final Integer target = ((OcuSyncChannelDroneCommand) command).ocuSyncChannel;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            link.setChannelNumber(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof OcuSyncChannelSelectionModeDroneCommand) {
            link.getChannelSelectionMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<ChannelSelectionMode>() {
                @Override
                public void execute(final ChannelSelectionMode current) {
                    final ChannelSelectionMode target = DronelinkDJI.getOcuSyncChannelSelectionMode(((OcuSyncChannelSelectionModeDroneCommand) command).ocuSyncChannelSelectionMode);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            link.setChannelSelectionMode(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof OcuSyncFrequencyBandDroneCommand) {
            link.getFrequencyBand(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<OcuSyncFrequencyBand>() {
                @Override
                public void execute(final OcuSyncFrequencyBand current) {
                    final OcuSyncFrequencyBand target = DronelinkDJI.getOcuSyncFrequencyBand(((OcuSyncFrequencyBandDroneCommand) command).ocuSyncFrequencyBand);
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            link.setFrequencyBand(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof OcuSyncVideoFeedSourcesDroneCommand) {
            link.assignSourceToPrimaryChannel(
                    DronelinkDJI.getOcuSyncFeedSource((OcuSyncVideoFeedSourcesDroneCommand) command, 0),
                    DronelinkDJI.getOcuSyncFeedSource((OcuSyncVideoFeedSourcesDroneCommand) command, 1),
                    createCompletionCallback(finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeAccessoryDroneCommand(final AccessoryDroneCommand command, final com.dronelink.core.command.Command.Finisher finished) {
        final AccessoryAggregation accessoryAggregation = adapter.getDrone().getAccessoryAggregation();
        if (accessoryAggregation == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_accessory_aggregation_unavailable_title));
        }

        if (command instanceof BeaconDroneCommand) {
            final Beacon beacon = accessoryAggregation.getBeacon();
            if (beacon == null) {
                return new CommandError(context.getString(R.string.MissionDisengageReason_drone_beacon_unavailable_title));
            }

            beacon.getEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((BeaconDroneCommand) command).enabled;
                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            beacon.setEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof SpotlightDroneCommand || command instanceof SpotlightBrightnessDroneCommand) {
            final Spotlight spotlight = accessoryAggregation.getSpotlight();
            if (spotlight == null) {
                return new CommandError(context.getString(R.string.MissionDisengageReason_drone_spotlight_unavailable_title));
            }

            if (command instanceof SpotlightDroneCommand) {
                spotlight.getEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
                    @Override
                    public void execute(final Boolean current) {
                        final Boolean target = ((SpotlightDroneCommand) command).enabled;
                        com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                            @Override
                            public void execute() {
                                spotlight.setEnabled(target, createCompletionCallback(finished));
                            }
                        });
                    }
                }, finished));
                return null;
            }

            spotlight.setBrightness((int)(((SpotlightBrightnessDroneCommand) command).spotlightBrightness * 100), createCompletionCallback(finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeRemoteControllerCommand(final RemoteControllerCommand command, final com.dronelink.core.command.Command.Finisher finished) {
        final RemoteController remoteController = DronelinkDJI.getRemoteController(adapter.getDrone(), command.channel);
        if (remoteController == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_remote_controller_unavailable_title));
        }

        if (command instanceof TargetGimbalChannelRemoteControllerCommand) {
            remoteController.getControllingGimbalIndex(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final int target = ((TargetGimbalChannelRemoteControllerCommand) command).targetGimbalChannel;
                    com.dronelink.core.command.Command.conditionallyExecute(target != current, finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            remoteController.setControllingGimbalIndex(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

//    private CommandError executeCameraCommand(final CameraCommand command, final com.dronelink.core.command.Command.Finisher finished) {
//        final Camera camera = DronelinkDJI.getCamera(adapter.getDrone(), command.channel);
//        final DatedValue<CameraStateAdapter> state = getCameraState(command.channel);
//        if (camera == null || state == null || !(state.value instanceof DJICameraStateAdapter)) {
//            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_camera_unavailable_title));
//        }
//        final DJICameraStateAdapter djiState = (DJICameraStateAdapter)state.value;
//
//        if (command instanceof AEBCountCameraCommand) {
//            camera.getPhotoAEBCount(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.PhotoAEBCount>() {
//                @Override
//                public void execute(final SettingsDefinitions.PhotoAEBCount current) {
//                    final SettingsDefinitions.PhotoAEBCount target = DronelinkDJI.getCameraAEBCount(((AEBCountCameraCommand) command).aebCount);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setPhotoAEBCount(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof ApertureCameraCommand) {
//            final SettingsDefinitions.Aperture target = DronelinkDJI.getCameraAperture(((ApertureCameraCommand) command).aperture);
//            com.dronelink.core.command.Command.conditionallyExecute(djiState.exposureSettings.getAperture() != target, finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                @Override
//                public void execute() {
//                    camera.setAperture(target, createCompletionCallback(finished));
//                }
//            });
//            return null;
//        }
//
//        if (command instanceof AutoLockGimbalCameraCommand) {
//            camera.getAutoLockGimbalEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((AutoLockGimbalCameraCommand) command).enabled;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setAutoLockGimbalEnabled(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof AutoExposureLockCameraCommand) {
//            camera.getAELock(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((AutoExposureLockCameraCommand) command).enabled;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setAELock(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof DisplayModeCameraCommand) {
//            if (adapter.drone.getModel() == Model.MAVIC_2_ENTERPRISE_DUAL || camera.getDisplayName() == Camera.DisplayNameXT2_IR) {
//                camera.getDisplayMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.DisplayMode>() {
//                    @Override
//                    public void execute(final SettingsDefinitions.DisplayMode current) {
//                        final SettingsDefinitions.DisplayMode target = DronelinkDJI.getCameraDisplayMode(((DisplayModeCameraCommand) command).displayMode);
//                        com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                            @Override
//                            public void execute() {
//                                camera.setDisplayMode(target, createCompletionCallback(finished));
//                            }
//                        });
//                    }
//                }, finished));
//                return null;
//            }
//
//            final Lens lens = DronelinkDJI.getLens(camera, ((DisplayModeCameraCommand) command).lensIndex);
//            if (lens == null) {
//                return new CommandError(context.getString(R.string.MissionDisengageReason_drone_lens_unavailable_title));
//            }
//
//            lens.getDisplayMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.DisplayMode>() {
//                @Override
//                public void execute(final SettingsDefinitions.DisplayMode current) {
//                    final SettingsDefinitions.DisplayMode target = DronelinkDJI.getCameraDisplayMode(((DisplayModeCameraCommand) command).displayMode);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            lens.setDisplayMode(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof ColorCameraCommand) {
//            camera.getColor(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.CameraColor>() {
//                @Override
//                public void execute(final SettingsDefinitions.CameraColor current) {
//                    final SettingsDefinitions.CameraColor target = DronelinkDJI.getCameraColor(((ColorCameraCommand) command).color);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setColor(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof ContrastCameraCommand) {
//            camera.getContrast(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Integer>() {
//                @Override
//                public void execute(final Integer current) {
//                    final Integer target = ((ContrastCameraCommand) command).contrast;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setContrast(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof ExposureCompensationCameraCommand) {
//            final SettingsDefinitions.ExposureCompensation target = DronelinkDJI.getCameraExposureCompensation(((ExposureCompensationCameraCommand) command).exposureCompensation);
//            com.dronelink.core.command.Command.conditionallyExecute(DronelinkDJI.getCameraExposureCompensation(djiState.getExposureCompensation()) != target, finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                @Override
//                public void execute() {
//                    final Lens lens = DronelinkDJI.getLens(camera, djiState.getLensIndex());
//                    if (lens == null) {
//                        camera.setExposureCompensation(target, createCompletionCallback(finished));
//                    }
//                    else {
//                        lens.setExposureCompensation(target, createCompletionCallback(finished));
//                    }
//                }
//            });
//            return null;
//        }
//
//        if (command instanceof ExposureCompensationStepCameraCommand) {
//            final SettingsDefinitions.ExposureCompensation target = DronelinkDJI.getCameraExposureCompensation(djiState.getExposureCompensation().offset(((ExposureCompensationStepCameraCommand) command).exposureCompensationSteps));
//            com.dronelink.core.command.Command.conditionallyExecute(DronelinkDJI.getCameraExposureCompensation(djiState.getExposureCompensation()) != target, finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                @Override
//                public void execute() {
//                    final Lens lens = DronelinkDJI.getLens(camera, djiState.getLensIndex());
//                    if (lens == null) {
//                        camera.setExposureCompensation(target, createCompletionCallback(finished));
//                    }
//                    else {
//                        lens.setExposureCompensation(target, createCompletionCallback(finished));
//                    }
//                }
//            });
//            return null;
//        }
//
//        if (command instanceof ExposureModeCameraCommand) {
//            camera.getExposureMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.ExposureMode>() {
//                @Override
//                public void execute(final SettingsDefinitions.ExposureMode current) {
//                    final SettingsDefinitions.ExposureMode target = DronelinkDJI.getCameraExposureMode(((ExposureModeCameraCommand) command).exposureMode);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setExposureMode(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof FileIndexModeCameraCommand) {
//            camera.getFileIndexMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.FileIndexMode>() {
//                @Override
//                public void execute(final SettingsDefinitions.FileIndexMode current) {
//                    final SettingsDefinitions.FileIndexMode target = DronelinkDJI.getCameraFileIndexMode(((FileIndexModeCameraCommand) command).fileIndexMode);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setFileIndexMode(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof FocusCameraCommand) {
//            final FocusCameraCommand focusCameraCommand = (FocusCameraCommand)command;
//            camera.setFocusTarget(new PointF((float)focusCameraCommand.focusTarget.x, (float)focusCameraCommand.focusTarget.y), new CommonCallbacks.CompletionCallback() {
//                @Override
//                public void onResult(final DJIError djiError) {
//                    if (djiError != null) {
//                        finished.execute(DronelinkDJI.createCommandError(djiError));
//                        return;
//                    }
//
//                    new Handler().postDelayed(new Runnable() {
//                        @Override
//                        public void run() {
//                            cameraCommandFinishFocusTargetVerifyRing(focusCameraCommand, finished);
//                        }
//                    }, 500);
//                }
//            });
//
//            return null;
//        }
//
//        if (command instanceof FocusDistanceCameraCommand) {
//            final FocusDistanceCameraCommand focusDistanceCameraCommand = (FocusDistanceCameraCommand)command;
//            final CameraFocusCalibration cameraFocusCalibration = Dronelink.getInstance().getCameraFocusCalibration(focusDistanceCameraCommand.focusCalibration.withDroneSerialNumber(getSerialNumber()));
//            if (cameraFocusCalibration == null) {
//                return new CommandError(context.getString(R.string.DJIDroneSession_cameraCommand_focus_distance_error));
//            }
//            camera.setFocusRingValue(cameraFocusCalibration.ringValue.intValue(), createCompletionCallback(finished));
//            return null;
//        }
//
//        if (command instanceof FocusModeCameraCommand) {
//            camera.getFocusMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.FocusMode>() {
//                @Override
//                public void execute(final SettingsDefinitions.FocusMode current) {
//                    final SettingsDefinitions.FocusMode target = DronelinkDJI.getCameraFocusMode(((FocusModeCameraCommand) command).focusMode);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setFocusMode(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof FocusRingCameraCommand) {
//            final Double focusRingMax = djiState.getFocusRingMax();
//            camera.setFocusRingValue((int)(((FocusRingCameraCommand)command).focusRingPercent * (focusRingMax == null ? 0 : focusRingMax)), createCompletionCallback(finished));
//            return null;
//        }
//
//        if (command instanceof ISOCameraCommand) {
//            final SettingsDefinitions.ISO target = DronelinkDJI.getCameraISO(((ISOCameraCommand) command).iso);
//            com.dronelink.core.command.Command.conditionallyExecute(djiState.exposureSettings.getISO() != target.value(), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                @Override
//                public void execute() {
//                    camera.setISO(target, createCompletionCallback(finished));
//                }
//            });
//            return null;
//        }
//
//        if (command instanceof MechanicalShutterCameraCommand) {
//            camera.getMechanicalShutterEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((MechanicalShutterCameraCommand) command).enabled;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setMechanicalShutterEnabled(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof MeteringModeCameraCommand) {
//            camera.getMeteringMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.MeteringMode>() {
//                @Override
//                public void execute(final SettingsDefinitions.MeteringMode current) {
//                    final SettingsDefinitions.MeteringMode target = DronelinkDJI.getCameraMeteringMode(((MeteringModeCameraCommand) command).meteringMode);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setMeteringMode(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof ModeCameraCommand) {
//            if (camera.isFlatCameraModeSupported()) {
//                camera.getFlatMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.FlatCameraMode>() {
//                    @Override
//                    public void execute(final SettingsDefinitions.FlatCameraMode current) {
//                        final SettingsDefinitions.FlatCameraMode target = DronelinkDJI.getCameraModeFlat(((ModeCameraCommand) command).mode);
//                        com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                            @Override
//                            public void execute() {
//                                camera.setFlatMode(target, createCompletionCallback(finished));
//                            }
//                        });
//                    }
//                }, finished));
//            }
//            else {
//                final CameraMode target = ((ModeCameraCommand) command).mode;
//                com.dronelink.core.command.Command.conditionallyExecute(djiState.getMode() != target, finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                    @Override
//                    public void execute() {
//                        camera.setMode(DronelinkDJI.getCameraMode(target), createCompletionCallback(finished));
//                    }
//                });
//            }
//            return null;
//        }
//
//        if (command instanceof PhotoAspectRatioCameraCommand) {
//            camera.getPhotoAspectRatio(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.PhotoAspectRatio>() {
//                @Override
//                public void execute(final SettingsDefinitions.PhotoAspectRatio current) {
//                    final SettingsDefinitions.PhotoAspectRatio target = DronelinkDJI.getCameraPhotoAspectRatio(((PhotoAspectRatioCameraCommand) command).photoAspectRatio);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setPhotoAspectRatio(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof PhotoFileFormatCameraCommand) {
//            camera.getPhotoFileFormat(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.PhotoFileFormat>() {
//                @Override
//                public void execute(final SettingsDefinitions.PhotoFileFormat current) {
//                    final SettingsDefinitions.PhotoFileFormat target = DronelinkDJI.getCameraPhotoFileFormat(((PhotoFileFormatCameraCommand) command).photoFileFormat);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setPhotoFileFormat(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof PhotoIntervalCameraCommand) {
//            camera.getPhotoTimeIntervalSettings(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.PhotoTimeIntervalSettings>() {
//                @Override
//                public void execute(final SettingsDefinitions.PhotoTimeIntervalSettings current) {
//                    final SettingsDefinitions.PhotoTimeIntervalSettings target = new SettingsDefinitions.PhotoTimeIntervalSettings(255, ((PhotoIntervalCameraCommand) command).photoInterval);
//                    com.dronelink.core.command.Command.conditionallyExecute(current.getCaptureCount() != target.getCaptureCount() || current.getTimeIntervalInSeconds() != target.getTimeIntervalInSeconds(), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setPhotoTimeIntervalSettings(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof PhotoModeCameraCommand) {
//            if (camera.isFlatCameraModeSupported()) {
//                camera.getFlatMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.FlatCameraMode>() {
//                    @Override
//                    public void execute(final SettingsDefinitions.FlatCameraMode current) {
//                        final SettingsDefinitions.FlatCameraMode target = DronelinkDJI.getCameraModeFlat(((PhotoModeCameraCommand) command).photoMode);
//                        com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                            @Override
//                            public void execute() {
//                                camera.setFlatMode(target, createCompletionCallback(finished));
//                            }
//                        });
//                    }
//                }, finished));
//            }
//            else {
//                camera.getShootPhotoMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.ShootPhotoMode>() {
//                    @Override
//                    public void execute(final SettingsDefinitions.ShootPhotoMode current) {
//                        final SettingsDefinitions.ShootPhotoMode target = DronelinkDJI.getCameraPhotoMode(((PhotoModeCameraCommand) command).photoMode);
//                        com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                            @Override
//                            public void execute() {
//                                camera.setShootPhotoMode(target, createCompletionCallback(finished));
//                            }
//                        });
//                    }
//                }, finished));
//            }
//            return null;
//        }
//
//        if (command instanceof SaturationCameraCommand) {
//            camera.getSaturation(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Integer>() {
//                @Override
//                public void execute(final Integer current) {
//                    final Integer target = ((SaturationCameraCommand) command).saturation;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setSaturation(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof SharpnessCameraCommand) {
//            camera.getSharpness(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Integer>() {
//                @Override
//                public void execute(final Integer current) {
//                    final Integer target = ((SharpnessCameraCommand) command).sharpness;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setSharpness(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof ShutterSpeedCameraCommand) {
//            final SettingsDefinitions.ShutterSpeed target = DronelinkDJI.getCameraShutterSpeed(((ShutterSpeedCameraCommand) command).shutterSpeed);
//            com.dronelink.core.command.Command.conditionallyExecute(djiState.exposureSettings.getShutterSpeed() != target, finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                @Override
//                public void execute() {
//                    camera.setShutterSpeed(target, createCompletionCallback(finished));
//                }
//            });
//            return null;
//        }
//
//        if (command instanceof SpotMeteringTargetCameraCommand) {
//            final SpotMeteringTargetCameraCommand spotMeteringTargetCameraCommand = (SpotMeteringTargetCameraCommand)command;
//            camera.setSpotMeteringTarget(new Point((int)Math.round(spotMeteringTargetCameraCommand.spotMeteringTarget.x * 11), (int)Math.round(spotMeteringTargetCameraCommand.spotMeteringTarget.y * 7)), createCompletionCallback(finished));
//            return null;
//        }
//
//        if (command instanceof StartCaptureCameraCommand) {
//            switch (djiState.getMode()) {
//                case PHOTO:
//                    if (djiState.isCapturingPhotoInterval()) {
//                        Log.d(TAG, "Camera start capture skipped, already shooting interval photos");
//                        finished.execute(null);
//                    }
//                    else {
//                        Log.d(TAG, "Camera start capture photo");
//                        final Date started = new Date();
//                        camera.startShootPhoto(new CommonCallbacks.CompletionCallback() {
//                            boolean resultReceived = false;
//
//                            @Override
//                            public void onResult(final DJIError djiError) {
//                                //seeing two calls to onResult when taking interval photos!
//                                if (resultReceived) {
//                                    Log.d(TAG, "Camera start capture received multiple results!");
//                                    return;
//                                }
//                                resultReceived = true;
//
//                                if (djiError != null) {
//                                    finished.execute(DronelinkDJI.createCommandError(djiError));
//                                    return;
//                                }
//
//                                //waiting since isBusy will still be false for a bit
//                                new Handler().postDelayed(new Runnable() {
//                                    @Override
//                                    public void run() {
//                                        final StartCaptureCameraCommand startCaptureCameraCommand = (StartCaptureCameraCommand)command;
//                                        if (startCaptureCameraCommand.verifyFileCreated) {
//                                            cameraCommandFinishStartShootPhotoVerifyFile(startCaptureCameraCommand, started, finished);
//                                        }
//                                        else {
//                                            cameraCommandFinishNotBusy(startCaptureCameraCommand, finished);
//                                        }
//                                    }
//                                }, 500);
//                            }
//                        });
//                    }
//                    break;
//
//                case VIDEO:
//                    if (djiState.isCapturingVideo()) {
//                        Log.d(TAG, "Camera start capture skipped, already recording video");
//                        finished.execute(null);
//                    }
//                    else {
//                        Log.d(TAG, "Camera start capture video");
//                        camera.startRecordVideo(new CommonCallbacks.CompletionCallback() {
//                            @Override
//                            public void onResult(final DJIError djiError) {
//                                if (djiError != null) {
//                                    finished.execute(DronelinkDJI.createCommandError(djiError));
//                                    return;
//                                }
//
//                                //waiting since isBusy will still be false for a bit
//                                new Handler().postDelayed(new Runnable() {
//                                    @Override
//                                    public void run() {
//                                        cameraCommandFinishNotBusy(command, finished);
//                                    }
//                                }, 500);
//                            }
//                        });
//                    }
//                    break;
//
//                case PLAYBACK:
//                case DOWNLOAD:
//                case BROADCAST:
//                case UNKNOWN:
//                    Log.i(TAG, "Camera start capture invalid mode: " + djiState.getMode().toString());
//                    return new CommandError(context.getString(R.string.MissionDisengageReason_drone_camera_mode_invalid_title));
//            }
//            return null;
//        }
//
//        if (command instanceof StopCaptureCameraCommand) {
//            switch (djiState.getMode()) {
//                case PHOTO:
//                    if (djiState.isCapturingPhotoInterval()) {
//                        Log.d(TAG, "Camera stop capture interval photo");
//                        camera.stopShootPhoto(new CommonCallbacks.CompletionCallback() {
//                            @Override
//                            public void onResult(final DJIError djiError) {
//                                if (djiError != null) {
//                                    finished.execute(DronelinkDJI.createCommandError(djiError));
//                                    return;
//                                }
//
//                                cameraCommandFinishStopCapture(command, finished);
//                            }
//                        });
//                    }
//                    else {
//                        Log.d(TAG, "Camera stop capture skipped, not shooting interval photos");
//                        finished.execute(null);
//                    }
//                    break;
//
//                case VIDEO:
//                    if (djiState.isCapturingVideo()) {
//                        Log.d(TAG, "Camera stop capture video");
//                        camera.stopRecordVideo(new CommonCallbacks.CompletionCallback() {
//                            @Override
//                            public void onResult(final DJIError djiError) {
//                                if (djiError != null) {
//                                    finished.execute(DronelinkDJI.createCommandError(djiError));
//                                    return;
//                                }
//
//                                cameraCommandFinishStopCapture(command, finished);
//                            }
//                        });
//                    }
//                    else {
//                        Log.d(TAG, "Camera stop capture skipped, not recording video");
//                        finished.execute(null);
//                    }
//                    break;
//
//                case PLAYBACK:
//                case DOWNLOAD:
//                case BROADCAST:
//                case UNKNOWN:
//                    Log.i(TAG, "Camera start capture invalid mode: " + djiState.getMode().toString());
//                    return new CommandError(context.getString(R.string.MissionDisengageReason_drone_camera_mode_invalid_title));
//            }
//            return null;
//        }
//
//        if (command instanceof StorageLocationCameraCommand) {
//            camera.getStorageLocation(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.StorageLocation>() {
//                @Override
//                public void execute(final SettingsDefinitions.StorageLocation current) {
//                    final SettingsDefinitions.StorageLocation target = DronelinkDJI.getCameraStorageLocation(((StorageLocationCameraCommand) command).storageLocation);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setStorageLocation(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof VideoCaptionCameraCommand) {
//            camera.getVideoCaptionEnabled(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((VideoCaptionCameraCommand) command).enabled;
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setVideoCaptionEnabled(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof VideoFileCompressionStandardCameraCommand) {
//            camera.getVideoFileCompressionStandard(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.VideoFileCompressionStandard>() {
//                @Override
//                public void execute(final SettingsDefinitions.VideoFileCompressionStandard current) {
//                    final SettingsDefinitions.VideoFileCompressionStandard target = DronelinkDJI.getCameraVideoFileCompressionStandard(((VideoFileCompressionStandardCameraCommand) command).videoFileCompressionStandard);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setVideoFileCompressionStandard(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof VideoFileFormatCameraCommand) {
//            camera.getVideoFileFormat(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.VideoFileFormat>() {
//                @Override
//                public void execute(final SettingsDefinitions.VideoFileFormat current) {
//                    final SettingsDefinitions.VideoFileFormat target = DronelinkDJI.getCameraVideoFileFormat(((VideoFileFormatCameraCommand) command).videoFileFormat);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setVideoFileFormat(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof VideoModeCameraCommand) {
//            if (camera.isFlatCameraModeSupported()) {
//                camera.getFlatMode(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.FlatCameraMode>() {
//                    @Override
//                    public void execute(final SettingsDefinitions.FlatCameraMode current) {
//                        final SettingsDefinitions.FlatCameraMode target = DronelinkDJI.getCameraModeFlat(((VideoModeCameraCommand) command).videoMode);
//                        com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                            @Override
//                            public void execute() {
//                                camera.setFlatMode(target, createCompletionCallback(finished));
//                            }
//                        });
//                    }
//                }, finished));
//            }
//            else {
//                com.dronelink.core.command.Command.conditionallyExecute(djiState.getMode() != CameraMode.VIDEO, finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                    @Override
//                    public void execute() {
//                        camera.setMode(SettingsDefinitions.CameraMode.RECORD_VIDEO, createCompletionCallback(finished));
//                    }
//                });
//            }
//            return null;
//        }
//
//        if (command instanceof VideoResolutionFrameRateCameraCommand) {
//            camera.getVideoResolutionAndFrameRate(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<ResolutionAndFrameRate>() {
//                @Override
//                public void execute(final ResolutionAndFrameRate current) {
//                    final ResolutionAndFrameRate target = new ResolutionAndFrameRate(
//                            DronelinkDJI.getCameraVideoVideoResolution(((VideoResolutionFrameRateCameraCommand) command).videoResolution),
//                            DronelinkDJI.getCameraVideoVideoFrameRate(((VideoResolutionFrameRateCameraCommand) command).videoFrameRate),
//                            DronelinkDJI.getCameraVideoVideoFieldOfView(((VideoResolutionFrameRateCameraCommand) command).videoFieldOfView)
//                    );
//                    com.dronelink.core.command.Command.conditionallyExecute(current.getResolution() != target.getResolution() || current.getFrameRate() != target.getFrameRate() || current.getFov() != target.getFov(), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setVideoResolutionAndFrameRate(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof VideoStandardCameraCommand) {
//            camera.getVideoStandard(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<SettingsDefinitions.VideoStandard>() {
//                @Override
//                public void execute(final SettingsDefinitions.VideoStandard current) {
//                    final SettingsDefinitions.VideoStandard target = DronelinkDJI.getCameraVideoStandard(((VideoStandardCameraCommand) command).videoStandard);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setVideoStandard(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof VideoStreamSourceCameraCommand) {
//            camera.getCameraVideoStreamSource(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<CameraVideoStreamSource>() {
//                @Override
//                public void execute(final CameraVideoStreamSource current) {
//                    final CameraVideoStreamSource target = DronelinkDJI.getCameraVideoStreamSource(((VideoStreamSourceCameraCommand) command).videoStreamSource);
//                    com.dronelink.core.command.Command.conditionallyExecute(!target.equals(current), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setCameraVideoStreamSource(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof WhiteBalanceCustomCameraCommand) {
//            camera.getWhiteBalance(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<WhiteBalance>() {
//                @Override
//                public void execute(final WhiteBalance current) {
//                    final WhiteBalance target = new WhiteBalance(SettingsDefinitions.WhiteBalancePreset.CUSTOM, ((WhiteBalanceCustomCameraCommand) command).whiteBalanceCustom);
//                    com.dronelink.core.command.Command.conditionallyExecute(current.getWhiteBalancePreset() != target.getWhiteBalancePreset() || current.getColorTemperature() != target.getColorTemperature(), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setWhiteBalance(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        if (command instanceof WhiteBalancePresetCameraCommand) {
//            camera.getWhiteBalance(createCompletionCallbackWith(new com.dronelink.core.command.Command.FinisherWith<WhiteBalance>() {
//                @Override
//                public void execute(final WhiteBalance current) {
//                    final WhiteBalance target = new WhiteBalance(DronelinkDJI.getCameraWhiteBalancePreset(((WhiteBalancePresetCameraCommand) command).whiteBalancePreset));
//                    com.dronelink.core.command.Command.conditionallyExecute(current.getWhiteBalancePreset() != target.getWhiteBalancePreset(), finished, new com.dronelink.core.command.Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            camera.setWhiteBalance(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
//            return null;
//        }
//
//        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
//    }
}
