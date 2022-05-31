package com.dronelink.autel;

import android.content.Context;
import android.location.Location;
import android.os.AsyncTask;
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
import com.dronelink.core.kernel.command.camera.CameraCommand;
import com.dronelink.core.kernel.command.camera.ModeCameraCommand;
import com.dronelink.core.kernel.command.drone.DroneCommand;
import com.dronelink.core.kernel.command.gimbal.GimbalCommand;
import com.dronelink.core.kernel.command.gimbal.ModeGimbalCommand;
import com.dronelink.core.kernel.command.remotecontroller.RemoteControllerCommand;
import com.dronelink.core.kernel.core.Message;
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
    private Location _lastKnownGroundLocation;

    private AutelCoordinate3D _location;
    private EvoAttitudeInfo _attitude;

    private final List<Listener> listeners = new LinkedList<>();
    private final ExecutorService listenerExecutor = Executors.newSingleThreadExecutor();
    private final CommandQueue _droneCommands = new CommandQueue();
    private final MultiChannelCommandQueue _remoteControllerCommands = new MultiChannelCommandQueue();
    private final MultiChannelCommandQueue _cameraCommands = new MultiChannelCommandQueue();
    private final MultiChannelCommandQueue _gimbalCommands = new MultiChannelCommandQueue();

    private final ExecutorService _mainControllerSerialQueue = Executors.newSingleThreadExecutor();
    private DatedValue<FlyControllerInfo> _mainControllerState;

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
                EvoGpsInfo gpsInfo = flyControllerInfo.getGpsInfo();
                if (null != gpsInfo) {
                    AutelCoordinate3D coord3D = new AutelCoordinate3D(gpsInfo.getLatitude(), gpsInfo.getLongitude(), gpsInfo.getAltitude());
                    if (null != coord3D) {
                        updateAircraftLocation(coord3D, flyControllerInfo.getAttitudeInfo());
                    }
                }
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

    private void updateAircraftLocation(AutelCoordinate3D coord3D, EvoAttitudeInfo attitudeInfo)  {
        _location = coord3D;
        _attitude = attitudeInfo;
    }

    public DatedValue<FlyControllerInfo> getMainControllerState() {
        try {
            AutelDroneSession self = this;
            return _mainControllerSerialQueue.submit(new Callable<DatedValue<FlyControllerInfo>>() {
                @Override
                public DatedValue<FlyControllerInfo> call() throws Exception {
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

                if (state.initialized) {
                    listener.onInitialized(self);
                }

                if (state.located) {
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
}
