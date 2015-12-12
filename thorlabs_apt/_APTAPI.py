import ctypes

# lHWType definitions - used with GetNumHWUnitsEx & GetHWSerialNumEx.
HWTYPE_BSC001 = 11 # 1 Ch benchtop stepper driver
HWTYPE_BSC101 = 12	# 1 Ch benchtop stepper driver
HWTYPE_BSC002 = 13	# 2 Ch benchtop stepper driver
HWTYPE_BDC101 = 14	# 1 Ch benchtop DC servo driver
HWTYPE_SCC001 = 21	# 1 Ch stepper driver card (used within BSC102,103 units)
HWTYPE_DCC001 = 22	# 1 Ch DC servo driver card (used within BDC102,103 units)
HWTYPE_ODC001 = 24	# 1 Ch DC servo driver cube
HWTYPE_OST001 = 25	# 1 Ch stepper driver cube
HWTYPE_MST601 = 26	# 2 Ch modular stepper driver module
HWTYPE_TST001 = 29	# 1 Ch Stepper driver T-Cube
HWTYPE_TDC001 = 31	# 1 Ch DC servo driver T-Cube
HWTYPE_LTSXXX = 42	# LTS300/LTS150 Long Travel Integrated Driver/Stages
HWTYPE_L490MZ = 43	# L490MZ Integrated Driver/Labjack
HWTYPE_BBD10X = 44	# 1/2/3 Ch benchtop brushless DC servo driver

# Channel idents - used with MOT_SetChannel.
CHAN1_INDEX = 0		# Channel 1.
CHAN2_INDEX = 1		# Channel 2.

# Home direction (lDirection) - used with MOT_Set(Get)HomeParams.
HOME_FWD = 1		# Home in the forward direction.
HOME_REV = 2		# Home in the reverse direction.

# Home limit switch (lLimSwitch) - used with MOT_Set(Get)HomeParams.
HOMELIMSW_FWD = 4		# Use forward limit switch for home datum.
HOMELIMSW_REV = 1		# Use reverse limit switch for home datum.

# Stage units (lUnits) - used with MOT_Set(Get)StageAxisInfo.
STAGE_UNITS_MM = 1		# Stage units are in mm.
STAGE_UNITS_DEG = 2		# Stage units are in degrees.

# Hardware limit switch settings (lRevLimSwitch, lFwdLimSwitch) - used with MOT_Set(Get)HWLimSwitches
HWLIMSWITCH_IGNORE = 1		# Ignore limit switch (e.g. for stages with only one or no limit switches).
HWLIMSWITCH_MAKES = 2		# Limit switch is activated when electrical continuity is detected.
HWLIMSWITCH_BREAKS = 3		# Limit switch is activated when electrical continuity is broken.
HWLIMSWITCH_MAKES_HOMEONLY = 4		# As per HWLIMSWITCH_MAKES except switch is ignored other than when homing (e.g. to support rotation stages).
HWLIMSWITCH_BREAKS_HOMEONLY = 5		# As per HWLIMSWITCH_BREAKS except switch is ignored other than when homing (e.g. to support rotation stages).

# Move direction (lDirection) - used with MOT_MoveVelocity.
MOVE_FWD = 1		# Move forward.
MOVE_REV = 2		# Move reverse.

# Profile mode settings - used with MOT_Set(Get)DCProfileModeParams.
DC_PROFILEMODE_TRAPEZOIDAL = 0
DC_PROFILEMODE_SCURVE = 2

# Joystick direction sense settings - used with MOT_Set(Get)DCJoystickParams.
DC_JS_DIRSENSE_POS = 1
DC_JS_DIRSENSE_NEG = 2

def set_ctypes_argtypes(lib):
    c_long = ctypes.c_long
    c_long_p = ctypes.POINTER(ctypes.c_long)
    c_char_p = ctypes.c_char_p
    c_bool = ctypes.c_bool
    c_float = ctypes.c_float
    c_float_p = ctypes.POINTER(ctypes.c_float)

    # System Level Exports.
    lib.APTInit.argtypes = None
    lib.APTInit.restype = c_long
    lib.APTCleanUp.argtypes = None
    lib.APTCleanUp.restype = c_long
    lib.GetNumHWUnitsEx.argtypes = [c_long, c_long_p]
    lib.GetNumHWUnitsEx.restype = c_long
    lib.GetHWSerialNumEx.argtypes = [c_long, c_long, c_long_p]
    lib.GetHWSerialNumEx.restype = c_long
    lib.GetHWInfo.argtypes = [c_long, c_char_p, c_long, c_char_p, c_long, 
            c_char_p, c_long]
    lib.GetHWInfo.restype = c_long
    lib.InitHWDevice.argtypes = [c_long]
    lib.InitHWDevice.restype = c_long
    lib.EnableEventDlg.argtypes = [c_bool]
    lib.EnableEventDlg.restype = c_long

    # Low Level Motor Specific Exports.
    lib.MOT_LLSetEncoderCount.argtypes = [c_long, c_long]
    lib.MOT_LLSetEncoderCount.restype = c_long
    lib.MOT_LLGetEncoderCount.argtypes = [c_long, c_long_p]
    lib.MOT_LLGetEncoderCount.restype = c_long

    # Motor Specific Exports.
    lib.MOT_SetChannel.argtypes = [c_long, c_long]
    lib.MOT_SetChannel.restype = c_long
    lib.MOT_Identify.argtypes = [c_long]
    lib.MOT_Identify.restype = c_long
    lib.MOT_EnableHWChannel.argtypes = [c_long]
    lib.MOT_EnableHWChannel.restype = c_long
    lib.MOT_DisableHWChannel.argtypes = [c_long]
    lib.MOT_DisableHWChannel.restype = c_long
    lib.MOT_SetVelParams.argtypes = [c_long, c_float, c_float, c_float]
    lib.MOT_SetVelParams.restype = c_long
    lib.MOT_GetVelParams.argtypes = [c_long, c_float_p, c_float_p, c_float_p]
    lib.MOT_GetVelParams.restype = c_long
    lib.MOT_GetVelParamLimits.argtypes = [c_long, c_float_p, c_float_p]
    lib.MOT_GetVelParamLimits.restype = c_long
    lib.MOT_SetHomeParams.argtypes = [c_long, c_long, c_long, c_float, c_float]
    lib.MOT_SetHomeParams.restype = c_long
    lib.MOT_GetHomeParams.argtypes = [c_long, c_long_p, c_long_p, c_float_p, 
            c_float_p]
    lib.MOT_GetHomeParams.restype = c_long
    lib.MOT_GetStatusBits.argtypes = [c_long, c_long_p]
    lib.MOT_GetStatusBits.restype = c_long
    lib.MOT_SetBLashDist.argtypes = [c_long, c_float]
    lib.MOT_SetBLashDist.restype = c_long
    lib.MOT_GetBLashDist.argtypes = [c_long, c_float_p]
    lib.MOT_GetBLashDist.restype = c_long
    lib.MOT_SetMotorParams.argtypes = [c_long, c_long, c_long]
    lib.MOT_SetMotorParams.restype = c_long
    lib.MOT_GetMotorParams.argtypes = [c_long, c_long_p, c_long_p]
    lib.MOT_GetMotorParams.restype = c_long
    lib.MOT_SetStageAxisInfo.argtypes = [c_long, c_float, c_float, c_long, 
            c_float]
    lib.MOT_SetStageAxisInfo.restype = c_long
    lib.MOT_GetStageAxisInfo.argtypes = [c_long, c_float_p, c_float_p, 
            c_long_p, c_float_p]
    lib.MOT_GetStageAxisInfo.restype = c_long
    lib.MOT_SetHWLimSwitches.argtypes = [c_long, c_long, c_long]
    lib.MOT_SetHWLimSwitches.restype = c_long
    lib.MOT_GetHWLimSwitches.argtypes = [c_long, c_long_p, c_long_p]
    lib.MOT_GetHWLimSwitches.restype = c_long
    lib.MOT_SetPIDParams.argtypes = [c_long, c_long, c_long, c_long, c_long]
    lib.MOT_SetPIDParams.restype = c_long
    lib.MOT_GetPIDParams.argtypes = [c_long, c_long_p, c_long_p, c_long_p, 
            c_long_p]
    lib.MOT_GetPIDParams.restype = c_long
    lib.MOT_GetPosition.argtypes = [c_long, c_float_p]
    lib.MOT_GetPosition.restype = c_long
    lib.MOT_MoveHome.argtypes = [c_long, c_bool]
    lib.MOT_MoveHome.restype = c_long
    lib.MOT_MoveRelativeEx.argtypes = [c_long, c_float, c_bool]
    lib.MOT_MoveRelativeEx.restype = c_long
    lib.MOT_MoveAbsoluteEx.argtypes = [c_long, c_float, c_bool]
    lib.MOT_MoveAbsoluteEx.restype = c_long
    lib.MOT_MoveVelocity.argtypes = [c_long, c_long]
    lib.MOT_MoveVelocity.restype = c_long
    lib.MOT_StopProfiled.argtypes = [c_long]
    lib.MOT_StopProfiled.restype = c_long

    # Brushless DC Servo Specific Exports.
    lib.MOT_SetDCCurrentLoopParams.argtypes = [c_long, c_long, c_long, c_long,
            c_long, c_long]
    lib.MOT_SetDCCurrentLoopParams.restype = c_long
    lib.MOT_GetDCCurrentLoopParams.argtypes = [c_long, c_long_p, c_long_p, 
            c_long_p, c_long_p, c_long_p]
    lib.MOT_GetDCCurrentLoopParams.restype = c_long
    lib.MOT_SetDCPositionLoopParams.argtypes = [c_long, c_long, c_long, 
            c_long, c_long, c_long, c_long, c_long, c_long, c_long]
    lib.MOT_SetDCPositionLoopParams.restype = c_long
    lib.MOT_GetDCPositionLoopParams.argtypes = [c_long, c_long_p, c_long_p, 
            c_long_p, c_long_p, c_long_p, c_long_p, c_long_p, c_long_p, 
            c_long_p]
    lib.MOT_GetDCPositionLoopParams.restype = c_long
    lib.MOT_SetDCMotorOutputParams.argtypes = [c_long, c_float, c_float, 
            c_float, c_float]
    lib.MOT_SetDCMotorOutputParams.restype = c_long
    lib.MOT_GetDCMotorOutputParams.argtypes = [c_long, c_float_p, c_float_p, 
            c_float_p, c_float_p]
    lib.MOT_GetDCMotorOutputParams.restype = c_long
    lib.MOT_SetDCTrackSettleParams.argtypes = [c_long, c_long, c_long, c_long]
    lib.MOT_SetDCTrackSettleParams.restype = c_long
    lib.MOT_GetDCTrackSettleParams.argtypes = [c_long, c_long_p, c_long_p, 
            c_long_p]
    lib.MOT_GetDCTrackSettleParams.restype = c_long
    lib.MOT_SetDCProfileModeParams.argtypes = [c_long, c_long, c_float]
    lib.MOT_SetDCProfileModeParams.restype = c_long
    lib.MOT_GetDCProfileModeParams.argtypes = [c_long, c_long_p, c_float_p]
    lib.MOT_GetDCProfileModeParams.restype = c_long
    lib.MOT_SetDCJoystickParams.argtypes = [c_long, c_float, c_float, c_float,
            c_float, c_long]
    lib.MOT_SetDCJoystickParams.restype = c_long
    lib.MOT_GetDCJoystickParams.argtypes = [c_long, c_float_p, c_float_p, 
            c_float_p, c_float_p, c_long_p]
    lib.MOT_GetDCJoystickParams.restype = c_long
    lib.MOT_SetDCSettledCurrentLoopParams.argtypes = [c_long, c_long, c_long, 
            c_long, c_long, c_long]
    lib.MOT_SetDCSettledCurrentLoopParams.restype = c_long
    lib.MOT_GetDCSettledCurrentLoopParams.argtypes = [c_long, c_long_p, 
            c_long_p, c_long_p, c_long_p, c_long_p]
    lib.MOT_GetDCSettledCurrentLoopParams.restype = c_long
