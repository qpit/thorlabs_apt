import _APTAPI

import ctypes
import os

def list_available_devices():
    # we have to check for all possible hardware types.
    # Unfortunately I couldn't find a list of all existing hardware types,
    # the list in the C header file is incomplete. Therefore we just check
    # the first 100 type values
    devices = []
    count = ctypes.c_long()
    for hwtype in range(100):
        if (_lib.GetNumHWUnitsEx(hwtype, ctypes.byref(count)) == 0):
            # found an existing hardware type
            if (count > 0):
                # devices are available!!
                # get their serial number
                serial_number = ctypes.c_long()
                for ii in range(count.value):
                    if (_lib.GetHWSerialNumEx(hwtype, ii, 
                        ctypes.byref(serial_number)) == 0):
                        devices.append((hwtype, serial_number.value))
    return devices

def hardware_info(serial_number):
    model = ctypes.c_buffer(255)
    swver = ctypes.c_buffer(255)
    hwnotes = ctypes.c_buffer(255)
    if (_lib.GetHWInfo(serial_number, model, len(model),
        swver, len(swver), hwnotes, len(hwnotes)) != 0):
        raise Exception("Getting hardware info failed.")
    return (model.value, swver.value, hwnotes.value)



class Motor(object):
    def __init__(self, serial_number):
        self._serial_number = serial_number
        self._active_channel = 0
        # initialize device
        if (_lib.InitHWDevice(serial_number) != 0):
            raise Exception("Could not initialize device.")

    def __property_from_index(index, get_func, set_func):
        def setter(self, value):
            params = list(get_func(self))
            params[index] = value
            set_func(self, *params)

        def getter(self):
            return get_func(self)[index]

        setter.get_func = get_func
        setter.index = index
        getter.index = index

        return property(getter, setter)

    @property
    def serial_number(self):
        return self._serial_number

    @property
    def hardware_info(self):
        return hardware_info(self._serial_number)

    @property
    def status(self):
        status_bits = ctypes.c_long()
        if (_lib.MOT_GetStatusBits(self._serial_number, 
                ctypes.byref(status_bits)) != 0):
            raise Exception("Getting status failed.")
        return status_bits.value

    @property
    def active_channel(self):
        return self._active_channel

    @active_channel.setter
    def active_channel(self, channel):
        if (_lib.MOT_SetChannel(self._serial_number, channel) != 0):
            raise Exception("Setting channel %d failed." % channel)
        self._active_channel = channel

    def enable(self):
        if (_lib.MOT_EnableHWChannel(self._serial_number) != 0):
            raise Exception("Enabling channel failed.")
    
    def disable(self):
        if (_lib.MOT_DisableHWChannel(self._serial_number) != 0):
            raise Exception("Disabling channel failed.")

    def identify(self):
        if (_lib.MOT_Identify(self._serial_number) != 0):
            raise Exception("Identifing device failed.")

    def get_velocity_parameters(self):
        min_vel = ctypes.c_float()
        accn = ctypes.c_float()
        max_vel = ctypes.c_float()
        if (_lib.MOT_GetVelParams(self._serial_number, ctypes.byref(min_vel),
            ctypes.byref(accn), ctypes.byref(max_vel)) != 0):
            raise Exception("Getting velocity parameters failed:")
        return (min_vel.value, accn.value, max_vel.value)

    def set_velocity_parameters(self, min_vel, accn, max_vel):
        if (_lib.MOT_SetVelParams(self._serial_number, min_vel, accn, 
            max_vel) != 0):
            raise Exception("Setting velocity parameters failed.")

    minimum_velocity = __property_from_index(0, get_velocity_parameters,
            set_velocity_parameters)
    acceleration = __property_from_index(1, get_velocity_parameters,
            set_velocity_parameters)
    maximum_velocity = __property_from_index(2, get_velocity_parameters,
            set_velocity_parameters)

    def get_velocity_parameter_limits(self):
        max_accn = ctypes.c_float()
        max_vel = ctypes.c_float()
        if (_lib.MOT_GetVelParamLimits(self._serial_number, 
            ctypes.byref(max_accn), ctypes.byref(max_vel)) != 0):
            raise Exception("Getting velocity parameter limits failed:")
        return (max_accn.value, max_vel.value)

    @property
    def velocity_upper_limit(self):
        return self.get_velocity_parameter_limits()[0]

    @property
    def acceleration_upper_limit(self):
        return self.get_velocity_parameter_limits()[1]
 

    def get_move_home_parameters(self):
        direction = ctypes.c_long()
        lim_switch = ctypes.c_long()
        velocity = ctypes.c_float()
        zero_offset = ctypes.c_float()
        if (_lib.MOT_GetHomeParams(self._serial_number,
                ctypes.byref(direction),
                ctypes.byref(lim_switch),
                ctypes.byref(velocity),
                ctypes.byref(zero_offset)) != 0):
            raise Exception("Getting move home parameters failed.")
        return (direction.value, lim_switch.value, velocity.value,
                zero_offset.value)
    
    def set_move_home_parameters(self, direction, lim_switch, velocity,
            zero_offset):
        if (_lib.MOT_SetHomeParams(self._serial_number, direction,
                lim_switch, velocity, zero_offset) != 0):
            raise Exception("Setting move home parameters failed.")

    move_home_direction = __property_from_index(0,
            get_move_home_parameters, set_move_home_parameters)
    move_home_lim_switch = __property_from_index(1,
            get_move_home_parameters, set_move_home_parameters)
    move_home_velocity = __property_from_index(2,
            get_move_home_parameters, set_move_home_parameters)
    move_home_zero_offset = __property_from_index(3,
            get_move_home_parameters, set_move_home_parameters)

    def get_motor_parameters(self):
        steps_per_rev = ctypes.c_long()
        gear_box_ratio = ctypes.c_long()
        if (_lib.MOT_GetMotorParams(self._serial_number, 
                ctypes.byref(steps_per_rev), 
                ctypes.byref(gear_box_ratio)) != 0):
            raise Exception("Failed getting motor parameters.")
        else:
            return (steps_per_rev.value, gear_box_ratio.value)

    def set_motor_parameters(self, steps_per_rev, gear_box_ratio):
        if (_lib.MOT_SetMotorParams(self._serial_number, steps_per_rev,
                gear_box_ratio) != 0):
            raise Exception("Setting motor parameters failed.")

    steps_per_revolution = __property_from_index(0, get_motor_parameters,
            set_motor_parameters)
    gear_box_ratio = __property_from_index(1, get_motor_parameters,
            set_motor_parameters)
    @property
    def backlash_correction(self):
        backlash = ctypes.c_float()
        if (_lib.MOT_GetBLashDist(self._serial_number, 
            ctypes.byref(backlash)) != 0):
            raise Exception("Failed getting backlash correction value.")
        else:
            return backlash.value

    def get_stage_axis_info(self):
        min_pos = ctypes.c_float()
        max_pos = ctypes.c_float()
        units = ctypes.c_long()
        pitch = ctypes.c_float()
        if (_lib.MOT_GetStageAxisInfo(self._serial_number, 
                ctypes.byref(min_pos),
                ctypes.byref(max_pos),
                ctypes.byref(units),
                ctypes.byref(pitch)) != 0):
            raise Exception("Failed getting stage axis information.")
        return (min_pos.value, max_pos.value, units.value, pitch.value)

    def set_stage_axis_info(self, min_pos, max_pos, units, pitch):
        if (_lib.MOT_SetStageAxisInfo(self._serial_number,
                min_pos, max_pos, units, pitch) != 0):
            raise Exception("Setting stage axis info failed.")

    minimum_position = __property_from_index(0, get_stage_axis_info,
            set_stage_axis_info)
    maximum_position = __property_from_index(1, get_stage_axis_info,
            set_stage_axis_info)
    units = __property_from_index(2, get_stage_axis_info,
            set_stage_axis_info)
    pitch = __property_from_index(3, get_stage_axis_info,
            set_stage_axis_info)

    def get_hardware_limit_switches(self):
        rev = ctypes.c_long()
        fwd = ctypes.c_long()
        if (_lib.MOT_GetHWLimSwitches(self._serial_number,
                ctypes.byref(rev), ctypes.byref(fwd)) != 0):
            raise Exception("Getting hardware limit switches failed.")
        return (rev.value, fwd.value)

    def set_hardware_limit_switches(self, rev, fwd):
        if (_lib.MOT_SetHWLimSwitches(self._serial_number, rev, fwd) != 0):
            raise Exception("Setting hardware limit switches failed.")

    reverse_limit_switch = __property_from_index(0, 
            get_hardware_limit_switches,
            set_hardware_limit_switches)
    forward_limit_switch = __property_from_index(1, 
            get_hardware_limit_switches,
            set_hardware_limit_switches)

    def get_pid_parameters(self):
        proportional = ctypes.c_long()
        integrator = ctypes.c_long()
        differentiator = ctypes.c_long()
        integrator_limit = ctypes.c_long()
        if (_lib.MOT_GetPIDParams(self._serial_number,
            ctypes.byref(proportional),
            ctypes.byref(integrator),
            ctypes.byref(differentiator),
            ctypes.byref(integrator_limit)) != 0):
            raise Exception("Getting PID parameters failed.")
        return (proportional.value, integrator.value, differentiator.value,
                integrator_limit.value)

    def set_pid_parameters(self, proportional, integrator, differentiator,
            integrator_limit):
        if (_lib.MOT_SetPIDParams(self._serial_number, proportional,
            integrator, differentiator, integrator_limit) != 0):
            raise Exception("Setting PID parameters failed.")

    pid_proportional = __property_from_index(0, get_pid_parameters,
            set_pid_parameters)
    pid_integrator = __property_from_index(1, get_pid_parameters,
            set_pid_parameters)
    pid_differentiator = __property_from_index(2, get_pid_parameters,
            set_pid_parameters)
    pid_integrator_limit = __property_from_index(3, get_pid_parameters,
            set_pid_parameters)

    def move_to_absolute_position(self, value, blocking = False):
        if (_lib.MOT_MoveAbsoluteEx(self._serial_number, value, 
                blocking) != 0):
            raise Exception("Setting absolute position failed.")
    
    def move_relative(self, value, blocking = False):
        if (_lib.MOT_MoveRelativeEx(self._serial_number, value, 
                blocking) != 0):
            raise Exception("Setting relative position failed.")

    @property
    def position(self):
        pos = ctypes.c_float()
        if (_lib.MOT_GetPosition(self._serial_number, 
                ctypes.byref(pos)) != 0):
            raise Exception("Getting position failed.")
        return pos.value

    @position.setter
    def position(self, value):
        self.move_to_absolute_position(value, False)

    
    def move_home(self, blocking = False):
        if (_lib.MOT_MoveHome(self._serial_number, blocking) != 0):
            raise Exception("Moving velocity failed.")
    
    def move_velocity(self, direction):
        if (_lib.MOT_MoveVelocity(self._serial_number, direction) != 0):
            raise Exception("Moving velocity failed.")
    
    def stop_profiled(self):
        if (_lib.MOT_StopProfiled(self._serial_number) != 0):
            raise Exception("Stop profiled failed: ")
    

def _load_library():
    # load library
    if (os.name != 'nt'):
        raise Exception("Your operating system is not supported. " \
                "Thorlabs' APT API only works on Windows.")  
    filename = "%s/APT.dll" % os.path.dirname(__file__)
    #lib = ctypes.util.find_library(filename)
    #if (lib is None):
    #    raise Exception("Could not find shared library APT.dll.")
    #else:
    lib = ctypes.windll.LoadLibrary(filename)
    if (lib is not None):
        _APTAPI.set_ctypes_argtypes(lib)
        if (lib.APTInit() != 0):
            raise Exception("Thorlabs APT Initialization failed.")
        if (lib.EnableEventDlg(False) != 0):
            raise Exception("Couldn't disable event dialog.")
    return lib

_lib = None
_lib = _load_library()

import atexit
@atexit.register
def _cleanup():
    if (_lib is not None):
        _lib.APTCleanUp()

