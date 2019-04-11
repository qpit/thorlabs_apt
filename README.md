thorlabs_apt
============
python package wrapping Thorlabs' APT.dll shared library.

**Installation**

Since this package is based on APT.dll it only works on Windows. For Linux and Mac you can try [thorpy](https://github.com/UniNE-CHYN/thorpy)

1. Install thorlabs_apt using setup.py

2. Check whether your python is a 32 bit or 64 bit version and install the corresponding [Thorlabs' APT software](http://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=APT)

3. Copy APT.dll from the "APT installation path\APT Server" directory to one of the following locations:
    - Windows\System32
    - into the "thorlabs_apt" folder
    - your python application directory

**List of reported working devices**

I have tested thorlabs_apt only with the K10CR1 rotation stage, but it should work with all motors supported by APT. If it works with other motors as well, please let me know and I will add it here. Otherwise file a bug report or fix the problem yourself and open a pull request.

- BSC101
    - NRT150
- BSC201
- K10CR1 (*)
- KDC101
- LTS150
- NT100 (#12)
- TDC001

(*) see known issues.

**Known issues**

- If the package has been imported and a new device is plugged into the USB port, `list_available_devices()` cannot find it. The reason is that the list of available devices in APT.DLL is initialized when APTInit() is called. However, calling APTCleanUp() and APTInit() will kill all existing communication to other devices. If you have a good solution for that, I'm happy to hear it, but I fear that this is a limitation of APT.DLL.
- K10CR1: default move home parameters are invalid which prevents homing. Use `set_move_home_parameters()` to set them manually.

**Example**

The following example checks for all connected devices and then connects
to the one specified by its serial number. The motor is first homed (blocking)
and then moved relative by 45 degree.

```python
    >>> import thorlabs_apt as apt
    >>> apt.list_available_devices()
    [(50, 55000038)]
    >>> motor = apt.Motor(55000038)
    >>> motor.move_home(True)
    >>> motor.move_by(45)
```

**References**

- [Thorlabs APT software](http://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=APT)
- [github](https://github.com/qpit/thorlabs_apt)
