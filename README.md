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

**Known issues**

I have tested thorlabs_apt only with the K10CR1 rotation stage, but it should work with all motors supported by APT. If it works with other motors as well, please let me know and I will add it here. Otherwise file a bug report or fix the problem yourself and open a pull request.

**Example**

The following example checks for all connected devices and then connects
to the one specified by its serial number. The motor is first homed (blocking)
and then moved relative by 45 degree.

```python
    >>> import thorlabs_apt as apt
    >>> apt.list_available_devices()
    [[50, 55000038]
    >>> motor = apt.Motor(55000038)
    >>> motor.move_home(True)
    >>> motor.move_by(45)
```

**References**

- [Thorlabs APT software](http://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=APT)
- [github](https://github.com/qpit/thorlabs_apt)
