MG17_UNKNOWN_ERR = 10000
MG17_INTERNAL_ERR = 10001
MG17_FAILED = 10002
MG17_INVALIDPARAM_ERR = 10003
MG17_ETHERNET_ERR = 10051
MG17_REGISTRY_ERR = 10052
MG17_MEMORY_ERR = 10053
MG17_COM_ERR = 10054
MG17_USB_ERR = 10055
MG17_SERIALNUMUNKNOWN_ERR = 10100
MG17_DUPLICATESERIALNUM_ERR = 10101
MG17_DUPLICATEDEVICEIDENT_ERR = 10102
MG17_INVALIDMSGSRC_ERR = 10103
MG17_UNKNOWNMSGIDENT_ERR = 10104
MG17_INVALIDSERIALNUM_ERR = 10106
MG17_INVALIDMSGDEST_ERR = 10107
MG17_INVALIDINDEX_ERR = 10108
MG17_CTRLCOMMSDISABLED_ERR = 10109
MG17_HWRESPONSE_ERR = 10110
MG17_HWTIMEOUT_ERR = 10111
MG17_INCORRECTVERSION_ERR = 10112
MG17_INCOMPATIBLEHARDWARE_ERR = 10115
MG17_NOSTAGEAXISINFO = 10150
MG17_CALIBTABLE_ERR = 10151
MG17_ENCCALIB_ERR = 10152
MG17_ENCNOTPRESENT_ERR = 10153

error_message = {
    MG17_UNKNOWN_ERR: "Unknown error.",
    MG17_INTERNAL_ERR: "Internal error.",
    MG17_FAILED: "Call has failed.",
    MG17_INVALIDPARAM_ERR: "Invalid or out-of-range parameter.",
    MG17_ETHERNET_ERR: "Error while accessing hard disk.",
    MG17_REGISTRY_ERR: "Error while accessing registry.",
    MG17_MEMORY_ERR: "Internal memory allocation or de-allocation error.",
    MG17_COM_ERR: "COM system error.",
    MG17_USB_ERR: "USB communication error.",
    MG17_SERIALNUMUNKNOWN_ERR: "Unknown serial number.",
    MG17_DUPLICATESERIALNUM_ERR: "Duplicate serial number.",
    MG17_DUPLICATEDEVICEIDENT_ERR: "Duplicate device identifier.",
    MG17_INVALIDMSGSRC_ERR: "Invalid message source.",
    MG17_UNKNOWNMSGIDENT_ERR: "Message received with unknown identifier.",
    MG17_INVALIDSERIALNUM_ERR: "Serial number invalid.",
    MG17_INVALIDMSGDEST_ERR: "Invalid message destination ident.",
    MG17_INVALIDINDEX_ERR: "Invalid index.",
    MG17_CTRLCOMMSDISABLED_ERR: "Control is currently not communicating.",
    MG17_HWRESPONSE_ERR: "Hardware fault or illegal command or parameter has been sent to hardware.",
    MG17_HWTIMEOUT_ERR: "Time out while waiting for hardware unit to respond.",
    MG17_INCORRECTVERSION_ERR: "Incorrect firmware version.",
    MG17_INCOMPATIBLEHARDWARE_ERR: "Your hardware is not compatible.",
    MG17_NOSTAGEAXISINFO: "No stage has been assigned.",
    MG17_CALIBTABLE_ERR: "Internal error when using an encoded stage.",
    MG17_ENCCALIB_ERR: "Internal error when using an encoded stage.",
    MG17_ENCNOTPRESENT_ERR: "Call only applicable to encoded stages."
}
