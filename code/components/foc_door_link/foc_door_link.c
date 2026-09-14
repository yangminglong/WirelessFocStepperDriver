#include "foc_door_link.h"

const char *foc_door_cmd_name(uint8_t cmd)
{
    switch (cmd) {
    case FOC_DOOR_CMD_NONE:  return "NONE";
    case FOC_DOOR_CMD_WAKE:  return "WAKE";
    case FOC_DOOR_CMD_OPEN:  return "OPEN";
    case FOC_DOOR_CMD_CLOSE: return "CLOSE";
    case FOC_DOOR_CMD_STOP:  return "STOP";
    case FOC_DOOR_CMD_SET_T: return "SET_T";
    default:                 return "?";
    }
}
