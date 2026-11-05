# Published once by the team on /parc/start when the robot starts its run.
# QoS: reliable, depth 10. The run clock starts when the referee receives it.

builtin_interfaces/Time stamp

uint8 METHOD_COMMAND = 0   # started by a typed command
uint8 METHOD_VOICE   = 1   # started by a spoken start word (bonus)
uint8 method

# For voice starts: the recognised word ("go", "allez" or "vai"). Empty otherwise.
string heard
