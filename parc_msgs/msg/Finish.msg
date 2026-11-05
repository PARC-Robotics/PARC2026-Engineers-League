# Published by the team on /parc/finish to end the run early, e.g. after
# skipping a tag. Not needed if all four tags are reported: the referee ends
# the run automatically after the fourth report.
# QoS: reliable, depth 10.

builtin_interfaces/Time stamp

uint8 REASON_COMPLETED = 0   # the robot has done everything it is going to do
uint8 REASON_GAVE_UP   = 1   # the robot cannot continue
uint8 reason
