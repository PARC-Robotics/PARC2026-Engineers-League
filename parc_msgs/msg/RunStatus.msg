# Published by the referee on /parc/run_status at 2 Hz and on every change.
# QoS: reliable, transient local, depth 1.
# Teams can use it to confirm their start and visits were received, and should
# stop the robot when the state is FINISHED, TIMED_OUT or TERMINATED.

builtin_interfaces/Time stamp

uint8 IDLE       = 0   # no run in progress
uint8 ARMED      = 1   # order published, waiting for /parc/start
uint8 RUNNING    = 2   # clock running
uint8 FINISHED   = 3   # fourth tag reported or /parc/finish received
uint8 TIMED_OUT  = 4   # time limit reached
uint8 TERMINATED = 5   # ended by a judge (safety stop, contact with a person, loss of control)
uint8 state

float32 elapsed_s      # seconds since the start signal
float32 time_limit_s   # 600

uint8[] tag_order      # same as /parc/tag_order
uint8[] tags_reported  # tag IDs reported, in the order received (the judges' verdict is separate)

uint8 start_method     # StartSignal.METHOD_*, valid once RUNNING
bool false_start       # robot moved before the start signal
