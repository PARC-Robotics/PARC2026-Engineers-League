# Published by the referee on /parc/tag_order when the judge arms the run.
# QoS: reliable, transient local, depth 1 (late subscribers still receive it).

builtin_interfaces/Time stamp

# The order in which the tags must be visited, e.g. [7, 1, 9, 4].
uint8[] tag_ids
