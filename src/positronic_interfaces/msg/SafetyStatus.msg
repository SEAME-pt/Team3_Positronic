# Status of the safety supervisor (#45), on /safety_supervisor/status. One message at every
# change of mode, latched (transient local): a late subscriber still gets the current one.
# The constants repeat Mode and Reason of positronic_safety/supervisor_logic.hpp; the
# supervisor does not build if the two disagree. Add new values at the end, never reuse one.

# When the mode changed.
builtin_interfaces/Time stamp

uint8 mode
# The arm is stopped and stays stopped until a person activates the supervisor.
uint8 MODE_STOPPED=0
# A person activated the supervisor: the resume requests are running.
uint8 MODE_RESUMING=1
# The arm may move and every trigger is watched.
uint8 MODE_MONITORING=2
# A trigger fired: the halt requests are running.
uint8 MODE_HALTING=3

# Why the last halt started. REASON_NONE before the first halt and after a resume.
uint8 reason
uint8 REASON_NONE=0
uint8 REASON_STOP_SERVICE=1
uint8 REASON_HEARTBEAT_LOST=2
uint8 REASON_DRIVER_FAULT=3
uint8 REASON_FAULT_DATA_LOST=4
uint8 REASON_DEACTIVATED=5
uint8 REASON_RESUME_FAILED=6

# The last driver/fault_code the supervisor read (0 = no fault), NaN before the first one.
float64 fault_code

# True when the last halt got the hardware component out of ACTIVE. False while moving, and
# after a halt the hardware component did not confirm.
bool hardware_stopped
