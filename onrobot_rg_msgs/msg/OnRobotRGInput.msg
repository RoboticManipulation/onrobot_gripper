# gFOF : Current fingertip offset in 1/10 millimeters. The value is a signed two's complement number.

uint16 gFOF

# gGWD : Current width between the gripper fingers in 1/10 millimeters.
#        The width is provided without any fingertip offset, as it is measured between the insides of the aluminum fingers.

uint16 gGWD

# gSTA : Current device status, which indicates the status of the gripper and its motion.
# Bit       - Name              - Description
# 0 (LSB)   - Busy              - High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
# 1         - Grip detected     - High (1) when an internal- or external grip is detected.
# 2         - S1 pushed         - High (1) when safety switch 1 is pushed.
# 3         - S1 trigged        - High (1) when safety circuit 1 is activated. The gripper will not move while this flag is high.
# 4         - S2 pushed         - High (1) when safety switch 2 is pushed.
# 5         - S2 trigged        - High (1) when safety circuit 2 is activated. The gripper will not move while this flag is high.
# 6         - Safety error      - High (1) when on power on any of the safety switch is pushed.
# 7 - 15    - Reserved          - Not used.

uint16 gSTA

# gWDF : Current width between the gripper fingers in 1/10 millimeters.
#        The set fingertip offset is considered.

uint16 gWDF

# All 4 status signals
uint16 StaFing_l
uint16 StaFing_r
uint16 StaProx_l
uint16 StaProx_r

# Signal that indicates if th gripper is busy (1) or accepts new commands (0)
uint16 Busy

# Signal that indicates whether an external or internal grip is detected (1)
uint16 GripDet

# Proximity offsets of both fingers in 1/10 millimeters

uint16 ProxOff_l
uint16 ProxOff_r

# Force values along all 3 axis of the left finger in 1/10 newton. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.

int32 Fx_l
int32 Fy_l
int32 Fz_l

# Torque values about all 3 axis of the left finger in 1/100 newton-meter. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.

int32 Tx_l
int32 Ty_l
int32 Tz_l

# Force Values along all 3 axis of the right finger in 1/10 newton. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.

int32 Fx_r
int32 Fy_r
int32 Fz_r

# Torque values about all 3 axis of the right finger in 1/100 newton-meter. 
#(According to the manual the value is a signed INT), it seems to be a 2 complement number.

int32 Tx_r
int32 Ty_r
int32 Tz_r

#Proximity values of both sensors in 1/10mm
int32 Prox_l
int32 Prox_r

# Actual gripper width without any offset in 1/10 millimeters
int32 GripWidth

