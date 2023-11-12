# Software

## Links
https://youtu.be/JBvnB0279-Q?si=LplhdbVDn-7sKslk

## Old PID coefficients
    P		I		D
    Pri		0,14	0,18	0,102
    Sec		0,14	0,18	0,102
    Yaw		0,01	0		0

## PID adjust

    **Preparation**

    Platformio.ini - Activate MINITERM and PID_ADJUST
    Open BT COM 
    Start the program, and the main menu opens.

### Select axis and coefficient.

**"X"** selects the primary axis and sets a "1" with setItemAxis()

Afterward<br>
**"P"** selects the first (P)IDParameter and sets a "10" with setItemOffset()
    setItemAxis() + setItemOffset() + setItemOffset() = PidType.

**"+" or "-"** selects coefficient_Up() or coefficient_Down()<br>  
select(getPidType(true)) +  setItemAxis() + setItemOffset() = PIDtype "11"<br>
                                    
In this case, the P-value of the primary-axis<br>
It is also decided whether the value
increased or decreased. In addition, the accuracy set.
    
The value is passed to the PIDType in the void select(uint8_t type) function.
In this case:<br> 
    pri_kP_value = 11

### **virtual void update() override**
starts entering various parameters

## Test codes

### axis_pri_test.h
### axis_sec_test.h
### all_axis_test.h
the primary and/or secondary axis is tested, including<br>
- engines<br>
- IMU sensor<br>
- PID settings<br>

### sensor_radio_test.h
The radio class is tested in combination with the IMU sensor.
