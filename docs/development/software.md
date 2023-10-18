# Software
## PID adjust

#   Setting the PID values
    **Preparation**

    Platformio.ini - Activate MINITERM and PID_ADJUST
    Open BT COM 
    Start the program, and the main menu opens.

##  Select axis and coefficient.
    "X" selects the primary axis and sets a "1" with setItemAxis()
    Afterward
    "P" selects the first (P)IDParameter and sets a "10" with setItemOffset()
        setItemAxis() + setItemOffset() + setItemOffset() = PidType.

    "+" or "-" selects coefficient_Up() or coefficient_Down()  
                                        select(getPidType(true)) added
                                        setItemAxis() + setItemOffset()
                                        and returns the PIDtype "11".
                                        In this case, the P-value of the primary-axis

                                        It is also decided whether the value
                                        increased or decreased. In addition, the accuracy
                                        set.
        
        The value is passed to the PIDType in the void **select(uint8_t type)** function.
        In this case 
            pri_kP_value

   **virtual void update() override**
    starts entering various parameters