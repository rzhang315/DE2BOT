; StartingCodeWithLog.asm
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program includes:
; - Wireless position logging using timer interrupt
; - Subroutines for
; -- 16x16 signed multiplication
; -- 16/16 signed division
; -- Arctangent (in appropriate robot units)
; -- Distance (L2 norm) approximation
; - Example of using tables (ILOAD, ISTORE)
; - Additional misc. examples

;***************************************************************
;* Jump Table (Starts at word 128)
;***************************************************************
; When an interrupt occurs, execution is redirected to one of
; these addresses (depending on the interrupt source), which
; needs to either contain RETI (return from interrupt) if not
; used, or a JUMP instruction to the desired interrupt service
; routine (ISR).  The first location is the reset vector, and
; should be a JUMP instruction to the beginning of your normal
; code.

ORG        &H000       ; Jump table is located in mem 0-4
	JUMP   Init        ; Reset vector
	RETI               ; Sonar interrupt (unused)
	JUMP   CTimer_ISR  ; Timer interrupt
	RETI               ; UART interrupt (unused)
	RETI               ; Motor stall interrupt (unused)
	
;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display batt voltage on LCD

WaitForSafety:
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
	JUMP   Main

;***************************************************************
;*User Values
;***************************************************************


;PointTable
PtrPoint: DW 0
PointTable: 
; Point 1
DW -4
DW 2
; Point 2
DW -3
DW -2
; Point 3
DW 2
DW -1
; Point 4
DW 4
DW 3
; Point 5
DW 2
DW -3
; Point 6
DW -4
DW 4
; Point 7
DW 1
DW 3
; Point 8
DW -3
DW 5
; Point 9
DW -1
DW -5
; Point 10
DW 2
DW 4
; Point 11
DW 1
DW 4
; Point 12
DW -4
DW 5

;OrderTable
PtrOrder: DW 0
OrderTable:  
DW 3
DW 5
DW 9
DW 2
DW 1
DW 6
DW 12
DW 8
DW 7
DW 11
DW 10
DW 4

; Max number of points to traverse
MaxPointNum:     DW 12
CurrentPointNum: DW 0
CurrentPointDisp: DW 1

;Intermediate Values used for Calculation
XCurr:       DW 0
XDest:       DW 0
YCurr:       DW 0
YDest:       DW 0
ThetaDest:   DW 0
ThetaCurr:   DW 0
DestLength:  DW 0
DestAngle:   DW 0
TimerVal:    DW 0
TempVal:     DW 0
WheelSpeedL: DW 0
WheelSpeedR: DW 0

; Robot Control Parameters
MaxSpeed:      DW 300  ; Must be divisible by 4
MidSpeed:      DW 300  ;
SlowSpeed:     DW 100  ;
MaxSpeed_temp: DW 0    ;
Margin_Mid_X:  DW 870
Margin_Mid_Y:  DW 1160
Margin_Slow_X: DW 1160
Margin_Slow_Y: DW 1450
Margin_Mid:    DW 2400
Margin_Slow:   DW 600
UpdateSpeed:   DW 4    ; In 1/10 of a second
DistProximity: DW 100   ; Distance to target recognized as at target
TurnProximity: DW 128  ; Distance to target that activates turn only mode
TurnAngle:     DW 80   ; Distance to target that activates turn only mode
    
;***************************************************************
;* Main code
;***************************************************************

; Main code
Main:
	OUT    RESETPOS    ; reset odometer in case wheels moved after programming
	CALL   UARTClear   ; empty the UART receive FIFO of any old data
	CALL   StartLog    ; enable the interrupt-based position logging
	
	; Initialize pointers to tables
	LOADI  OrderTable
	ADDI   -1
	STORE  PtrOrder
	
	LOADI  PointTable
	STORE  PtrPoint
	;JUMP   LengthAngleDebugging
	
	; Set MaxSpeed
	LOAD   MaxSpeed
	Store  MaxSpeed_temp
	
MainLoop:
	; Get the next point
	CALL   NextPoint
	
	; Go to the point
	CALL   GoToPoint
	
	; Indicate destination number
    ILOAD  PtrOrder
    CALL   IndicateDest

	; Check if max number of points reached    
    CALL   MaxPointsReached
    	
	; Loop back to next point
	JUMP   MainLoop

EndProgram:
	LOADI  0
	OUT    RVELCMD
	OUT    LVELCMD
    CALL   Wait1
    JUMP   Die

LengthAngleDebugging:
	CALL   NextPoint
LengthAngleDebuggingLoop:	
	CALL   GetAngleDist
	JUMP   LengthAngleDebuggingLoop

;***************************************************************
;* Helper subroutines
;***************************************************************
; Wait a short amount of time specified by UpdateSpeed, with AC value maintained
WaitShort:
    STORE  TempVal
	LOAD   UpdateSpeed
	CALL   WaitAC
	LOAD   TempVal
	RETURN

; Beep a short time
ShortBeep:
    STORE TempVal
    LOADI 4
    OUT   BEEP
    LOADI 1
    CALL  WaitAC
    LOADI 0
    OUT   BEEP
    Load  TempVal
    RETURN

; Display the value in the AC in the SSEG1 for 1 second
CheckVal:
    STORE  TempVal
	OUT    SSEG1
	LOADI  10
	CALL   WaitAC
	LOAD   TempVal
	RETURN

; Convert AC value from feet to robot units
ConvertToRobotUnits:                             
	STORE  m16sA    ; operand 1                      
	LOADI  290                                        
	STORE  m16sB    ; operand 2                      
	CALL   Mult16s  ; multiply                       
	LOAD   mres16sL ; low word of the 32-bit result  
    RETURN

SonarCheck:
	LOADI  500
    OUT    SONALARM             
    LOADI  48
    OUT    SONAREN
    OUT    SONARINT
    IN     SONALARM
    JZERO  SonarCheck_Move
    LOADI  0
    OUT    BEEP
	LOAD   MidSpeed
	STORE  MaxSpeed_temp
    RETURN
SonarCheck_Move:
    LOADI  0
    OUT    BEEP
	LOAD   MaxSpeed
	STORE  MaxSpeed_temp
    CALL   AdjustSpeedOdometry ; Adjust speed
	RETURN
    
;***************************************************************
;* Major subroutines
;***************************************************************
    
; Check if maximum number of points reached
; @param:  MaxPointNum max number of points before ending
MaxPointsReached:	
	LOAD   CurrentPointDisp
	OUT    LEDS
	LOAD   CurrentPointNum
	ADDI   1
	STORE  CurrentPointNum
	SUB    MaxPointNum
	JZERO   EndProgram
	LOAD   CurrentPointDisp
	SHIFT  1
	STORE  CurrentPointDisp
	RETURN

; Get the x and y value of the next point
; @result:  XDest x value of the destination in feet
; @result:  XDest y value of the destination in feet
NextPoint:
	; Reset Point pointer
	LOADI  PointTable
	STORE  PtrPoint
	
	; Increment the Order pointer
	LOAD   PtrOrder
	ADDI   1
	STORE  PtrOrder
	
	; Calculate the offset into the point table
	ILOAD  PtrOrder
	
	ADDI   -1
	SHIFT  1  
	ADD    PtrPoint
	STORE  PtrPoint
	
	; Set the destination x value
	ILOAD  PtrPoint
	CALL   ConvertToRobotUnits
	STORE  XDest
	
	; Set the destination y value
	; Increment to get the y value
	LOAD   PtrPoint
	ADDI   1
	STORE  PtrPoint
	ILOAD  PtrPoint
	CALL   ConvertToRobotUnits
	STORE  YDest
	
	RETURN

; Go to a specific point (X,Y) obtained from the pointer to table
; @param:  XDest x value of destination in feet
; @param:  YDest y value of destination in feet
GoToPoint:
	
	; Get the angle and distance to destination
	CALL   GetAngleDist
	LOAD   DestLength
	
	; Consider point reached if within a certain distance
	SUB    DistProximity
	JNEG   ExitGoToPoint
	
	; Adjust Speed depending on sonar reading
	;CALL   AdjustSpeedOdometry
	CALL   SonarCheck
	
	; Set wheel speed according to angle
    CALL   SpeedFunctionFO
    JUMP   GoToPoint
ExitGoToPoint:
	RETURN
	
; TODO: Make exit condition a simple comparison between current and destination position
	
; Calculate the angle and distance to destination
; @param:   XDest x value of destination in feet
; @param:   YDest y value of destination in feet
; @result:  DestLength length in robot units to destination
; @result:  DestAngle  angle in degrees to destination
; @display: SSEG1 DestLength value
; @display: SSEG2 DestAngle value
GetAngleDist:
	; Get the current angle
	IN     THETA
	STORE  ThetaCurr

	; Get the delta x value
	IN     XPOS
	STORE  XCurr
	LOAD   XDest
	SUB    XCurr
	STORE  AtanX
	STORE  L2X
	
	; Get the delta y value
	IN     YPOS
	STORE  YCurr
	LOAD   YDest
	SUB    YCurr
	STORE  AtanY
	STORE  L2Y
	
	; Calculate angle
	CALL   Atan2
	STORE  ThetaDest
	SUB    ThetaCurr
	JPOS   GetAngleDist_StoreResult
	ADDI   360
GetAngleDist_StoreResult:
	STORE  DestAngle
	OUT    SSEG2
	
	; Calculate distance
	CALL   L2Estimate
	STORE  DestLength
	
	OUT    SSEG1
	
	RETURN

; Scale angle (0 to 90) from (0 to Max) -> DestAngle * (MaxSpeed/2)/45
ScaleSpeed90toMax:
	STORE  m16sA    ; operand 1	
	LOAD   MaxSpeed_temp
	SHIFT  -1 ; More stable if divided by 180
	STORE  m16sB    ; operand 2
	CALL   Mult16s  ; multiply
	;LOAD   mres16sH ; high word of the 32-bit result
	LOAD   mres16sL ; low word of the 32-bit result

	STORE  d16sN    ; numerator
	LOADI  45
	STORE  d16sD    ; denominator
	CALL   Div16s   ; divide
	LOAD   dres16sQ ; quotient of division
	RETURN

; Scale angle (0 to 45) from (0 to Max) -> DestAngle * (MaxSpeed)/45
ScaleSpeed45toMax:
	STORE  m16sA    ; operand 1	
	LOAD   MaxSpeed_temp
	SHIFT  0 ; More stable if divided by 90
	STORE  m16sB    ; operand 2
	CALL   Mult16s  ; multiply
	;LOAD   mres16sH ; high word of the 32-bit result
	LOAD   mres16sL ; low word of the 32-bit result

	STORE  d16sN    ; numerator
	LOADI  45
	STORE  d16sD    ; denominator
	CALL   Div16s   ; divide
	LOAD   dres16sQ ; quotient of division
	RETURN
	

	; Change MaxSpeed_temp depending on sonar readings
AdjustSpeedOdometry:
	IN    XPOS
	CALL  ABS
    SUB   Margin_Mid_X
    JPOS  AdjustSpeedOdometry_Mid
    
	IN    XPOS
	CALL  ABS
    SUB   Margin_Slow_X
    JPOS  AdjustSpeedOdometry_Slow
    
	IN    YPOS
	CALL  ABS
    SUB   Margin_Mid_Y
    JPOS  AdjustSpeedOdometry_Mid
    
	IN    YPOS
	CALL  ABS
    SUB   Margin_Slow_Y
    JPOS  AdjustSpeedOdometry_Slow
    
	LOAD   MaxSpeed
	STORE  MaxSpeed_temp
    
	JUMP   AdjustSpeedOdometry_Exit
   	
AdjustSpeedOdometry_Mid:
	LOAD   MidSpeed
	STORE  MaxSpeed_temp
	JUMP   AdjustSpeedOdometry_Exit
	
AdjustSpeedOdometry_Slow:
	LOAD   SlowSpeed
	STORE  MaxSpeed_temp
	JUMP   AdjustSpeedOdometry_Exit
	
AdjustSpeedOdometry_Exit:
	LOAD   MaxSpeed_temp
	OUT    LCD
	RETURN


; Change MaxSpeed_temp depending on sonar readings
AdjustSpeedSonar:
    LOADI DestLength0
    SUB	Margin_Mid
    JNEG AdjustSpeedSonar_Mid
    
    LOADI DestLength2
    SUB	Margin_Mid
    JNEG AdjustSpeedSonar_Mid
    
    LOADI DestLength3
    SUB	Margin_Mid
    JNEG AdjustSpeedSonar_Mid
    
    LOADI DestLength5
    SUB	Margin_Mid
    JNEG AdjustSpeedSonar_Mid
    
	LOAD   MaxSpeed
	STORE  MaxSpeed_temp
	
	JUMP   AdjustSpeedSonar_Exit
   	
AdjustSpeedSonar_Mid:
    LOADI DestLength0
    SUB	Margin_Slow
    JNEG AdjustSpeedSonar_Slow
    
    LOADI DestLength2
    SUB	Margin_Mid
    JNEG AdjustSpeedSonar_Slow
    
    LOADI DestLength3
    SUB	Margin_Slow
    JNEG AdjustSpeedSonar_Slow
    
    LOADI DestLength5
    SUB	Margin_Slow
    JNEG AdjustSpeedSonar_Slow
   	
    ; Change the speed
	LOAD   MidSpeed
	STORE  MaxSpeed_temp
	
	JUMP   AdjustSpeedSonar_Exit
	
AdjustSpeedSonar_Slow:
	
	LOAD   SlowSpeed
	STORE  MaxSpeed_temp
	JUMP   AdjustSpeedSonar_Exit
	
AdjustSpeedSonar_Exit:
	LOAD   MaxSpeed_temp
	OUT    LCD
	RETURN

;***************************************************************
;* Implementations of Braitenberg functions
;***************************************************************
SpeedFunctionFO:
    LOAD   DestAngle
    ADDI   -180
    JNEG   SpeedFunctionFO_CCW
    JUMP   SpeedFunctionFO_CW
    
SpeedFunctionFO_CW: ; angle between 180 to 359
	
	; Turn in place if angle > TurnAngle and distance < TurnProximity
	;LOAD   DestLength
	;SUB    TurnProximity
	;JPOS   SpeedFunction_FO_CW_Main
	LOAD   DestAngle
	SUB    TurnAngle
	ADDI   -200
	JPOS   SpeedFunction_FO_CW_Main
	LOAD   MaxSpeed_temp
	STORE  WheelSpeedR
	JUMP   SpeedFunction_FO_CW_Reverse

SpeedFunction_FO_CW_Main:
	LOAD   DestAngle
    ADDI   -270
    CALL   ABS
 	CALL   ScaleSpeed90toMax
	STORE  WheelSpeedR

	LOAD   DestAngle
	ADDI   -270
	JNEG   SpeedFunction_FO_CW_Reverse
	LOAD   WheelSpeedR
	JUMP   SpeedFunction_FO_CW_MoveWheel
SpeedFunction_FO_CW_Reverse:
	LOADI  0
	SUB    WheelSpeedR
SpeedFunction_FO_CW_MoveWheel:
	;LOAD   dres16sR ; remainder of division
	OUT    RVELCMD
	LOAD   MaxSpeed_temp
	OUT    LVELCMD
 	RETURN
	
SpeedFunctionFO_CCW: ; angle between 0 to 180
	; Scale angle (0 to 180) from (0 to MaxSpeed) -> DestAngle * (MaxSpeed/4)/45)
    
	; Turn in place if angle > TurnAngle and distance < TurnProximity
	;LOAD   DestLength
	;SUB    TurnProximity
	;JPOS   SpeedFunction_FO_CCW_Main
	LOAD   DestAngle
	SUB    TurnAngle
	JNEG   SpeedFunctionFO_CCW_Main
	LOAD   MaxSpeed_temp
	STORE  WheelSpeedL
	JUMP   SpeedFunctionFO_CCW_Reverse

SpeedFunctionFO_CCW_Main:
	; DestAngle
	LOAD   DestAngle
    ADDI   -90
    CALL   ABS
    CALL   ScaleSpeed90toMax
	STORE  WheelSpeedL

	LOAD   DestAngle
	ADDI   -90
	JPOS   SpeedFunctionFO_CCW_Reverse
	LOAD   WheelSpeedL
	JUMP   SpeedFunctionFO_CCW_MoveWheel
SpeedFunctionFO_CCW_Reverse:
	LOADI  0
	SUB    WheelSpeedL
SpeedFunctionFO_CCW_MoveWheel:
	;LOAD   dres16sR ; remainder of division
	OUT    LVELCMD
	LOAD   MaxSpeed_temp
	OUT    RVELCMD
 	RETURN

 	
 	
 	
 	
; Move forwards and backwards
SpeedFunctionFB:
    LOAD   DestAngle
    ADDI   -90
    JNEG   SpeedFunctionFB_0to89
    ADDI   -90
    JNEG   SpeedFunctionFB_90to179
    ADDI   -90
    JNEG   SpeedFunctionFB_180to269
    JUMP   SpeedFunctionFB_270to359

SpeedFunctionFB_0to89:
	LOAD   DestAngle
	ADDI   -45
	CALL   ABS
    CALL   ScaleSpeed45toMax
    STORE  WheelSpeedL
    LOAD   DestAngle
    ADDI   -45
    JNEG   SpeedFunctionFB_0to44
    JNEG   SpeedFunctionFB_0to44
SpeedFunctionFB_0to44:
	LOAD   WheelSpeedL
	OUT    LVELCMD
	LOAD   MaxSpeed_temp
	OUT    RVELCMD
    JUMP   SpeedFunctionFB_Exit
SpeedFunctionFB_45to89:
	LOADI  0
	SUB    WheelSpeedL
	OUT    LVELCMD
	LOAD   MaxSpeed_temp
	OUT    RVELCMD
    JUMP   SpeedFunctionFB_Exit
    
    
SpeedFunctionFB_90to179:
	LOAD   DestAngle
	ADDI   -135
	CALL   ABS
    CALL   ScaleSpeed45toMax
    STORE  WheelSpeedL
    LOAD   DestAngle
    ADDI   -135
    JNEG   SpeedFunctionFB_90to134
    JNEG   SpeedFunctionFB_135to179
SpeedFunctionFB_90to134:
	LOAD   WheelSpeedL
	OUT    LVELCMD
	LOADI  0
	SUB    MaxSpeed_temp
	OUT    RVELCMD
    JUMP   SpeedFunctionFB_Exit
SpeedFunctionFB_135to179:
	LOADI  0
	SUB    WheelSpeedL
	OUT    LVELCMD
	LOADI  0
	SUB    MaxSpeed_temp
	OUT    RVELCMD
    JUMP   SpeedFunctionFB_Exit
    
    
SpeedFunctionFB_180to269:
	LOAD   DestAngle
	ADDI   -225
	CALL   ABS
    CALL   ScaleSpeed45toMax
    STORE  WheelSpeedL
    LOAD   DestAngle
    ADDI   -225
    JNEG   SpeedFunctionFB_180to224
    JNEG   SpeedFunctionFB_225to269
SpeedFunctionFB_180to224:
	LOADI  0
	SUB    WheelSpeedR
	OUT    RVELCMD
	LOADI  0
	SUB    MaxSpeed_temp
	OUT    LVELCMD
    JUMP   SpeedFunctionFB_Exit
SpeedFunctionFB_225to269:
	LOAD   WheelSpeedR
	OUT    RVELCMD
	LOADI  0
	SUB    MaxSpeed_temp
	OUT    LVELCMD
    JUMP   SpeedFunctionFB_Exit
    
SpeedFunctionFB_270to359:
	LOAD   DestAngle
	ADDI   -315
	CALL   ABS
    CALL   ScaleSpeed45toMax
    STORE  WheelSpeedL
    LOAD   DestAngle
    ADDI   -315
    JNEG   SpeedFunctionFB_270to314
    JNEG   SpeedFunctionFB_315to359
SpeedFunctionFB_270to314:
	LOADI  0
	SUB    WheelSpeedR
	OUT    RVELCMD
	LOAD   MaxSpeed_temp
	OUT    LVELCMD
    JUMP   SpeedFunctionFB_Exit
SpeedFunctionFB_315to359:
	LOAD   WheelSpeedR
	OUT    RVELCMD
	LOAD   MaxSpeed_temp
	OUT    LVELCMD
    JUMP   SpeedFunctionFB_Exit

SpeedFunctionFB_Exit:
	RETURN
	

	
;***************************************************************
;* Subroutines
;***************************************************************
Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero        ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD        ; An indication that we are dead
	OUT    SSEG2
	CALL   StopLog     ; Disable position logging
Forever:
	JUMP   Forever     ; Do this forever.
DEAD:      DW &HDEAD   ; Example of a "local variable"

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN

; Subroutine to wait the number of timer counts currently in AC
WaitAC:
	STORE  WaitTime
	OUT    Timer
WACLoop:
	IN     Timer
	SUB    WaitTime
	JNEG   WACLoop
	RETURN
	WaitTime: DW 0     ; "local" variable.
	
; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

; Subroutines to send AC value through the UART,
; Calling UARTSend2 will send both bytes of AC
; formatted for default base station code:
; [ AC(15..8) | AC(7..0)]
; Calling UARTSend1 will only send the low byte.
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend2:
	STORE  UARTTemp
	SHIFT  -8
	ADDI   -27   ; escape character
	JZERO  UEsc1
	ADDI   27
	OUT    UART_DAT
	JUMP   USend2
UEsc1:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
USend2:
	LOAD   UARTTemp
UARTSend1:
	AND    LowByte
	ADDI   -27   ; escape character
	JZERO  UEsc2
	ADDI   27
	OUT    UART_DAT
	RETURN
UEsc2:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
	RETURN
	UARTTemp: DW 0

; Subroutine to send a newline to the computer log
UARTNL:
	LOAD   NL
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NL: DW &H0A1B

; Subroutine to send a space to the computer log
UARTNBSP:
	LOAD   NBSP
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NBSP: DW &H201B

; Subroutine to clear the internal UART receive FIFO.
UARTClear:
	IN     UART_DAT
	JNEG   UARTClear
	RETURN

; Subroutine to tell the server that this position is one
; of the destinations.  Use AC=0 for generic indication,
; or AC=#1-12 for specific indication
IndicateDest:
	; AC contains which destination this is
	AND    LowNibl    ; keep only #s 0-15
	STORE  IDNumber
	LOADI  1
	STORE  IDFlag     ; set flag for indication
	RETURN
	IDNumber: DW 0
	IDFlag: DW 0
	

; Timer interrupt, used to send position data to the server
CTimer_ISR:
	CALL   UARTNL ; newline
	IN     XPOS
	CALL   UARTSend2
	IN     YPOS
	CALL   UARTSend2
	LOAD   IDFlag ; check if user has request a destination indication
	JPOS   CTIndicateDest ; if yes, do it; otherwise...
	RETI   ; return from interrupt
CTIndicateDest:
	LOAD   IDNumber
	CALL   UARTSend1 ; send the indicated destination
	LOADI  0
	STORE  IDFlag
	RETI

; Configure the interrupt timer and enable interrupts
StartLog:
	; See supporting information on the powersof2 site for how
	; SCOMP's communication system works.
	CALL   UARTNL      ; send a newline to separate data
	LOADI  0
	STORE  IDFlag      ; clear any pending flag
	LOADI  50
	OUT    CTIMER      ; configure timer for 0.01*50=0.5s interrupts
	CLI    &B0010      ; clear any pending interrupt from timer
	SEI    &B0010      ; enable interrupt from timer (source 1)
	RETURN

; Disable the interrupt timer and interrupts
StopLog:
	CLI    &B0010      ; disable interrupt source 1 (timer)
	LOADI  0
	OUT    CTIMER      ; reset configurable timer
	RETURN

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW -1                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;*******************************************************************************
; Mod180: modulo 180
; Returns AC%180 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************	
Mod180:
	JNEG   Mod180n      ; handle negatives
Mod180p:
	ADDI   -180
	JPOS   Mod180p      ; subtract 180 until negative
	ADDI   180          ; go back positive
	RETURN
Mod180n:
	ADDI   180          ; add 180 until positive
	JNEG   Mod180n
	ADDI   -180         ; go back negative
	RETURN
	
;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOADI  246       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOADI  104       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0
	
;***************************************************************
;* Variables
;***************************************************************
Temp:  DW 0 ; "Temp" is not a great name, but can be useful
Temp2: DW 0
Temp3: DW 0
CDX: DW 0      ; current desired X
CDY: DW 0      ; current desired Y
CDT: DW 0      ; current desired angle
CX:  DW 0      ; sampled X
CY:  DW 0      ; sampled Y
CT:  DW 0      ; sampled theta

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 952       ; ~1m in 1.05mm units
HalfMeter: DW 476      ; ~0.5m in 1.05mm units
OneFoot:  DW 290       ; ~1ft in 1.05mm robot units
TwoFeet:  DW 581       ; ~2ft in 1.05mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 130       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DestLength0:    EQU &HA8  ; the eight sonar distance readings
DestLength1:    EQU &HA9  ; ...
DestLength2:    EQU &HAA  ; ...
DestLength3:    EQU &HAB  ; ...
DestLength4:    EQU &HAC  ; ...
DestLength5:    EQU &HAD  ; ...
DestLength6:    EQU &HAE  ; ...
DestLength7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0