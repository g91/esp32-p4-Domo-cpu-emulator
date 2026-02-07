; M68K Assembly Test Application 3: Pure Assembly Test
; Tests basic 68000 instructions and addressing modes

    ORG     $0

; Reset vectors
    DC.L    $00FFFFFC       ; Initial Stack Pointer (top of 16MB RAM)
    DC.L    START           ; Initial Program Counter

; Constants
CONSOLE_OUT EQU $00F02000   ; Console output address

START:
    ; Initialize registers
    MOVEQ   #0,D0           ; Clear D0
    MOVEQ   #0,D1           ; Clear D1
    MOVEQ   #0,D2           ; Clear D2
    
    ; Print welcome message
    LEA     MSG_START,A0
    BSR     PRINT_STRING
    
    ; Test 1: MOVEQ instruction
    LEA     MSG_TEST1,A0
    BSR     PRINT_STRING
    MOVEQ   #42,D0
    BSR     PRINT_HEX_BYTE
    LEA     MSG_NEWLINE,A0
    BSR     PRINT_STRING
    
    ; Test 2: ADD instruction
    LEA     MSG_TEST2,A0
    BSR     PRINT_STRING
    MOVEQ   #10,D0
    MOVEQ   #20,D1
    ADD.W   D0,D1           ; D1 = D0 + D1 = 30
    MOVE.W  D1,D0
    BSR     PRINT_HEX_BYTE
    LEA     MSG_NEWLINE,A0
    BSR     PRINT_STRING
    
    ; Test 3: Loop counter
    LEA     MSG_TEST3,A0
    BSR     PRINT_STRING
    MOVEQ   #0,D0           ; Counter
LOOP:
    MOVE.B  D0,D1
    ADD.B   #'0',D1         ; Convert to ASCII
    MOVE.B  D1,CONSOLE_OUT  ; Print digit
    MOVE.B  #' ',CONSOLE_OUT ; Print space
    ADDQ.B  #1,D0           ; Increment counter
    CMP.B   #10,D0          ; Compare with 10
    BLT     LOOP            ; Loop if less than 10
    
    LEA     MSG_NEWLINE,A0
    BSR     PRINT_STRING
    
    ; Test 4: Memory operations
    LEA     MSG_TEST4,A0
    BSR     PRINT_STRING
    LEA     TEST_MEM,A0
    MOVE.L  #$DEADBEEF,D0
    MOVE.L  D0,(A0)         ; Write to memory
    MOVE.L  (A0),D1         ; Read from memory
    CMP.L   D0,D1
    BNE     MEMORY_FAIL
    LEA     MSG_PASS,A0
    BSR     PRINT_STRING
    BRA     DONE_TEST4
MEMORY_FAIL:
    LEA     MSG_FAIL,A0
    BSR     PRINT_STRING
DONE_TEST4:
    
    ; All tests done
    LEA     MSG_DONE,A0
    BSR     PRINT_STRING
    
    ; Hang
HANG:
    BRA     HANG

; Subroutine: Print null-terminated string
; A0 = pointer to string
PRINT_STRING:
    MOVEM.L D0/A0,-(SP)     ; Save registers
    LEA     CONSOLE_OUT,A1
.LOOP:
    MOVE.B  (A0)+,D0        ; Get character
    BEQ.S   .DONE           ; If zero, done
    MOVE.B  D0,(A1)         ; Write to console
    BRA.S   .LOOP
.DONE:
    MOVEM.L (SP)+,D0/A0     ; Restore registers
    RTS

; Subroutine: Print D0 as two hex digits
PRINT_HEX_BYTE:
    MOVEM.L D0-D2/A0,-(SP)  ; Save registers
    LEA     CONSOLE_OUT,A0
    MOVE.B  #'0',(A0)
    MOVE.B  #'x',(A0)
    MOVE.B  D0,D1           ; Copy value
    LSR.B   #4,D1           ; Get high nibble
    BSR.S   PRINT_HEX_DIGIT
    MOVE.B  D0,D1           ; Copy value
    AND.B   #$0F,D1         ; Get low nibble
    BSR.S   PRINT_HEX_DIGIT
    MOVEM.L (SP)+,D0-D2/A0  ; Restore registers
    RTS

; Print single hex digit (0-F) from D1
PRINT_HEX_DIGIT:
    LEA     HEX_DIGITS,A1
    MOVE.B  0(A1,D1.W),D2
    MOVE.B  D2,CONSOLE_OUT
    RTS

; Data
HEX_DIGITS:
    DC.B    '0123456789ABCDEF'
    
MSG_START:
    DC.B    '=== M68K Assembly Test ===',13,10
    DC.B    'Pure 68000 Assembly Program',13,10,13,10,0
    
MSG_TEST1:
    DC.B    'Test 1 - MOVEQ: ',0
    
MSG_TEST2:
    DC.B    'Test 2 - ADD (10+20): ',0
    
MSG_TEST3:
    DC.B    'Test 3 - Loop (0-9): ',0
    
MSG_TEST4:
    DC.B    'Test 4 - Memory: ',0
    
MSG_PASS:
    DC.B    'PASS',13,10,0
    
MSG_FAIL:
    DC.B    'FAIL',13,10,0
    
MSG_NEWLINE:
    DC.B    13,10,0
    
MSG_DONE:
    DC.B    13,10,'All tests completed!',13,10
    DC.B    '========================',13,10,0
    
    EVEN
    
; Test memory location
TEST_MEM:
    DS.L    1
    
    END
