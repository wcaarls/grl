; -------------------------- RANROT.ASM ---------------------------------
; random number generator by Agner Fog 1999
; 32-bit mode version for 80x86 and compatible microprocessors.
; Works with Window 95 and later, and possibly with other 32-bit systems.
;
; The code uses the RANROT type W algorithm with self-test.
; The resolution is 63 bits for WRandom and 32 bits for WBRandom.
;
; WRandomInit must be called before the first call to any of the random
; functions.
; WRandom returns a floating point number between 0 and 1.
; WBRandom returns an integer between 0 and 0xFFFFFFFF
; WIRandom returns an integer in the interval defined by min and max
;
; C++ prototypes:
; extern"C" void _cdecl WRandomInit (int seed);
; extern"C" double _cdecl WRandom (void);
; extern"C" unsigned _cdecl WBRandom (void);
; extern"C" int _cdecl WIRandom (int min, int max);
;
; -----------------------------------------------------------------------

INCLUDE segdef.asi            ; segment definitions

; define appropriate function for error-exit:
; (depends on system):
; ErrorExit equ <FatalAppExitA>

; define parameters
JJ      equ     10            ; lag 1
KK      equ     17            ; lag 2, size of circular buffer
R1      equ     19            ; rotate count
R2      equ     27            ; rotate count
SAFE    equ      1            ; define this if you want self-test

RAN_DATA SEGMENT
align   16
randp1  DT      1.5           ; used for conversion to float
        DW      0             ; alignment
p1      DD      0             ; pointer in circular buffer
p2      DD      0             ; pointer in circular buffer
randbuf DD      (2*KK) dup(?) ; circular buffer

IFDEF   SAFE
randbufcopy DD  (4*KK) dup(?) ; double copy of randbuf initial state
RandMessage1    DB 'Random number generator not initialized',0
RandMessage2    DB 'Random number generator returned to initial state',0
ENDIF
RAN_DATA ENDS


RAN_TEXT   SEGMENT
  
PUBLIC WRandomInit, _WRandomInit
PUBLIC WRandom, _WRandom, WBRandom, _WBRandom, WIRandom, _WIRandom

WBRandom PROC NEAR
_WBRandom LABEL NEAR
        PUSH    EBX
        MOV     EBX, [p1]       ; ring buffer pointers
        MOV     ECX, [p2]       ; ring buffer pointer
        MOV     EDX, [randbuf][EBX]
        MOV     EAX, [randbuf][EBX+4]
IFDEF   SAFE        
        CMP     EDX, [randbufcopy][0]   ; self-test first dword
        JE      SHORT R50       ; jump if further compare needed
ENDIF        
R20:    ROL     EDX, R1                  ; rotate bits
        ROL     EAX, R2
        ADD     EDX, [randbuf][ECX]      ; add two dwords
        ADD     EAX, [randbuf][ECX+4]
        MOV     [randbuf][EBX], EAX      ; save in swapped order
        MOV     [randbuf][EBX+4], EDX
        SUB     EBX, 8                   ; decrement p1
        JNC     SHORT R30
        MOV     EBX, (KK-1)*8            ; wrap around p1
R30:    SUB     ECX, 8                   ; decrement p2
        JNC     SHORT R40
        MOV     ECX, (KK-1)*8            ; wrap around p2
R40:    MOV     [p1], EBX                ; save updated pointers
        MOV     [p2], ECX
        POP     EBX
        RET
        
IFDEF   SAFE
R50:    ; first dword is identical. compare entire buffer
        PUSHAD
        MOV     ESI, OFFSET DS:randbuf
        MOV     EDI, OFFSET DS:randbufcopy+KK*8
        SUB     EDI, EBX                  ; buffer is rotated
        MOV     ECX, KK*2
        CLD
        REPE    CMPSD                     ; compare buffers
        JE      SHORT R60
        POPAD
        JMP     R20                       ; buffers are different. continue
R60:    ; error found. initial state is repeated
IFDEF   ErrorExit
        PUSH    OFFSET DS:RandMessage1
        MOV     EAX, [p1]
        CMP     EAX, [p2]
        JE      SHORT R70
        PUSH    OFFSET DS:RandMessage2
R70:    PUSH    0
        CALL    ErrorExit   ; name of abort procedure depends on system
        EXTRN   ErrorExit:NEAR
ELSE
        ; ErrorExit function not defined. Make division by 0 error
        DIV     ECX
ENDIF
ENDIF
WBRandom ENDP


WRandom PROC NEAR                        ; make random long double float
_WRandom LABEL NEAR
        CALL    WBRandom                 ; 64 bit random number in EDX:EAX
        OR      EDX, 80000000H           ; fast convert to float
        MOV     DWORD PTR [randp1],EAX
        MOV     DWORD PTR [randp1+4],EDX
        FLD1
        FLD     [randp1]
        FSUBR
        RET
WRandom ENDP


WIRandom PROC   NEAR
_WIRandom LABEL NEAR
        CALL    WBRandom             ; make random 64 bits
        PUSH    EBX
        MOV     EBX, [ESP+8+4]       ; max
        SUB     EBX, [ESP+4+4]       ; min
        JS      SHORT RERROR         ; max < min
        INC     EBX                  ; max - min + 1
        MOV     ECX, EDX             ; high bits of random number
        MUL     EBX                  ; multiply low 32 bits
        MOV     EAX, ECX
        MOV     ECX, EDX
        MUL     EBX                  ; multiply high 32 bits
        ADD     EAX, ECX
        ADC     EDX, [ESP+4+4]       ; add min
        MOV     EAX, EDX
        POP     EBX
        RET                          ; RET 8 if not _cdecl calling

RERROR: MOV     EAX, 80000000H       ; error exit   
        POP     EBX
        RET                          ; RET 8 if not _cdecl calling
WIRandom ENDP


WRandomInit PROC NEAR
_WRandomInit LABEL NEAR
        MOV     EAX, [ESP+4]    ; seed
        XOR     ECX, ECX
        ; make random numbers and put them into buffer
R80:    IMUL    EAX, 2891336453
        INC     EAX
        MOV     [randbuf][ECX*4], EAX
        INC     ECX
        CMP     ECX, KK*2
        JB      R80
        FLD1
        FSTP    [randp1]        ; initialize bit 64-79
        MOV     [p1], 0         ; initialize buffer pointers
        MOV     [p2], JJ*8
IFDEF   SAFE
        PUSHAD                  ; make 2 copies of initial state
        MOV     ECX, KK*2
        MOV     ESI, OFFSET DS:randbuf
        MOV     EDI, OFFSET DS:randbufcopy
        PUSH    ECX
        PUSH    ESI
        CLD
        REP     MOVSD
        POP     ESI
        POP     ECX
        REP     MOVSD
        INC     [randbufcopy][0]   ; avoid error message
        CALL    WBRandom           ; prepare first random number
        DEC     [randbufcopy][0]
        POPAD
ELSE
        CALL    WBRandom           ; prepare first random number
ENDIF
        PUSH    EDI
        MOV     EDI, 30
R90:    CALL    WBRandom
        DEC     EDI
        JNZ     R90
        POP     EDI                                                

        RET     0                  ; RET 4 if not _cdecl calling
WRandomInit ENDP

RAN_TEXT ENDS
        END



; Example of calling from C++ program:

; #include "arandom.h"
; #include <stdio.h>
;
; int seed = time(0);
; WRandomInit (seed);
; for (int i=0; i<20; i++) {
;   printf ("\n%14.10f  %2i", WRandom(), WIRandom(0,99));}
