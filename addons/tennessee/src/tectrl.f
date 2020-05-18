C Modifications for GRL:
C - Removed main loop
C - Captured fuctionality in two subroutines:
C   - TEINCT: reset EOLDs
C   - TECNTR: calculate XMV from XMEAS
C
C               Tennessee Eastman Process Control Test Problem
C
C                        Original codes written by
C
C                    James J. Downs and Ernest F. Vogel
C
C                  Process and Control Systems Engineering
C                        Tennessee Eastman Company
C                              P.O. Box 511
C                          Kingsport, Tennessee 37662
C
C--------------------------------------------------------------------
C
C  New version is a closed-loop plant-wide control scheme for 
C  the Tennessee Eastman Process Control Test Problem
C                                
C  The modifications are by:
C
C              Evan L. Russell, Leo H. Chiang and Richard D. Braatz
C
C                     Large Scale Systems Research Laboratory
C                        Department of Chemical Engineering
C                    University of Illinois at Urbana-Champaign
C                         600 South Mathews Avenue, Box C-3
C                              Urbana, Illinois 61801
C                             http://brahms.scs.uiuc.edu
C
C The modified text is Copyright 1998-2002 by The Board of Trustees 
C of the University of Illinois.  All rights reserved.
C 
C Permission hereby granted, free of charge, to any person obtaining a copy
C of this software and associated documentation files (the "Software"), to
C deal with the Software without restriction, including without limitation
C the rights to use, copy, modify, merge, publish, distribute, sublicense,
C and/or sell copies of the Software, and to permit persons to whom the 
C Software is furnished to do so, subject to the following conditions:
C                1. Redistributions of source code must retain the above copyright
C                   notice, this list of conditions and the following disclaimers.
C                2. Redistributions in binary form must reproduce the above 
C                   copyright notice, this list of conditions and the following 
C                   disclaimers in the documentation and/or other materials 
C                   provided with the distribution.
C                3. Neither the names of Large Scale Research Systems Laboratory,
C                   University of Illinois, nor the names of its contributors may
C                   be used to endorse or promote products derived from this 
C                   Software without specific prior written permission.
C
C THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
C OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
C FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
C THE CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
C OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
C ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
C DEALINGS IN THE SOFTWARE.
C----------------------------------------------------------------------
C
C  Users should cite the original code using the following references:
C
C    J.J. Downs and E.F. Vogel, "A plant-wide industrial process control 
C    problem." Presented at the AIChE 1990 Annual Meeting, Session on
C    Industrial Challenge Problems in Process Control, Paper #24a
C    Chicago, Illinois, November 14, 1990.
C
C    J.J. Downs and E.F. Vogel, "A plant-wide industrial process control 
C    problem," Computers and Chemical Engineering, 17:245-255 (1993).
C  
C  Users should cite the modified code using the following references:
C
C    E.L. Russell, L.H. Chiang, and R.D. Braatz. Data-driven Techniques 
C    for Fault Detection and Diagnosis in Chemical Processes, Springer-Verlag, 
C    London, 2000. 
C
C    L.H. Chiang, E.L. Russell, and R.D. Braatz. Fault Detection and 
C    Diagnosis in Industrial Systems, Springer-Verlag, London, 2001.  
C
C    L.H. Chiang, E.L. Russell, and R.D. Braatz. "Fault diagnosis in 
C    chemical processes using Fisher discriminant analysis, discriminant 
C    partial least squares, and principal component analysis," Chemometrics 
C    and Intelligent Laboratory Systems, 50:243-252, 2000. 
C
C    E.L. Russell, L.H. Chiang, and R.D. Braatz. "Fault detection in 
C    industrial processes using canonical variate analysis and dynamic 
C    principal component analysis," Chemometrics and Intelligent Laboratory 
C    Systems, 51:81-93, 2000. 
C
C=============================================================================
C
      SUBROUTINE TEINCT
C      
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
C
      INTEGER STEP
      COMMON/STEP/ STEP
C
      INTEGER FLAG
      COMMON/FLAG6/ FLAG
C
      DOUBLE PRECISION GAIN1, EOLD1
      COMMON/CTRL1/ GAIN1, EOLD1
      DOUBLE PRECISION GAIN2, EOLD2
      COMMON/CTRL2/ GAIN2, EOLD2
      DOUBLE PRECISION GAIN3, EOLD3
      COMMON/CTRL3/ GAIN3, EOLD3
      DOUBLE PRECISION  GAIN4, EOLD4
      COMMON/CTRL4/ GAIN4, EOLD4
      DOUBLE PRECISION GAIN5, TAUI5, EOLD5
      COMMON/CTRL5/ GAIN5, TAUI5, EOLD5
      DOUBLE PRECISION GAIN6, EOLD6
      COMMON/CTRL6/ GAIN6, EOLD6
      DOUBLE PRECISION GAIN7, EOLD7
      COMMON/CTRL7/  GAIN7, EOLD7
      DOUBLE PRECISION GAIN8, EOLD8
      COMMON/CTRL8/ GAIN8, EOLD8
      DOUBLE PRECISION GAIN9, EOLD9
      COMMON/CTRL9/ GAIN9, EOLD9
      DOUBLE PRECISION GAIN10, TAUI10, EOLD10
      COMMON/CTRL10/ GAIN10, TAUI10, EOLD10
      DOUBLE PRECISION GAIN11, TAUI11, EOLD11
      COMMON/CTRL11/ GAIN11, TAUI11, EOLD11
      DOUBLE PRECISION GAIN13, TAUI13, EOLD13
      COMMON/CTRL13/ GAIN13, TAUI13, EOLD13
      DOUBLE PRECISION GAIN14, TAUI14, EOLD14
      COMMON/CTRL14/ GAIN14, TAUI14, EOLD14
      DOUBLE PRECISION GAIN15, TAUI15, EOLD15
      COMMON/CTRL15/ GAIN15, TAUI15, EOLD15
      DOUBLE PRECISION GAIN16, TAUI16, EOLD16
      COMMON/CTRL16/ GAIN16, TAUI16, EOLD16
      DOUBLE PRECISION GAIN17, TAUI17, EOLD17
      COMMON/CTRL17/ GAIN17, TAUI17, EOLD17
      DOUBLE PRECISION GAIN18, TAUI18, EOLD18
      COMMON/CTRL18/ GAIN18, TAUI18, EOLD18
      DOUBLE PRECISION GAIN19, TAUI19, EOLD19
      COMMON/CTRL19/ GAIN19, TAUI19, EOLD19
      DOUBLE PRECISION GAIN20, TAUI20, EOLD20
      COMMON/CTRL20/ GAIN20, TAUI20, EOLD20
      DOUBLE PRECISION GAIN22, TAUI22, EOLD22
      COMMON/CTRL22/ GAIN22, TAUI22, EOLD22
C
      STEP=0
      FLAG=0
C
      SETPT(1)=3664.0        
      GAIN1=1.0
      EOLD1=0.0
      SETPT(2)=4509.3
      GAIN2=1.0
      EOLD2=0.0
      SETPT(3)=.25052
      GAIN3=1.
      EOLD3=0.0
      SETPT(4)=9.3477
      GAIN4=1.
      EOLD4=0.0
      SETPT(5)=26.902
      GAIN5=-0.083    
      TAUI5=1./3600.   
      EOLD5=0.0
      SETPT(6)=0.33712  
      GAIN6=1.22          
      EOLD6=0.0
      SETPT(7)=50.0
      GAIN7=-2.06      
      EOLD7=0.0
      SETPT(8)=50.0
      GAIN8=-1.62      
      EOLD8=0.0
      SETPT(9)=230.31
      GAIN9=0.41          
      EOLD9=0.0
      SETPT(10)=94.599
      GAIN10= -0.156     * 10.
      TAUI10=1452./3600. 
      EOLD10=0.0
      SETPT(11)=22.949    
      GAIN11=1.09  
      TAUI11=2600./3600.
      EOLD11=0.0
      SETPT(13)=32.188
      GAIN13=18.              
      TAUI13=3168./3600.   
      EOLD13=0.0
      SETPT(14)=6.8820
      GAIN14=8.3  
      TAUI14=3168.0/3600.
      EOLD14=0.0
      SETPT(15)=18.776   
      GAIN15=2.37  
      TAUI15=5069./3600.    
      EOLD15=0.0
      SETPT(16)=65.731
      GAIN16=1.69  / 10.
      TAUI16=236./3600.
      EOLD16=0.0
      SETPT(17)=75.000
      GAIN17=11.1/ 10.
      TAUI17=3168./3600.  
      EOLD17=0.0  
      SETPT(18)=120.40
      GAIN18=2.83* 10.
      TAUI18=982./3600.
      EOLD18=0.0
      SETPT(19)=13.823
      GAIN19=-83.2  / 5. /3.  
      TAUI19=6336./3600. 
      EOLD19=0.0
      SETPT(20)=0.83570  
      GAIN20=-16.3 / 5.   
      TAUI20=12408./3600.  
      EOLD20=0.0
      SETPT(12)=2633.7
      GAIN22=-1.0  * 5.   
      TAUI22=1000./3600.  
      EOLD22=0.0
C
      XMV(1) = 63.053 + 0.
      XMV(2) = 53.980 + 0.
      XMV(3) = 24.644 + 0.    
      XMV(4) = 61.302 + 0.
      XMV(5) = 22.210 + 0.
      XMV(6) = 40.064 + 0.
      XMV(7) = 38.100 + 0.
      XMV(8) = 46.534 + 0.
      XMV(9) = 47.446 + 0.
      XMV(10)= 41.106 + 0.
      XMV(11)= 18.114 + 0.
C
      END
C
C=============================================================================
C
      SUBROUTINE TECNTR
C      
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      INTEGER STEP
      COMMON/STEP/ STEP
C
      STEP=STEP+1
C
      TEST=MOD(STEP,3)
      IF (TEST.EQ.0) THEN
C
      CALL CNTR1
      CALL CNTR2
      CALL CNTR3
      CALL CNTR4
      CALL CNTR5
      CALL CNTR6
      CALL CNTR7
      CALL CNTR8
      CALL CNTR9
      CALL CNTR10
      CALL CNTR11
      CALL CNTR16
      CALL CNTR17
      CALL CNTR18
C
      ENDIF
C
      TEST1=MOD(STEP,360)
C
      IF (TEST1.EQ.0) THEN
C
      CALL CNTR13
      CALL CNTR14
      CALL CNTR15
      CALL CNTR19
C
      ENDIF
C
      TEST1=MOD(STEP,900)
      IF (TEST1.EQ.0) CALL CNTR20
C
      END
C
C=============================================================================
C
CC      SUBROUTINE CNTR
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
CC      DOUBLE PRECISION XMEAS, XMV
CC      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
CC      DOUBLE PRECISION SETPT, GAIN, TAUI, EOLD, DELTAT
CC      COMMON/CTRL/ SETPT, GAIN, TAUI, EOLD, DELTAT
C
CC      DOUBLE PRECISION ERR, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
CC      ERR = SETPT - XMEAS(15)
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
CC      DXMV = GAIN * ( ( ERR - EOLD ) + ERR * DELTAT * 60. / TAUI )
C
CC      XMV(8) = XMV(8) - DXMV
C
CC      EOLD = ERR
C
CC      RETURN
CC      END
C
C=============================================================================
C
      SUBROUTINE CNTR1
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN1, EOLD1
      COMMON/CTRL1/ GAIN1, EOLD1
C
      DOUBLE PRECISION ERR1, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR1 = (SETPT(1) - XMEAS(2)) * 100. / 5811.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN1 * ( ( ERR1 - EOLD1 ) )
C
      XMV(1) = XMV(1) + DXMV
C
      EOLD1 = ERR1
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR2
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN2, EOLD2
      COMMON/CTRL2/ GAIN2, EOLD2
C
      DOUBLE PRECISION ERR2, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR2 = (SETPT(2) - XMEAS(3)) * 100. / 8354. 
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN2 * ( ( ERR2 - EOLD2 ) )
C
      XMV(2) = XMV(2) + DXMV
C
      EOLD2 = ERR2
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR3
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN3, EOLD3
      COMMON/CTRL3/ GAIN3, EOLD3
C
      DOUBLE PRECISION ERR3, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR3 = (SETPT(3) - XMEAS(1)) * 100. / 1.017
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN3 * ( ( ERR3 - EOLD3 ) )
C
      XMV(3) = XMV(3) + DXMV
C
      EOLD3 = ERR3
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR4
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN4, EOLD4
      COMMON/CTRL4/ GAIN4, EOLD4
C
      DOUBLE PRECISION ERR4, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR4 = (SETPT(4) - XMEAS(4)) * 100. / 15.25
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN4 * ( ( ERR4 - EOLD4 ) )
C
      XMV(4) = XMV(4) + DXMV
C
      EOLD4 = ERR4
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR5
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN5, TAUI5, EOLD5
      COMMON/CTRL5/ GAIN5, TAUI5, EOLD5
C
      DOUBLE PRECISION ERR5, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR5 = (SETPT(5) - XMEAS(5))  * 100. / 53.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
C       PRINT *, 'GAIN5= ', GAIN5
C      PRINT *, 'TAUI5= ', TAUI5
C      PRINT *, 'ERR5= ', ERR5
C      PRINT *, 'EOLD5= ', EOLD5     
C
      DXMV = GAIN5 * ((ERR5 - EOLD5)+ERR5*DELTAT*3./TAUI5)
C
      XMV(5) = XMV(5) + DXMV
C
      EOLD5 = ERR5
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR6
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
      INTEGER FLAG
       COMMON/FLAG6/ FLAG
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN6, EOLD6
      COMMON/CTRL6/ GAIN6, EOLD6
C
      DOUBLE PRECISION ERR6, DXMV
C
C  Example PI Controller:
C     Stripper Level Controller
      IF (XMEAS(13).GE.2950.0) THEN
            XMV(6)=100.0
            FLAG=1
      ELSEIF (FLAG.EQ.1.AND.XMEAS(13).GE.2633.7) THEN
            XMV(6)=100.0
      ELSEIF (FLAG.EQ.1.AND.XMEAS(13).LE.2633.7) THEN
            XMV(6)=40.060
            SETPT(6)=0.33712
            EOLD6=0.0
             FLAG=0
      ELSEIF (XMEAS(13).LE.2300.) THEN
            XMV(6)=0.0
            FLAG=2
      ELSEIF (FLAG.EQ.2.AND.XMEAS(13).LE.2633.7) THEN
            XMV(6)=0.0
      ELSEIF (FLAG.EQ.2.AND.XMEAS(13).GE.2633.7) THEN
            XMV(6)=40.060
            SETPT(6)=0.33712
            EOLD6=0.0
            FLAG=0
      ELSE      
            FLAG=0
C
C    Calculate Error
C
       ERR6 = (SETPT(6) - XMEAS(10)) * 100. /1.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
C      PRINT *, 'XMV(6)= ', XMV(6)
      DXMV = GAIN6 * ( ( ERR6 - EOLD6 ) )
C
C       PRINT *, 'GAIN6= ', GAIN6
C      PRINT *, 'SETPT(6)= ', SETPT(6)      
C      PRINT *, 'XMEAS(10)= ', XMEAS(10)     
      XMV(6) = XMV(6) + DXMV
C
C       PRINT *, 'EOLD6= ', EOLD6     
C      PRINT *, 'ERR6= ', ERR6
C      PRINT *, 'XMV(6)== ', XMV(6)
      EOLD6 = ERR6
      ENDIF
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR7
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN7, EOLD7
      COMMON/CTRL7/ GAIN7, EOLD7
C
      DOUBLE PRECISION ERR7, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR7 = (SETPT(7) - XMEAS(12)) * 100. / 70.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN7 * ( ( ERR7 - EOLD7 ) )
C
      XMV(7) = XMV(7) + DXMV
C
      EOLD7 = ERR7
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR8
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN8, EOLD8
      COMMON/CTRL8/ GAIN8, EOLD8
C
      DOUBLE PRECISION ERR8, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR8 = (SETPT(8) - XMEAS(15)) * 100. / 70.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV =  GAIN8 * ( ( ERR8 - EOLD8 ) )
C
      XMV(8) = XMV(8) + DXMV
C
      EOLD8 = ERR8
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR9
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN9, EOLD9
      COMMON/CTRL9/ GAIN9, EOLD9
C
      DOUBLE PRECISION ERR9, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR9 = (SETPT(9) - XMEAS(19)) * 100. / 460. 
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN9 * ( ( ERR9 - EOLD9 ) )
C
      XMV(9) = XMV(9) + DXMV
C
      EOLD9 = ERR9
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR10
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN10, TAUI10, EOLD10
      COMMON/CTRL10/ GAIN10, TAUI10, EOLD10
C
      DOUBLE PRECISION ERR10, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR10 = (SETPT(10) - XMEAS(21)) * 100. / 150.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN10*((ERR10 - EOLD10)+ERR10*DELTAT*3./TAUI10)
C
      XMV(10) = XMV(10) + DXMV
C
      EOLD10 = ERR10
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR11
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN11, TAUI11, EOLD11
      COMMON/CTRL11/ GAIN11, TAUI11, EOLD11
C
      DOUBLE PRECISION ERR11, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR11 = (SETPT(11) - XMEAS(17)) * 100. / 46.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN11*((ERR11 - EOLD11)+ERR11*DELTAT*3./TAUI11)
C
      XMV(11) = XMV(11) + DXMV
C
      EOLD11 = ERR11
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR13
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN13, TAUI13, EOLD13
      COMMON/CTRL13/ GAIN13, TAUI13, EOLD13
C
      DOUBLE PRECISION ERR13, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR13 = (SETPT(13) - XMEAS(23)) * 100. / 100.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN13 * ((ERR13 - EOLD13)+ERR13*DELTAT*360./TAUI13)
C
      SETPT(3) = SETPT(3) + DXMV * 1.017 / 100.
C
      EOLD13 = ERR13
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR14
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN14, TAUI14, EOLD14
      COMMON/CTRL14/ GAIN14, TAUI14, EOLD14
C
      DOUBLE PRECISION ERR14, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR14 = (SETPT(14) - XMEAS(26)) * 100. /100.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN14*((ERR14 - EOLD14)+ERR14*DELTAT*360./TAUI14)
C
      SETPT(1) = SETPT(1) + DXMV * 5811. / 100.
C
      EOLD14 = ERR14
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR15
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN15, TAUI15, EOLD15
      COMMON/CTRL15/ GAIN15, TAUI15, EOLD15
C
      DOUBLE PRECISION ERR15, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR15 = (SETPT(15) - XMEAS(27)) * 100. / 100.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN15 * ((ERR15 - EOLD15)+ERR15*DELTAT*360./TAUI15)
C
      SETPT(2) = SETPT(2) + DXMV * 8354. / 100.
C
      EOLD15 = ERR15
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR16
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN16, TAUI16, EOLD16
      COMMON/CTRL16/ GAIN16, TAUI16, EOLD16
C
      DOUBLE PRECISION ERR16, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR16 = (SETPT(16) - XMEAS(18)) * 100. / 130.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN16 * ((ERR16 - EOLD16)+ERR16*DELTAT*3./TAUI16)
C
      SETPT(9) = SETPT(9) + DXMV * 460. / 100.
C
      EOLD16 = ERR16
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR17
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN17, TAUI17, EOLD17
      COMMON/CTRL17/ GAIN17, TAUI17, EOLD17
C
      DOUBLE PRECISION ERR17, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR17 = (SETPT(17) - XMEAS(8)) * 100. / 50.
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV =GAIN17*((ERR17 - EOLD17)+ERR17*DELTAT*3./TAUI17)
C
      SETPT(4) = SETPT(4) + DXMV * 15.25 / 100.
C
      EOLD17 = ERR17
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR18
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN18, TAUI18, EOLD18
      COMMON/CTRL18/ GAIN18, TAUI18, EOLD18
C
      DOUBLE PRECISION ERR18, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR18 = (SETPT(18) - XMEAS(9)) * 100. / 150. 
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN18 * ((ERR18 - EOLD18)+ERR18*DELTAT*3./TAUI18)
C
      SETPT(10) = SETPT(10) + DXMV * 150. / 100.
C
      EOLD18 = ERR18
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR19
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN19, TAUI19, EOLD19
      COMMON/CTRL19/ GAIN19, TAUI19, EOLD19
C
      DOUBLE PRECISION ERR19, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR19 = (SETPT(19) - XMEAS(30)) * 100. / 26.
C      PRINT *, 'EOLD19= ', EOLD19
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN19*((ERR19 - EOLD19)+ERR19*DELTAT*360./TAUI19)
C
      SETPT(6) = SETPT(6) + DXMV * 1. / 100.
C      PRINT *, 'SETPT(6)= ', SETPT(6)
C
      EOLD19 = ERR19
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR20
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN20, TAUI20, EOLD20
      COMMON/CTRL20/  GAIN20, TAUI20, EOLD20
C    
      DOUBLE PRECISION ERR20, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR20 = (SETPT(20) - XMEAS(38)) * 100. / 1.6
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN20*((ERR20 - EOLD20)+ERR20*DELTAT*900./TAUI20)
C
      SETPT(16) = SETPT(16) + DXMV  * 130. / 100.
C
      EOLD20 = ERR20
C
      RETURN
      END
C
C=============================================================================
C
      SUBROUTINE CNTR22
C
C  Discrete control algorithms
C
C
C   MEASUREMENT AND VALVE COMMON BLOCK
C
      DOUBLE PRECISION XMEAS, XMV
      COMMON/PV/ XMEAS(41), XMV(12)
C
C   CONTROLLER COMMON BLOCK
C
      DOUBLE PRECISION SETPT, DELTAT
      COMMON/CTRLALL/ SETPT(20), DELTAT
      DOUBLE PRECISION GAIN22, TAUI22, EOLD22
      COMMON/CTRL22/  GAIN22, TAUI22, EOLD22
C    
      DOUBLE PRECISION ERR22, DXMV
C
C  Example PI Controller:
C    Stripper Level Controller
C
C    Calculate Error
C
      ERR22 = SETPT(12) - XMEAS(13)
C
C    Proportional-Integral Controller (Velocity Form)
C         GAIN = Controller Gain
C         TAUI = Reset Time (min)
C
      DXMV = GAIN22*((ERR22 - EOLD22)+ERR22*DELTAT*3./TAUI22)
C
      XMV(6) = XMV(6) + DXMV
C
      EOLD22 = ERR22
C
      RETURN
      END
C
C=============================================================================
