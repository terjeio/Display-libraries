// portmacros.h - some useful macros for Texas Instruments MSP430 processor family

#ifndef __portmacros_h__

#define __portmacros_h__

#define portIn(p) portI(p)
#define portI(p) P ## p ## IN
#define portOut(p) portO(p)
#define portO(p) P ## p ## OUT
#define portDir(p) portD(p)
#define portD(p) P ## p ## DIR
#define portSel(p, q) portS(p, q)
#define portS(p, q) P ## p ## SEL ## q
#define portRen(p) portR(p)
#define portR(p) P ## p ## REN
#define portIe(p) portF(p)
#define portF(p) P ## p ## IE
#define portEs(p) portG(p)
#define portG(p) P ## p ## IES
#define portIfg(p) portH(p)
#define portH(p) P ## p ## IFG
#define portInt(p) portW(p)
#define portW(p) PORT ## p ## _VECTOR

#define timerR(p, i) timerRx(p, i)
#define timerRx(p, i) T ## p ## i ## R
#define timerCtl(p, i) timerC(p, i)
#define timerC(p, i) T ## p ## i ## CTL
#define timerCCtl(p, i, q) timerD(p, i, q)
#define timerD(p, i, q) T ## p ## i ## CCTL ## q
#define timerCcr(p, i, q) timerA(p, i, q)
#define timerA(p, i, q) T ## p ## i ## CCR ## q
#define timerIv(p, i) timerI(p, i)
#define timerI(p, i) T ## p ## i ## IV
#define timerEx(p, i) timerE(p, i)
#define timerE(p, i) T ## p ## i ## EX0
#define timerInt(p, i, q) timerW(p, i, q)
#define timerW(p, i, q) TIMER ## i ## _ ## p ## q ## _VECTOR

#define usciCTL(p, i) usciC(p, i)
#define usciC(p, i) UC ## p ## CTL ## i
#define usciBR(p, i) usciB(p, i)
#define usciB(p, i) UC ## p ## BR ## i
#define usciIE(p) usciE(p)
#define usciE(p) UC ## p ## IE
#define usciIFG(p) usciF(p)
#define usciF(p) UC ## p ## IFG
#define usciIV(p) usciV(p)
#define usciV(p) UC ## p ## IV
#define usciRXBUF(p) usciR(p)
#define usciR(p) UC ## p ## RXBUF
#define usciTXBUF(p) usciT(p)
#define usciT(p) UC ## p ## TXBUF
#define usciSTAT(p) usciS(p)
#define usciS(p) UC ## p ## STAT
#define usciInt(p) usciW(p)
#define usciW(p) USCI_ ## p ## _VECTOR
#define usciSADDR(p) usciA(p)
#define usciA(p) UC ## p ## I2CSA

#endif
