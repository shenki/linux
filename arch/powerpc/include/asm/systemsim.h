/*
 *
 * Copyright (C) 2001, 2005 IBM
 *
 * Filename	: systemsim.h
 *
 * Originator	: Patrick Bohrer and Charles Lefurgy
 * Modified By	: Eric Van Hensbegren <ericvh@gmail.com>
 *
 * Purpose	:
 *
 *   This file is compiled with programs that are run under the
 *   PowerPC Full System simulator.  For example, stand-alone programs
 *   or operating systems.  The programs call the callthru wrapper
 *   functions which use an illegal PowerPC instruction to signal the
 *   simulator to emulate special support.
 *
 */

#ifndef _SYSTEMSIM_CONFIG_H_
#define _SYSTEMSIM_CONFIG_H_
#ifdef __KERNEL__

/* The functions callthru0 to callthru5 setup up the arguments for the
 * simulator callthru and then use the callthru instruction.  Note that
 * 0-5 specify the number of arguments after the command */

/* Note: Arguments are cast as void* to prevent casting by the
   compiler.  This way, you can pass pointers, integers, etc. in
   machine register and have the simulator interpret what the
   register is supposed to be.  To help with typing errors when using
   callthrus, we provide wrapper functions for each callthru.  The
   wrappers cast all arguments to void*.  Unfortunately, this results
   in a lot of compiler warnings that I do not know how to remove.  If
   you modify this code, be aware that we are trying to pick a type
   that is the size of the registers (32-bit or 64-bit) and that is
   why are choosing to cast to a void* (it should be the size of a
   machine register) */

static inline int callthru0(int command)
{
	register int c asm("r3") = command;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c));
	return (c);
}
static inline int callthru1(int command, unsigned long arg1)
{
	register int c asm("r3") = command;
	register unsigned long a1 asm("r4") = arg1;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c), "r"(a1));
	return (c);
}
static inline int callthru2(int command, unsigned long arg1, unsigned long arg2)
{
	register int c asm("r3") = command;
	register unsigned long a1 asm("r4") = arg1;
	register unsigned long a2 asm("r5") = arg2;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c), "r"(a1), "r"(a2));
	return (c);
}
static inline int callthru3(int command, unsigned long arg1, unsigned long arg2,
			    unsigned long arg3)
{
	register int c asm("r3") = command;
	register unsigned long a1 asm("r4") = arg1;
	register unsigned long a2 asm("r5") = arg2;
	register unsigned long a3 asm("r6") = arg3;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c), "r"(a1), "r"(a2),
		      "r"(a3));
	return (c);
}
static inline int callthru4(int command, unsigned long arg1, unsigned long arg2,
			    unsigned long arg3, unsigned long arg4)
{
	register int c asm("r3") = command;
	register unsigned long a1 asm("r4") = arg1;
	register unsigned long a2 asm("r5") = arg2;
	register unsigned long a3 asm("r6") = arg3;
	register unsigned long a4 asm("r7") = arg4;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c), "r"(a1), "r"(a2),
		      "r"(a3), "r"(a4));
	return (c);
}
static inline int callthru5(int command, unsigned long arg1, unsigned long arg2,
			    unsigned long arg3, unsigned long arg4,
			    unsigned long arg5)
{
	register int c asm("r3") = command;
	register unsigned long a1 asm("r4") = arg1;
	register unsigned long a2 asm("r5") = arg2;
	register unsigned long a3 asm("r6") = arg3;
	register unsigned long a4 asm("r7") = arg4;
	register unsigned long a5 asm("r8") = arg5;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c), "r"(a1), "r"(a2),
		      "r"(a3), "r"(a4), "r"(a5));
	return (c);
}
static inline int callthru6(int command, unsigned long arg1, unsigned long arg2,
			    unsigned long arg3, unsigned long arg4,
			    unsigned long arg5, unsigned long arg6)
{
	register int c asm("r3") = command;
	register unsigned long a1 asm("r4") = arg1;
	register unsigned long a2 asm("r5") = arg2;
	register unsigned long a3 asm("r6") = arg3;
	register unsigned long a4 asm("r7") = arg4;
	register unsigned long a5 asm("r8") = arg5;
	register unsigned long a6 asm("r9") = arg6;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c), "r"(a1), "r"(a2),
		      "r"(a3), "r"(a4), "r"(a5), "r"(a6));
	return (c);
}
static inline int callthru7(int command, unsigned long arg1, unsigned long arg2,
			    unsigned long arg3, unsigned long arg4,
			    unsigned long arg5, unsigned long arg6,
			    unsigned long arg7)
{
	register int c asm("r3") = command;
	register unsigned long a1 asm("r4") = arg1;
	register unsigned long a2 asm("r5") = arg2;
	register unsigned long a3 asm("r6") = arg3;
	register unsigned long a4 asm("r7") = arg4;
	register unsigned long a5 asm("r8") = arg5;
	register unsigned long a6 asm("r9") = arg6;
	register unsigned long a7 asm("r10") = arg7;
	asm volatile (".long 0x000eaeb0":"=r" (c):"r"(c), "r"(a1), "r"(a2),
		      "r"(a3), "r"(a4), "r"(a5), "r"(a6), "r"(a7));
	return (c);
}

#endif /* __KERNEL__ */
#endif/* _SYSTEMSIM_CONFIG_H_ */
