#include <config.h>

/* BUG Turnarround code from NEXELL's turnarround.S */
@-------------------------------------------------------------------------------
@	current program status register mode definition
@-------------------------------------------------------------------------------
#define	I_Bit			(0x80)			@ when I bit is set, IRQ is disabled

#define	ST_MEM_BASE		(0x10000000)
#define	ST_MEM_OFFS		(0x08000000)

@-------------------------------------------------------------------------------
@	void	WriteIODW( Addr, Data )
@-------------------------------------------------------------------------------

.globl	WriteIODW
WriteIODW:
	@ R0:IOA, R1:IOD, R2:UCA, R3:CPSR, R4:TMP
	PUSH	{R2-R4}

	MOV		R2, #ST_MEM_BASE
	ORR		R2, r2, #ST_MEM_OFFS

	MRS		R3, CPSR			@ read interrupt enable status
	ORR		R4, R3, #I_Bit		@ clear interrupt enable bit
	MSR		CPSR_cxsf, R4		@ save and interrupt disable

	STR		R4, [R2]			@ before memory write
	LDR		R4, [R2]			@ before memory read

	STR		R1, [R0]			@ write io data

	LDR		R4, [R2]			@ after memory read

	MSR		CPSR_cxsf, R3		@ restore CPSR

	POP		{R2-R4}

	BX		LR

@-------------------------------------------------------------------------------
@	void	WriteIOW( Addr, Data )
@-------------------------------------------------------------------------------

.globl	WriteIOW
WriteIOW:

	@ R0:IOA, R1:IOD, R2:UCA, R3:CPSR, R4:TMP

	PUSH	{R2-R4}

	MOV		R2, #ST_MEM_BASE
	ORR		R2, r2, #ST_MEM_OFFS

	MRS		R3, CPSR			@ read interrupt enable status
	ORR		R4, R3, #I_Bit		@ clear interrupt enable bit
	MSR		CPSR_cxsf, R4		@ save and interrupt disable

	STR		R4, [R2]			@ before memory write
	LDR		R4, [R2]			@ before memory read

	STRH	R1, [R0]			@ write io data

	LDR		R4, [R2]			@ after memory read

	MSR		CPSR_cxsf, R3		@ restore CPSR

	POP		{R2-R4}

	BX		LR

@-------------------------------------------------------------------------------
@	void	WriteIOB( Addr, Data )
@-------------------------------------------------------------------------------

.globl	WriteIOB
WriteIOB:

	@ R0:IOA, R1:IOD, R2:UCA, R3:CPSR, R4:TMP

	PUSH	{R2-R4}

	MOV		R2, #ST_MEM_BASE
	ORR		R2, r2, #ST_MEM_OFFS

	MRS		R3, CPSR			@ read interrupt enable status
	ORR		R4, R3, #I_Bit		@ clear interrupt enable bit
	MSR		CPSR_cxsf, R4		@ save and interrupt disable

	STR		R4, [R2]			@ before memory write
	LDR		R4, [R2]			@ before memory read

	STRB	R1, [R0]			@ write io data

	LDR		R4, [R2]			@ after memory read

	MSR		CPSR_cxsf, R3		@ restore CPSR

	POP		{R2-R4}

	BX		LR

#if 0
/* Below code is for ARM926EJS and ARM1026EJS */
	.globl cleanDCache
cleanDCache:
	mrc	p15, 0, pc, c7, c10, 3	/* test/clean D-Cache */
	bne	cleanDCache
	mov	pc, lr

	.globl cleanFlushDCache
cleanFlushDCache:
	mrc	p15, 0, pc, c7, c14, 3	/* test/cleanflush D-Cache */
	bne	cleanFlushDCache
	mov	pc, lr

	.globl cleanFlushCache
cleanFlushCache:
	mrc	p15, 0, pc, c7, c14, 3	/* test/cleanflush D-Cache */
	bne	cleanFlushCache
	mcr	p15, 0, r0, c7, c5, 0	/* flush I-Cache */
	mov	pc, lr
#endif

	.ltorg

