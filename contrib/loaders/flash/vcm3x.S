/***************************************************************************
 *   Copyright (C) 2018 by Vertexcom Technologies. Inc                     *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m3
	.thumb
	.thumb_func

/*
 * To assemble:
 * arm-none-eabi-gcc -c vcm3x.S
 *
 * To disassemble:
 * arm-none-eabi-objdump -d vcm3x.o
 */
 
/*
 * Params :
 * r0 = workarea start
 * r1 = workarea end
 * r2 = flash address
 * r3 = byte count
 * r4 = flash base [0x40020000]
 *
 * Clobbered:
 * r5 - temp
 * r6 - temp
 * r7 - rp
 * r8 - wp, tmp
 */

#define VC_FLASH_PGADDR_OFFSET   0x48
#define VC_FLASH_PGDATA_OFFSET   0x4c
#define VC_FLASH_STS_OFFSET      0x88

wait_fifo:
	ldr 	r8, [r0, #0]	/* read wp */
	cmp 	r8, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r7, [r0, #4]	/* read rp */
	cmp 	r7, r8			/* wait until rp != wp */
	beq 	wait_fifo
    mov     r6, r2
    adds    r2, #0x4                            /* increment the address */
    str     r6, [r4, #VC_FLASH_PGADDR_OFFSET]
    ldr     r6, [r7], #0x04                     /* read one word from src, increment ptr */
    str     r6, [r4, #VC_FLASH_PGDATA_OFFSET]
busy:
    ldr     r6, [r4, #VC_FLASH_STS_OFFSET]
    mov     r5, #0x1
    cmp     r5, r6
    bne     busy
	cmp 	r7, r1			/* wrap rp at end of buffer */
    bcc     no_wrap
    mov     r7, r0
	adds	r7, #8		    /* skip loader args */
no_wrap:
	str 	r7, [r0, #4]	/* store rp */
	subs	r3, #4          /* decrement byte count */
	bne		wait_fifo
exit:
	bkpt	#0x00
