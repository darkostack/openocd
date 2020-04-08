/***************************************************************************
 *   Copyright (C) 2020 by Vertexcom Technologies. Inc                     *
 ***************************************************************************/

    .text
    .syntax unified
    .cpu cortex-m4
    .thumb
    .thumb_func

/*
 * To assemble:
 * arm-none-eabi-gcc -c vcm4x.S
 *
 * To disassemble:
 * arm-none-eabi-objdump -d vcm4x.o
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
 * r6 - temp
 * r7 - rp
 * r8 - wp, tmp
 */

#define FCSR_FLASH_CMD_OFFSET   0x000
#define FCSR_FLASH_ADDR_OFFSET  0x004
#define FCSR_FLASH_BUF0_OFFSET  0x100

wait_fifo:
    ldr     r8, [r0, #0]                        /* read wp */
    cmp     r8, #0                              /* abort if wp == 0 */
    beq     exit
    ldr     r7, [r0, #4]                        /* read rp */
    cmp     r7, r8                              /* wait until rp != wp */
    beq     wait_fifo
    ldr     r6,=0x81000006                      /* write enable */
    str     r6, [r4, #FCSR_FLASH_CMD_OFFSET]
wait_write_enable:
    ldr     r6, [r4, #FCSR_FLASH_CMD_OFFSET]    /* load CMD register */
    tst     r6, #0x80000000                     /* ACT (bit31) == 1 => operation in progress */
    bne     wait_write_enable                   /* wait more ... */
    ldr     r6, [r7], #0x4                      /* read one word from src, increment ptr */
    str     r6, [r4, #FCSR_FLASH_BUF0_OFFSET]
    mov     r6, r2
    adds    r2, #0x4                            /* increment the addres */
    str     r6, [r4, #FCSR_FLASH_ADDR_OFFSET]
    ldr     r6,=0x81030702
    str     r6, [r4, #FCSR_FLASH_CMD_OFFSET]
wait_write_program:
    ldr     r6, [r4, #FCSR_FLASH_CMD_OFFSET]    /* load CMD register */
    tst     r6, #0x80000000                     /* ACT (bit31) == 1 => operation in progress */
    bne     wait_write_program                  /* wait more ... */
    cmp     r7, r1                              /* wrap rp at end of buffer */
    bcc     no_wrap
    mov     r7, r0
    adds    r7, #8                              /* skip loader args */
no_wrap:
    str     r7, [r0, #4]                        /* store rp */
    subs    r3, #4                              /* decrement byte count */
    bne     wait_fifo
exit:
    bkpt    #0x00
