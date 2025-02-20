/*******************************************************************************
 * 
 *   [ c02.h ]
 *   Single file, minimalist 6502 emulator library.
 *   v1.0.0 - 2025/02/20
 * 
 *   AUTHOR: Emanuele Parlangeli
 * 
 *   LICENSE: MIT-0
 *   See the end of this file for the license.
 * 
 *******************************************************************************
 * 
 * ///// HOW TO USE ///////////////////
 * 
 *   Before using it, do the following in *one* C/C++ file:
 * 
 *      #define C02_IMPLEMENTATION
 *      #include "c02.h"
 * 
 *   to create the implementation.
 * 
 * ///// ABOUT ////////////////////////
 * 
 *   This library provides a very simple API to emulate a 6502 processor.
 *   Only supports legal opcodes.
 * 
 *   Tested with Klaus Dormann's functional test suite and Bruce Clark's
 *   decimal mode test program:
 *      - https://github.com/Klaus2m5/6502_65C02_functional_tests
 * 
 * ///// EXAMPLE //////////////////////
 * 
 *   Here's a little example that shows how to run Klaus Dormann's
 *   "6502_functional_test.bin" test program:
 * 
 *      #include <stdint.h>
 *      #include <stdio.h>
 *      #include <string.h>
 *      
 *      #define C02_IMPLEMENTATION
 *      #include "c02.h"
 *      
 *      #define ADDR_SUCCESS 0x3469
 *      const uint8_t program[] = { ... };
 *      
 *      uint8_t memory[C02_MEMORY_SIZE];
 *      
 *      int main(int argc, char *argv[])
 *      {
 *          C02Context ctx;
 *          c02_InitContext(&ctx, memory);
 *          
 *          // Load the program into memory, and set the PC to the start
 *          memcpy(memory, program, sizeof program);
 *          c02_SetPC(&ctx, 0x0400);
 *          
 *          // Run until we're stuck in place
 *          for (;;)
 *          {
 *              if (!c02_Step(&ctx))
 *              {
 *                  // Print registers, zero-page, stack, ...
 *                  return -1;
 *              }
 *              
 *              if (c02_DetectTrap(&ctx))
 *                  break;
 *          }
 *          
 *          uint16_t pc = c02_GetPC(&ctx);
 *          
 *          // Where are we stuck?
 *          if (pc == ADDR_SUCCESS)
 *              printf("SUCCESS!!");
 *          else
 *              printf("Trapped at 0x%04X", pc);
 *          
 *          printf("  total cycles : %ld\n", c02_GetTotalCycles(&ctx));
 *          return 0;
 *      }
 * 
 ******************************************************************************/

#ifndef _C02_H
#define _C02_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/* --- Emulator context --------------------------------------------------------
 * 
 *   A context encapsulates a single 6502 processor instance.
 *   All the functions in this library work with a context.
 *   You can create as many as you like!
 * 
 *   No memory allocation is performed, so you don't need to worry about
 *   cleanup.
 *
 *   c02_InitContext  : Initialize a context.
 *                      You must provide a contiguous array of 65536 bytes
 *                      to act as "memory".
 *                      Keep in mind that the stack range and the system
 *                      vectors are hard-coded to be from $0100 to $01FF and
 *                      from $FFFA to $FFFF, respectively.
 *                      All registers are cleared except the stack pointer,
 *                      which is set to $FF.
 * 
 *   c02_ResetContext : Reset a context.
 *                      Only the registers (and flags) are reset.
 * 
 * -------------------------------------------------------------------------- */

/**
 * The amount of memory needed for the full 16-bit address range.
 */
#define C02_MEMORY_SIZE (0xFFFF + 1)

/**
 * Opaque type.
 */
typedef struct C02Context C02Context;

void c02_InitContext  (C02Context *ctx, uint8_t memory[C02_MEMORY_SIZE]);
void c02_ResetContext (C02Context *ctx);


/* --- General usage -----------------------------------------------------------
 * 
 *   c02_Step             : Step forward one instruction.
 *                          When an illegal opcode is found, emulation is
 *                          halted and this function returns false.
 *                          You're free to mess around with the 
 *                          processor's state, otherwise you should
 *                          call c02_ResetContext() and restart your program.
 * 
 *   c02_GetElapsedCycles : Returns how many cycles the last successfully
 *                          executed instruction took.
 * 
 *   c02_GetTotalCycles   : Returns the total number of cycles that have
 *                          passed since the start.
 * 
 *   c02_TriggerNMI       : Simulate a Non-Maskable Interrupt.
 * 
 *   c02_TriggerRES       : Simulate a RESET signal.
 * 
 *   c02_TriggerIRQ       : Simulate an Interrupt Request.
 * 
 * -------------------------------------------------------------------------- */

bool     c02_Step             (C02Context *ctx);

int32_t  c02_GetElapsedCycles (C02Context *ctx);
uint64_t c02_GetTotalCycles   (C02Context *ctx);

void     c02_TriggerNMI       (C02Context *ctx);
void     c02_TriggerRES       (C02Context *ctx);
void     c02_TriggerIRQ       (C02Context *ctx);


/* --- Registers' getters & setters ----------------------------------------- */

/**
 * Status register flags.
 */
#define C02_FLAG_CARRY     (1 << 0)
#define C02_FLAG_ZERO      (1 << 1)
#define C02_FLAG_IQRDIS    (1 << 2)
#define C02_FLAG_DECMODE   (1 << 3)
#define C02_FLAG_OVERFLOW  (1 << 6)
#define C02_FLAG_NEGATIVE  (1 << 7)

void     c02_SetAccumulator    (C02Context *ctx, uint8_t  value);
void     c02_SetX              (C02Context *ctx, uint8_t  value);
void     c02_SetY              (C02Context *ctx, uint8_t  value);
void     c02_SetStatusRegister (C02Context *ctx, uint8_t  value);
void     c02_SetStackPointer   (C02Context *ctx, uint8_t  value);
void     c02_SetPC             (C02Context *ctx, uint16_t value);

uint8_t  c02_GetAccumulator    (C02Context *ctx);
uint8_t  c02_GetX              (C02Context *ctx);
uint8_t  c02_GetY              (C02Context *ctx);
uint8_t  c02_GetStatusRegister (C02Context *ctx);
uint8_t  c02_GetStackPointer   (C02Context *ctx);
uint16_t c02_GetPC             (C02Context *ctx);


/* --- Bonus debugging utilities -----------------------------------------------
 * 
 *   Here are some of the functions I created while testing this library.
 *   I've decided to include them. Perhaps, you may find them useful!
 * 
 *   c02_DetectTrap                   : Check if the program counter is stuck
 *                                      at the same address.
 * 
 *   c02_DisassembleInstruction       : Disassemble the instruction starting
 *                                      at [address] into a string of 12 chars
 *                                      (including the terminator character).
 * 
 *   c02_GetOpcodeMnemonic            : Get the assembler mnemonic of an
 *                                      [opcode] as a string of 4 chars
 *                                      (including the terminator character).
 * 
 *   c02_GetInstructionAddressingMode : Lookup the addressing mode of
 *                                      an instruction.
 * 
 *   c02_GetInstructionLength         : Lookup how many bytes long an
 *                                      instruction is.
 * 
 *   c02_GetInstructionCycleCount     : Lookup the base cycle cost of
 *                                      an instruction.
 *                                      Does not check for page crossing or
 *                                      branch evaluation, you should figure
 *                                      that out yourself.
 * 
 * -------------------------------------------------------------------------- */

/**
 * All 6502 addressing modes.
 */
typedef enum C02AddressingMode
{
    kC02AddrMode_Unknown = 0,
    
    kC02AddrMode_Accumulator,             // OPC A
    
    kC02AddrMode_Absolute,                // OPC $HHLL
    kC02AddrMode_Absolute_X_Indexed,      // OPC $HHLL,X
    kC02AddrMode_Absolute_Y_Indexed,      // OPC $HHLL,Y
    
    kC02AddrMode_Immediate,               // OPC #$BB
    
    kC02AddrMode_Implied,                 // OPC
    
    kC02AddrMode_Indirect,                // OPC ($HHLL)
    kC02AddrMode_Indirect_X_Indexed,      // OPC ($LL,X)
    kC02AddrMode_Indirect_Y_Indexed,      // OPC ($LL),Y
    
    kC02AddrMode_Relative,                // OPC $BB
    
    kC02AddrMode_Zeropage,                // OPC $LL
    kC02AddrMode_Zeropage_X_Indexed,      // OPC $LL,X
    kC02AddrMode_Zeropage_Y_Indexed,      // OPC $LL,Y
} C02AddressingMode;

bool              c02_DetectTrap                   (C02Context *ctx);

void              c02_DisassembleInstruction       (C02Context *ctx, uint16_t address, char outstr[12]);

void              c02_GetOpcodeMnemonic            (uint8_t opcode, char outstr[4]);
C02AddressingMode c02_GetInstructionAddressingMode (uint8_t opcode);
int32_t           c02_GetInstructionLength         (uint8_t opcode);
int32_t           c02_GetInstructionCycleCount     (uint8_t opcode);


#ifdef __cplusplus
}
#endif

#endif /* _C02_H */



/***********************************************************************************************************************
 * 
 *      IMPLEMENTATION
 * 
 **********************************************************************************************************************/

#ifdef C02_IMPLEMENTATION
#ifndef _C02_C
#define _C02_C

#include <stdio.h>    // sprintf
#include <string.h>   // memset, memcpy


/* --- Private definitions -------------------------------------------------- */

/**
 * Unofficial status register flags.
 */
#define C02__FLAG_BREAK     (1 << 4)
#define C02__FLAG_RESERVED  (1 << 5)

/**
 * System vectors.
 */
#define C02__NMI_VECTOR_LB  0xFFFA
#define C02__NMI_VECTOR_HB  0xFFFB
#define C02__RES_VECTOR_LB  0xFFFC
#define C02__RES_VECTOR_HB  0xFFFD
#define C02__IRQ_VECTOR_LB  0xFFFE
#define C02__IRQ_VECTOR_HB  0xFFFF

/**
 * Stack stuff.
 */
#define C02__STACK_PAGE     0x01
#define C02__STACK_TOP      0x00
#define C02__STACK_BOTTOM   0xFF

/**
 * The number of "unique" 6502 instructions.
 */
#define C02__NUM_INSTRUCTIONS 56

/**
 * Get the low byte of a 16-bit value.
 */
#define C02__LB(hhll) \
    ((uint8_t)((hhll) & 0xFF))

/**
 * Get the high byte of a 16-bit value.
 */
#define C02__HB(hhll) \
    ((uint8_t)(((hhll) >> 8) & 0xFF))

/**
 * Combine a low byte and a high byte into a 16-bit address.
 */
#define C02__MKADDR(hb, lb) \
    ((uint16_t)(((hb) << 8) | (lb)))

/**
 * Push something into the stack.
 */
#define C02__STACK_PUSH(val)                                                \
    do                                                                      \
    {                                                                       \
        ctx->memory[ C02__MKADDR(C02__STACK_PAGE, ctx->reg.SP) ] = val;     \
        ctx->reg.SP--;                                                      \
    } while (0)

/**
 * Pop the stack.
 */
#define C02__STACK_POP(dest)                                                \
    do                                                                      \
    {                                                                       \
        ctx->reg.SP++;                                                      \
        dest = ctx->memory[ C02__MKADDR(C02__STACK_PAGE, ctx->reg.SP) ];    \
    } while (0)

/**
 * If (expr) is true, set (flag) in the status register, otherwise clear it.
 */
#define C02__TEST_FOR_FLAG(flag, expr)          \
    do                                          \
    {                                           \
        if ((expr)) ctx->reg.SR |=  (flag);     \
        else        ctx->reg.SR &= ~(flag);     \
    } while (0)


/* --- Private types -------------------------------------------------------- */

/**
 * Represents the "implementation" of an instruction.
 */
typedef void (*C02InstructionHandler)(C02Context *ctx);

typedef struct C02Context
{
    struct
    {
        uint8_t  A, X, Y, SR, SP;
        uint16_t prev_PC, PC;
    } reg;
    
    uint8_t *memory;
    
    struct
    {
        uint8_t opcode;
        uint8_t handler_idx;
        uint8_t addressing_mode;
        uint8_t cycles_needed;
        uint8_t length;
    } curr_instruction;
    
    int32_t  elapsed_cycles;
    uint64_t total_cycles;
} C02Context;


/* --- Private functions declaration ---------------------------------------- */

/**
 * Evaluate the operands of an instruction based on its addressing mode,
 * and return the final address.
 */
uint16_t c02__ParseOperands(C02Context *ctx);

/**
 * All instruction handlers.
 */
void c02__ADC(C02Context *ctx);
void c02__AND(C02Context *ctx);
void c02__ASL(C02Context *ctx);
void c02__BCC(C02Context *ctx);
void c02__BCS(C02Context *ctx);
void c02__BEQ(C02Context *ctx);
void c02__BIT(C02Context *ctx);
void c02__BMI(C02Context *ctx);
void c02__BNE(C02Context *ctx);
void c02__BPL(C02Context *ctx);
void c02__BRK(C02Context *ctx);
void c02__BVC(C02Context *ctx);
void c02__BVS(C02Context *ctx);
void c02__CLC(C02Context *ctx);
void c02__CLD(C02Context *ctx);
void c02__CLI(C02Context *ctx);
void c02__CLV(C02Context *ctx);
void c02__CMP(C02Context *ctx);
void c02__CPX(C02Context *ctx);
void c02__CPY(C02Context *ctx);
void c02__DEC(C02Context *ctx);
void c02__DEX(C02Context *ctx);
void c02__DEY(C02Context *ctx);
void c02__EOR(C02Context *ctx);
void c02__INC(C02Context *ctx);
void c02__INX(C02Context *ctx);
void c02__INY(C02Context *ctx);
void c02__JMP(C02Context *ctx);
void c02__JSR(C02Context *ctx);
void c02__LDA(C02Context *ctx);
void c02__LDX(C02Context *ctx);
void c02__LDY(C02Context *ctx);
void c02__LSR(C02Context *ctx);
void c02__NOP(C02Context *ctx);
void c02__ORA(C02Context *ctx);
void c02__PHA(C02Context *ctx);
void c02__PHP(C02Context *ctx);
void c02__PLA(C02Context *ctx);
void c02__PLP(C02Context *ctx);
void c02__ROL(C02Context *ctx);
void c02__ROR(C02Context *ctx);
void c02__RTI(C02Context *ctx);
void c02__RTS(C02Context *ctx);
void c02__SBC(C02Context *ctx);
void c02__SEC(C02Context *ctx);
void c02__SED(C02Context *ctx);
void c02__SEI(C02Context *ctx);
void c02__STA(C02Context *ctx);
void c02__STX(C02Context *ctx);
void c02__STY(C02Context *ctx);
void c02__TAX(C02Context *ctx);
void c02__TAY(C02Context *ctx);
void c02__TSX(C02Context *ctx);
void c02__TXA(C02Context *ctx);
void c02__TXS(C02Context *ctx);
void c02__TYA(C02Context *ctx);


/* --- Private global variables --------------------------------------------- */

/**
 * Table of instruction handlers.
 * 
 * Each opcode references its "implementation" function through its index
 * in this table.
 */
const C02InstructionHandler g_instruction_handler_table[C02__NUM_INSTRUCTIONS] = {
    c02__ADC, c02__AND, c02__ASL, c02__BCC, c02__BCS, c02__BEQ, c02__BIT,
    c02__BMI, c02__BNE, c02__BPL, c02__BRK, c02__BVC, c02__BVS, c02__CLC,
    c02__CLD, c02__CLI, c02__CLV, c02__CMP, c02__CPX, c02__CPY, c02__DEC,
    c02__DEX, c02__DEY, c02__EOR, c02__INC, c02__INX, c02__INY, c02__JMP,
    c02__JSR, c02__LDA, c02__LDX, c02__LDY, c02__LSR, c02__NOP, c02__ORA,
    c02__PHA, c02__PHP, c02__PLA, c02__PLP, c02__ROL, c02__ROR, c02__RTI,
    c02__RTS, c02__SBC, c02__SEC, c02__SED, c02__SEI, c02__STA, c02__STX,
    c02__STY, c02__TAX, c02__TAY, c02__TSX, c02__TXA, c02__TXS, c02__TYA
};

/**
 * Opcode lookup table.
 * 
 * For every opcode, it defines:
 *    - what function to call as an index into the previous table (handler_idx)
 *    - the addressing mode of the instruction (addressing_mode)
 *    - how many bytes long it is (length)
 *    - the base cycle cost required to execute it (cycles_needed)
 */
const uint16_t g_opcode_lookup[256] = {
    
// Opcode information is packed into 16-bits like this:
#define LEGAL(handler_idx, addressing_mode, length, cycles_needed) \
    ( (length << 14) | (handler_idx << 8) | (addressing_mode << 4) | cycles_needed )

// Placeholder with a fake handler index for all undocumented opcodes
#define ILLEGAL  LEGAL(63, 0, 0, 0)
    
    /* $00: BRK         */ LEGAL( 10, kC02AddrMode_Implied,            1, 7 ),
    /* $01: ORA ($LL,X) */ LEGAL( 34, kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $02: ---         */ ILLEGAL,
    /* $03: ---         */ ILLEGAL,
    /* $04: ---         */ ILLEGAL,
    /* $05: ORA $LL     */ LEGAL( 34, kC02AddrMode_Zeropage,           2, 3 ),
    /* $06: ASL $LL     */ LEGAL( 2,  kC02AddrMode_Zeropage,           2, 5 ),
    /* $07: ---         */ ILLEGAL,
    /* $08: PHP         */ LEGAL( 36, kC02AddrMode_Implied,            1, 3 ),
    /* $09: ORA #$BB    */ LEGAL( 34, kC02AddrMode_Immediate,          2, 2 ),
    /* $0A: ASL A       */ LEGAL( 2,  kC02AddrMode_Accumulator,        1, 2 ),
    /* $0B: ---         */ ILLEGAL,
    /* $0C: ---         */ ILLEGAL,
    /* $0D: ORA $HHLL   */ LEGAL( 34, kC02AddrMode_Absolute,           3, 4 ),
    /* $0E: ASL $HHLL   */ LEGAL( 2,  kC02AddrMode_Absolute,           3, 6 ),
    /* $0F: ---         */ ILLEGAL,
    
    /* $10: BPL $BB     */ LEGAL( 9,  kC02AddrMode_Relative,           2, 2 ),
    /* $11: ORA ($LL),Y */ LEGAL( 34, kC02AddrMode_Indirect_Y_Indexed, 2, 5 ),
    /* $12: ---         */ ILLEGAL,
    /* $13: ---         */ ILLEGAL,
    /* $14: ---         */ ILLEGAL,
    /* $15: ORA $LL,X   */ LEGAL( 34, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $16: ASL $LL,X   */ LEGAL( 2,  kC02AddrMode_Zeropage_X_Indexed, 2, 6 ),
    /* $17: ---         */ ILLEGAL,
    /* $18: CLC         */ LEGAL( 13, kC02AddrMode_Implied,            1, 2 ),
    /* $19: ORA $HHLL,Y */ LEGAL( 34, kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $1A: ---         */ ILLEGAL,
    /* $1B: ---         */ ILLEGAL,
    /* $1C: ---         */ ILLEGAL,
    /* $1D: ORA $HHLL,X */ LEGAL( 34, kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $1E: ASL $HHLL,X */ LEGAL( 2,  kC02AddrMode_Absolute_X_Indexed, 3, 7 ),
    /* $1F: ---         */ ILLEGAL,
    
    /* $20: JSR $HHLL   */ LEGAL( 28, kC02AddrMode_Absolute,           3, 6 ),
    /* $21: AND ($LL,X) */ LEGAL( 1,  kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $22: ---         */ ILLEGAL,
    /* $23: ---         */ ILLEGAL,
    /* $24: BIT $LL     */ LEGAL( 6,  kC02AddrMode_Zeropage,           2, 3 ),
    /* $25: AND $LL     */ LEGAL( 1,  kC02AddrMode_Zeropage,           2, 3 ),
    /* $26: ROL $LL     */ LEGAL( 39, kC02AddrMode_Zeropage,           2, 5 ),
    /* $27: ---         */ ILLEGAL,
    /* $28: PLP         */ LEGAL( 38, kC02AddrMode_Implied,            1, 4 ),
    /* $29: AND #$BB    */ LEGAL( 1,  kC02AddrMode_Immediate,          2, 2 ),
    /* $2A: ROL A       */ LEGAL( 39, kC02AddrMode_Accumulator,        1, 2 ),
    /* $2B: ---         */ ILLEGAL,
    /* $2C: BIT $HHLL   */ LEGAL( 6,  kC02AddrMode_Absolute,           3, 4 ),
    /* $2D: AND $HHLL   */ LEGAL( 1,  kC02AddrMode_Absolute,           3, 4 ),
    /* $2E: ROL $HHLL   */ LEGAL( 39, kC02AddrMode_Absolute,           3, 6 ),
    /* $2F: ---         */ ILLEGAL,
    
    /* $30: BMI $BB     */ LEGAL( 7,  kC02AddrMode_Relative,           2, 2 ),
    /* $31: AND ($LL),Y */ LEGAL( 1,  kC02AddrMode_Indirect_Y_Indexed, 2, 5 ),
    /* $32: ---         */ ILLEGAL,
    /* $33: ---         */ ILLEGAL,
    /* $34: ---         */ ILLEGAL,
    /* $35: AND $LL,X   */ LEGAL( 1,  kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $36: ROL $LL,X   */ LEGAL( 39, kC02AddrMode_Zeropage_X_Indexed, 2, 6 ),
    /* $37: ---         */ ILLEGAL,
    /* $38: SEC         */ LEGAL( 44, kC02AddrMode_Implied,            1, 2 ),
    /* $39: AND $HHLL,Y */ LEGAL( 1,  kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $3A: ---         */ ILLEGAL,
    /* $3B: ---         */ ILLEGAL,
    /* $3C: ---         */ ILLEGAL,
    /* $3D: AND $HHLL,X */ LEGAL( 1,  kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $3E: ROL $HHLL,X */ LEGAL( 39, kC02AddrMode_Absolute_X_Indexed, 3, 7 ),
    /* $3F: ---         */ ILLEGAL,
    
    /* $40: RTI         */ LEGAL( 41, kC02AddrMode_Implied,            1, 6 ),
    /* $41: EOR ($LL,X) */ LEGAL( 23, kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $42: ---         */ ILLEGAL,
    /* $43: ---         */ ILLEGAL,
    /* $44: ---         */ ILLEGAL,
    /* $45: EOR $LL     */ LEGAL( 23, kC02AddrMode_Zeropage,           2, 3 ),
    /* $46: LSR $LL     */ LEGAL( 32, kC02AddrMode_Zeropage,           2, 5 ),
    /* $47: ---         */ ILLEGAL,
    /* $48: PHA         */ LEGAL( 35, kC02AddrMode_Implied,            1, 3 ),
    /* $49: EOR #$BB    */ LEGAL( 23, kC02AddrMode_Immediate,          2, 2 ),
    /* $4A: LSR A       */ LEGAL( 32, kC02AddrMode_Accumulator,        1, 2 ),
    /* $4B: ---         */ ILLEGAL,
    /* $4C: JMP $HHLL   */ LEGAL( 27, kC02AddrMode_Absolute,           3, 3 ),
    /* $4D: EOR $HHLL   */ LEGAL( 23, kC02AddrMode_Absolute,           3, 4 ),
    /* $4E: LSR $HHLL   */ LEGAL( 32, kC02AddrMode_Absolute,           3, 6 ),
    /* $4F: ---         */ ILLEGAL,
    
    /* $50: BVC $BB     */ LEGAL( 11, kC02AddrMode_Relative,           2, 2 ),
    /* $51: EOR ($LL),Y */ LEGAL( 23, kC02AddrMode_Indirect_Y_Indexed, 2, 5 ),
    /* $52: ---         */ ILLEGAL,
    /* $53: ---         */ ILLEGAL,
    /* $54: ---         */ ILLEGAL,
    /* $55: EOR $LL,X   */ LEGAL( 23, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $56: LSR $LL,X   */ LEGAL( 32, kC02AddrMode_Zeropage_X_Indexed, 2, 6 ),
    /* $57: ---         */ ILLEGAL,
    /* $58: CLI         */ LEGAL( 15, kC02AddrMode_Implied,            1, 2 ),
    /* $59: EOR $HHLL,Y */ LEGAL( 23, kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $5A: ---         */ ILLEGAL,
    /* $5B: ---         */ ILLEGAL,
    /* $5C: ---         */ ILLEGAL,
    /* $5D: EOR $HHLL,X */ LEGAL( 23, kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $5E: LSR $HHLL,X */ LEGAL( 32, kC02AddrMode_Absolute_X_Indexed, 3, 7 ),
    /* $5F: ---         */ ILLEGAL,
    
    /* $60: RTS         */ LEGAL( 42, kC02AddrMode_Implied,            1, 6 ),
    /* $61: ADC ($LL,X) */ LEGAL( 0,  kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $62: ---         */ ILLEGAL,
    /* $63: ---         */ ILLEGAL,
    /* $64: ---         */ ILLEGAL,
    /* $65: ADC $LL     */ LEGAL( 0,  kC02AddrMode_Zeropage,           2, 3 ),
    /* $66: ROR $LL     */ LEGAL( 40, kC02AddrMode_Zeropage,           2, 5 ),
    /* $67: ---         */ ILLEGAL,
    /* $68: PLA         */ LEGAL( 37, kC02AddrMode_Implied,            1, 4 ),
    /* $69: ADC #$BB    */ LEGAL( 0,  kC02AddrMode_Immediate,          2, 2 ),
    /* $6A: ROR A       */ LEGAL( 40, kC02AddrMode_Accumulator,        1, 2 ),
    /* $6B: ---         */ ILLEGAL,
    /* $6C: JMP ($HHLL) */ LEGAL( 27, kC02AddrMode_Indirect,           3, 5 ),
    /* $6D: ADC $HHLL   */ LEGAL( 0,  kC02AddrMode_Absolute,           3, 4 ),
    /* $6E: ROR $HHLL   */ LEGAL( 40, kC02AddrMode_Absolute,           3, 6 ),
    /* $6F: ---         */ ILLEGAL,
    
    /* $70: BVS $BB     */ LEGAL( 12, kC02AddrMode_Relative,           2, 2 ),
    /* $71: ADC ($LL),Y */ LEGAL( 0,  kC02AddrMode_Indirect_Y_Indexed, 2, 5 ),
    /* $72: ---         */ ILLEGAL,
    /* $73: ---         */ ILLEGAL,
    /* $74: ---         */ ILLEGAL,
    /* $75: ADC $LL,X   */ LEGAL( 0,  kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $76: ROR $LL,X   */ LEGAL( 40, kC02AddrMode_Zeropage_X_Indexed, 2, 6 ),
    /* $77: ---         */ ILLEGAL,
    /* $78: SEI         */ LEGAL( 46, kC02AddrMode_Implied,            1, 2 ),
    /* $79: ADC $HHLL,Y */ LEGAL( 0,  kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $7A: ---         */ ILLEGAL,
    /* $7B: ---         */ ILLEGAL,
    /* $7C: ---         */ ILLEGAL,
    /* $7D: ADC $HHLL,X */ LEGAL( 0,  kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $7E: ROR $HHLL,X */ LEGAL( 40, kC02AddrMode_Absolute_X_Indexed, 3, 7 ),
    /* $7F: ---         */ ILLEGAL,
    
    /* $80: ---         */ ILLEGAL,
    /* $81: STA ($LL,X) */ LEGAL( 47, kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $82: ---         */ ILLEGAL,
    /* $83: ---         */ ILLEGAL,
    /* $84: STY $LL     */ LEGAL( 49, kC02AddrMode_Zeropage,           2, 3 ),
    /* $85: STA $LL     */ LEGAL( 47, kC02AddrMode_Zeropage,           2, 3 ),
    /* $86: STX $LL     */ LEGAL( 48, kC02AddrMode_Zeropage,           2, 3 ),
    /* $87: ---         */ ILLEGAL,
    /* $88: DEY         */ LEGAL( 22, kC02AddrMode_Implied,            1, 2 ),
    /* $89: ---         */ ILLEGAL,
    /* $8A: TXA         */ LEGAL( 53, kC02AddrMode_Implied,            1, 2 ),
    /* $8B: ---         */ ILLEGAL,
    /* $8C: STY $HHLL   */ LEGAL( 49, kC02AddrMode_Absolute,           3, 4 ),
    /* $8D: STA $HHLL   */ LEGAL( 47, kC02AddrMode_Absolute,           3, 4 ),
    /* $8E: STX $HHLL   */ LEGAL( 48, kC02AddrMode_Absolute,           3, 4 ),
    /* $8F: ---         */ ILLEGAL,
    
    /* $90: BCC $BB     */ LEGAL( 3,  kC02AddrMode_Relative,           2, 2 ),
    /* $91: STA ($LL),Y */ LEGAL( 47, kC02AddrMode_Indirect_Y_Indexed, 2, 6 ),
    /* $92: ---         */ ILLEGAL,
    /* $93: ---         */ ILLEGAL,
    /* $94: STY $LL,X   */ LEGAL( 49, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $95: STA $LL,X   */ LEGAL( 47, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $96: STX $LL,Y   */ LEGAL( 48, kC02AddrMode_Zeropage_Y_Indexed, 2, 4 ),
    /* $97: ---         */ ILLEGAL,
    /* $98: TYA         */ LEGAL( 55, kC02AddrMode_Implied,            1, 2 ),
    /* $99: STA $HHLL,Y */ LEGAL( 47, kC02AddrMode_Absolute_Y_Indexed, 3, 5 ),
    /* $9A: TXS         */ LEGAL( 54, kC02AddrMode_Implied,            1, 2 ),
    /* $9B: ---         */ ILLEGAL,
    /* $9C: ---         */ ILLEGAL,
    /* $9D: STA $HHLL,X */ LEGAL( 47, kC02AddrMode_Absolute_X_Indexed, 3, 5 ),
    /* $9E: ---         */ ILLEGAL,
    /* $9F: ---         */ ILLEGAL,
    
    /* $A0: LDY #$BB    */ LEGAL( 31, kC02AddrMode_Immediate,          2, 2 ),
    /* $A1: LDA ($LL,X) */ LEGAL( 29, kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $A2: LDX #$BB    */ LEGAL( 30, kC02AddrMode_Immediate,          2, 2 ),
    /* $A3: ---         */ ILLEGAL,
    /* $A4: LDY $LL     */ LEGAL( 31, kC02AddrMode_Zeropage,           2, 3 ),
    /* $A5: LDA $LL     */ LEGAL( 29, kC02AddrMode_Zeropage,           2, 3 ),
    /* $A6: LDX $LL     */ LEGAL( 30, kC02AddrMode_Zeropage,           2, 3 ),
    /* $A7: ---         */ ILLEGAL,
    /* $A8: TAY         */ LEGAL( 51, kC02AddrMode_Implied,            1, 2 ),
    /* $A9: LDA #$BB    */ LEGAL( 29, kC02AddrMode_Immediate,          2, 2 ),
    /* $AA: TAX         */ LEGAL( 50, kC02AddrMode_Implied,            1, 2 ),
    /* $AB: ---         */ ILLEGAL,
    /* $AC: LDY $HHLL   */ LEGAL( 31, kC02AddrMode_Absolute,           3, 4 ),
    /* $AD: LDA $HHLL   */ LEGAL( 29, kC02AddrMode_Absolute,           3, 4 ),
    /* $AE: LDX $HHLL   */ LEGAL( 30, kC02AddrMode_Absolute,           3, 4 ),
    /* $AF: ---         */ ILLEGAL,
    
    /* $B0: BCS $BB     */ LEGAL( 4,  kC02AddrMode_Relative,           2, 2 ),
    /* $B1: LDA ($LL),Y */ LEGAL( 29, kC02AddrMode_Indirect_Y_Indexed, 2, 5 ),
    /* $B2: ---         */ ILLEGAL,
    /* $B3: ---         */ ILLEGAL,
    /* $B4: LDY $LL,X   */ LEGAL( 31, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $B5: LDA $LL,X   */ LEGAL( 29, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $B6: LDX $LL,Y   */ LEGAL( 30, kC02AddrMode_Zeropage_Y_Indexed, 2, 4 ),
    /* $B7: ---         */ ILLEGAL,
    /* $B8: CLV         */ LEGAL( 16, kC02AddrMode_Implied,            1, 2 ),
    /* $B9: LDA $HHLL,Y */ LEGAL( 29, kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $BA: TSX         */ LEGAL( 52, kC02AddrMode_Implied,            1, 2 ),
    /* $BB: ---         */ ILLEGAL,
    /* $BC: LDY $HHLL,X */ LEGAL( 31, kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $BD: LDA $HHLL,X */ LEGAL( 29, kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $BE: LDX $HHLL,Y */ LEGAL( 30, kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $BF: ---         */ ILLEGAL,
    
    /* $C0: CPY #$BB    */ LEGAL( 19, kC02AddrMode_Immediate,          2, 2 ),
    /* $C1: CMP ($LL,X) */ LEGAL( 17, kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $C2: ---         */ ILLEGAL,
    /* $C3: ---         */ ILLEGAL,
    /* $C4: CPY $LL     */ LEGAL( 19, kC02AddrMode_Zeropage,           2, 3 ),
    /* $C5: CMP $LL     */ LEGAL( 17, kC02AddrMode_Zeropage,           2, 3 ),
    /* $C6: DEC $LL     */ LEGAL( 20, kC02AddrMode_Zeropage,           2, 5 ),
    /* $C7: ---         */ ILLEGAL,
    /* $C8: INY         */ LEGAL( 26, kC02AddrMode_Implied,            1, 2 ),
    /* $C9: CMP #$BB    */ LEGAL( 17, kC02AddrMode_Immediate,          2, 2 ),
    /* $CA: DEX         */ LEGAL( 21, kC02AddrMode_Implied,            1, 2 ),
    /* $CB: ---         */ ILLEGAL,
    /* $CC: CPY $HHLL   */ LEGAL( 19, kC02AddrMode_Absolute,           3, 4 ),
    /* $CD: CMP $HHLL   */ LEGAL( 17, kC02AddrMode_Absolute,           3, 4 ),
    /* $CE: DEC $HHLL   */ LEGAL( 20, kC02AddrMode_Absolute,           3, 6 ),
    /* $CF: ---         */ ILLEGAL,
    
    /* $D0: BNE $BB     */ LEGAL( 8,  kC02AddrMode_Relative,           2, 2 ),
    /* $D1: CMP ($LL),Y */ LEGAL( 17, kC02AddrMode_Indirect_Y_Indexed, 2, 5 ),
    /* $D2: ---         */ ILLEGAL,
    /* $D3: ---         */ ILLEGAL,
    /* $D4: ---         */ ILLEGAL,
    /* $D5: CMP $LL,X   */ LEGAL( 17, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $D6: DEC $LL,X   */ LEGAL( 20, kC02AddrMode_Zeropage_X_Indexed, 2, 6 ),
    /* $D7: ---         */ ILLEGAL,
    /* $D8: CLD         */ LEGAL( 14, kC02AddrMode_Implied,            1, 2 ),
    /* $D9: CMP $HHLL,Y */ LEGAL( 17, kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $DA: ---         */ ILLEGAL,
    /* $DB: ---         */ ILLEGAL,
    /* $DC: ---         */ ILLEGAL,
    /* $DD: CMP $HHLL,X */ LEGAL( 17, kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $DE: DEC $HHLL,X */ LEGAL( 20, kC02AddrMode_Absolute_X_Indexed, 3, 7 ),
    /* $DF: ---         */ ILLEGAL,
    
    /* $E0: CPX #$BB    */ LEGAL( 18, kC02AddrMode_Immediate,          2, 2 ),
    /* $E1: SBC ($LL,X) */ LEGAL( 43, kC02AddrMode_Indirect_X_Indexed, 2, 6 ),
    /* $E2: ---         */ ILLEGAL,
    /* $E3: ---         */ ILLEGAL,
    /* $E4: CPX $LL     */ LEGAL( 18, kC02AddrMode_Zeropage,           2, 3 ),
    /* $E5: SBC $LL     */ LEGAL( 43, kC02AddrMode_Zeropage,           2, 3 ),
    /* $E6: INC $LL     */ LEGAL( 24, kC02AddrMode_Zeropage,           2, 5 ),
    /* $E7: ---         */ ILLEGAL,
    /* $E8: INX         */ LEGAL( 25, kC02AddrMode_Implied,            1, 2 ),
    /* $E9: SBC #$BB    */ LEGAL( 43, kC02AddrMode_Immediate,          2, 2 ),
    /* $EA: NOP         */ LEGAL( 33, kC02AddrMode_Implied,            1, 2 ),
    /* $EB: ---         */ ILLEGAL,
    /* $EC: CPX $HHLL   */ LEGAL( 18, kC02AddrMode_Absolute,           3, 4 ),
    /* $ED: SBC $HHLL   */ LEGAL( 43, kC02AddrMode_Absolute,           3, 4 ),
    /* $EE: INC $HHLL   */ LEGAL( 24, kC02AddrMode_Absolute,           3, 6 ),
    /* $EF: ---         */ ILLEGAL,
    
    /* $F0: BEQ $BB     */ LEGAL( 5,  kC02AddrMode_Relative,           2, 2 ),
    /* $F1: SBC ($LL),Y */ LEGAL( 43, kC02AddrMode_Indirect_Y_Indexed, 2, 5 ),
    /* $F2: ---         */ ILLEGAL,
    /* $F3: ---         */ ILLEGAL,
    /* $F4: ---         */ ILLEGAL,
    /* $F5: SBC $LL,X   */ LEGAL( 43, kC02AddrMode_Zeropage_X_Indexed, 2, 4 ),
    /* $F6: INC $LL,X   */ LEGAL( 24, kC02AddrMode_Zeropage_X_Indexed, 2, 6 ),
    /* $F7: ---         */ ILLEGAL,
    /* $F8: SED         */ LEGAL( 45, kC02AddrMode_Implied,            1, 2 ),
    /* $F9: SBC $HHLL,Y */ LEGAL( 43, kC02AddrMode_Absolute_Y_Indexed, 3, 4 ),
    /* $FA: ---         */ ILLEGAL,
    /* $FB: ---         */ ILLEGAL,
    /* $FC: ---         */ ILLEGAL,
    /* $FD: SBC $HHLL,X */ LEGAL( 43, kC02AddrMode_Absolute_X_Indexed, 3, 4 ),
    /* $FE: INC $HHLL,X */ LEGAL( 24, kC02AddrMode_Absolute_X_Indexed, 3, 7 ),
    /* $FF: ---         */ ILLEGAL

#undef LEGAL
#undef ILLEGAL

};

/**
 * Assembler mnemonics for all instructions.
 */
const char g_opcode_mnemonics[][4] = {
    "ADC", "AND", "ASL", "BCC", "BCS", "BEQ", "BIT",
    "BMI", "BNE", "BPL", "BRK", "BVC", "BVS", "CLC",
    "CLD", "CLI", "CLV", "CMP", "CPX", "CPY", "DEC",
    "DEX", "DEY", "EOR", "INC", "INX", "INY", "JMP",
    "JSR", "LDA", "LDX", "LDY", "LSR", "NOP", "ORA",
    "PHA", "PHP", "PLA", "PLP", "ROL", "ROR", "RTI",
    "RTS", "SBC", "SEC", "SED", "SEI", "STA", "STX",
    "STY", "TAX", "TAY", "TSX", "TXA", "TXS", "TYA"
};


/* --- Public API implementation -------------------------------------------- */

void c02_InitContext(C02Context *ctx, uint8_t memory[C02_MEMORY_SIZE])
{
    ctx->memory = memory;
    c02_ResetContext(ctx);
}

void c02_ResetContext(C02Context *ctx)
{
    ctx->reg.A  = 0;
    ctx->reg.X  = 0;
    ctx->reg.Y  = 0;
    ctx->reg.SR = 0;
    ctx->reg.SP = C02__STACK_BOTTOM;
    ctx->reg.PC = 0;
    
    ctx->reg.prev_PC = 0;
    
    ctx->elapsed_cycles = 0;
    ctx->total_cycles   = 0;
}

bool c02_Step(C02Context *ctx)
{
    // Read the next instruction opcode
    uint8_t opc = ctx->memory[ctx->reg.PC];
    
    // Lookup all the necessary information about it
    ctx->curr_instruction.opcode          = opc;
    ctx->curr_instruction.handler_idx     = (g_opcode_lookup[opc] >>  8) & 0x3F;
    ctx->curr_instruction.addressing_mode = (g_opcode_lookup[opc] >>  4) & 0x0F;
    ctx->curr_instruction.cycles_needed   =  g_opcode_lookup[opc]        & 0x0F;  // Will be incremented later if necessary
    ctx->curr_instruction.length          = (g_opcode_lookup[opc] >> 14) & 0x03;
    
    // Catch illegal opcodes
    if (ctx->curr_instruction.handler_idx >= C02__NUM_INSTRUCTIONS)
    {
        ctx->elapsed_cycles = 0;
        return false;
    }
    
    ctx->reg.prev_PC = ctx->reg.PC;
    
    // Execute the function associated with this instruction
    g_instruction_handler_table[ ctx->curr_instruction.handler_idx ](ctx);
    
    ctx->elapsed_cycles = ctx->curr_instruction.cycles_needed;
    ctx->total_cycles  += ctx->elapsed_cycles;
    return true;
}

int32_t c02_GetElapsedCycles(C02Context *ctx)
{
    return ctx->elapsed_cycles;
}

uint64_t c02_GetTotalCycles(C02Context *ctx)
{
    return ctx->total_cycles;
}

void c02_TriggerNMI(C02Context *ctx)
{
    // Push the program counter and the status register to the stack
    C02__STACK_PUSH( C02__HB(ctx->reg.PC) );
    C02__STACK_PUSH( C02__LB(ctx->reg.PC) );
    
    // When pushed, set bit 5 and clear the "break" flag
    ctx->reg.SR |= C02__FLAG_RESERVED;
    ctx->reg.SR &= ~C02__FLAG_BREAK;
    C02__STACK_PUSH(ctx->reg.SR);
    
    // Disable interrupts
    ctx->reg.SR |= C02_FLAG_IQRDIS;
    
    // Read NMI vector
    uint8_t lb = ctx->memory[C02__NMI_VECTOR_LB];
    uint8_t hb = ctx->memory[C02__NMI_VECTOR_HB];
    
    // Jump to that address
    ctx->reg.PC = C02__MKADDR(hb, lb);
}

void c02_TriggerRES(C02Context *ctx)
{
    // Read RESET vector
    uint8_t lb = ctx->memory[C02__RES_VECTOR_LB];
    uint8_t hb = ctx->memory[C02__RES_VECTOR_HB];
    
    // Jump to that address
    ctx->reg.PC = C02__MKADDR(hb, lb);
}

void c02_TriggerIRQ(C02Context *ctx)
{
    // Don't do anything if the "interrupt disable" flag is set
    if (ctx->reg.SR & C02_FLAG_IQRDIS)
        return;
    
    // Push the program counter and the status register to the stack
    C02__STACK_PUSH( C02__HB(ctx->reg.PC) );
    C02__STACK_PUSH( C02__LB(ctx->reg.PC) );
    
    // When pushed, set bit 5 and clear the "break" flag
    ctx->reg.SR |= C02__FLAG_RESERVED;
    ctx->reg.SR &= ~C02__FLAG_BREAK;
    C02__STACK_PUSH(ctx->reg.SR);
    
    // Disable interrupts
    ctx->reg.SR |= C02_FLAG_IQRDIS;
    
    // Read IRQ vector
    uint8_t lb = ctx->memory[C02__IRQ_VECTOR_LB];
    uint8_t hb = ctx->memory[C02__IRQ_VECTOR_HB];
    
    // Jump to that address
    ctx->reg.PC = C02__MKADDR(hb, lb);
}

void c02_SetAccumulator(C02Context *ctx, uint8_t value)
{
    ctx->reg.A = value;
}

void c02_SetX(C02Context *ctx, uint8_t value)
{
    ctx->reg.X = value;
}

void c02_SetY(C02Context *ctx, uint8_t value)
{
    ctx->reg.Y = value;
}

void c02_SetStatusRegister(C02Context *ctx, uint8_t value)
{
    ctx->reg.SR = value & 0xCF;
}

void c02_SetStackPointer(C02Context *ctx, uint8_t value)
{
    ctx->reg.SP = value;
}

void c02_SetPC(C02Context *ctx, uint16_t value)
{
    ctx->reg.PC = value;
}

uint8_t c02_GetAccumulator(C02Context *ctx)
{
    return ctx->reg.A;
}

uint8_t c02_GetX(C02Context *ctx)
{
    return ctx->reg.X;
}

uint8_t c02_GetY(C02Context *ctx)
{
    return ctx->reg.Y;
}

uint8_t c02_GetStatusRegister(C02Context *ctx)
{
    return ctx->reg.SR & 0xCF;
}

uint8_t c02_GetStackPointer(C02Context *ctx)
{
    return ctx->reg.SP;
}

uint16_t c02_GetPC(C02Context *ctx)
{
    return ctx->reg.PC;
}

bool c02_DetectTrap(C02Context *ctx)
{
    return ctx->reg.prev_PC == ctx->reg.PC;
}

void c02_DisassembleInstruction(C02Context *ctx, uint16_t address, char outstr[12])
{
    memset(outstr, 0, 12);
    
    uint8_t opcode = ctx->memory[address];
    c02_GetOpcodeMnemonic(opcode, outstr);
    
    uint16_t PC = ctx->reg.PC;
    
    uint8_t addrmode = (g_opcode_lookup[opcode] >> 4) & 0x0F;
    switch (addrmode)
    {
        default:
        case kC02AddrMode_Unknown:
        case kC02AddrMode_Implied:
        case kC02AddrMode_Accumulator:
            return;
        
        case kC02AddrMode_Absolute:
            sprintf(outstr + 3, " $%02X%02X", ctx->memory[PC + 2], ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Absolute_X_Indexed:
            sprintf(outstr + 3, " $%02X%02X,X", ctx->memory[PC + 2], ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Absolute_Y_Indexed:
            sprintf(outstr + 3, " $%02X%02X,Y", ctx->memory[PC + 2], ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Immediate:
            sprintf(outstr + 3, " #$%02X", ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Indirect:
            sprintf(outstr + 3, " $(%02X%02X)", ctx->memory[PC + 2], ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Indirect_X_Indexed:
            sprintf(outstr + 3, " ($%02X,X)", ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Indirect_Y_Indexed:
            sprintf(outstr + 3, " ($%02X),Y", ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Relative:
            sprintf(outstr + 3, " $%04X", (uint16_t)((int16_t)(PC) + (int8_t)(ctx->memory[PC + 1])) + 2);
            return;
        
        case kC02AddrMode_Zeropage:
            sprintf(outstr + 3, " $%02X", ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Zeropage_X_Indexed:
            sprintf(outstr + 3, " $%02X,X", ctx->memory[PC + 1]);
            return;
        
        case kC02AddrMode_Zeropage_Y_Indexed:
            sprintf(outstr + 3, " $%02X,Y", ctx->memory[PC + 1]);
            return;
    }
}

void c02_GetOpcodeMnemonic(uint8_t opcode, char outstr[4])
{
    uint8_t idx = (g_opcode_lookup[opcode] >> 8) & 0x3F;
    
    if (idx >= C02__NUM_INSTRUCTIONS) memcpy(outstr, "---", 4);
    else                              memcpy(outstr, g_opcode_mnemonics[idx], 4);
}

C02AddressingMode c02_GetInstructionAddressingMode(uint8_t opcode)
{
    return (g_opcode_lookup[opcode] >> 4) & 0x0F;
}

int32_t c02_GetInstructionLength(uint8_t opcode)
{
    return (g_opcode_lookup[opcode] >> 14) & 0x03;
}

int32_t c02_GetInstructionCycleCount(uint8_t opcode)
{
    return g_opcode_lookup[opcode] & 0x0F;
}


/* --- Private functions implementation ------------------------------------- */

uint16_t c02__ParseOperands(C02Context *ctx)
{
    switch (ctx->curr_instruction.addressing_mode)
    {
        case kC02AddrMode_Immediate:
            return ctx->reg.PC + 1;
        
        case kC02AddrMode_Zeropage:
            return (uint16_t)(ctx->memory[ctx->reg.PC + 1]);
        
        case kC02AddrMode_Zeropage_X_Indexed:
            return (uint16_t)(ctx->memory[ctx->reg.PC + 1] + ctx->reg.X) & 0xFF;
        
        case kC02AddrMode_Zeropage_Y_Indexed:
            return (uint16_t)(ctx->memory[ctx->reg.PC + 1] + ctx->reg.Y) & 0xFF;
        
        case kC02AddrMode_Absolute:
        {
            uint8_t lb = ctx->memory[ctx->reg.PC + 1];
            uint8_t hb = ctx->memory[ctx->reg.PC + 2];
            return C02__MKADDR(hb, lb);
        }
        
        case kC02AddrMode_Absolute_X_Indexed:
        {
            uint8_t lb = ctx->memory[ctx->reg.PC + 1];
            uint8_t hb = ctx->memory[ctx->reg.PC + 2];
            
            uint16_t effective = C02__MKADDR(hb, lb) + ctx->reg.X;
            
            // Account for page crossing
            if (hb != C02__HB(effective))
                ctx->curr_instruction.cycles_needed++;
            
            return effective;
        }
        
        case kC02AddrMode_Absolute_Y_Indexed:
        {
            uint8_t lb = ctx->memory[ctx->reg.PC + 1];
            uint8_t hb = ctx->memory[ctx->reg.PC + 2];
            
            uint16_t effective = C02__MKADDR(hb, lb) + ctx->reg.Y;
            
            // Account for page crossing
            if (hb != C02__HB(effective))
                ctx->curr_instruction.cycles_needed++;
            
            return effective;
        }
        
        case kC02AddrMode_Indirect:
        {
            uint8_t lb = ctx->memory[ctx->reg.PC + 1];
            uint8_t hb = ctx->memory[ctx->reg.PC + 2];
            uint16_t lookup = C02__MKADDR(hb, lb);
            
            lb = ctx->memory[lookup];
            hb = ctx->memory[lookup + 1];
            
            return C02__MKADDR(hb, lb);
        }
        
        case kC02AddrMode_Indirect_X_Indexed:
        {
            uint16_t lookup = (ctx->memory[ctx->reg.PC + 1] + ctx->reg.X) & 0xFF;
            
            uint8_t lb = ctx->memory[lookup];
            uint8_t hb = ctx->memory[lookup + 1];
            
            return C02__MKADDR(hb, lb);
        }
        
        case kC02AddrMode_Indirect_Y_Indexed:
        {
            uint16_t zpaddr = ctx->memory[ctx->reg.PC + 1];
            
            uint8_t lb = ctx->memory[zpaddr];
            uint8_t hb = ctx->memory[zpaddr + 1];
            
            uint16_t effective = C02__MKADDR(hb, lb) + ctx->reg.Y;
            
            // Account for page crossing
            if (hb != C02__HB(effective))
                ctx->curr_instruction.cycles_needed++;
            
            return effective;
        }
        
        case kC02AddrMode_Relative:
        {
            int8_t offset = (int8_t)(ctx->memory[ctx->reg.PC + 1]);
            return (int16_t)(ctx->reg.PC + 2 + offset);
        }
        
        default:
            // How did we get here?!?!
            return 0;
    }
}

void c02__ADC(C02Context *ctx)
{
    ////////////////////////////////////////////
    //  Add Memory to Accumulator with Carry  //
    ////////////////////////////////////////////
    
    /**
     * This feels needlessly difficult >.<
     *  - http://www.6502.org/tutorials/decimal_mode.html#A
     */
    
    uint16_t A = ctx->reg.A;
    uint16_t B = ctx->memory[ c02__ParseOperands(ctx) ];  // Load memory value based on the addressing mode
    uint16_t C = ctx->reg.SR & C02_FLAG_CARRY;
    
    // Binary arithmetic
    uint16_t binary_res = A + B + C;
    
    uint16_t old_A = A;                                           // Needed to calculate the "overflow" flag in BCD mode
    uint8_t binary_res_8 = (uint8_t)A + (uint8_t)B + (uint8_t)C;  // Needed to calculate the "zero" flag in BCD mode
    
    // BCD arithmetic
    if (ctx->reg.SR & C02_FLAG_DECMODE)
    {
        // 1a
        uint16_t AL = (A & 0x0F) + (B & 0x0F) + C;
        
        // 1b
        if (AL >= 0x0A)
            AL = ((AL + 0x06) & 0x0F) + 0x10;
        
        // 2a
        int16_t AL_signed = (int16_t)(A & 0x0F) + (int16_t)(B & 0x0F) + (int16_t)C;
        
        // 2b
        if (AL_signed >= 0x0A)
            AL_signed = ((AL_signed + 0x06) & 0x0F) + 0x10;
        
        // 2c
        int16_t A_signed = (int16_t)(A & 0xF0) + (int16_t)(B & 0xF0) + AL_signed;
        
        // 1c
        A = (A & 0xF0) + (B & 0xF0) + AL;
        
        // 1e
        if (A >= 0xA0)
            A += 0x60;
        
        // 1f
        ctx->reg.A = C02__LB(A);
        
        // 1g
        C02__TEST_FOR_FLAG( C02_FLAG_CARRY, A >= 0x100 );
        
        // 2e
        C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, A_signed & 0x80 );
        
        /**
         * When I followed Bruce Clark's decimal mode tutorial, I wrote the "overflow" flag check like this:
         * 
         *      // 2f
         *      C02__TEST_FOR_FLAG( C02_FLAG_OVERFLOW, A_signed < -128 || A_signed > 127 );
         * 
         * It didn't pass his decimal test program.
         * 
         * I hate this.
         * 
         * https://stackoverflow.com/a/29193951
         */
        C02__TEST_FOR_FLAG( C02_FLAG_OVERFLOW, (A_signed ^ old_A) & (A_signed ^ B) & 0x80 );
        
        /**
         * Oh, yeah, I hate this too!
         * It's a miracle I found this:
         *  - https://stardot.org.uk/forums/viewtopic.php?p=96927&sid=0aa5033185c341b504421823d50cf967#p96927
         */
        C02__TEST_FOR_FLAG( C02_FLAG_ZERO, binary_res_8 == 0 );
    }
    else
    {
        ctx->reg.A = C02__LB(binary_res);
        
        C02__TEST_FOR_FLAG( C02_FLAG_CARRY,    binary_res >= 0x100                        );
        C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0                            );
        C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80                          );
        C02__TEST_FOR_FLAG( C02_FLAG_OVERFLOW, (binary_res ^ A) & (binary_res ^ B) & 0x80 );
    }
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__AND(C02Context *ctx)
{
    ///////////////////////////////////
    //  AND Memory with Accumulator  //
    ///////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    ctx->reg.A &= M;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__ASL(C02Context *ctx)
{
    //////////////////////////////////////////////////
    //  Shift Left One Bit (Memory or Accumulator)  //
    //////////////////////////////////////////////////
    
    uint8_t  M     = ctx->reg.A;
    uint16_t Maddr = 0;
    
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
    {
        // Load memory value based on the addressing mode
        Maddr = c02__ParseOperands(ctx);
        M     = ctx->memory[Maddr];
    }
    
    // Update "carry" flag
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY, M & 0x80 );
    
    M <<= 1;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     M == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, M & 0x80 );
    
    // Write back to memory (or accumulator)
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
        ctx->memory[Maddr] = M;
    else
        ctx->reg.A = M;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__BCC(C02Context *ctx)
{
    /////////////////////////////
    //  Branch on Carry Clear  //
    /////////////////////////////
    
    if (ctx->reg.SR & C02_FLAG_CARRY)
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__BCS(C02Context *ctx)
{
    ///////////////////////////
    //  Branch on Carry Set  //
    ///////////////////////////
    
    if (!(ctx->reg.SR & C02_FLAG_CARRY))
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__BEQ(C02Context *ctx)
{
    /////////////////////////////
    //  Branch on Result Zero  //
    /////////////////////////////
    
    if (!(ctx->reg.SR & C02_FLAG_ZERO))
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__BIT(C02Context *ctx)
{
    ////////////////////////////////////////////
    //  Test Bits in Memory with Accumulator  //
    ////////////////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    // Set "negative" flag to bit 7 and "overflow" flag to bit 6
    ctx->reg.SR = (ctx->reg.SR & 0x3F) | (uint8_t)(M & 0xC0);
    
    // Update "zero" flag
    uint8_t res = ctx->reg.A & M;
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO, res == 0 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__BMI(C02Context *ctx)
{
    //////////////////////////////
    //  Branch on Result Minus  //
    //////////////////////////////
    
    if (!(ctx->reg.SR & C02_FLAG_NEGATIVE))
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__BNE(C02Context *ctx)
{
    /////////////////////////////////
    //  Branch on Result not Zero  //
    /////////////////////////////////
    
    if (ctx->reg.SR & C02_FLAG_ZERO)
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__BPL(C02Context *ctx)
{
    /////////////////////////////
    //  Branch on Result Plus  //
    /////////////////////////////
    
    if (ctx->reg.SR & C02_FLAG_NEGATIVE)
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__BRK(C02Context *ctx)
{
    ///////////////////
    //  Force Break  //
    ///////////////////
    
    ctx->reg.PC += ctx->curr_instruction.length + 1;
    
    // Push the program counter and the status register to the stack
    C02__STACK_PUSH( C02__HB(ctx->reg.PC) );
    C02__STACK_PUSH( C02__LB(ctx->reg.PC) );
    
    // When pushed, set bit 5 and the "break" flag
    ctx->reg.SR |= C02__FLAG_RESERVED;
    ctx->reg.SR |= C02__FLAG_BREAK;
    C02__STACK_PUSH(ctx->reg.SR);
    
    // Disable interrupts
    ctx->reg.SR |= C02_FLAG_IQRDIS;
    
    // Read IRQ vector
    uint8_t lb = ctx->memory[C02__IRQ_VECTOR_LB];
    uint8_t hb = ctx->memory[C02__IRQ_VECTOR_HB];
    
    // Jump to that address
    ctx->reg.PC = C02__MKADDR(hb, lb);
}

void c02__BVC(C02Context *ctx)
{
    ////////////////////////////////
    //  Branch on Overflow Clear  //
    ////////////////////////////////
    
    if (ctx->reg.SR & C02_FLAG_OVERFLOW)
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__BVS(C02Context *ctx)
{
    //////////////////////////////
    //  Branch on Overflow Set  //
    //////////////////////////////
    
    if (!(ctx->reg.SR & C02_FLAG_OVERFLOW))
    {
        // Don't branch...
        ctx->reg.PC += ctx->curr_instruction.length;
        return;
    }
    
    // Calculate final address from the relative offset
    uint16_t destination = c02__ParseOperands(ctx);
    
    // Base cycle cost for taking the branch
    ctx->curr_instruction.cycles_needed++;
    
    // Check if the branch occurs on the same page or on a different one
    uint16_t curr_PC = ctx->reg.PC + ctx->curr_instruction.length;
    if (curr_PC & 0xFF00 != destination & 0xFF00)
        ctx->curr_instruction.cycles_needed++;
    
    ctx->reg.PC = destination;
}

void c02__CLC(C02Context *ctx)
{
    ////////////////////////
    //  Clear Carry Flag  //
    ////////////////////////
    
    ctx->reg.SR &= ~C02_FLAG_CARRY;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__CLD(C02Context *ctx)
{
    //////////////////////////
    //  Clear Decimal Mode  //
    //////////////////////////
    
    ctx->reg.SR &= ~C02_FLAG_DECMODE;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__CLI(C02Context *ctx)
{
    ///////////////////////////////////
    //  Clear Interrupt Disable Bit  //
    ///////////////////////////////////
    
    ctx->reg.SR &= ~C02_FLAG_IQRDIS;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__CLV(C02Context *ctx)
{
    ///////////////////////////
    //  Clear Overflow Flag  //
    ///////////////////////////
    
    ctx->reg.SR &= ~C02_FLAG_OVERFLOW;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__CMP(C02Context *ctx)
{
    ///////////////////////////////////////
    //  Compare Memory with Accumulator  //
    ///////////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    uint16_t res = ctx->reg.A - M;
    
    // Update "zero", "negative" and "carry" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     res == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, res & 0x80 );
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY,    !C02__HB(res) );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__CPX(C02Context *ctx)
{
    //////////////////////////////////
    //  Compare Memory and Index X  //
    //////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    uint16_t res = ctx->reg.X - M;
    
    // Update "zero", "negative" and "carry" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     res == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, res & 0x80 );
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY,    !C02__HB(res) );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__CPY(C02Context *ctx)
{
    //////////////////////////////////
    //  Compare Memory and Index Y  //
    //////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    uint16_t res = ctx->reg.Y - M;
    
    // Update "zero", "negative" and "carry" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     res == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, res & 0x80 );
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY,    !C02__HB(res) );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__DEC(C02Context *ctx)
{
    ///////////////////////////////
    //  Decrement Memory by One  //
    ///////////////////////////////
    
    // Load memory value based on the addressing mode
    uint16_t Maddr = c02__ParseOperands(ctx);
    uint8_t  M     = ctx->memory[Maddr];
    
    M--;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     M == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, M & 0x80 );
    
    // Write back to memory
    ctx->memory[Maddr] = M;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__DEX(C02Context *ctx)
{
    ////////////////////////////////
    //  Decrement Index X by One  //
    ////////////////////////////////
    
    ctx->reg.X--;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.X == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.X & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__DEY(C02Context *ctx)
{
    ////////////////////////////////
    //  Decrement Index Y by One  //
    ////////////////////////////////
    
    ctx->reg.Y--;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.Y == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.Y & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__EOR(C02Context *ctx)
{
    ////////////////////////////////////////////
    //  Exclusive-OR Memory with Accumulator  //
    ////////////////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    ctx->reg.A ^= M;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__INC(C02Context *ctx)
{
    ///////////////////////////////
    //  Increment Memory by One  //
    ///////////////////////////////
    
    // Load memory value based on the addressing mode
    uint16_t Maddr = c02__ParseOperands(ctx);
    uint8_t  M     = ctx->memory[Maddr];
    
    M++;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     M == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, M & 0x80 );
    
    // Write back to memory
    ctx->memory[Maddr] = M;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__INX(C02Context *ctx)
{
    ////////////////////////////////
    //  Increment Index X by One  //
    ////////////////////////////////
    
    ctx->reg.X++;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.X == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.X & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__INY(C02Context *ctx)
{
    ////////////////////////////////
    //  Increment Index Y by One  //
    ////////////////////////////////
    
    ctx->reg.Y++;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.Y == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.Y & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__JMP(C02Context *ctx)
{
    ////////////////////////////
    //  Jump to New Location  //
    ////////////////////////////
    
    ctx->reg.PC = c02__ParseOperands(ctx);
}

void c02__JSR(C02Context *ctx)
{
    //////////////////////////////////////////////////
    //  Jump to New Location Saving Return Address  //
    //////////////////////////////////////////////////
    
    uint16_t dest = c02__ParseOperands(ctx);
    
    // Push return address to the stack
    ctx->reg.PC += 2;
    C02__STACK_PUSH( C02__HB(ctx->reg.PC) );
    C02__STACK_PUSH( C02__LB(ctx->reg.PC) );
    
    ctx->reg.PC = dest;
}

void c02__LDA(C02Context *ctx)
{
    ////////////////////////////////////
    //  Load Accumulator with Memory  //
    ////////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    ctx->reg.A = M;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__LDX(C02Context *ctx)
{
    ////////////////////////////////
    //  Load Index X with Memory  //
    ////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    ctx->reg.X = M;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.X == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.X & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__LDY(C02Context *ctx)
{
    ////////////////////////////////
    //  Load Index Y with Memory  //
    ////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    ctx->reg.Y = M;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.Y == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.Y & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__LSR(C02Context *ctx)
{
    ///////////////////////////////////////////////////
    //  Shift One Bit Right (Memory or Accumulator)  //
    ///////////////////////////////////////////////////
    
    uint8_t  M     = ctx->reg.A;
    uint16_t Maddr = 0;
    
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
    {
        // Load memory value based on the addressing mode
        Maddr = c02__ParseOperands(ctx);
        M     = ctx->memory[Maddr];
    }
    
    // Update "carry" flag
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY, M & 0x01 );
    
    M >>= 1;
    
    // Update "zero" flag
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO, M == 0 );
    
    // Clear "negative" flag
    ctx->reg.SR &= ~C02_FLAG_NEGATIVE;
    
    // Write back to memory (or accumulator)
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
        ctx->memory[Maddr] = M;
    else
        ctx->reg.A = M;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__NOP(C02Context *ctx)
{
    ////////////////////
    //  No Operation  //
    ////////////////////
    
    // :)
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__ORA(C02Context *ctx)
{
    //////////////////////////////////
    //  OR Memory with Accumulator  //
    //////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint8_t M = ctx->memory[ c02__ParseOperands(ctx) ];
    
    ctx->reg.A |= M;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__PHA(C02Context *ctx)
{
    /////////////////////////////////
    //  Push Accumulator on Stack  //
    /////////////////////////////////
    
    C02__STACK_PUSH(ctx->reg.A);
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__PHP(C02Context *ctx)
{
    //////////////////////////////////////
    //  Push Processor Status on Stack  //
    //////////////////////////////////////
    
    // Set the "break" flag and bit 5 before pushing
    ctx->reg.SR |= C02__FLAG_BREAK | C02__FLAG_RESERVED;
    C02__STACK_PUSH(ctx->reg.SR);
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__PLA(C02Context *ctx)
{
    ///////////////////////////////////
    //  Pull Accumulator from Stack  //
    ///////////////////////////////////
    
    C02__STACK_POP(ctx->reg.A);
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__PLP(C02Context *ctx)
{
    ////////////////////////////////////////
    //  Pull Processor Status from Stack  //
    ////////////////////////////////////////
    
    C02__STACK_POP(ctx->reg.SR);
    
    // Ignore "break" flag and bit 5
    ctx->reg.SR &= ~C02__FLAG_BREAK;
    ctx->reg.SR &= ~C02__FLAG_RESERVED;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__ROL(C02Context *ctx)
{
    ///////////////////////////////////////////////////
    //  Rotate One Bit Left (Memory or Accumulator)  //
    ///////////////////////////////////////////////////
    
    uint8_t  M     = ctx->reg.A;
    uint16_t Maddr = 0;
    
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
    {
        // Load memory value based on the addressing mode
        Maddr = c02__ParseOperands(ctx);
        M     = ctx->memory[Maddr];
    }
    
    bool last_bit_set = !!(M & 0x80);
    
    // Shift left and insert the carry bit
    M <<= 1;
    M |= (ctx->reg.SR & C02_FLAG_CARRY);
    
    // Update "carry", "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY,    last_bit_set );
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     M == 0       );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, M & 0x80     );
    
    // Write back to memory (or accumulator)
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
        ctx->memory[Maddr] = M;
    else
        ctx->reg.A = M;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__ROR(C02Context *ctx)
{
    ////////////////////////////////////////////////////
    //  Rotate One Bit Right (Memory or Accumulator)  //
    ////////////////////////////////////////////////////
    
    uint8_t  M     = ctx->reg.A;
    uint16_t Maddr = 0;
    
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
    {
        // Load memory value based on the addressing mode
        Maddr = c02__ParseOperands(ctx);
        M     = ctx->memory[Maddr];
    }
    
    bool first_bit_set = !!(M & 0x01);
    
    // Shift right and insert the carry bit
    M >>= 1;
    M |= (ctx->reg.SR & C02_FLAG_CARRY) << 7;
    
    // Update "carry", "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY,    first_bit_set );
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     M == 0       );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, M & 0x80     );
    
    // Write back to memory (or accumulator)
    if (ctx->curr_instruction.addressing_mode != kC02AddrMode_Accumulator)
        ctx->memory[Maddr] = M;
    else
        ctx->reg.A = M;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__RTI(C02Context *ctx)
{
    /////////////////////////////
    //  Return from Interrupt  //
    /////////////////////////////
    
    C02__STACK_POP(ctx->reg.SR);
    
    // Ignore "break" flag and bit 5
    ctx->reg.SR &= ~C02__FLAG_BREAK;
    ctx->reg.SR &= ~C02__FLAG_RESERVED;
    
    uint8_t hb, lb;
    C02__STACK_POP(lb);
    C02__STACK_POP(hb);
    ctx->reg.PC = C02__MKADDR(hb, lb);
}

void c02__RTS(C02Context *ctx)
{
    //////////////////////////////
    //  Return from Subroutine  //
    //////////////////////////////
    
    uint8_t hb, lb;
    C02__STACK_POP(lb);
    C02__STACK_POP(hb);
    ctx->reg.PC = C02__MKADDR(hb, lb) + 1;
}

void c02__SBC(C02Context *ctx)
{
    ////////////////////////////////////////////////////
    //  Subtract Memory from Accumulator with Borrow  //
    ////////////////////////////////////////////////////
    
    /**
     * This feels needlessly difficult T-T
     *  - http://www.6502.org/tutorials/decimal_mode.html#A
     */
    
    int16_t A = ctx->reg.A;
    int16_t B = ctx->memory[ c02__ParseOperands(ctx) ];  // Load memory value based on the addressing mode
    int16_t C = ctx->reg.SR & C02_FLAG_CARRY;
    
    uint16_t old_A = A;  // Needed to calculate the "overflow" flag
    
    // Binary arithmetic
    uint16_t binary_res = A - B + C-1;
    
    // BCD arithmetic
    if (ctx->reg.SR & C02_FLAG_DECMODE)
    {
        // 3a
        int16_t AL = (A & 0x0F) - (B & 0x0F) + C-1;
        
        // 3b
        if (AL < 0)
            AL = ((AL - 0x06) & 0x0F) - 0x10;
        
        // 3c
        A = (A & 0xF0) - (B & 0xF0) + AL;
        
        // 3d
        if (A < 0)
            A -= 0x60;
        
        // 3e
        ctx->reg.A = C02__LB(A);
    }
    else
    {
        ctx->reg.A = C02__LB(binary_res);
    }
    
    // bin
    C02__TEST_FOR_FLAG( C02_FLAG_CARRY,    binary_res < 0x100                                    );
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     !(binary_res & 0x00FF)                                );
    C02__TEST_FOR_FLAG( C02_FLAG_OVERFLOW, ((old_A ^ binary_res) & 0x80) && ((old_A ^ B) & 0x80) );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, binary_res & 0x80                                     );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__SEC(C02Context *ctx)
{
    //////////////////////
    //  Set Carry Flag  //
    //////////////////////
    
    ctx->reg.SR |= C02_FLAG_CARRY;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__SED(C02Context *ctx)
{
    ////////////////////////
    //  Set Decimal Flag  //
    ////////////////////////
    
    ctx->reg.SR |= C02_FLAG_DECMODE;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__SEI(C02Context *ctx)
{
    ////////////////////////////////////
    //  Set Interrupt Disable Status  //
    ////////////////////////////////////
    
    ctx->reg.SR |= C02_FLAG_IQRDIS;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__STA(C02Context *ctx)
{
    ///////////////////////////////////
    //  Store Accumulator in Memory  //
    ///////////////////////////////////
    
    // Load memory value based on the addressing mode
    uint16_t Maddr = c02__ParseOperands(ctx);
    
    ctx->memory[Maddr] = ctx->reg.A;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__STX(C02Context *ctx)
{
    ///////////////////////////////
    //  Store Index X in Memory  //
    ///////////////////////////////
    
    // Load memory value based on the addressing mode
    uint16_t Maddr = c02__ParseOperands(ctx);
    
    ctx->memory[Maddr] = ctx->reg.X;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__STY(C02Context *ctx)
{
    ///////////////////////////////
    //  Store Index Y in Memory  //
    ///////////////////////////////
    
    // Load memory value based on the addressing mode
    uint16_t Maddr = c02__ParseOperands(ctx);
    
    ctx->memory[Maddr] = ctx->reg.Y;
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__TAX(C02Context *ctx)
{
    ///////////////////////////////////////
    //  Transfer Accumulator to Index X  //
    ///////////////////////////////////////
    
    ctx->reg.X = ctx->reg.A;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.X == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.X & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__TAY(C02Context *ctx)
{
    ///////////////////////////////////////
    //  Transfer Accumulator to Index Y  //
    ///////////////////////////////////////
    
    ctx->reg.Y = ctx->reg.A;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.Y == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.Y & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__TSX(C02Context *ctx)
{
    /////////////////////////////////////////
    //  Transfer Stack Pointer to Index X  //
    /////////////////////////////////////////
    
    ctx->reg.X = ctx->reg.SP;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.X == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.X & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__TXA(C02Context *ctx)
{
    ///////////////////////////////////////
    //  Transfer Index X to Accumulator  //
    ///////////////////////////////////////
    
    ctx->reg.A = ctx->reg.X;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__TXS(C02Context *ctx)
{
    //////////////////////////////////////////
    //  Transfer Index X to Stack Register  //
    //////////////////////////////////////////
    
    ctx->reg.SP = ctx->reg.X;
    ctx->reg.PC += ctx->curr_instruction.length;
}

void c02__TYA(C02Context *ctx)
{
    ///////////////////////////////////////
    //  Transfer Index Y to Accumulator  //
    ///////////////////////////////////////
    
    ctx->reg.A = ctx->reg.Y;
    
    // Update "zero" and "negative" flags
    C02__TEST_FOR_FLAG( C02_FLAG_ZERO,     ctx->reg.A == 0   );
    C02__TEST_FOR_FLAG( C02_FLAG_NEGATIVE, ctx->reg.A & 0x80 );
    
    ctx->reg.PC += ctx->curr_instruction.length;
}


#endif /* _C02_C */
#endif /* C02_IMPLEMENTATION */

/*
==========
 LICENSE
==========

MIT No Attribution

Copyright 2025 Emanuele Parlangeli

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/