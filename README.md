# c02

c02 is a single header C library that provides a very simple API to emulate a 6502 processor.
Only supports legal opcodes.

Tested with Klaus Dormann's functional test suite and Bruce Clark's decimal mode test program:

https://github.com/Klaus2m5/6502_65C02_functional_tests

# Example

Here's a little example that shows how to run Klaus Dormann's "6502_functional_test.bin" test program:

```c
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define C02_IMPLEMENTATION
#include "c02.h"

#define ADDR_SUCCESS 0x3469
const uint8_t program[] = { ... };

uint8_t memory[C02_MEMORY_SIZE];

int main(int argc, char *argv[])
{
    C02Context ctx;
    c02_InitContext(&ctx, memory);
    
    // Load the program into memory, and set the PC to the start
    memcpy(memory, program, sizeof program);
    c02_SetPC(&ctx, 0x0400);
    
    // Run until we're stuck in place
    for (;;)
    {
        if (!c02_Step(&ctx))
        {
            // Print registers, zero-page, stack, ...
            return -1;
        }
        
        if (c02_DetectTrap(&ctx))
            break;
    }
    
    uint16_t pc = c02_GetPC(&ctx);
    
    // Where are we stuck?
    if (pc == ADDR_SUCCESS)
        printf("SUCCESS!!");
    else
        printf("Trapped at 0x%04X", pc);
    
    printf("  total cycles : %ld\n", c02_GetTotalCycles(&ctx));
    return 0;
}
```
