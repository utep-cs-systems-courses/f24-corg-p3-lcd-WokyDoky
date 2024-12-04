    .global doubleNumber
doubleNumber:
    ; Input: R12 contains the number to double
    ; Output: R12 contains the doubled value

    add     r12, r12    ; Double the number (R12 = R12 + R12)

    ret                 ; Return to caller
