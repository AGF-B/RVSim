loop:
addi x0, x0, 1
addi x1, x1, 2
addi x2, x2, 4
add x3, x1, x2
add x1, x2, x3
sw x1, 0(zero)
lw x4, 0(zero)
li x5, 0x100000
sw x1, 0(x5)
lw x6, 0(x5)
j loop