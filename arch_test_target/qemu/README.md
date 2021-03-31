# Using the QEMU Simulator as an Architectural test model

This is a reference for running QEMU as a target for the RISC-V Architectural Test framework.


## Adding QEMU as a target to the Architectural Test framework

Now clone the architectural test framework repo and copy the updated Makefile.include to it:

```
  $ git clone https://github.com/riscv/riscv-arch-test.git
  $ cd riscv-arch-test
  $ cp <custom-path>/plct-qemu/arch_test_target/qemu ./riscv-target/
```

```
make RISCV_TARGET=qemu compile simulate verify
```
If there is no default qemu-system-riscv32/64, you can use "TARGET_SIM=<path-to-qemu-system-riscv32/64>" to indicate the path of QEMU bin.













