# Using the QEMU Simulator as an Architectural test model

This is a reference for running QEMU as a target for the RISC-V Architectural Test framework.


## Adding QEMU as a target to the Architectural Test framework

Now clone the architectural test framework repo and copy the updated Makefile.include to it:

```
  $ git clone https://github.com/riscv/riscv-arch-test.git
  $ cd riscv-arch-test
  $ cp -r <custom-path>/plct-qemu/arch_test_target/qemu ./riscv-target/
```

```
make RISCV_TARGET=qemu compile simulate verify
```

## Using different options to custom your test

Now set the options to make it suitable for your test

`TARGET_SIM`: If there is no default qemu-system-riscv32/64, you can use "TARGET_SIM=<path-to-qemu-system-riscv32/64>" to indicate the path of QEMU bin.

`RISCV_DEVICE`: Keep it blank if you want to iterate through all the supported extensions. Allowed values are the individual names of the extensions supported by your target like: **I, M, C, K_unratified,privilege or Zifencei**.Only one extension can be selected at once.

`RISCV_PREFIX`: Keep it blank if there is a default compiler, or you can use "RISCV_PREFIX=<path-to-riscv64-unknown-elf-?>" to indicate the path of riscv64-unknown-elf-gcc bin.

`XLEN`:set XLEN to max supported XLEN. Allowed values are 32 and 64

There is an example for K test and 64 bits xlen.

```
make RISCV_TARGET=qemu RISCV_DEVICE=K_unratified TARGET_SIM=<your-path>/qemu-system-riscv64 RISCV_PREFIX=<your-path>/riscv64-unknown-elf- XLEN=64 compile simulate verify
```

If you want to ignore some instructions in the test you can delete it in the Makefrag file such as K_32
in the `./riscv-test-suite/rv32i_m/K_unratified/Makefrag` file the `rv32k_sc_tests` structure. 









