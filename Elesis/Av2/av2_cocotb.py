# Adapted from https://github.com/cocotb/cocotb/blob/master/examples/doc_examples/quickstart/test_my_design.py

import cocotb
from cocotb.triggers import Timer, FallingEdge
from cocotb.clock import Clock

@cocotb.test()
async def tb_jokenpo(dut):

    inX =   [0b00, 0b01, 0b10, 0b00, 0b01, 0b10, 0b01, 0b10, 0b00]
    inY =   [0b00, 0b01, 0b10, 0b01, 0b10, 0b00, 0b00, 0b01, 0b10]
    outG =  [0b11, 0b11, 0b11, 0b01, 0b01, 0b01, 0b10, 0b10, 0b10]


    for i in range(len(inX)):
        dut.x.value = inX[i]
        dut.y.value = inY[i]

        await Timer(1, units="ns")
        condition = (dut.ganhador.value == outG[i])
        if not condition:
            dut._log.error("Expected value: " + "{0:02b}".format(outG[i]) + " Obtained value: " + str(dut.ganhador.value) )
            assert condition, "Error in test {0}!".format(i)
        await Timer(1, units="ns")
        
        
@cocotb.test()
async def tb_cadeado(dut):

    inKEY    = [0b0000      ,0b0001      ,0b0010      ,0b0010      ,0b0010      ]
    inSW     = [0b0101010101,0b0101010101,0b0000000000,0b0110000110,0b0101010101]
    outLEDR  = [0b0000000000,0b0000000000,0b0000000000,0b0000000000,0b0101010101]
    outHEX0  = [0b1111111   ,0b1111111   ,0b1111111   ,0b1111111   ,0b0010010   ]
    outHEX1  = [0b1111111   ,0b1111111   ,0b1111111   ,0b1111111   ,0b0000110   ]
    outHEX2  = [0b1111111   ,0b1111111   ,0b1111111   ,0b1111111   ,0b0010001   ]
    
    clock = Clock(dut.clock, len(inSW), units="ns")
    await cocotb.start(clock.start())    

    await FallingEdge(dut.clock)
    for i in range(len(inSW)):
        dut.SW.value = inSW[i]
        dut.KEY.value = inKEY[i]

        await FallingEdge(dut.clock)

        condition = (dut.LEDR.value == outLEDR[i] and dut.HEX0.value == outHEX0[i] and dut.HEX1.value == outHEX1[i] and dut.HEX2.value == outHEX2[i])
        if not condition:
            if not (dut.LEDR.value == outLEDR[i]):
                dut._log.error("Expected value LEDR: " + "{0:010b}".format(outLEDR[i]) + " Obtained value soma: " + str(dut.LEDR.value) )
            if not (dut.HEX0.value == outHEX0[i]):
                dut._log.error("Expected value HEX0: " + "{0:07b}".format(outHEX0[i]) + " Obtained value vaium: " + str(dut.HEX0.value) )
            if not (dut.HEX1.value == outHEX1[i]):
                dut._log.error("Expected value HEX1: " + "{0:07b}".format(outHEX1[i]) + " Obtained value vaium: " + str(dut.HEX1.value) )
            if not (dut.HEX2.value == outHEX2[i]):
                dut._log.error("Expected value HEX2: " + "{0:07b}".format(outHEX2[i]) + " Obtained value vaium: " + str(dut.HEX2.value) )
            assert condition, "Error in test {0}!".format(i)
        await Timer(1, units="ns")

@cocotb.test()
async def tb_alu(dut):

    in_a = [0x0000000000000123, 0x0000000000000123, 0x0000000000000123, 0x0000000000000123]
    in_b = [0x0000000000000123, 0x0000000000000123, 0x0000000000000123, 0x0000000000000123]
    alu_ctrl_values = ["10", "11", "01", "00"]
    expected_results = [0x0000000000000123, 0x0000000000000123, 0x0000000000000000, 0x0000000000000246]
    expected_carry_out = [0, 0, 0, 0]

    for i in range(len(in_a)):
        dut.i_a.value = in_a[i]
        dut.i_b.value = in_b[i]
        dut.i_alu_ctrl.value = int(alu_ctrl_values[i], 2)

        await Timer(1, units="ns")

        condition = (dut.o_result.value == expected_results[i])
        if not condition:
            dut._log.error(f"Erro teste {i}: Esperado: {hex(expected_results[i])}, Obtido: {hex(dut.o_result.value)}")

        if alu_ctrl_values[i] == "00":
            carry_condition = (dut.o_carry_out.value == expected_carry_out[i])
            if not carry_condition:
                dut._log.error(f"Erro carry_out teste {i}: Esperado: {expected_carry_out[i]}, Obtido: {dut.o_carry_out.value}")
            assert carry_condition, f"Erro no carry_out no teste {i}"

        assert condition, f"Erro teste {i}: Esperado {hex(expected_results[i])}, Obtido : {hex(dut.o_result.value)}"
        
        await Timer(1, units="ns")