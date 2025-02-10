from cocotb_test.simulator import run
import pytest
import os

from telemetry import telemetryMark
pytestmark = telemetryMark()


def source(name):
	dir = os.path.dirname(__file__)
	src_dir = os.path.join(dir, 'src' )
	return os.path.join(src_dir, name)

@pytest.mark.telemetry_files(source('jokenpo.vhd'))
def test_jokenpo():
    run(vhdl_sources=[source("jokenpo.vhd")], toplevel="jokenpo", module="av2_cocotb" , testcase='tb_jokenpo', toplevel_lang="vhdl")

@pytest.mark.telemetry_files(source('cadeado.vhd'))
def test_cadeado():
    run(vhdl_sources=[source("cadeado.vhd")], toplevel="cadeado", module="av2_cocotb" , testcase='tb_cadeado', toplevel_lang="vhdl")

if __name__ == "__main__":
    test_gray()
    test_fullsub()
