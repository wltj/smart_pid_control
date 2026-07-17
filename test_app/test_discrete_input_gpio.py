from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read_source(name):
    return (ROOT / name).read_text(encoding="utf-8")


class DiscreteInputGpioConfigTest(unittest.TestCase):
    def test_device_running_di_reads_external_status_input_pin(self):
        board_h = read_source("board.h")

        self.assertRegex(
            board_h,
            r"#define\s+DEVICE_RUNNING_READ\(\)\s+PIN_RD\(\s*IO_WORK_STATUS_IN_PORT\s*,\s*IO_WORK_STATUS_IN_PIN\s*\)",
        )

    def test_work_status_input_pin_is_configured_as_high_impedance_input(self):
        sources = read_source("main.c") + "\n" + read_source("alarm_manager.c")

        self.assertRegex(
            sources,
            r"P5M0\s*&=\s*~\s*BIT\(\s*IO_WORK_STATUS_IN_PIN\s*\)",
        )
        self.assertRegex(
            sources,
            r"P5M1\s*\|=\s*BIT\(\s*IO_WORK_STATUS_IN_PIN\s*\)",
        )

    def test_maintenance_input_high_level_masks_fault_discrete_inputs(self):
        alarm_c = read_source("alarm_manager.c")

        self.assertRegex(alarm_c, r"if\s*\(\s*MAINTENANCE_READ\(\)\s*\)")
        self.assertNotRegex(alarm_c, r"if\s*\(\s*!\s*MAINTENANCE_READ\(\)\s*\)")

    def test_over_current_input_uses_pullup_and_active_low_fault_logic(self):
        alarm_c = read_source("alarm_manager.c")

        self.assertRegex(
            alarm_c,
            r"P5M0\s*&=\s*~\s*BIT\(\s*IO_OVER_CURRENT_IN_PIN\s*\)",
        )
        self.assertRegex(
            alarm_c,
            r"P5M1\s*&=\s*~\s*BIT\(\s*IO_OVER_CURRENT_IN_PIN\s*\)",
        )
        self.assertRegex(
            alarm_c,
            r"SET_PIN\(\s*IO_OVER_CURRENT_IN_PORT\s*,\s*IO_OVER_CURRENT_IN_PIN\s*\)",
        )
        self.assertRegex(
            alarm_c,
            r"if\s*\(\s*!\s*OVER_CURRENT_IN_READ\(\)\s*\)\s*g_discrete_inputs\[DI_OVER_CURRENT_FAULT_OFFSET\]\s*=\s*1\s*;",
        )

    def test_alarm_io_debug_logging_can_be_enabled_at_compile_time(self):
        alarm_c = read_source("alarm_manager.c")
        debug_uart_c = read_source("debugUart.c")

        self.assertRegex(alarm_c, r"#define\s+ALARM_IO_DEBUG_ENABLE\s+1")
        self.assertIn('debug_out("IO "', alarm_c)
        self.assertIn('alarm_debug_log_io_changes();', alarm_c)
        self.assertRegex(
            alarm_c,
            r"alarm_debug_log_io_changes\(\);\s*/\*.*?\*/\s*if\s*\(\s*IS_DEBUG_MODE\(\)\s*\)",
        )
        self.assertRegex(debug_uart_c, r"void\s+debug_out\s*\(\s*char\s+\*msg\s*\)")
        self.assertIn("SBUF = *msg++;", debug_uart_c)


if __name__ == "__main__":
    unittest.main()
