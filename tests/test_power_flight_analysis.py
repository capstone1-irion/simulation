"""Small synthetic acceptance tests; these are not real telemetry."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from power_flight_analysis import validate_fixed


def fixed_row(component=1):
    return {'source_system': 36, 'source_component': component, 'phase': 'fixed',
            'fields': {'mavpackettype': 'BATTERY_STATUS', 'id': 0,
                       'voltages': [4010, 4020, 4030, 4040] + [65535] * 6,
                       'current_battery': 725, 'battery_remaining': 73,
                       'temperature': 2850, 'current_consumed': 123}}


class FixedAcceptance(unittest.TestCase):
    def test_requires_autopilot_generated_matching_battery(self):
        self.assertEqual(validate_fixed([fixed_row() for _ in range(3)], 36)['battery_packets'], 3)
        with self.assertRaises(ValueError):
            validate_fixed([fixed_row(180) for _ in range(3)], 36)
        wrong = fixed_row()
        wrong['fields']['temperature'] = 2500
        with self.assertRaises(ValueError):
            validate_fixed([wrong for _ in range(3)], 36)


def flight_rows():
    rows = []
    for phase, armed, landed, z, ca, mv, used in [
        ('preflight', 0, 1, 0.0, 50, 16000, 0),
        ('hover', 128, 2, -3.0, 2050, 15200, 40),
        ('landed', 0, 1, 0.0, 50, 15990, 80)]:
        for kind, fields in [
            ('HEARTBEAT', {'base_mode': armed}),
            ('EXTENDED_SYS_STATE', {'landed_state': landed}),
            ('LOCAL_POSITION_NED', {'z': z, 'vz': 0.0}),
            ('ESC_STATUS', {'index': 0, 'rpm': [6000 if armed else 0] * 4,
                            'voltage': [16.0] * 4, 'current': [5.0 if armed else 0.0] * 4}),
            ('BATTERY_STATUS', {'id': 0, 'voltages': [mv // 4] * 4 + [65535] * 6,
                                'current_battery': ca, 'current_consumed': used,
                                'battery_remaining': 95})]:
            rows.append({'phase': phase, 'source_system': 36, 'source_component': 1,
                         'receive_monotonic_ns': len(rows) * 10000000,
                         'fields': {'mavpackettype': kind, **fields}})
    return rows


class FlightAcceptance(unittest.TestCase):
    def test_flight_requires_landing_and_load_response(self):
        from power_flight_analysis import validate_flight
        self.assertGreater(validate_flight(flight_rows(), 36)['max_relative_altitude_m'], 2.0)
        missing = [r for r in flight_rows() if r['fields']['mavpackettype'] != 'EXTENDED_SYS_STATE']
        with self.assertRaises(ValueError):
            validate_flight(missing, 36)
        fixed = flight_rows()
        for row in fixed:
            if row['fields']['mavpackettype'] == 'BATTERY_STATUS':
                row['fields']['current_battery'] = 50
        with self.assertRaises(ValueError):
            validate_flight(fixed, 36)


if __name__ == '__main__':
    unittest.main()
