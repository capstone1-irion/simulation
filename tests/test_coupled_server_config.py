"""Narrow regression for the standalone server's missing flight sensors."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class StandaloneSensorConfigTest(unittest.TestCase):
    def test_headless_server_loads_flight_sensors_without_rendering(self):
        config = Path(__file__).resolve().parents[1] / 'config/coupled-headless.config'
        self.assertTrue(config.is_file(), 'Standalone Gazebo needs an explicit flight-sensor config')
        plugins = ET.parse(config).getroot().findall('plugins/plugin')
        names = {p.attrib['name'] for p in plugins}
        for required in ['Physics', 'UserCommands', 'SceneBroadcaster', 'Imu',
                         'AirPressure', 'NavSat', 'Magnetometer']:
            self.assertIn('gz::sim::systems::' + required, names)
        self.assertNotIn('gz::sim::systems::Sensors', names)  # No camera/rendering needed.
        self.assertTrue(all(p.attrib['entity_type'] == 'world' for p in plugins))


if __name__ == '__main__':
    unittest.main()
