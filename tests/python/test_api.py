import unittest
import cityflow


class TestAPI(unittest.TestCase):

    config_file = "./examples/config.json"
    period = 3600

    def test_data_api(self):
        """Single save and single load with single threading engine"""
        eng = cityflow.Engine(config_file=self.config_file, thread_num=1)

        for _ in range(self.period):
            eng.next_step()
            running_count = len(eng.get_vehicles())
            total_count = len(eng.get_vehicles(include_waiting=True))
            self.assertTrue(running_count <= total_count)
            self.assertTrue(running_count, eng.get_vehicle_count())
            eng.get_lane_vehicle_count()
            eng.get_lane_waiting_vehicle_count()
            eng.get_lane_vehicles()
            eng.get_vehicle_speed()
            eng.get_vehicle_distance()
            eng.get_current_time()

        del eng

    def test_set_replay(self):
        """change replay path on the fly"""
        eng = cityflow.Engine(config_file=self.config_file, thread_num=1)

        for _ in range(100):
            eng.next_step()

        eng.set_replay_file("replay2.txt")

        for _ in range(100):
            eng.next_step()

        del eng

if __name__ == '__main__':
    unittest.main(verbosity=2)
