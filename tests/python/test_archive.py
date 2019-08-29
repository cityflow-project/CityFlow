import unittest
import cityflow
import time

class TestArchive(unittest.TestCase):

    config_file = "./examples/config.json"
    period = 100

    @staticmethod
    def run_steps(engine, steps):
        for i in range(steps):
            engine.next_step()

    def run_and_check(self, engine, record):
        self.run_steps(engine, self.period)
        newRecord = engine.get_lane_vehicle_count()
        self.assertEqual(newRecord, record)

    def test_save_and_load(self):
        """Single save and single load with single threading engine"""
        engine = cityflow.Engine(config_file = self.config_file, thread_num = 1)
        self.run_steps(engine, self.period)

        startTime = time.time()
        archive = engine.snapshot()
        saveTime = time.time() - startTime

        self.run_steps(engine, self.period)
        record0 = engine.get_lane_vehicle_count()

        startTime = time.time()
        engine.load(archive)
        loadTime = time.time() - startTime

        self.run_and_check(engine, record0)

        del engine
        print("\nsave: %.4fs load: %.4fs" % (saveTime, loadTime))

    def test_save_and_load_multithread(self):
        """Single save and single load with multi-threading engine"""
        engine = cityflow.Engine(config_file = self.config_file, thread_num = 4)

        self.run_steps(engine, self.period)
        archive = engine.snapshot()

        self.run_steps(engine, self.period)
        record0 = engine.get_lane_vehicle_count()

        engine.load(archive)
        self.run_and_check(engine, record0)

        del engine

    def test_save_and_multi_load(self):
        """Multiple saves and multiple loads with multi-threading engine"""
        engine = cityflow.Engine(config_file = self.config_file, thread_num = 4)

        self.run_steps(engine, self.period)
        archive = engine.snapshot()

        self.run_steps(engine, self.period)
        record0 = engine.get_lane_vehicle_count()

        repeats = 2
        for i in range(repeats):
            engine.load(archive)
            self.run_and_check(engine, record0)
        del engine

    def test_multi_save_and_multi_load(self):
        """ Multiple save and multiple loads with multi-threading engine") """
        engine = cityflow.Engine(config_file = self.config_file, thread_num = 4)
        archives, records = [], []
        repeats = 5

        for i in range(repeats + 1):
            archives.append(engine.snapshot())
            records.append(engine.get_lane_vehicle_count())
            self.run_steps(engine, self.period)

        for i in range(repeats):
            for j in range(repeats):
                engine.load(archives[j])
                self.run_and_check(engine, records[j + 1])

        del engine

if __name__ == '__main__':
    unittest.main(verbosity = 2)
