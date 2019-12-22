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
        new_record = self.get_record(engine)
        self.assertEqual(new_record, record)

    @staticmethod
    def get_record(engine):
        return engine.get_lane_vehicle_count(), engine.get_average_travel_time()

    def test_save_and_load(self):
        """Single save and single load with single threading engine"""
        engine = cityflow.Engine(config_file=self.config_file, thread_num=1)
        self.run_steps(engine, self.period)

        start_time = time.time()
        archive = engine.snapshot()
        save_time = time.time() - start_time

        self.run_steps(engine, self.period)
        record0 = self.get_record(engine)

        start_time = time.time()
        engine.load(archive)
        load_time = time.time() - start_time

        self.run_and_check(engine, record0)

        del engine
        print("\nsave: %.4fs load: %.4fs" % (save_time, load_time))

    def test_save_and_load_multithread(self):
        """Single save and single load with multi-threading engine"""
        engine = cityflow.Engine(config_file=self.config_file, thread_num=4)

        self.run_steps(engine, self.period)
        archive = engine.snapshot()

        self.run_steps(engine, self.period)
        record0 = self.get_record(engine)

        engine.load(archive)
        self.run_and_check(engine, record0)

        del engine

    def test_save_and_multi_load(self):
        """Multiple saves and multiple loads with multi-threading engine"""
        engine = cityflow.Engine(config_file=self.config_file, thread_num=4)

        self.run_steps(engine, self.period)
        archive = engine.snapshot()

        self.run_steps(engine, self.period)
        record0 = self.get_record(engine)

        repeats = 2
        for i in range(repeats):
            engine.load(archive)
            self.run_and_check(engine, record0)
        del engine

    def test_multi_save_and_multi_load(self):
        """ Multiple save and multiple loads with multi-threading engine") """
        engine = cityflow.Engine(config_file=self.config_file, thread_num=4)
        archives, records = [], []
        repeats = 5

        for i in range(repeats + 1):
            archives.append(engine.snapshot())
            records.append(self.get_record(engine))
            self.run_steps(engine, self.period)

        for i in range(repeats):
            for j in range(repeats):
                engine.load(archives[j])
                self.run_and_check(engine, records[j + 1])

        del engine

    def test_save_to_file(self):
        """ Disk IO test """
        engine = cityflow.Engine(config_file=self.config_file, thread_num=4)
        self.run_steps(engine, self.period)
        engine.snapshot().dump("save.json")
        self.run_steps(engine, self.period)
        record = self.get_record(engine)
        engine.load_from_file("save.json")
        self.run_and_check(engine, record)
        del engine

    def test_multi_save_to_file(self):
        """ Disk IO test 2"""
        engine = cityflow.Engine(config_file=self.config_file, thread_num=4)
        for i in range(2):
            self.run_steps(engine, self.period)
            engine.snapshot().dump("save.json")
            self.run_steps(engine, self.period)
            record = self.get_record(engine)
            for j in range(2):
                engine.load_from_file("save.json")
                self.run_and_check(engine, record)

        del engine

if __name__ == '__main__':
    unittest.main(verbosity=2)
