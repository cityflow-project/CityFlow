import cityflow
import pandas as pd
import os
import csv
import json

config_dict = {
            "interval": 1.0,
            "warning": True,
            "seed": 0,
            "dir": "data/",
            "roadnetFile": "atlanta_out.json", 
            "flowFile": "atlanta_flow.json",
            "rlTrafficLight": False,
            "laneChange": False,
            "saveReplay": True,
            "roadnetLogFile":"frontend/test_atlanta.json", 
            "replayLogFile": "frontend/atlanta_replay.txt"
        }
# config_dict["roadnetFile"] = "hangzhou_no_uturn.json"
config_path = os.path.join('data', 'config_engine.json')
with open(config_path, 'w') as config_file:
    json.dump(config_dict, config_file, indent=4)

eng = cityflow.Engine(config_path,thread_num=1)

for step in range(500):
    print(step)

    # print(step,eng.get_vehicle_count())
    eng.get_vehicle_count()
    eng.get_lane_vehicle_count()
    eng.get_lane_waiting_vehicle_count()
    eng.get_current_time()
    eng.get_vehicle_speed()

    eng.next_step()

