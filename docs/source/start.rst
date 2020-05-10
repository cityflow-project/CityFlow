.. _start:

Quick Start
===========

Installation
------------

If you have not installed CityFlow yet, :ref:`install` is a simple guide for installation.


Create Engine
-------------

.. code-block:: python
    
    import cityflow
    eng = cityflow.Engine(config_path, thread_num=1)


- ``config_path``: path for config file.
- ``thread_num``: number of threads.

Arguments In Config File
^^^^^^^^^^^^^^^^^^^^^^^^
- ``interval``: time of each simulation step (in seconds). An ``interval`` of 0.5 means for each simulation step, the system will move 0.5 seconds forward. For example, if a car is at location 0 with speed 10m/s, after one simulation step, it will move to location 10*0.5=5. 
- ``seed``: random seed.
- ``dir``: root directory, all file path will be relative to this directory.
- ``roadnetFile``: path for roadnet file.
- ``flowFile``: path for flow file.
- ``rlTrafficLight``: whether to enable traffic light control through python API. If set to ``false``, default traffic light plan defined in ``roadnetFile`` will be used.
- ``saveReplay``: whether to save simulation for replay. If set to ``true``, ``roadnetLogFile`` and ``replayLogFile`` are required.
- ``roadnetLogFile``: path for roadnet replay file. This is a special roadnet file for replay, not the same as ``roadnetFile``.
- ``replayLogFile``: path for replay. This file contains vehicle positions and traffic light situation of each simulation step.
- ``laneChange``: whether to enable lane changing. The default value is 'false'.

For format of ``roadnetFile`` and ``flowFile``, please see :ref:`roadnet`, :ref:`flow`

.. note::
    Runnable sample roadnet and flow files can be found in ``examples`` folder.

You can generate grid roadnet and flow files using `tools/generate_grid_scenario.py`

For example, you can generate a 2x3 roadnet with predefined traffic light plan and a corresponding flow file with

.. code-block:: shell

    python generate_grid_scenario.py 2 3 --roadnetFile roadnet.json --flowFile flow.json --dir . --tlPlan

Sample Config File
^^^^^^^^^^^^^^^^^^^

.. note::
    Runnable sample config files can be found in ``examples`` folder.

.. code-block:: json

    {
        "interval": 1.0,
        "seed": 0,
        "dir": "data/",
        "roadnetFile": "roadnet/testcase_roadnet_3x3.json",
        "flowFile": "flow/testcase_flow_3x3.json",
        "rlTrafficLight": false,
        "saveReplay": true,
        "roadnetLogFile": "frontend/web/testcase_roadnet_3x3.json",
        "replayLogFile": "frontend/web/testcase_replay_3x3.txt"
    }

Simulation
----------

To simulate one step, simply call ``eng.next_step()``

.. code-block:: python

    eng.next_step()

Data Access API
---------------

``get_vehicle_count()``:

- Get number of total running vehicles.
- Return an ``int``

``get_vehicles(include_waiting=False)``:

- Get all vehicle ids
- Include vehicles in lane's waiting buffer if ``include_waiting=True``
- Return an ``list`` of vehicle ids

``get_lane_vehicle_count()``: 

- Get number of running vehicles on each lane.
- Return a ``dict`` with lane id as key and corresponding number as value.

``get_lane_waiting_vehicle_count()``:

- Get number of waiting vehicles on each lane. Currently, vehicles with speed less than 0.1m/s is considered as waiting.
- Return a ``dict`` with lane id as key and corresponding number as value.

``get_lane_vehicles()``:

- Get vehicle ids on each lane.
- Return a ``dict`` with lane id as key and list of vehicle id as value.

``get_vehicle_info(vehicle_id)``:

- Return a ``dict`` which contains information of the given vehicle.
- The items include:

    + ``running``: whether the vehicle is running.
    + If the vehicle is running:

        * ``speed``: The speed of the vehicle.
        * ``distance``: The distance the vehicle has travelled on the current lane or lanelink.
        * ``drivable``: The id of the current drivable(lane or lanelink)
        * ``road``: The id of the current road if the vehicle is running on a lane.
        * ``intersection``: The next intersection if the vehicle is running on a lane.
        * ``route``: A string contains ids of following roads in the vehicle's route which are separated by ``' '``.

- Note that all items are stored as ``str``.

``get_vehicle_speed()``:

- Get speed of each vehicle
- Return a ``dict`` with vehicle id as key and corresponding speed as value.

``get_vehicle_distance()``:

- Get distance travelled on current lane of each vehicle.
- Return a ``dict`` with vehicle id as key and corresponding distance as value.

``get_leader(vehicle_id)``

- Return the id of the vehicle in front of ``vehicle_id``.
- Return an empty string ``""`` when ``vehicle_id`` does not have a leader

``get_current_time()``:

- Get simulation time (in seconds)
- Return a ``double``

``get_average_travel_time()``:

- Get average travel time (in seconds)
- Return a ``double``

Control API
-----------

``set_tl_phase(intersection_id, phase_id)``: 

- Set the phase of traffic light of ``intersection_id`` to ``phase_id``. Only works when ``rlTrafficLight`` is set to ``true``.
- The ``intersection_id`` should be defined in ``roadnetFile``
- ``phase_id`` is the index of phase in array ``"lightphases"``, defined in ``roadnetFile``.

``set_vehicle_speed(vehicle_id, speed)``:

- Set the speed of ``vehicle_id`` to ``speed``.
- The vehicles have to obey fundamental rules to avoid collisions so the real speed might be different from ``speed``.

``reset(seed=False)``: 

- Reset the simulation (clear all vehicles and set simulation time back to zero)
- Reset random seed if ``seed`` is set to ``True``
- This does not clear old replays, instead, it appends new replays to ``replayLogFile``.

``snapshot()``:

- Take a snapshot of current simulation state
- This will generate an ``Archive`` object which can be loaded later
- You can save an ``Archive`` object to a file using its ``dump`` method.

``load(archive)``:

- Load an ``Archive`` object and restore simulation state

``load_from_file(path)``

- Load a snapshot file created by ``dump`` method and restore simulation state.
- The whole process of saving and loading file is like:

  .. code-block:: python

      archive = eng.snapshot() # create an archive object
      archive.dump("save.json") # if you want to save the snapshot to a file

      # do something

      eng.load(archive)
      # load 'archive' and the simulation will start from the status when 'archive'is created

      # or if you want to load from 'save.json'
      eng.load_from_file("save.json")


``set_random_seed(seed)``:

- Set seed of random generator to ``seed``

.. _set-replay-file:


``set_vehicle_route(vehicle_id, route)``:

- To change the route of a vehicle during its travelling.
- `route` is a list of road ids (doesn't include the current road)
- Return true if the route is available and can be connected.



Other API
---------

``set_replay_file(replay_file)``: 

- ``replay_file`` should be a path related to ``dir`` in config file
- Set ``replayLogFile`` to ``replay_file``, newly generated replays will be output into ``replay_file``
- This is useful when you want to look at a specific episode for debugging purposes
- This API works only when ``saveReplay`` is ``true`` in config json

``set_save_replay(open)``:

- Open or close replay saving
- Set ``open`` to False to stop replay saving
- Set ``open`` to True to start replay saving
- This API works only when ``saveReplay`` is ``true`` in config json