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

``get_vehicle_speed()``:

- Get speed of each vehicle
- Return a ``dict`` with vehicle id as key and corresponding speed as value.

``get_vehicle_distance()``:

- Get distance travelled on current lane of each vehicle.
- Return a ``dict`` with vehicle id as key and corresponding distance as value.

``get_current_time()``:

- Get simulation time (in seconds)
- Return a ``double``


Control API
-----------

``set_tl_phase(intersection_id, phase_id)``: 

- Set the phase of traffic light of ``intersection_id`` to ``phase_id``. Only works when ``rlTrafficLight`` is set to ``true``.
- The ``intersection_id`` should be defined in ``roadnetFile``
- ``phase_id`` is the index of phase in array ``"lightphases"``, defined in ``roadnetFile``.

``reset()``: 

- Reset the simulation (clear all vehicles and set simulation time back to zero)
- Notice that this does not reset random state, so each simulation after reset may be different.
- This does not clear old replays, instead, it appends new replays to ``replayLogFile``.