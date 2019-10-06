.. _replay:

Replay
======

Start
------

1. enter the ``frontend`` folder and open ``index.html`` in your browser.

2. choose the roadnet log file (as defined by ``roadnetLogFile`` field in the config file, **not 'roadnetFile'**) and wait for it to be loaded. When it has finished loading, there will be a message shown in the info box.

3. choose the replay file (as defined by ``replayLogFile`` field in the config file) .

4. choose the chart data file (optional, see section *Chart* below).

5. press ``Start`` button to start the replay.

Control
-------

- Use the mouse to navigate. Dragging and mouse wheel zooming are supported.

- Move the slider in Control Box to adjust the replay speed. You can also press ``1`` on keyboard to slow down or ``2`` to speed up.

- Press ``Pause`` button in Control Box to pause/resume. You can also double-click on the map to pause and resume.

- Press ``[`` or ``]`` on keyboard to take a step backward or forward.

- To restart the replay, just press ``Start`` button again.

- The ``debug`` option enables displaying the ID of vehicles, roads and intersections during a mouse hover. **This will cause a slower replaying**, so we suggest using it only for debugging purposes.

Chart
------

The player supports showing the change of different metrics in a chart simultaneously with the replay process.

To provide required data, a log file in a format as shown below is needed:

.. code-block::

  title
  0.3            0.4              0.1          ...(step1)
  0.5            0.2              0.2          ...(step2)
  ...(metric1)   ...(metric2)     ...(metric3)

The first line is the title of the chart.

Each row stands for a time step and each column stands for a specific metric.
For example, to track vehicle numbers of three crossroads respectively, we need three columns and each column stands for the vehicle number of a certain crossroads.

In one row, numbers are separated by one or more spaces or tabs.

The numbers in one column will be shown as points connected by one line in the chart.

.. note::
  Make sure that each row is corresponding with the right time step.

Notes
------

- To get the example replay files, run ``download_replay.py`` under ``frontend`` folder.

- If you create a new Engine object with same ``replayLogFile``, it will clear the old replay file first

- Using ``eng.reset()`` won't clear old replays, it will append newly generated replay to the end of ``replayLogFile``

- You can change ``replayLogFile`` during runtime using ``set_replay_file``, see :ref:`set-replay-file`
