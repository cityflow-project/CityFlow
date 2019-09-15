.. _replay:

Replay
======

Start
------

1. enter the ``frontend`` folder and download required libraries

.. code-block:: shell

    python init.py

2. open the ``index.html`` under ``frontend`` folder in your browser.

3. choose the roadnet log file (as defined by ``roadnetLogFile`` field in the config file, **not 'roadnetFile'**) and wait for it to be loaded. When it has finished loading, there will be a message shown in the info box.

4. choose the replay file (as defined by ``replayLogFile`` field in the config file) .

5. press ``Start`` button to start the replay.

Control
-------

- Use the mouse to navigate. Dragging and mouse wheel zooming are supported.

- Move the slider in Control Box to adjust the replay speed. You can also press ``1`` on keyboard to slow down or ``2`` to speed up.

- Press ``Pause`` button in Control Box to pause/resume. You can also double-click on the map to pause and resume.

- Press ``[`` or ``]`` on keyboard to take a step backward or forward.

- To restart the replay, just press ``Start`` button again.

Notes
------

- To get the example replay files, run ``download_replay.py`` under ``frontend`` folder.

- If you create a new Engine object with same ``replayLogFile``, it will clear the old replay file first

- Using ``eng.reset()`` won't clear old replays, it will append newly generated replay to the end of ``replayLogFile``

- You can change ``replayLogFile`` during runtime using ``set_replay_file``, see :ref:`set-replay-file`
