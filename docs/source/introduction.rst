Introduction
============

CityFlow is a multi-agent reinforcement learning environment for large scale city traffic scenario. 

Checkout these features!

- a microscopic traffic simulator which simulates the behavior of each vehicle, providing highest level detail of traffic evolution.
- support flexible definitions for road network and traffic flow
- provides friendly python interface for reinforcement learning
- **Fast!** Elaborately designed data structure and simulation algorithm with multithreading. Capable of simulating city-wide traffic. See the performance comparison with SUMO [#sumo]_.

.. figure:: https://github.com/cityflow-project/data/raw/master/docs/images/performance.png
    :align: center

    Performance comparison between CityFlow with different number of threads (1, 2, 4, 8) and SUMO. From small 1x1 grid roadnet to city-level 30x30 roadnet. Even faster when you need to interact with the simulator through python API.

See :ref:`start` to get started.

.. [#paper] `WWW 2019 Demo Paper <https://arxiv.org/abs/1905.05217>`_
.. [#sumo] `SUMO home page <https://sumo.dlr.de/index.html>`_