CityFlow
============

.. image:: https://readthedocs.org/projects/cityflow/badge/?version=latest
    :target: https://cityflow.readthedocs.io/en/latest/?badge=latest
    :alt: Documentation Status

.. image:: https://dev.azure.com/CityFlow/CityFlow/_apis/build/status/cityflow-project.CityFlow?branchName=master
    :target: https://dev.azure.com/CityFlow/CityFlow/_build/latest?definitionId=2&branchName=master
    :alt: Build Status

CityFlow is a multi-agent reinforcement learning environment for large-scale city traffic scenario.

Checkout these features!

- A microscopic traffic simulator which simulates the behavior of each vehicle, providing highest level detail of traffic evolution.
- Supports flexible definitions for road network and traffic flow
- Provides friendly python interface for reinforcement learning
- **Fast!** Elaborately designed data structure and simulation algorithm with multithreading. Capable of simulating city-wide traffic. See the performance comparison with SUMO [#sumo]_.

.. figure:: https://user-images.githubusercontent.com/44251346/54403537-5ce16b00-470b-11e9-928d-76c8ba0ab463.png
    :align: center
    :alt: performance compared with SUMO

    Performance comparison between CityFlow with different number of threads (1, 2, 4, 8) and SUMO. From small 1x1 grid roadnet to city-level 30x30 roadnet. Even faster when you need to interact with the simulator through python API.

Screencast
----------

.. figure:: https://user-images.githubusercontent.com/44251346/62375390-c9e98600-b570-11e9-8808-e13dbe776f1e.gif
    :align: center
    :alt: demo

Featured Research and Projects Using CityFlow
---------------------------------------------
- `PressLight: Learning Max Pressure Control to Coordinate Traffic Signals in Arterial Network (KDD 2019) <http://personal.psu.edu/hzw77/publications/presslight-kdd19.pdf>`_
- `CoLight: Learning Network-level Cooperation for Traffic Signal Control <https://arxiv.org/abs/1905.05717>`_
- `Traffic Signal Control Benchmark <https://traffic-signal-control.github.io/>`_
- `TSCC2050: A Traffic Signal Control Game by Tianrang Intelligence (in Chinese) <http://game.tscc2050.com/>`_ [#tianrang]_

Links
-----

- `WWW 2019 Demo Paper <https://arxiv.org/abs/1905.05217>`_
- `Home Page <http://cityflow-project.github.io/>`_
- `Documentation and Quick Start <https://cityflow.readthedocs.io/en/latest/>`_
- `Docker <https://hub.docker.com/r/cityflowproject/cityflow>`_


.. [#sumo] `SUMO home page <https://sumo.dlr.de/index.html>`_
.. [#tianrang] `Tianrang Intelligence home page <https://www.tianrang.com/>`_
