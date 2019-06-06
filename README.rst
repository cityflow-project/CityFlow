CityFlow
============

.. figure:: https://readthedocs.org/projects/cityflow/badge/?version=latest
    :target: https://cityflow.readthedocs.io/en/latest/?badge=latest
    :alt: Documentation Status


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

.. figure:: https://user-images.githubusercontent.com/44251346/57910296-8a879380-78b7-11e9-849e-b8544be3e312.gif
    :align: center
    :alt: demo

Links
-----

- `WWW 2019 Demo Paper <https://arxiv.org/abs/1905.05217>`_
- `Home Page <http://cityflow-project.github.io/>`_
- `Documentation and Quick Start <https://cityflow.readthedocs.io/en/latest/>`_
- `Docker <https://hub.docker.com/r/cityflowproject/cityflow>`_


.. [#sumo] `SUMO home page <https://sumo.dlr.de/index.html>`_
