.. _install:

Installation Guide
==================

Docker
------

The easiest way to use CityFlow is via docker.

.. code-block:: shell
    
    docker pull cityflowproject/cityflow:v0.1

This will create docker image ``cityflow:v0.1``.

.. code-block:: shell
    
    docker run -it cityflowproject/cityflow:v0.1

Create and start a container, CityFlow is out-of-the-box along with miniconda with python3.6.

.. code-block:: python
    
    import cityflow
    eng = cityflow.Engine

Build From Source
-----------------

If you want to get nightly version of CityFlow or running on native system, you can build CityFlow from source. Currently, we only support building on Unix systems. This guide is based on Ubuntu 16.04.

CityFlow has little dependencies, so building from source is not scary.

1. Check that you have python 3.6 installed. Other version of python might work, however, we only tested on python with version >= 3.6.


2. Install cpp dependencies

.. code-block:: shell
    
    apt update && apt-get install -y build-essential libboost-all-dev cmake

3. Clone CityFlow project from github.

.. code-block:: shell
    
    git clone --recursive git@github.com:cityflow-project/CityFlow.git
    
Notice that CityFlow uses pybind11 to integrate C++ code with python, the repo has pybind11 as a submodule, please use ``--recursive`` to clone all codes.

4. Go to CityFlow project's root directory and run 

.. code-block:: shell
    
    pip install .

5. Wait for installation to complete and CityFlow should be successfully installed.

.. code-block:: python
    
    import cityflow
    eng = cityflow.Engine