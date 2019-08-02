.. _flow:

Flow File Format
===================
    
Flow file defines the traffic flow. Each flow contains following field:

- ``vehicle``: defines the parameter of vehicle.
    - length: length of the vehicle
    - width: width of the vehicle
    - maxPosAcc: maximum acceleration (in m/s)
    - maxNegAcc: maximum deceleration (in m/s)
    - usualPosAcc: usual acceleration (in m/s)
    - usualNegAcc: usual deceleration (in m/s)
    - minGap: minimum acceptable gap with leading vehicle (in meter)
    - maxSpeed: maximum cruising speed (in m/s)
    - headwayTime: desired headway time (in seconds) with leading vehicle, keep *current speed \* headwayTime* gap.
- ``route``: defines the route, all vehicles of this flow will follow the route. Specify the source and the destination, optionally some anchor points and the router will connect them with shortest paths automatically.
- ``interval``: defines the interval of consecutive vehicles (in seconds). If the interval is too small, vehicles may not be able to enter the road due to blockage, it will be held and let go once there are enough space.
- ``startTime``, ``endTime``: Flow will generate vehicles between time [startTime, endTime] (in seconds), including ``startTime`` and ``endTime``.

.. note::
  Runnable sample flow files can be found in ``examples`` folder.
