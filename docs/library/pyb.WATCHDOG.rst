.. _pyb.WATCHDOG:

class WATCHDOG -- variable watchdog timer
=========================================

WATCHDOG implements the standard STM32 WATCHDOG.
Once enabled it cannot be halted except by a reset.
Even a soft-reboot won't stop the watchdog.

WATCHDOG objects can be created and initialised using::

    from pyb import WATCHDOG

    WATCHDOG = WATCHDOG(1000000)          # init with given timeout in microseconds

The WATCHDOG object should be triggered before the timer terminates using::

    WATCHDOG.trig()     # resets the watchdog timer to the value at initialization

Constructors
------------

.. only:: port_mokpy

    .. class:: pyb.WATCHDOG(timeout)
    
       Construct a WATCHDOG object with timeout microseconds.

Methods
-------

.. only:: port_mokpy

    .. method:: WATCHDOG.trig()
    
       Reset the WATCHDOG timer to it's initial value.

Constants
---------

   None.
