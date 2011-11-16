# Thread-safe AX3500 Motor Controller Interface

## Overview

This program allows software to interface with the Roboteq AX3500 motor controller. The motor controller is presented in a completely thread-safe way. Serial ports are synchronous devices, and it is important that this behavior be monitored so that multiple clients can utilize the same piece of hardware at once.

The methods in this program are either commands to be written to the motor controller or requests for data from the motor controller. The multi-threaded nature allows writes to be asynchronous and reads to be synchronous. In other words, the time it takes for a program to send a command to the motor controller is essentally zero. On the other hand, reads take a finite amount of time (usually on the order of milliseconds). The benefit of synchronous reads is that they always return with the data requested, as opposed to an ugly callback framework required by async reads.

Another important feature of the thread-safe nature means that commands can be prioritized over queries. For example, a thread in charge of logging distance travelled by the motors can poll the controller at 100% utilization. Normally this would exclude all other communications with the device. However, if another thread attempts to send a command to the controller, the command will take priority over the queries and execute without hesitation.

This program is a small piece of a larger puzzle. Now that thread-safe hardware access is gauranteed, the interface can be presented to external clients. Coming up soon is a novel way of controlling the motor controller.

## Related Work

1. Roboteq AX2550 Motor Controller by the Auburn Automow team
   <https://github.com/Auburn-Automow/au_automow_drivers>
