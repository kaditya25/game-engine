# Software Patterns
This file serves as informal documentation of the various software patterns used
within GameEngine. 

## ROS
ROS has a tendency to spread its tendrils throughout a code base.  To manage
ROS and keep software components weakly linked, almost all ROS code is limited
to the (ros)[../src/ros] directory.

### Publishers and Subscribers
Any new publishers and subscribers should be wrapped in a new class instance.
Classes may choose to include an Options struct to configure specific traits of
the publishers/subscribers.

### PublisherGuard
The GameEngine utilizes many threads. It is possible that multiple threads may
attempt to simultaneously publish via the same ROS publisher instance. It is
unclear to the original author whether ros::publisher::publish() is thread-safe.
To ensure this, a PublisherGuard is utilized. The PublisherGuard protects an
instance of a ros::publisher with a mutex and allows only one publish() to be
called at a time.

## Dependency Injection
The GameEngine was designed for Dependency Injection. Object instances should be
created up front and injected as references/pointers into objects that need
access to this data. The up-front cost is managed in a larger main() function.

## Patterns
### Guard Pattern
The Guard Pattern is designed to enable thread-safe read/write access to a
wrapped object instance. For example, the PublisherGuard protects a
ros::publisher instance with a mutex to ensure that multiple threads are not
writing to the publisher simultaneously.

### Warden Pattern
The Warden Pattern maintains a container of Guards are provides thread-safe
read, write, and await interfaces. The Warden may be interpreted as a
thread-safe buffer with convenient interfaces. Components with a reference to
the Warden may query it for access to a specific underlying guard. 

### Watchdog Pattern
The Watchdog Pattern encapsulates an if-then check within an individual thread.
For example, the QuadStateWatchdog periodically checks to see if the
QuadcopterState has violated one of the prescribed constraints. If so, it
triggers a shutdown of the system. Watchdogs tend to be very lightweight and
were preferred over increasing the complexity/responsibilities of other objects.