# cozmo-tools

## Tools for programming Anki's Cozmo robot.

* __simple_cli__ provides a _Command Line Interface_ for the Cozmo SDK
so you can evaluate expressions in the context of an active SDK connection
to a robot. Run it by typing: `python3 simple_cli`

* __event_monitor.py__ provides Cozmo event monitoring.
Type `monitor(robot)` to start monitoring.  See doc for more options.

* __world_viewer.py__ is an OpenGL viewer for Cozmo's world map.
Requires the PyOpenGL and PyOpenGL_accelerate packages (from pip), and
freeglut3. (On Linux you can install freeglut3 via apt-get, and on mac osx with brew).
 Run it by typing: `viewer(robot)`; type 'h' in the graphics window for a list of
commands. May not work on Macs due to Tkinter brokenness.

* __cozmo_fsm__ is a Finite State Machine package for Cozmo programming.

* __genfsm__ is a preprocessor that converts .fsm files written in
the cozmo_fsm notation to .py files that are ready to run.

__Note__: you can install all the python dependencies by running `pip3 install -r requirements.txt`
