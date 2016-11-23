# cozmo-tools
Tools for programming Anki's Cozmo robot.

simple_cli.py provides a Command Line Interface for the Cozmo SDK so you can evaluate
    expressions in the context of an active SDK connection to a robot.
    Run it by typing:  python3 simple_cli

event_monitor.py provides Cozmo event monitoring.
    Type monitor(robot) to start monitoring.  See doc for more options.

world_viewer.py is an OpenGL viewer for Cozmo's world map.
    Requires the PyOpenGL package.
    Run it by typing: viewer(robot)

