# cozmo-tools

See the INSTALL.txt file for installation instructions.

For a radically different approach to Cozmo programming more suited to beginners, try Calypso at https://Calypso.software

## Tools for programming Anki's Cozmo robot via the Python SDK.

* __simple_cli__ provides a _Command Line Interface_ for the Cozmo SDK
so you can evaluate expressions in the context of an active SDK connection
to a robot. It also provides a variety of visualization tools, such as a
camera viewer, worldmap viewer, particle viewer, and path viewer.
Run it by typing: `python3 simple_cli`

* __cozmo_fsm__ is a Finite State Machine package for Cozmo programming.

* __genfsm__ is a preprocessor that converts .fsm files written in
the cozmo_fsm notation to .py files that are ready to run.

__Note__: you can install most of the Python dependencies by simply running `pip3 install -r requirements.txt`,
but see the INSTALL.txt file for some exceptions.
