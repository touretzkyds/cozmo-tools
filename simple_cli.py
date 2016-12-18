#!/usr/bin/python3 -i
"""

Simple Cozmo CLI
================
A simple (and crude) Command Line Interface for Cozmo.
If you exit the CLI loop, first close the TK window and then
type reconnect() to re-establish the CLI.

Strings sent to TCP port 4242 (including '\n' characters) will be 
executed in python's context. Globals can be exchanged with the 
keyboard CLI environment.  This can be used in a Node-Red 'Template' node.

Node-Red:  http://nodered.org

With both node-red and this CLI started on the same device (Arch Linux 
on a rooted Android phone, for example), then there is no more need for 
interactions with a python shell on the phone or on a computer. All 
interactions can be done via the Node-Red web server application 
exposed by the phone on Cozmo's internal network. Multiple machines
can be simultaneously connected to Cozmo's WiFi access point, thus
sharing control of Cozmo via Node-Red.

The variable 'res' is defined as a default global, and its value
is returned to the TCP client after command execution. The TCP CLI 
data packet can spread over more than one line. For example, we can have
the following text in a Node-Red 'template' node connected to a TCP
node and triggered by some injection node:

    # Example of a Node-Red 'template' node passed to the TCP node
    global q_turn, do_square

    def q_turn():
        robot.drive_straight(distance_mm(20),speed_mmps(50)).wait_for_completed()
        robot.turn_in_place(degrees(90)).wait_for_completed()

    def do_square():
        q_turn(); q_turn(); q_turn(); q_turn()
        return({{payload}})

    res=do_square() 

When triggered by the injection node's message, this code will make Cozmo 
perform a square dance, and return the trigger's payload (an echo) in
the 'res' object, which can then used in the keyboard CLI.
The functions do_square and q_turn also become available to the
keyboard CLI side.


*********

Available CLI commands:

    (python code)
        Executable statements are directly passed to python's interpreter.  Some magic global variables:
            ans:    results from the keyboard CLI can be returned in this global object
            res:    results from the TCP server CLI can be returned in this global object
        
    exit    
        Quit all the running activities

    monitor(robot[, Event])
        Monitor all event types in the dispatch table, or a specified type of Event
    
    unmonitor(robot[, Event])
        Turn off monitoring

    viewer(robot)
        Start a 3D real-time model of the World around Cozmo.  See the
        world_viewer documentation for a list of commands.

    runfsm(module_name)
        Imports or reloads a state machine module and runs the
        state machine.

    tracefsm(trace_level)
        Sets the FSM tracing level (0-9). With no argument, returns
        the current level.

*********



Author:     David S. Touretzky, Carnegie Mellon University
=======

Changelog
=========
* 12/13/2016: More cozmo_fsm development
    Dave Touretzky
        - Changed TRACE to tracefsm
        - Rewrote runfsm to match changes in cozmo_fsm

* 12/11/2016: Imported TRACE from cozmo_fsm.trace
    Dave Touretzky
        - Allows user to set TRACE.trace_level before calling runfsm

* 12/10/2016:  Add runfsm('myfsm') to re-load and run a state machine.
    Real Ouellet and Dave Touretzky
        - Imports the module if it's not already loaded, else reloads it.
        - Then does myfsm.run(robot) to run the state machine.

* 12/04/2016:  Provide useful user-visible variables.
    Dave Touretzky
        - Added charger, cube1-cube3, light_cubes, and world.
        - Announce these variables on start-up.
        - Removed bogus call to world_viewer.init()

*   Healthier exit code to allow quitting without drama
    Real Ouellet
        - Push Cozmo's code in a daemon thread
        - Use 'return' genereously
        - Forbid exiting from the GL window with 'ESC': it messes up the host console
        - Add some doc

* Added OS test because Tkinter breaks console input
    Dave Touretzky
        - Use sys.stdin.readline() on Macs as a workaround, although
          this means we don't get command line editing.

* Added world_viewer
    Dave Touretzky
        - Imports the world_viewer module.  Do viewer(robot) to start it.

* Added event moniitoring
    Dave Touretzky
        - Imports monitor and unmonitor functions from event_monitor.py
        - Also restored the -i switch on line 1 to prevent unwanted exits

* TCP socket interface + 'exit' command + optional 'tk' mode
    Real Ouellet, ABB inc.
        - TCP socket listens on port 4242
        - Added new command 'exit' to close socket cleanly
        - Removed the '-i' for a better python exit
        - Do not start in tk mode if an argument is passed

* Synchronous instead of async calls. 
    Dave Touretzky
        - Not thread-safe but works much more smoothly.

"""

# All that stuff we really need in order to get going.
import threading
import socket
import socketserver
import signal
import sys
import platform
import code
import traceback
import time
import os 
from importlib import __import__, reload

# This should be in your '.pythonstartup' file, but I put it also here just in case...
import readline 
import rlcompleter 
import atexit 

import cozmo
from cozmo.util import *

from event_monitor import monitor, unmonitor

try:
    import cozmo_fsm
    from cozmo_fsm import tracefsm
except: pass

try:
    import world_viewer
    from world_viewer import viewer
except: pass

# tab completion 
readline.parse_and_bind('tab: complete') 
# history file 
histfile = os.path.join(os.environ['HOME'], '.pythonhistory') 
try: 
    readline.read_history_file(histfile) 
except IOError: 
    pass 
atexit.register(readline.write_history_file, histfile) 
del os, histfile, readline, rlcompleter

os_version = platform.system()
del platform


res = 0
ans = None
RUNNING = True

# Helper class for TCP requests
# The TCP CLI is handled here.
class ThreadedExecRequestHandler( socketserver.BaseRequestHandler ):
    def handle(self):
        global res
         
        try:
            exec('global res\n' + self.request.recv(1024).strip().decode('utf-8') )
        except Exception as e:
            traceback.print_exc()
            print()
        self.request.send(bytes(str(res).encode('utf-8')))
        return

# Another TCP helper class
class ThreadedExecServer(socketserver.ThreadingMixIn,
                         socketserver.TCPServer,
                         ):
    pass

running_fsm = None

def runfsm(module_name, running_modules=dict()):
    """runfsm('modname') reloads that module and calls its setup_fsm() function."""
    global cozmo_fsm, running_fsm
    try:
        reload(running_modules[module_name])
    except:
        running_modules[module_name] = __import__(module_name)
    # Call the parent node class's constructor; it must match the module name
    parent_node = running_modules[module_name].__getattribute__(module_name)
    running_fsm = parent_node()
    robot.loop.call_soon(running_fsm.start)
    return running_fsm


def run(sdk_conn):
    global robot
    robot = sdk_conn.wait_for_robot()
    if len(sys.argv) <= 1:
        time.sleep(1.5) # allow time for Tk to set up the window
    try:
        robot.erouter = cozmo_fsm.evbase.EventRouter()
        robot.erouter.robot = robot
        cozmo_fsm.evbase.robot_for_loading = robot
    except: pass
    cli_loop(robot)

def cli_loop(robot):
    global ans, RUNNING
    cli_loop.charger_warned = False

    world = robot.world
    light_cubes = world.light_cubes
    cube1 = light_cubes[cozmo.objects.LightCube1Id]
    cube2 = light_cubes[cozmo.objects.LightCube2Id]
    cube3 = light_cubes[cozmo.objects.LightCube3Id]
    charger = robot.world.charger
    exec("print();print('Variables:', dir())")
    cli_loop._console = code.InteractiveConsole()

    while True:
        if RUNNING == False:
            return
        cli_loop._line = ''
        while cli_loop._line == '':
            if robot.is_on_charger:
                if not cli_loop.charger_warned:
                    print("On charger. Type robot.drive_off_charger_contacts() to enable motion.\n")
                    cli_loop.charger_warned = True
            else:
                cli_loop.charger_warned = False
            try:
                if os_version == 'Darwin':   # Tkinter breaks console on Macs
                    print('c> ', end='')
                    cli_loop._line = sys.stdin.readline().strip()
                else:
                    cli_loop._line = cli_loop._console.raw_input('C> ').strip()
            except EOFError:
                print("EOF.\nType 'exit' to exit.\n")
                continue
        cli_loop._do_await = False
        if cli_loop._line[0:7] == 'import ' or cli_loop._line[0:5] == 'from '  or \
               cli_loop._line[0:7] == 'global ' or cli_loop._line[0:4] == 'del '   or \
               cli_loop._line[0:4] == 'for ' or \
               cli_loop._line[0:4] == 'def '    or cli_loop._line[0:6] == 'async ' :
            # Can't use assignment to capture a return value, so None.
            ans = None
        elif cli_loop._line[0:6] == 'await ':
            cli_loop._do_await = True
            cli_loop._line = 'global ans, res; ans=' + cli_loop._line[6:]
        elif cli_loop._line[0:5] == 'exit':
            # Clean up
            try:
                world_viewer.exited = True
            except: pass
            server.shutdown()
            server.socket.close()
            RUNNING=False
        else:
            cli_loop._line = 'global ans, res; ans=' + cli_loop._line
        try:
            exec(cli_loop._line)
            if cli_loop._do_await:
                print("Can't use await outside of an async def.")
                ans = None # ans = await ans
            if not ans is None:
                print(ans,end='\n\n')
        except KeyboardInterrupt: raise
        except Exception:
            traceback.print_exc()
            print()


logging_is_setup = False

def reconnect():
    global logging_is_setup
    if not logging_is_setup:
        cozmo.setup_basic_logging()
        logging_is_setup = True
    cozmo.robot.Robot.drive_off_charger_on_connect = False
    try:
        if len(sys.argv) <= 1:
            cozmo.connect_with_tkviewer(run)
        else:
            cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)


def signal_handler(signal, frame):
        print('Please use "exit" to quit.')

if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    # Fair warning
    if os_version == 'Darwin':
        print("WARNING!! This program does not really work on OSX, because Not Linux. You have been warned.")
    
    # Start the TCP CLI server
    address = ('localhost', 4242)  # let the kernel assign a port
    server = ThreadedExecServer(address,
                                ThreadedExecRequestHandler)
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ip, port = server.server_address  # what port was assigned?
    CLI_server = threading.Thread(target=server.serve_forever)
    CLI_server.setDaemon(True)  # don't hang on exit
    CLI_server.start()
    print('CLI server loop running on port', port)
    
    # Start the Cozmo thread:  connect to Cozmo and start the keyboard CLI
    COZMO_server = threading.Thread(target=reconnect)
    COZMO_server.setDaemon(True)  # don't hang on exit
    COZMO_server.start()
    
    # Start doing nothing until not anymore ...
    while RUNNING:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            if running_fsm:
                print('\nKeyboardInterrupt: stopping', running_fsm.name)
                running_fsm.stop()
                running_fsm = None
            else:
                print("\nKeyboardInterrupt. Type 'exit' to exit.")
            print('\nC> ',end='')
