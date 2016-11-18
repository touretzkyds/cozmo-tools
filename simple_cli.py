#!/usr/bin/python3 -i
'''

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



Author:     David S. Touretzky, Carnegie Mellon University
=======

Changelog
=========

*   Added event moniitoring
        Dave Touretzky
            - Imports monitor and unmonitor functions from event_monitor.py
            - Also restored the -i switch on line 1 to prevent unwanted exits

*   TCP socket interface + 'exit' command + optional 'tk' mode
        Real Ouellet, ABB inc.
            - TCP socket listens on port 4242
            - Added new command 'exit' to close socket cleanly
            - Removed the '-i' for a better python exit
            - Do not start in tk mode if an argument is passed

*   Synchronous instead of async calls. 
        Dave Touretzky
            - Not thread-safe but works much more smoothly.

'''

# All that stuff we really need in order to get going.
import threading
import socket
import socketserver
import sys
import code
import traceback
import time
import cozmo
from cozmo.util import *

# This should be in your '.pythonstartup' file, but I put it also here just in case...
import readline 
import rlcompleter 
import atexit 
import os 

from event_monitor import monitor, unmonitor

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


# 'robot' might not have to be global. Feel free to modify this, 
# but remember that it must still be available to the TCP side.
global res, ans, robot;
res=0

# Helper class for TCP requests
# The TCP CLI is handled here.
# This code was written by a Python ignoramus... Feel free to make it better
# in any way!
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



def run(sdk_conn):
    global robot
    robot = sdk_conn.wait_for_robot()
    if len(sys.argv) <= 1:
        time.sleep(1) # allow time for Tk to set up the window
    cli_loop(robot)

def cli_loop(robot):
    console = code.InteractiveConsole()
    global ans
    while True:
        line = ''
        while line == '':
           line = console.raw_input('C> ').strip()
        do_await = False
        if line[0:7] == 'import ' or line[0:5] == 'from ' or line[0:7] == 'global ' \
           or line[0:4] == 'del ':
            # Can't use assignment to capture a return value, so None.
            ans = None
        elif line[0:6] == 'await ':
            do_await = True
            line = 'global ans, res; ans=' + line[6:]
        elif line[0:5] == 'exit':
            # Clean up
            server.shutdown()
            server.socket.close()
            sys.exit()
        else:
            line = 'global ans, res; ans=' + line
        try:
            exec(line)
            if do_await:
                print("Can't use await outside of an async def.")
                ans = None # ans = await ans
            if not ans is None:
                print(ans,end='\n\n')
        except Exception:
            traceback.print_exc()
            print()

global logging_is_setup
logging_is_setup = False

def reconnect():
    global logging_is_setup
    if not logging_is_setup:
        cozmo.setup_basic_logging()
        logging_is_setup = True
    try:
        if len(sys.argv) <= 1:
            cozmo.connect_with_tkviewer(run)
        else:
            cozmo.connect(run)

    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)



if __name__ == '__main__':
    
    # Start the TCP CLI server
    address = ('localhost', 4242)  # let the kernel assign a port
    server = ThreadedExecServer(address,
                                ThreadedExecRequestHandler)
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ip, port = server.server_address  # what port was assigned?
    t = threading.Thread(target=server.serve_forever)
    t.setDaemon(True)  # don't hang on exit
    t.start()
    print('Server loop running on port', port)
    
    # Connect to Cozmo and start the keyboard CLI
    reconnect()

