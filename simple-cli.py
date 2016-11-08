#!/usr/bin/python3 -i
'''Simple Cozmo CLI

A simple (and crude) Command Line Interface for Cozmo.
Version 3: synchronous instead of async calls. Not thread-safe
   but works much more smoothly.
If you exit the CLI loop, first close the TK window and then
type reconnect() to re-establish the CLI.

Author: David S. Touretzky, Carnegie Mellon University
'''

import sys
import code
import traceback
import time
import cozmo
from cozmo.util import *

def run(sdk_conn):
  robot = sdk_conn.wait_for_robot()
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
    if line[0:7] == 'import ' or line[0:5] == 'from ' or line[0:7] == 'global ':
      # Can't use assignment to capture a return value, so None.
      ans = None
    elif line[0:6] == 'await ':
      do_await = True
      line = 'global ans; ans=' + line[6:]
    else:
      line = 'global ans; ans=' + line
    try:
      exec(line)
      if do_await:
        print("Can't use await outside of an async def.")
        ans = None # ans = await ans
      if not ans is None:
        print(ans,end='\n\n')
    except Exception as e:
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
    cozmo.connect_with_tkviewer(run)
  except cozmo.ConnectionError as e:
    sys.exit("A connection error occurred: %s" % e)

if __name__ == '__main__':
  reconnect()
