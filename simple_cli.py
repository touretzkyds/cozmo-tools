#!/usr/bin/python3 -i
'''Simple Cozmo CLI

A simple (and crude) Command Line Interface for Cozmo
'''

import sys
import code
import traceback
import asyncio
import cozmo

async def run(sdk_conn):
  robot = await sdk_conn.wait_for_robot()
  console = code.InteractiveConsole()
  global ans
  while True:
    line = ''
    while line == '':
      await asyncio.sleep(0.1)
      line = console.raw_input('C> ').strip()
    await asyncio.sleep(0.1)
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
        ans = await ans
      if not ans is None:
        print(ans,end='\n\n')
    except Exception as e:
      traceback.print_exc()
      print()

def reconnect():
  cozmo.setup_basic_logging()
  try:
    cozmo.connect_with_tkviewer(run)
  except cozmo.ConnectionError as e:
    sys.exit("A connection error occurred: %s" % e)

if __name__ == '__main__':
  reconnect()
