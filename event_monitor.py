'''

Event Monitor Tool for Cozmo
============================

Usage:
    monitor(robot) to monitor all event types in the dispatch table
    monitor(robot, Event) to monitor a specific type of event

    unmonitor(robot[, Event]) to turn off monitoring

Author: David S. Touretzky, Carnegie Mellon University
=====

ChangeLog
=========

*   Renaming and more face support
        Dave Touretzky
            - Renamed module to event_monitor
            - Renamed monitor_on/off to monitor/unmonitor
            - Added monitor_face to handle face events

*   Created
        Dave Touretzky

'''

import cozmo

def print_prefix(evt):
    print('-> ', evt.event_name, ' ', sep='', end='')

def print_object(obj):
    if isinstance(obj,cozmo.objects.LightCube):
        cube_id = next(k for k,v in robot.world.light_cubes.items() if v==obj)
        print('LightCube-',cube_id,sep='',end='')
    else:
        r = obj.__repr__()
        r = r[1:r.index(" ")]
        print(r,end='')

def monitor_generic(evt, **args):
    print_prefix(evt)
    if 'behavior_type_name' in args:
        print(args['behavior_type_name'], '', end='')
        print(' ', end='')
    if 'obj' in args:
        print_object(args['obj'])
        print(' ', end='')
    print(set(args.keys()))

def monitor_EvtActionCompleted(evt, action, state, failure_code, failure_reason, **args):
    print_prefix(evt)
    print_object(action)
    print('',state,end='')
    if not failure_code is None:
        print('',failure_code,failure_reason,end='')
    print()

def monitor_EvtObjectTapped(evt, *, obj, tap_count, tap_duration, tap_intensity, **kwargs):
    print_prefix(evt)
    print_object(obj)
    print(' count=', tap_count, \
          ' duration=', tap_duration, ' intensity=', tap_intensity, sep='')

def monitor_face(evt, face, **args):
    print_prefix(evt)
    print("'", face.name, "' face_id=", face.face_id, sep='')

dispatch_table = {                                                    \
  cozmo.action.EvtActionStarted        : monitor_generic,             \
  cozmo.action.EvtActionCompleted      : monitor_EvtActionCompleted,  \
  cozmo.behavior.EvtBehaviorStarted    : monitor_generic,             \
  cozmo.behavior.EvtBehaviorStopped    : monitor_generic,             \
  cozmo.anim.EvtAnimationsLoaded       : monitor_generic,             \
  cozmo.anim.EvtAnimationCompleted     : monitor_generic,             \
  cozmo.objects.EvtObjectAvailable     : monitor_generic,             \
  cozmo.objects.EvtObjectAppeared      : monitor_generic,             \
  cozmo.objects.EvtObjectDisappeared   : monitor_generic,             \
  cozmo.objects.EvtObjectObserved      : monitor_generic,             \
  cozmo.objects.EvtObjectTapped        : monitor_EvtObjectTapped,     \
  cozmo.faces.EvtFaceAppeared          : monitor_face,                \
  cozmo.faces.EvtFaceObserved          : monitor_face,                \
  cozmo.faces.EvtFaceDisappeared       : monitor_face,                \
}

excluded_events = {    # Occur too frequently to monitor by default   \
    cozmo.objects.EvtObjectObserved,    \
    cozmo.faces.EvtFaceObserved,        \
}

def monitor(_robot, evt_class=None):
    if not isinstance(_robot, cozmo.robot.Robot):
        raise TypeError('First argument must be a Robot instance')
    if not ( evt_class is None or issubclass(evt_class, cozmo.event.Event) ):
        raise TypeError('Second argument must be an Event subclass')
    global robot
    robot = _robot
    if evt_class in dispatch_table:
        robot.add_event_handler(evt_class,dispatch_table[evt_class])
    elif not evt_class is None:
        robot.add_event_handler(evt_class,monitor_generic)
    else:
        for k,v in dispatch_table.items():
            if not k in excluded_events:
                robot.add_event_handler(k,v)

def unmonitor(_robot, evt_class=None):
    if not isinstance(_robot, cozmo.robot.Robot):
        raise TypeError('First argument must be a Robot instance')
    if not ( evt_class is None or issubclass(evt_class, cozmo.event.Event) ):
        raise TypeError('Second argument must be an Event subclass')
    global robot
    robot = _robot
    try:
        if evt_class in dispatch_table:
            robot.remove_event_handler(evt_class,dispatch_table[evt_class])
        elif not evt_class is None:
            robot.remove_event_handler(evt_class,monitor_generic)
        else:
            for k,v in dispatch_table.items():
                robot.remove_event_handler(k,v)
    except Exception:
        pass

