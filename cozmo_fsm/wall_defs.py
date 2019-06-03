from .worldmap import *

def make_walls():

    # ~12 inch walls

    w1 = WallSpec(length=600, height=190, door_width = 77, door_height=110,
                  marker_specs={
                      'Aruco-1' : (+1, ( 65., 50.)),
                      'Aruco-2' : (+1, (153.,150.)),
                      'Aruco-3' : (+1, (241., 50.)),
                      'Aruco-4' : (+1, (359., 50.)),
                      'Aruco-5' : (+1, (447.,150.)),
                      'Aruco-6' : (+1, (535., 50.)),
                      'Aruco-12' : (-1, ( 65., 50.)),
                      'Aruco-11' : (-1, (153.,150.)),
                      'Aruco-10' : (-1, (241., 50.)),
                      'Aruco-9' : (-1, (359., 50.)),
                      'Aruco-8' : (-1, (447.,150.)),
                      'Aruco-7' : (-1, (535., 50.))
                  },
                  doorways = [ (153., 77.), (447., 77.) ],
                  door_ids = [ (2, 8), (5, 11) ])

    w13 = WallSpec(length=600, height=190, door_width = 77, door_height=110,
                   marker_specs={
                       'Aruco-13' : (+1, ( 65., 50.)),
                       'Aruco-14' : (+1, (153.,150.)),
                       'Aruco-15' : (+1, (241., 50.)),
                       'Aruco-16' : (+1, (359., 50.)),
                       'Aruco-17' : (+1, (447.,150.)),
                       'Aruco-18' : (+1, (535., 50.)),
                       'Aruco-24' : (-1, ( 65., 50.)),
                       'Aruco-23' : (-1, (153.,150.)),
                       'Aruco-22' : (-1, (241., 50.)),
                       'Aruco-21' : (-1, (359., 50.)),
                       'Aruco-20' : (-1, (447.,150.)),
                       'Aruco-19' : (-1, (535., 50.))
                   },
                   doorways = [ (153., 77.), (447., 77.) ],
                   door_ids = [ (14, 20), (17, 23) ])

    w25 = WallSpec(length=600, height=190, door_width = 77, door_height=110,
                   marker_specs={
                       'Aruco-25' : (+1, ( 65., 50.)),
                       'Aruco-26' : (+1, (153.,150.)),
                       'Aruco-27' : (+1, (241., 50.)),
                       'Aruco-28' : (+1, (359., 50.)),
                       'Aruco-29' : (+1, (456.,150.)),
                       'Aruco-30' : (+1, (535., 50.)),
                       'Aruco-36' : (-1, ( 65., 50.)),
                       'Aruco-35' : (-1, (153.,150.)),
                       'Aruco-34' : (-1, (241., 50.)),
                       'Aruco-33' : (-1, (359., 50.)),
                       'Aruco-32' : (-1, (447.,150.)),
                       'Aruco-31' : (-1, (535., 50.))
                   },
                   doorways = [ (153., 77.), (447., 77.) ],
                   door_ids = [ (26, 32), (29, 35) ])

    
    # ~9 inch walls
    w49 = WallSpec(length=400, height=190, door_width = 77, door_height=110,
                   marker_specs={
                       'Aruco-49' : (+1, (112., 50.)),
                       'Aruco-50' : (+1, (200.,150.)),
                       'Aruco-51' : (+1, (288., 50.)),
                       'Aruco-54' : (-1, (112., 50.)),
                       'Aruco-53' : (-1, (200.,150.)),
                       'Aruco-52' : (-1, (288., 50.))
                   },
                   doorways = [ (200., 77.) ],
                   door_ids = [ (50, 53) ])

    w55 = WallSpec(length=400, height=190, door_width = 77, door_height=110,
                   marker_specs={
                       'Aruco-55' : (+1, (112., 50.)),
                       'Aruco-56' : (+1, (200.,150.)),
                       'Aruco-57' : (+1, (288., 50.)),
                       'Aruco-60' : (-1, (112., 50.)),
                       'Aruco-59' : (-1, (200.,150.)),
                       'Aruco-58' : (-1, (288., 50.))
                   },
                   doorways = [ (200., 77.) ],
                   door_ids = [ (56, 59) ])
    
    
    # ~6 inch walls

    w37 = WallSpec(length=300, height=190, door_width = 77, door_height=110,
                   marker_specs={
                       'Aruco-37' : (+1, ( 62., 50.)),
                       'Aruco-38' : (+1, (150.,150.)),
                       'Aruco-39' : (+1, (238., 50.)),
                       'Aruco-42' : (-1, ( 62., 50.)),
                       'Aruco-41' : (-1, (150.,150.)),
                       'Aruco-40' : (-1, (238., 50.))
                   },
                   doorways = [ (150., 77.) ],
                   door_ids = [ (38, 41) ])

    w43 = WallSpec(length=300, height=190, door_width = 77, door_height=110,
                   marker_specs={
                       'Aruco-43' : (+1, ( 62., 50.)),
                       'Aruco-44' : (+1, (150.,150.)),
                       'Aruco-45' : (+1, (238., 50.)),
                       'Aruco-48' : (-1, ( 62., 50.)),
                       'Aruco-47' : (-1, (150.,150.)),
                       'Aruco-46' : (-1, (238., 50.))}
                   ,
                   doorways = [ (150., 77.) ],
                   door_ids = [ (44, 47) ])

    
    wA = WallSpec(label='A', length=300, height=80)

    wB = WallSpec(label='B', length=400, height=80)

make_walls()
