from .worldmap import *

# Disabled ArUco ids: 17 and 37

def make_walls():

    # ~12 inch walls

    w1 = WallSpec(length=600, height=190, door_width=77, door_height=110,
                  marker_specs={
                      'Aruco-1' :  (+1, ( 62., 30.)),
                      'Aruco-2' :  (+1, (150.,150.)),
                      'Aruco-3' :  (+1, (238., 30.)),
                      'Aruco-4' :  (+1, (362., 30.)),
                      'Aruco-5' :  (+1, (450.,150.)),
                      'Aruco-6' :  (+1, (538., 30.)),
                      'Aruco-12' : (-1, ( 62., 30.)),
                      'Aruco-11' : (-1, (150.,150.)),
                      'Aruco-10' : (-1, (238., 30.)),
                      'Aruco-9' :  (-1, (362., 30.)),
                      'Aruco-8' :  (-1, (450.,150.)),
                      'Aruco-7' :  (-1, (538., 30.))
                  },
                  doorways = [ (150., 77.), (450., 77.) ],
                  door_ids = [ (2, 8), (5, 11) ])

    w13 = WallSpec(length=600, height=190, door_width=77, door_height=110,
                   marker_specs={
                       'Aruco-13' : (+1, ( 62., 30.)),
                       'Aruco-14' : (+1, (150.,150.)),
                       'Aruco-15' : (+1, (238., 30.)),
                       'Aruco-16' : (+1, (362., 30.)),
                       'Aruco-18' : (+1, (450.,150.)),
                       'Aruco-19' : (+1, (538., 30.)),
                       'Aruco-25' : (-1, ( 62., 30.)),
                       'Aruco-24' : (-1, (150.,150.)),
                       'Aruco-23' : (-1, (238., 30.)),
                       'Aruco-22' : (-1, (362., 30.)),
                       'Aruco-21' : (-1, (450.,150.)),
                       'Aruco-20' : (-1, (538., 30.))
                   },
                   doorways = [ (150., 77.), (450., 77.) ],
                   door_ids = [ (14, 21), (18, 24) ])

    w26 = WallSpec(length=600, height=190, door_width=77, door_height=110,
                   marker_specs={
                       'Aruco-26' : (+1, ( 62., 30.)),
                       'Aruco-27' : (+1, (150.,150.)),
                       'Aruco-28' : (+1, (238., 30.)),
                       'Aruco-29' : (+1, (362., 30.)),
                       'Aruco-30' : (+1, (450.,150.)),
                       'Aruco-31' : (+1, (538., 30.)),
                       'Aruco-38' : (-1, ( 62., 30.)),
                       'Aruco-36' : (-1, (150.,150.)),
                       'Aruco-35' : (-1, (238., 30.)),
                       'Aruco-34' : (-1, (362., 30.)),
                       'Aruco-33' : (-1, (450.,150.)),
                       'Aruco-32' : (-1, (538., 30.))
                   },
                   doorways = [ (150., 77.), (450., 77.) ],
                   door_ids = [ (27, 33), (30, 36) ])

    
    # ~6 inch walls

    w39 = WallSpec(length=300, height=190, door_width=77, door_height=110,
                   marker_specs={
                       'Aruco-39' : (+1, ( 62., 30.)),
                       'Aruco-40' : (+1, (150.,150.)),
                       'Aruco-41' : (+1, (238., 30.)),
                       'Aruco-44' : (-1, ( 62., 30.)),
                       'Aruco-43' : (-1, (150.,150.)),
                       'Aruco-42' : (-1, (238., 30.))
                   },
                   doorways = [ (150., 77.) ],
                   door_ids = [ (40, 43) ])

    w45 = WallSpec(length=300, height=190, door_width=77, door_height=110,
                   marker_specs={
                       'Aruco-45' : (+1, ( 62., 30.)),
                       'Aruco-46' : (+1, (150.,150.)),
                       'Aruco-47' : (+1, (238., 30.)),
                       'Aruco-50' : (-1, ( 62., 30.)),
                       'Aruco-49' : (-1, (150.,150.)),
                       'Aruco-48' : (-1, (238., 30.))}
                   ,
                   doorways = [ (150., 77.) ],
                   door_ids = [ (46, 49) ])

    
    # ~9 inch walls
    w51 = WallSpec(length=400, height=190, door_width=77, door_height=110,
                   marker_specs={
                       'Aruco-51' : (+1, (112., 30.)),
                       'Aruco-52' : (+1, (200.,150.)),
                       'Aruco-53' : (+1, (288., 30.)),
                       'Aruco-56' : (-1, (112., 30.)),
                       'Aruco-55' : (-1, (200.,150.)),
                       'Aruco-54' : (-1, (288., 30.))
                   },
                   doorways = [ (200., 77.) ],
                   door_ids = [ (52, 55) ])

    w57 = WallSpec(length=400, height=190, door_width=77, door_height=110,
                   marker_specs={
                       'Aruco-57' : (+1, (112., 30.)),
                       'Aruco-58' : (+1, (200.,150.)),
                       'Aruco-59' : (+1, (288., 30.)),
                       'Aruco-62' : (-1, (112., 30.)),
                       'Aruco-61' : (-1, (200.,150.)),
                       'Aruco-60' : (-1, (288., 30.))
                   },
                   doorways = [ (200., 77.) ],
                   door_ids = [ (58, 61) ])
    
    
    # Walls without markers

    wA = WallSpec(label='A', length=300, height=190)

    wB = WallSpec(label='B', length=400, height=190)

make_walls()
