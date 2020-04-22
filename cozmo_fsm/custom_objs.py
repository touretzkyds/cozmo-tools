import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

custom_marker_types = []
custom_container_types = []
custom_cube_types = []

async def declare_objects(robot):

    """
    await robot.world.define_custom_box(
        CustomObjectTypes.CustomType00,
        CustomObjectMarkers.Hexagons4,     # front
        CustomObjectMarkers.Triangles5,    # back
        CustomObjectMarkers.Circles2,      # top
        CustomObjectMarkers.Diamonds3,     # bottom
        CustomObjectMarkers.Circles4,      # left
        CustomObjectMarkers.Diamonds5,     # right
        50, 20, 1,   # depth, width, height
        40, 40,      # marker width and height
        True)        # is_unique
    return
    """

    global custom_marker_types, custom_cube_types
    
    decl_marker = robot.world.define_custom_wall
    custom_marker_types = [
        CustomObjectTypes.CustomType00,
        CustomObjectTypes.CustomType01,
        CustomObjectTypes.CustomType02,
        CustomObjectTypes.CustomType03
        ]

    await decl_marker(CustomObjectTypes.CustomType00,
                      CustomObjectMarkers.Circles2,
                      40, 40, 40, 40, True)

    await decl_marker(CustomObjectTypes.CustomType01,
                      CustomObjectMarkers.Triangles2,
                      40, 40, 40, 40, True)

    await decl_marker(CustomObjectTypes.CustomType02,
                      CustomObjectMarkers.Diamonds2,
                      40, 40, 40, 40, True)

    await decl_marker(CustomObjectTypes.CustomType03,
                      CustomObjectMarkers.Hexagons2,
                      40, 40, 40, 40, True)


# Markers for containers
    custom_container_types = [
      CustomObjectTypes.CustomType04,
      CustomObjectTypes.CustomType05
      ]

    await decl_marker(CustomObjectTypes.CustomType04,
                      CustomObjectMarkers.Circles3,
                      40, 40, 40, 40, False)

    await decl_marker(CustomObjectTypes.CustomType05,
                      CustomObjectMarkers.Triangles3,
                      40, 40, 40, 40, False)



# Markers for cubes

    decl_cube = robot.world.define_custom_cube

    custom_cube_types = [
        CustomObjectTypes.CustomType10,
        CustomObjectTypes.CustomType11,
        CustomObjectTypes.CustomType12,
        CustomObjectTypes.CustomType13,
        CustomObjectTypes.CustomType14,
        CustomObjectTypes.CustomType15
        ]

    await decl_cube(CustomObjectTypes.CustomType10,
                    CustomObjectMarkers.Circles5,
                    50, 40, 40, True)
    await decl_cube(CustomObjectTypes.CustomType11,
                    CustomObjectMarkers.Diamonds5,
                    50, 40, 40, True)
    await decl_cube(CustomObjectTypes.CustomType12,
                    CustomObjectMarkers.Hexagons5,
                    50, 40, 40, True)
    await decl_cube(CustomObjectTypes.CustomType13,
                    CustomObjectMarkers.Triangles4,
                    50, 40, 40, True)
    await decl_cube(CustomObjectTypes.CustomType14,
                    CustomObjectMarkers.Circles4,
                    50, 40, 40, True)
    await decl_cube(CustomObjectTypes.CustomType15,
                    CustomObjectMarkers.Diamonds4,
                    50, 40, 40, True)
