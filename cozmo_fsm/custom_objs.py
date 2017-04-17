import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

async def declare_objects(robot):
    decl = robot.world.define_custom_cube
    await decl(CustomObjectTypes.CustomType00,
               CustomObjectMarkers.Circles2,
               50, 40, 40, True)
    await decl(CustomObjectTypes.CustomType01,
               CustomObjectMarkers.Diamonds3,
               50, 40, 40, True)
    await decl(CustomObjectTypes.CustomType02,
               CustomObjectMarkers.Hexagons4,
               50, 40, 40, True)
    await decl(CustomObjectTypes.CustomType03,
               CustomObjectMarkers.Triangles5,
               50, 40, 40, True)
    await decl(CustomObjectTypes.CustomType04,
               CustomObjectMarkers.Circles4,
               50, 40, 40, True)
    await decl(CustomObjectTypes.CustomType05,
               CustomObjectMarkers.Diamonds5,
               50, 40, 40, True)

