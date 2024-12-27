from map_pose import MapPose

# Class definition
class PointOfInterest:
    def __init__(self, name, id, type, audio_path, audio_name, x, y, z, w): # Constructor
        self.map_pose = MapPose(x, y, z, w)
        self.name = name
        self.type = type
        self.ID = id
        self.audio_path = "none"
        self.audio_file = "none"
        

    def set_point_of_interest(self, name, id, type, x, y, z, w):
        self.name = name
        self.ID = id
        self.type = type
        self.map_pose.set_pose(x, y, z, w)

    def get_point_of_interest(self):
        return self.name, self.type, self.map_pose.get_pose()


# List of PointOfInterest objects
POIx = [ PointOfInterest(name="POI1", id=1, type="type1", audio_path="null", audio_name="null", x=7.3302,  y=-6.3986, z=-0.59279122, w=0.80535617),
         PointOfInterest(name="POI2", id=2, type="type2", audio_path="null", audio_name="null", x=0.8706,  y=-7.1419, z=-0.98494684, w=0.12987554),
         PointOfInterest(name="POI3", id=3, type="type3", audio_path="null", audio_name="null", x=-6.4854, y=-6.4669, z=0.987313451, w=0.15878333),
         PointOfInterest(name="POI4", id=4, type="type4", audio_path="null", audio_name="null", x=-6.6813, y=5.19896, z=0.717991671, w=0.69605169),
         PointOfInterest(name="POI5", id=5, type="type5", audio_path="null", audio_name="null", x=7.4463,  y=5.1589,  z=-0.09426977, w=0.99554668),
         PointOfInterest(name="POI6", id=6, type="type6", audio_path="null", audio_name="null", x=1.8342,  y=2.01792, z=0.980541241, w=0.19631320)]
