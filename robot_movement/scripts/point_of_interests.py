from map_pose import MapPose

# Class definition
class PointOfInterest:
    def __init__(self, name, id, type, audio_path, audio_name, x, y, z, w): # Constructor
        self.map_pose = MapPose(x, y, z, w)
        self.name = "none"
        self.type = "none"
        self.ID = 0
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
POIx = [ PointOfInterest(name="POI1", id=1, type="type1", audio_path="null", audio_name="null", x=1.0, y=2.0, z=3.0, w=4.0),
         PointOfInterest(name="POI2", id=2, type="type2", audio_path="null", audio_name="null", x=5.0, y=6.0, z=7.0, w=8.0),
         PointOfInterest(name="POI3", id=3, type="type3", audio_path="null", audio_name="null", x=9.0, y=10.0, z=11.0, w=12.0),
         PointOfInterest(name="POI4", id=4, type="type4", audio_path="null", audio_name="null", x=13.0, y=14.0, z=15.0, w=16.0),
         PointOfInterest(name="POI5", id=5, type="type5", audio_path="null", audio_name="null", x=17.0, y=18.0, z=19.0, w=20.0)]