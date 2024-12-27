#!/usr/bin/env python3


class MapPose:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def set_pose(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def get_pose(self):
        return self.x, self.y, self.z, self.w
