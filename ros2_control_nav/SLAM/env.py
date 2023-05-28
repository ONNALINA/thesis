#!/usr/bin/python3
import math
import pygame

class buildEnvilonment:
    def __init__(self,MapDimensions):
        pygame.init()
        self.pointClound = []
        self.externalMap = pygame.image.load('map.png')
        self.maph, self.mapw = MapDimensions
        self.MapWindowName = 'RRT_Path_Planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw,self.maph))
        self.map.blit(self.externalMap,(0,0))
        #colors
        self.black = (0,0,0)
        self.gray = (70,70,70)
        self.blue = (0,0,255)
        self.green = (0,255,0)
        self.red = (255,0,0)
        self.write = (255,255,255)

    def AD2pos(self,distance,angle,robotPosition):
        x = distance * math.cos(angle)+robotPosition[0]
        y = -distance * math.cos(angle)+robotPosition[1]
        return (int(x),int(y))

    def dataStorage(self,data):
        print("Data received:", data)
        if data != False:
            for element in data:
                point = self.AD2pos(element[0],element[1],element[2])
                if point not in self.pointClound:
                    self.pointClound.append(point)

    def show_sensorData(self):
        self.infomap = self.map.copy()
        for point in self.pointClound:
            self.infomap.set_at((int(point[0]),int(point[1])),(255,0,0))