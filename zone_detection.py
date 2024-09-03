#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  2 18:10:33 2024

@author: obarrero
"""

import shapefile
from shapely.geometry import shape, Point


def check(lon, lat):
    # build a shapely point from your geopoint
    point = Point(lon, lat)

    # the contains function does exactly what you want
    return polygon.contains(point)


# read your shapefile
poly_file='poligono_casona.shp'
r = shapefile.Reader(poly_file)
# get the shapes
shapes = r.shapes()
lon =-75.198452
lat = 4.448837
for k in range(len(shapes)):

    # build a shapely polygon from your shape
    polygon = shape(shapes[k])    
    zone_def = check(lon, lat)
    if zone_def :
        zone=k
        
print('El punto corresponde a la zona ' + str(zone+1))




