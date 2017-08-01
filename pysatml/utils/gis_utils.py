# numeric packages
import numpy as np
import pandas as pd

# filesystem and OS
import sys, os, time
import glob

# plotting
from matplotlib import pyplot as plt
import matplotlib

# compression
import gzip
import cPickle as pickle
import copy

# geo stuff
import geopandas as gpd
from shapely.geometry import Point, Polygon
from osgeo import gdal, osr
from pyproj import Proj, transform


def km_to_deg_at_location(loc, sizeKm):
    latMin, lonMin, latMax, lonMax = bounding_box_at_location(loc, sizeKm)
    return latMax - latMin, lonMax - lonMin


# Bounding box surrounding the point at given coordinates,
# assuming local approximation of Earth surface as a sphere
# of radius given by WGS84
def bounding_box_at_location(latlon, sizeKm):
    widthKm, heightKm = sizeKm
    latitudeInDegrees, longitudeInDegrees = latlon
    # degrees to radians
    def deg2rad(degrees):
        return math.pi*degrees/180.0

    # radians to degrees
    def rad2deg(radians):
        return 180.0*radians/math.pi

    # Semi-axes of WGS-84 geoidal reference
    WGS84_a = 6378137.0  # Major semiaxis [m]
    WGS84_b = 6356752.3  # Minor semiaxis [m]

    # Earth radius at a given latitude, according to the WGS-84 ellipsoid [m]
    def WGS84EarthRadius(lat):
        # http://en.wikipedia.org/wiki/Earth_radius
        An = WGS84_a*WGS84_a * math.cos(lat)
        Bn = WGS84_b*WGS84_b * math.sin(lat)
        Ad = WGS84_a * math.cos(lat)
        Bd = WGS84_b * math.sin(lat)
        return math.sqrt( (An*An + Bn*Bn)/(Ad*Ad + Bd*Bd) )

    lat = deg2rad(latitudeInDegrees)
    lon = deg2rad(longitudeInDegrees)
    widthMeters = 1000*widthKm/2.0
    heightMeters = 1000*heightKm/2.0

    # Radius of Earth at given latitude
    radius = WGS84EarthRadius(lat)
    # Radius of the parallel at given latitude
    pradius = radius*math.cos(lat)

    latMin = lat - heightMeters/radius
    latMax = lat + heightMeters/radius
    lonMin = lon - widthMeters/pradius
    lonMax = lon + widthMeters/pradius

    return (rad2deg(latMin), rad2deg(lonMin), rad2deg(latMax), rad2deg(lonMax))


def read_prj_file(prj_file):
    '''
    Read ESRI projection file into string.
    '''
    prj_text = open(prj_file, 'r').read()
    srs = osr.SpatialReference()
    if srs.ImportFromWkt(prj_text):
        raise ValueError("Error importing PRJ information from: %s" % prj_file)
    prj = srs.ExportToProj4()
    if prj == "":
        return '+proj=merc +lon_0=0 +lat_ts=0 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs '
    else:
        return prj
    

def xy2lonlat(xy, prj=""):
    '''
    Convert northing/easting coordinates to commonly-used lat/lon.
    '''
    x, y = xy
    inProj = Proj(prj) 
    outProj = Proj(init='epsg:4326')
    lon, lat = transform(inProj,outProj,x,y) 
    return lon, lat
    

def lonlat2xy(lonlat, prj=""):
    '''
    Convert commonly-used lat/lon to northing/easting coordinates.
    '''
    lon, lat = lonlat
    inProj = Proj(init='epsg:4326')
    outProj = Proj(prj) 
    x, y = transform(inProj,outProj,lon,lat) 
    return x, y
    

def polygon_xy2lonlat(p, prj=""):
    '''
    Convert polygon coordinates from meter to lon/lat.
    '''
    inProj = Proj(prj) 
    outProj = Proj(init='epsg:4326')
    x, y = p.exterior.coords.xy
    locs_meter = zip(x, y)
    locs_lonlat= [transform(inProj,outProj,x1,y1) for x1,y1 in locs_meter]
    return Polygon(locs_lonlat)


def polygon_lonlat2xy(p, prj=""):
    '''
    Convert polygon coordinates from lon/lat to meter.
    '''
    inProj = Proj(init='epsg:4326')
    outProj = Proj(prj) 
    lon, lat = p.exterior.coords.xy
    locs_lonlat = zip(lon, lat)
    locs_meter = [transform(inProj,outProj,x,y) for x,y in locs_lonlat]
    return Polygon(locs_meter)


def convert_geoWindow_to_pixelWindow(geoWindow, gt):
    geoLeft, geoTop, geoRight, geoBottom = geoWindow
    pixelLeft, pixelTop = geoLoc_to_pixLoc((geoLeft, geoTop), gt)
    pixelRight, pixelBottom = geoLoc_to_pixLoc((geoRight, geoBottom), gt)
    pixelLeft, pixelRight = sorted((pixelLeft, pixelRight))
    pixelTop, pixelBottom = sorted((pixelTop, pixelBottom))
    return pixelLeft, pixelTop, pixelRight, pixelBottom


def convert_pixelWindow_to_geoWindow(pixWindow, gt):
    pixelLeft, pixelTop, pixelRight, pixelBottom = pixWindow
    geoLeft, geoTop = geoLoc_to_pixLoc((pixelLeft, pixelTop), gt)
    geoRight, geoBottom = geoLoc_to_pixLoc((pixelRight, pixelBottom), gt)
    geoLeft, geoRight = sorted((geoLeft, geoRight))
    geoTop, geoBottom = sorted((geoTop, geoBottom))
    return geoLeft, geoTop, geoRight, geoBottom


def geoLoc_to_pixLoc(geoLoc, gt):
    """
    Transforms a geographical location geoLoc (lat,lon) to pixel coordinates in a geographic reference given by the geotransform gt.
    """
    g0, g1, g2, g3, g4, g5 = gt
    xGeo, yGeo = geoLoc
    if g2 == 0:
        xPixel = (xGeo - g0) / float(g1)
        yPixel = (yGeo - g3 - xPixel*g4) / float(g5)
    else:
        xPixel = (yGeo*g2 - xGeo*g5 + g0*g5 - g2*g3) / float(g2*g4 - g1*g5)
        yPixel = (xGeo - g0 - xPixel*g1) / float(g2)
    return int(round(xPixel)), int(round(yPixel))


def geoSize_to_pixSize(geoSize, gt):
    g0, g1, g2, g3, g4, g5 = gt
    geoWidth, geoHeight = geoSize
    return int(round(abs(float(geoWidth) / g1))), int(round(abs(float(geoHeight) / g5)))

