# numeric packages
import numpy as np
import pandas as pd

# geo stuff
import geopandas as gpd
from shapely.geometry import Point, Polygon
from osgeo import gdal, osr
from pyproj import Proj, transform

# local modules
import gis_utils as gu


def compute_gdf_bounds(gdf):
    bounds = np.array(gdf['geometry'].apply(lambda p: list(p.bounds)).values.tolist())
    xmin = bounds[:,[0,2]].min()
    xmax = bounds[:,[0,2]].max()
    ymin = bounds[:,[1,3]].min()
    ymax = bounds[:,[1,3]].max()
    return xmin, ymin, xmax, ymax


def filter_gdf_by_polygon(gdf, polygon):
    spatial_index = gdf.sindex
    possible_matches_index = list(spatial_index.intersection(polygon.bounds))
    possible_matches = gdf.iloc[possible_matches_index]
    precise_matches = possible_matches[possible_matches.intersects(polygon)]
    return precise_matches


def filter_gdf_by_centered_window(gdf, center, window):
    latmin, lonmin, latmax, lonmax = gu.bounding_box_at_location(center, window)
    pbox = Polygon([(lonmin,latmin), (lonmax,latmin), (lonmax,latmax), (lonmin,latmax)])
    return filter_gdf_by_polygon(gdf, pbox)


    