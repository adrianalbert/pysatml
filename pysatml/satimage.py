# for local I/O
import sys, os

# for handling geometry and AOIs
from shapely.geometry import Polygon, Point
from shapely.wkt import dumps as wkt_dumps
import geojson
import math

# wrapper for GoogleMaps geocoding service
import geocoder

# for reading GeoTiffs
from osgeo import gdal, osr
from gdal import gdalconst
from gdalconst import * 

# for processing image data
import skimage
from skimage import exposure, io

# the data will be exported as numpy arrays
import numpy as np
import struct

# local modules
from utils import gis_utils as gu
from utils import vector_utils as vu 
from utils import raster_utils as ru

# constants
WHOLE_WORLD_BOUNDS = (-180, -85, 180, 85)

# Types used by struct.unpack
typeByGDT = {
    gdal.GDT_Byte: 'B',
    gdal.GDT_UInt16: 'H', 
    gdal.GDT_Float32: 'f'
}
modeByType = {
    gdal.GDT_Byte: 'L',
    gdal.GDT_UInt16: 'I;16', 
    gdal.GDT_Float32: 'I;16'
}

class SatImage(object):

	def __init__(self, rasterFile, force_wgs84=False):
		"""
		Initialize with either one or multiple raster files where the data of interest resides. Some times large raster files are broken down into several tiles to facilitate processing and distribution.
		force_wgs84: project raster to commonly-used WGS84 format.
					 Note: this takes a bit of time, for repeated use it's better to reproject rasters separately (done just once).
		"""
		self._rasterPaths = rasterFile
		self._raster = {}
		if type(rasterFile) == str:
			rasterFile = [rasterFile]
		for rf in rasterFile:
			data = gdal.Open(rf, GA_ReadOnly)	
			if data is None:
				continue
			if force_wgs84:
				data = ru.project_to_wgs84(data) 
				# if gdal version >= 2.0, Warp is available
				# data = gdal.Warp("tmp.tif",data,dstSRS='EPSG:4326')
			print ru.get_geotiff_bounds(data)
			self._raster[ru.get_geotiff_bounds(data)] = data


	def polygonize_raster(self):
		pass

	def get_value_at_location(self, loc):
		return self.get_image_at_location(loc)


	def get_image_at_location(self, loc, w=None, dumpPath=None, pickle=False):
		"""
		Crop raster at location loc (lat,lon) with size w x w (in meters). Return a data matrix or save to file. 
		If w is not set, return just the pixel values at <loc>. 
		"""
		if type(loc) == str:
			# if location is given by a string, interpret that as an address
			# or city name and try to turn it to lat/lon via Google geocoding
			loc = tuple(geocoder.google(loc).latlng)
		if len(loc) == 0:
			return None

		img = [img for bounds, img in self._raster.iteritems() \
			if (loc[0]>=bounds[0] and loc[0]<bounds[2]) and \
				(loc[1]>=bounds[1] and loc[1]<bounds[3])]
		if len(img) == 0:
			return None

		w = 0 if w is None else w
		wLat, wLon = gu.km_to_deg_at_location(loc, (w,w))
		# note that all the GDAL-based code assumes locations are given as (lon,lat), so we must reverse loc
		img = ru.extract_centered_image_lonlat(img[0], loc[::-1], (wLon, wLat))

		if dumpPath is not None:
			dumpPath += "%2.6f_%2.6f_%dkm"%(loc[0], loc[1], w)
			save_image_data(img, dumpPath, pickle=False)
		return img


	def get_image_at_locations(self,locs,w=None,dumpPath=None,pickle=False):
		""" Obtain sample images at different locations.
		"""
		if type(locs) == tuple:
			return self.get_image_at_location(locs, w=w, \
				dumpPath=dumpPath, pickle=pickle)
		else:
			return {loc:self.get_image_at_location(loc, w=w, \
				dumpPath=dumpPath, pickle=pickle) for loc in locs}


	def sample_images_around_location(self, loc, w=None, W=None, nSamples=1,\
		dumpPath=None, pickle=False):
		""" 
		Sample nSamples images of size w x w within a bounding box of W x W around location loc. Returns the list of sampled locations.
		"""
		locs = generate_locations_around_latlon(loc, W=W, nSamples=nSamples)
		return self.get_image_at_locations(locs, w=w, \
			dumpPath=dumpPath, pickle=pickle)

	
	def sample_images_within_shape(self, shape, w=None, nSamples=1, \
		dumpPath=None, pickle=False):
		""" 
		Generate nSamples candidate (lat,lon) locations to sample images at. 
		"""
		locs = generate_locations_within_polygon(polygon, nSamples=1)
		return self.get_image_at_locations(locs, w=w, \
			dumpPath=dumpPath, pickle=pickle)


def generate_locations_around_latlon(latlon, W=None, nSamples=1):
	boundingBox = gu.bounding_box_at_location(latlon, (W,W))
	locs = generate_locations_within_bounding_box(boundingBox, nSamples)
	return locs


def generate_locations_within_bounding_box(bbox, nSamples=1, seed=None):
	np.random.seed(seed)
	minX, minY, maxX, maxY = bbox
	x = np.random.uniform(minX, maxX, nSamples)
	y = np.random.uniform(minY, maxY, nSamples)
	return zip(x,y)


def generate_locations_within_polygon(polygon, nSamples=1, seed=None, \
	strict=True):
	"""
	There doesn't seem to be an efficient way to do this sampling for an arbitrary polygon. We use a rejection sampling method instead.
	"""
	bbox = polygon.bounds
	shape = polygon if strict else polygon.convex_hull
	points = []
	while len(points) < nSamples:
		ps = generate_locations_within_bounding_box(bbox, \
			nSamples=nSamples, seed=seed)
		ps = [p for p in ps if Point(p).within(shape)]
		points += ps
	return points[:nSamples]


def save_image_data(img, dumpPath, pickle=False):
    basedir = os.path.dirname(dumpPath)
    # img = exposure.rescale_intensity(img, out_range='float')
    # img = skimage.img_as_uint(img)
    img = img.astype(np.uint8)
    if not os.path.exists(basedir):
        os.makedirs(basedir)
    if pickle:
        with gzip.open(dumpPath+".pickle.gz", 'w'):
            pickle.dump(img, dumpPath)
    elif img.shape[0] in [3,4]:
        io.imsave(dumpPath+".jpg", img.reshape(img.shape[::-1]))
    elif img.shape[0] == 1:
        io.imsave(dumpPath+".png", np.squeeze(img))
    else:
        io.imsave(dumpPath+".tif", img, compress=6)

