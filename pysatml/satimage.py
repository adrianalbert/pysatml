# for local I/O
import sys, os

# for handling geometry and AOIs
from shapely.geometry import Polygon, Point
from shapely.geometry.multipolygon import MultiPolygon
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
from PIL import Image, ImageDraw

# the data will be exported as numpy arrays
import numpy as np
import struct

# local modules
from .utils.gis_utils import km_to_deg_at_location,geoLoc_to_pixLoc
from .utils.raster_utils import get_image_center_pix,project_to_wgs84,get_geotiff_bounds,extract_centered_image_lonlat

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

	def __init__(self, rasterFile, force_wgs84=False, resolution=None):
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
				data = project_to_wgs84(data) 
				# if gdal version >= 2.0, Warp is available
				# data = gdal.Warp("tmp.tif",data,dstSRS='EPSG:4326')
			print(data.GetGeoTransform())
			if resolution is not None:
				if type(resolution) is not tuple:
					resolution = (resolution, resolution)
				data = resize_raster(data, resolution)
			self._raster[get_geotiff_bounds(data)] = data


	def polygonize_raster(self):
		pass

	def get_value_at_location(self, loc):
		return self.get_image_at_location(loc)

	def _get_raster(self, loc):
		raster = [ds for bounds, ds in self._raster.iteritems() \
			if (loc[0]>=bounds[0] and loc[0]<bounds[2]) and \
				(loc[1]>=bounds[1] and loc[1]<bounds[3])]
		if len(raster) == 0:
			return None
		return raster[0]

	def get_image_at_location(self, loc, w=None, toKm=True, dumpPath=None, pickle=False):
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

		w = 0 if w is None else w
		w = w if isinstance(w, tuple) else (w,w)
		wLat, wLon = km_to_deg_at_location(loc, w) if toKm else w
		# note that all the GDAL-based code assumes locations are given as (lon,lat), so we must reverse loc
		raster = self._get_raster(loc)
		if raster is None:
			return None
		img = extract_centered_image_lonlat(raster, loc[::-1], (wLon, wLat))

		if dumpPath is not None:
			dumpPath += "%2.6f_%2.6f_%dkm"%(loc[0], loc[1], w)
			save_image_data(img, dumpPath, pickle=False)
		return img

	def extract_polygon_mask(self, poly, w=None):
		'''
		Given a shapely polygon, extract an cropping from the raster that contains the input geometry. If w (km) is specified, extract a window of size w. Mask all values outside the given polygon by NaN.
		'''
		# if the polygon is actually a collection of polygons (MultiPolygon)
		# we take the largest polygon
		if isinstance(poly, MultiPolygon):
		    polygons = [polygon for polygon in poly]
		    polygons.sort(key=lambda p: p.area, reverse=True)
		    poly = polygons[0]

		# crop an image from raster containing the polygon
		lon, lat = poly.centroid.xy
		center = (lat[0], lon[0]) # note lat,lon to work with satimg
		toKm = True
		if w is None:
		    bounds = poly.bounds
		    wLon,wLat = np.abs(bounds[2]-bounds[0]),np.abs(bounds[3]-bounds[1])
		    w = max([wLon, wLat])
		    w = (w,w)
		    toKm = False
		elif not isinstance(w, tuple):
		    w = (w,w)
		img = self.get_image_at_location(center, w=w, toKm=toKm)
		C, W, H = img.shape # again, it's wLat, wLon

		# convert polygon coordinates from latlon to image coordinates
		ds = self._get_raster(center)
		gt = ds.GetGeoTransform()
		poly_list = zip(poly.boundary.xy[0],poly.boundary.xy[1])
		poly_list = [geoLoc_to_pixLoc(p, gt=gt) for p in poly_list]
		center_pix= geoLoc_to_pixLoc(center[::-1], gt=gt)
		xmin, ymin = get_image_center_pix(center_pix, (H,W))[:2]
		poly_list = [(x-xmin,y-ymin) for (x,y) in poly_list]

		# create mask of polygon
		mask = Image.new('L', (H, W), 0)
		ImageDraw.Draw(mask).polygon(poly_list, outline=2, fill=1)
		mask = np.array(mask)
		# img[:,mask==0] = np.nan
		return np.vstack([img, np.expand_dims(mask,0)])

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

def resize_raster(raster, resolution):
	pass
