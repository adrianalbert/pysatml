# the data will be exported as numpy arrays
import numpy as np

# for reading GeoTiffs
from osgeo import gdal, osr
from gdal import gdalconst
from gdalconst import * 
import osr

import rasterio
from rasterio.features import shapes

from .gis_utils import geoLoc_to_pixLoc,geoSize_to_pixSize

def raster2polygon(raster_file, band=1):
	mask = None
	with rasterio.drivers():
	    with rasterio.open(raster_file) as src:
	        image = src.read(band) # first band
	        results = (
	        	{'properties': {'raster_val': v}, 'geometry': s}
	        		for i, (s, v) in enumerate(shapes(image, mask=mask, transform=src.affine)))
	geoms = list(results)
	gpd_polygonized_raster = gp.GeoDataFrame.from_features(geoms)
	return gpd_polygonized_raster


def get_geotiff_bounds(raster):

	# get the existing coordinate system
	old_cs= osr.SpatialReference()
	old_cs.ImportFromWkt(raster.GetProjectionRef())

	# create the new coordinate system
	wgs84_wkt = """
	GEOGCS["WGS 84",
	    DATUM["WGS_1984",
	        SPHEROID["WGS 84",6378137,298.257223563,
	            AUTHORITY["EPSG","7030"]],
	        AUTHORITY["EPSG","6326"]],
	    PRIMEM["Greenwich",0,
	        AUTHORITY["EPSG","8901"]],
	    UNIT["degree",0.01745329251994328,
	        AUTHORITY["EPSG","9122"]],
	    AUTHORITY["EPSG","4326"]]"""
	new_cs = osr.SpatialReference()
	new_cs.ImportFromWkt(wgs84_wkt)

	# create a transform object to convert between coordinate systems
	transform = osr.CoordinateTransformation(old_cs,new_cs) 

	#get the point to transform, pixel (0,0) in this case
	width = raster.RasterXSize
	height = raster.RasterYSize
	gt = raster.GetGeoTransform()
	minx = gt[0]
	miny = gt[3] + width*gt[4] + height*gt[5] 
	maxx = gt[0] + width*gt[1] + height*gt[2]
	maxy = gt[3] 

	#get the coordinates in lat long
	latlongMin = transform.TransformPoint(minx,miny) 
	latlongMax = transform.TransformPoint(maxx,maxy) 

	return latlongMin[1], latlongMin[0], latlongMax[1], latlongMax[0]


def project_to_wgs84(src_ds):
	# Define target SRS
	dst_srs = osr.SpatialReference()
	dst_srs.ImportFromEPSG(4326)
	dst_wkt = """
	GEOGCS["WGS 84",
	    DATUM["WGS_1984",
	        SPHEROID["WGS 84",6378137,298.257223563,
	            AUTHORITY["EPSG","7030"]],
	        AUTHORITY["EPSG","6326"]],
	    PRIMEM["Greenwich",0,
	        AUTHORITY["EPSG","8901"]],
	    UNIT["degree",0.01745329251994328,
	        AUTHORITY["EPSG","9122"]],
	    AUTHORITY["EPSG","4326"]]"""

	error_threshold = 0.125  # error threshold --> use same value as in gdalwarp
	resampling = gdal.GRA_NearestNeighbour

	# Call AutoCreateWarpedVRT() to fetch default values for target raster dimensions and geotransform
	new_ds = gdal.AutoCreateWarpedVRT( src_ds,
	                                   None, # src_wkt : left to default value --> will use the one from source
	                                   dst_wkt,
	                                   resampling,
	                                   error_threshold )
	gdal.ReprojectImage(src_ds, new_ds, None, dst_wkt)

	return new_ds


def extract_centered_image_lonlat(raster, lonlat, geoSize):
	gt = raster.GetGeoTransform()
	pixCenter = geoLoc_to_pixLoc(lonlat, gt)
	geoWidth, geoHeight = geoSize
	if geoWidth==0 or geoHeight==0 or geoWidth is None or geoHeight is None:
		return extract_centered_image_pix(raster, pixCenter, (1,1))[:,0,0]
	else:
		pixWidth, pixHeight = geoSize_to_pixSize(geoSize, gt)
		return extract_centered_image_pix(raster, pixCenter, (pixWidth, pixHeight))


def extract_centered_image_pix(raster, pixLoc, pixSize):
	pixWidth, pixHeight = pixSize
	pixUpperLeft = get_image_center_pix(pixLoc, (pixWidth, pixHeight))[:2]
	return extract_image_pix(raster, pixUpperLeft, (pixWidth, pixHeight))


def get_image_center_pix(pixLoc, pixSize):
    # Compute frame
    pixelWidth, pixelHeight = pixSize
    pixelLeft = max([0,pixLoc[0] - pixelWidth / 2])
    pixelTop = max([0,pixLoc[1] - pixelHeight / 2])
    pixelRight = pixelLeft + pixelWidth
    pixelBottom = pixelTop + pixelHeight
    # Return
    return pixelLeft, pixelTop, pixelRight, pixelBottom


def extract_image_pix(raster, pixLoc, pixSize):
	# Set bounds
	pixWidth, pixHeight = pixSize
	iLeft   = int(pixLoc[0])
	iTop    = int(pixLoc[1])
	iWidth  = int(pixWidth)
	iHeight = int(pixHeight)
	rWidth  = raster.RasterXSize
	rHeight = raster.RasterYSize
	iWidth  = iWidth if iLeft + iWidth <= rWidth \
				else rWidth - iLeft
	iHeight = iHeight if iTop + iHeight <= rHeight \
				else rHeight - iTop
	# Extract data
	bands = map(raster.GetRasterBand, xrange(1, raster.RasterCount + 1))
	img = [b.ReadAsArray(iLeft, iTop, iWidth, iHeight).astype(np.float) \
		for b in bands]
	img = np.asarray(img)
	return img

