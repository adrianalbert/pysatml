# satimg

This repository contains utilities for handling common tasks related to the processing of GIS data formats for machine learning applications. 

Geospatial and remote-sensing data typically comes in GIS raster formats (such as GeoTiff) that contain information about coordinate systems, location, etc. These files are typically large and unwieldy for current ML algorithms and pipelines. 

Typically, machine learning algorithms are trained using many small samples (images), or with data processed in very specific ways (such as masks for image segmentation tasks). This package contains functionality to simplify the following common tasks:
* sampling small images from large GeoTiff files, appropriately handling geographic coordinate systems and units of distance. [example notebook](examples/SatImage-tutorial-sampling.ipynb)
* constructing spatial masks for image segmentation tasks

