# pysatml

This repository contains utilities for handling common tasks related to the processing of GIS data formats for machine learning applications. The development of this package was motivated by the following two observations:

* Geospatial and remote-sensing data typically comes in GIS raster formats (such as GeoTiff) that contain information about coordinate systems, location, etc. These files are typically large and unwieldy for current ML algorithms and pipelines, and a good amount of custom preprocessing is required (especially for handling geographic projections and combining vector with raster data)
* Typically, computer vision and machine learning algorithms are trained using many small samples (images), or with data processed in very specific ways (such as masks for image segmentation tasks). 

As such, this package contains functionality to simplify the following common tasks:
* extracting samples from large GeoTiff files, appropriately handling geographic coordinate systems and units of distance. [example notebook](examples/SatImage-tutorial-sampling.ipynb)
* constructing spatial masks for image segmentation tasks, handling geometry data provided in the most common formats such as GeoJSON or GeoPandas GeoDataFrame

## Installation

The easiest way to install `pysatml` is via `pip`:

```bash
pip install pysatml
```

You can also clone the master branch of this repository and install the latest development version:

```bash
git clone https://github.com/adrianalbert/pysatml.git
cd pysatml/
python setup.py
```

You can check whether the module has installed successfuly by opening a Python console and typing:
```python
import pysatml
```

## Example usage

The main functionality in `pysatml` is provided via the `SatImage` class.  
```python
from pysatml import SatImage
```

An example of using this class is provided in an [example Jupyter notebook](examples/SatImage-tutorial-sampling.ipynb).

## Citation

This package has been developed around a related [paper](https://arxiv.org/abs/1704.02965) that showcases its usage for performing large-scale analysis of satellite imagery for land use classification in cities. If you find this package useful, please consider citing the related paper as:

> _Using convolutional networks and satellite imagery to identify patterns in urban environments at a large scale_. A. Toni Albert, J. Kaur, M.C. Gonzalez, 2017. In Proceedings of the ACM SigKDD 2017 Conference, Halifax, Nova Scotia, Canada.
