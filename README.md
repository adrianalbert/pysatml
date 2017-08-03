# pysatml

**Note: this package is under active development, and some functionality is incomplete or untested.**

This repository contains utilities for handling common tasks related to the processing of GIS data formats for machine learning applications. 

Geospatial and remote-sensing data typically comes in GIS raster formats (such as GeoTiff) that contain information about coordinate systems, location, etc. These files are typically large and unwieldy for current ML algorithms and pipelines. 

Typically, machine learning algorithms are trained using many small samples (images), or with data processed in very specific ways (such as masks for image segmentation tasks). This package contains functionality to simplify the following common tasks:
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
