 #!/bin/bash
 for f in *.tif; do
 	f1=$(basename "$f")
 	echo "$f", "$1"

 	# gdalwarp -t_srs EPSG:4326 $i $i.wsg84.tif
 done