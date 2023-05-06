## About
This work is heavily influenced by a repository located here:
https://github.com/muneebaadil/how-to-sfm

main.py uses two of their classes/functions, feature match and SFM. There are more details located in the repo listed above. This code can be run using python main.py and will run the default configuration.

The repo above comes with a solid tutorial that introduces you to the flow required for SFM. This includes feature calculation using sift, camera pose estimation and projections.

It should be noted that, if you do NOT have access to SIFT or SURF, ORB is a good alternative.

## Converting Point Cloud output to Mesh
The function responsible for conversion is ConvertToMesh(). It isn't being called right now since it produces an undesirable output. It takes in as input pcd (the generated point cloud) and the method to be used for conversion - "ball pivoting" or "poisson". The function returns a mesh which can be visualized using o3d.visualization.draw_geometries().

<!---
## TODO

1. THERE ARE A TON OF POORLY CALCULATED POINTS THAT NEED TO BE CLEANED.
2. CONVERTING THE POINT CLOUDS TO STL. THIS SHOULDNT BE TOO HARD AFTER CLEANING. MIGHT HAVE TO ASSUME AN ARBITRARY DEPTH.
3. CALCULATING THE K VALUES FOR TOLGA'S VIDEOS

-->

## TEST CASES
### GOOD
> python main.py --features SIFT

>python main.py --features SIFT --dataset Herz-Jesus-P25 --data_dirFt ../data/Herz-Jesus-P25/images/ --out_dirFt ../data/Herz-Jesus-P25 --reproj_err_thresh 5

>python main.py --features SIFT --dataset castle-P19 --data_dirFt ../data/castle-P19/images/ --out_dirFt ../data/castle-P19 --reproj_err_thresh 5

>python main.py --features SIFT --dataset Herz-Jesus-P8 --data_dirFt ../data/Herz-Jesus-P8/images/ --out_dirFt ../data/Herz-Jesus-P8 --reproj_err_thresh 5 
#This one is OKAY, not great

>python main.py --features SIFT --dataset mevlana --data_dirFt ../Data/mevlana/images/ --out_dirFt ../Data/mevlana --reproj_err_thresh 5 --calibration_mat galaxy_a30_crop

>python main.py --features SIFT --dataset mevlana --data_dirFt ../Data/mevlana/images/ --out_dirFt ../Data/mevlana --reproj_err_thresh 5 --calibration_mat galaxy_a30

For virtual camera and synthetic dataset:

>python main.py --features SIFT --dataset synth --data_dirFt ../Data/synth/images/ --out_dirFt ../Data/synth --calibration_mat virtual_cam

### Terrible Example(s)
Doesnt work well with ORB either. After zooming in some, these arent bad, just need post processing.

>python main.py --features SIFT --dataset castle-P30 --data_dirFt ../data/castle-P30/images/ --out_dirFt ../data/castle-P30 --reproj_err_thresh 1

>python main.py --features SIFT --dataset entry-P10 --data_dirFt ../data/entry-P10/images/ --out_dirFt ../data/entry-P10 --reproj_err_thresh 5
