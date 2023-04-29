This work is heavily influenced by a repository located here:
https://github.com/muneebaadil/how-to-sfm

main.py uses two of their classes/functions, feature match and SFM. There are more details located in the repo listed above. This code can be run using python main.py and will run the default configuration.

The repo above comes with a solid tutorial that introduces you to the flow required for SFM. This includes feature calculation using sift, camera pose estimation and projections. 

It should be noted that, if you do NOT have access to SIFT or SURF, ORB is a good alternative.
#TODO###
1. THERE ARE A TON OF POORLY CALCULATED POINTS THAT NEED TO BE CLEANED.
2. CONVERTING THE POINT CLOUDS TO STL. THIS SHOULDNT BE TOO HARD AFTER CLEANING. MIGHT HAVE TO ASSUME AN ARBITRARY DEPTH.
3. CALCULATING THE K VALUES FOR TOLGA'S VIDEOS


#TEST CASES####
#Currently a pretty good example, some strange outliers.
python SfM2STL.py --features SIFT

#Great case, needs cleaning BAD
python SfM2STL.py --features SIFT --dataset Herz-Jesus-P25 --data_dirFt ../data/Herz-Jesus-P25/images/ --out_dirFt ../data/Herz-Jesus-P25

#Forgot the significance of this case, was likely just testing params
python SfM2STL.py --features SIFT --outlier_thres 1 --reprojection_thres 1 #Try this one after adding code to remove things with a high reproj eror

#Terrible Example(s), doesnt work well with ORB either. After zooming in some, these arent bad, just need post processing.

python SfM2STL.py --features SIFT --dataset castle-P30 --data_dirFt ../data/castle-P30/images/ --out_dirFt ../data/castle-P30
python SfM2STL.py --features SIFT --dataset castle-P19 --data_dirFt ../data/castle-P19/images/ --out_dirFt ../data/castle-P19
python SfM2STL.py --features SIFT --dataset Herz-Jesus-P25 --data_dirFt ../data/Herz-Jesus-P25/images/ --out_dirFt ../data/Herz-Jesus-P25
python SfM2STL.py --features SIFT --dataset Herz-Jesus-P8 --data_dirFt ../data/Herz-Jesus-P8/images/ --out_dirFt ../data/Herz-Jesus-P8
python SfM2STL.py --features SIFT --dataset entry-P10 --data_dirFt ../data/entry-P10/images/ --out_dirFt ../data/entry-P10
