import utils as ut  # Using the utils that were described in the tutorial
import SFM as sfmnp  # Importing the SFM module from the tutorial

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import numpy as np

# Auto-reloading external modules, dont know what this does actually
# %load_ext autoreload
# %autoreload 2

# SECTION - Reading in a pair of images and comparing SIFT matches (technically they use SURF in this tutorial)
# Reading in two images
im1 = cv2.imread("../../Data/fountain-P11/images/0004.jpg")
im2 = cv2.imread("../../Data/fountain-P11/images/0006.jpg")

# Converting from BGR to RGB format
im1 = im1[:, :, ::-1]
im2 = im2[:, :, ::-1]

# Getting SIFT/SURF features for image mapping (this might take a while)
kp1, desc1, kp2, desc2, matches = ut.GetImageMatches(im1, im2)

# Algining the KEYPOINT vectors
img1pts, img2pts, img1idx, img2idx = ut.GetAlignedMatches(
    kp1, kp2, matches)

# SECTION - Fundamental Matrix Computation
img1pts_, img2pts_ = img1pts, img2pts

# The fundamental matrix is, in essence, a translation matrix
# CV2 ransac alg versus homebrew method, will likely be using homebrew for future work
# [F, mask] = cv2.findFundamentalMat(img1pts_, img2pts_, method=cv2.FM_RANSAC, ransacReprojThreshold=3, confidence=.1)
# mask = mask.astype(bool).flatten()

F, mask = sfmnp.EstimateFundamentalMatrixRANSAC(
    img1pts, img2pts, .1, iters=20000)

# Epipolar Lines Calculations
# epipolar lines are basically the line intersect of points on separate image planes
# There is also a cv2 implementation here that is worth looking into.
# lines2 = cv.computeCorrespondingEpilines(img1Pts)

lines2 = sfmnp.ComputeEpiline(img1pts[mask], 1, F)
lines1 = sfmnp.ComputeEpiline(img2pts[mask], 2, F)

# Visualizing the epipolar lines, this function was ripped entirely from tutorial
# tup = ut.drawlines(im2, im1, lines2, img2pts[mask], img1pts[mask], drawOnly=10,
#                    linesize=10, circlesize=30)
# epilines2 = np.concatenate(tup[::-1], axis=1)
# plt.figure(figsize=(9, 4))
# plt.imshow(epilines2)

# TODO - Figure out what the K array will be for Tolga's video
K = np.array([[2759.48, 0, 1520.69], [0, 2764.16, 1006.81], [0, 0, 1]])
E = K.T.dot(F.dot(K))

# Extract the camera position from the K array
R1, R2, t = sfmnp.ExtractCameraPoses(E)
t = t[:, np.newaxis]

# Visualizing the camera pose.
# for R_ in [R1, R2]:
#     for t_ in [t, -t]:

#         fig = plt.figure(figsize=(9, 6))
#         ax = fig.add_subplot(111, projection='3d')
#         ax.set_xlabel('X')
#         ax.set_ylabel('Y')
#         ax.set_zlabel('Z')

#         ut.PlotCamera(np.eye(3, 3), np.zeros((3,)), ax)
#         ut.PlotCamera(R_, t_[:, 0], ax)


# SECTION - 3D scene estimations
# The way this will be done is Direct Linear Transformation
pts3dgt = sfmnp.GetTriangulatedPts(
    img1pts[mask], img2pts[mask], K, R2, t, cv2.triangulatePoints)

# Getting the camera poses for each possible configuration
configSet = [None, None, None, None]
configSet[0] = (R1, t, sfmnp.GetTriangulatedPts(
    img1pts[mask], img2pts[mask], K, R1, t, cv2.triangulatePoints))
configSet[1] = (R1, -t, sfmnp.GetTriangulatedPts(img1pts[mask],
                img2pts[mask], K, R1, -t, cv2.triangulatePoints))
configSet[2] = (R2, t, sfmnp.GetTriangulatedPts(
    img1pts[mask], img2pts[mask], K, R2, t, cv2.triangulatePoints))
configSet[3] = (R2, -t, sfmnp.GetTriangulatedPts(img1pts[mask],
                img2pts[mask], K, R2, -t, cv2.triangulatePoints))

# Visualizing the triangulated points of configurations
# for cs in configSet:
#     fig = plt.figure(figsize=(9, 6))
#     ax = fig.add_subplot(111, projection='3d')
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')

#     ut.PlotCamera(np.eye(3, 3), np.zeros((3,)), ax, scale=5, depth=5)
#     ut.PlotCamera(cs[0], cs[1][:, 0], ax, scale=5, depth=5)

#     pts3d = cs[-1]
#     ax.scatter3D(pts3d[:, 0], pts3d[:, 1], pts3d[:, 2])

#     ax.set_xlim(left=-50, right=50)
#     ax.set_ylim(bottom=-50, top=50)
#     ax.set_zlim(bottom=-20, top=20)
# plt.show()

# Camera Pose Disambiguation (selecting one of the corresponding potential poses)
_, Rgt, tgt, mask2 = cv2.recoverPose(E, img1pts[mask], img2pts[mask], K)
R, t, count = sfmnp.DisambiguateCameraPose(configSet)

# We are viewing the point cloud using meshlab instead of matplotlb
pts3d = sfmnp.GetTriangulatedPts(
    img1pts[mask], img2pts[mask], K, R, t, cv2.triangulatePoints)

# Helper function that converts the points to meshlab format
print("POINTS SHAPE: ", pts3d.shape)
ut.pts2ply(pts3d, 'castle_2view.ply')

# SECTION - Reprojection error, evaluation. This is the error shown when a point is repreojected into the 2D plane
img1ptsReproj = sfmnp.ComputeReprojections(
    pts3d, np.eye(3, 3), np.zeros((3, 1)), K)
img2ptsReproj = sfmnp.ComputeReprojections(pts3d, R, t, K)

err2 = sfmnp.ComputeReprojectionError(img2pts[mask], img2ptsReproj)
err1 = sfmnp.ComputeReprojectionError(img1pts[mask], img1ptsReproj)

# Visualizing the reprojection error
fig, ax = plt.subplots(figsize=(9, 5))
ut.DrawCorrespondences(im1, img1pts[mask], img1ptsReproj, ax)

fig, ax = plt.subplots(figsize=(9, 5))
ut.DrawCorrespondences(im2, img2pts[mask], img2ptsReproj, ax)

# NEW CAMERA REGISTRATION
img3 = cv2.imread('../../data/castle-P19/images/0006.jpg')
img3 = img3[:, :, ::-1]
surfer = cv2.ORB_create()
kp3, desc3 = surfer.detectAndCompute(img3, None)

# plt.figure()
# plt.imshow(img3)
# plt.show()

# Using the surf algorithm to find features and then matching them to ones we already know
img3pts, pts3dpts = ut.Find2D3DMatches(
    desc1, img1idx, desc2, img2idx, desc3, kp3, mask, pts3d)

Rtest, ttest = ut.LinearPnP(pts3dpts, img3pts, K)

# This says using RANSAC, but I dont quite know how its in use for camera pose esimations
retval, Rvec, tnew, mask3gt = cv2.solvePnPRansac(pts3dpts[:, np.newaxis].astype('float32'), img3pts[:, np.newaxis].astype('float32'),
                                                 K, None, confidence=.99, flags=cv2.SOLVEPNP_DLS)
Rnew, _ = cv2.Rodrigues(Rvec)  # modifies the rotation vector

tnew = tnew[:, 0]

fig = plt.figure(figsize=(9, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ut.PlotCamera(np.eye(3, 3), np.zeros((3,)), ax)
ut.PlotCamera(R, t[:, 0], ax)
ut.PlotCamera(Rnew, tnew, ax, faceColor='red')

# NEXT SECTION IS RETRIANGULATION
# Retriangulation improves the 3D estimate of the points with the new information
kpNew, descNew = kp3, desc3
kpOld, descOld = kp1, desc1
ROld, tOld = np.eye(3), np.zeros((3, 1))

accPts = []
for (ROld, tOld, kpOld, descOld) in [(np.eye(3), np.zeros((3, 1)), kpOld, desc1), (R, t, kp2, desc2)]:
    matcher = cv2.BFMatcher(crossCheck=True)
    matches = matcher.match(descOld, desc3)
    matches = sorted(matches, key=lambda x: x.distance)
    imgOldPts, imgNewPts, _, _ = ut.GetAlignedMatches(
        kpOld, kpNew, matches)

    # Pruning the matches using the fundamental matrix.
    F, mask = sfmnp.EstimateFundamentalMatrixRANSAC(
        imgOldPts, imgNewPts, .1, iters=20000)
    # F, mask = cv2.findFundamentalMat(
    #     imgOldPts, imgNewPts, method=cv2.FM_RANSAC, param1=.1, param2=.99)
    mask = mask.flatten().astype(bool)
    imgOldPts = imgOldPts[mask]
    imgNewPts = imgNewPts[mask]

    # Triangulating new points
    newPts = sfmnp.GetTriangulatedPts(
        imgOldPts, imgNewPts, K, Rnew, tnew[:, np.newaxis], cv2.triangulatePoints, ROld, tOld)
    accPts.append(newPts)
accPts.append(pts3d)
print("NPly points shape: ", np.concatenate((accPts), axis=0).shape)
ut.pts2ply(np.concatenate((accPts), axis=0), 'castle_nview.ply')
plt.show()
