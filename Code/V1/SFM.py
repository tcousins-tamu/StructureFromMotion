import numpy as np
import cv2
# from itertools import zip

# SECTION - Fundamental Matrix Estimation
# 8 Point Algorithm


def EstimateFundamentalMatrix(x1, x2):
    # This function is an alternative to CV2 estimate fund matrix options,
    # This would likely be what we use for submission. This is the 8-pt alg.

    if x1.shape[1] == 2:  # Converting to homogenous coords
        x1 = cv2.convertPointsToHomogeneous(x1)[:, 0, :]
        x2 = cv2.convertPointsToHomogeneous(x2)[:, 0, :]

    A = np.zeros((x1.shape[0], 9))

    # Constructing A matrix
    x1_ = x1.repeat(3, axis=1)
    x2_ = np.tile(x2, (1, 3))

    A = x1_ * x2_

    u, s, v = np.linalg.svd(A)
    F = v[-1, :].reshape((3, 3), order='F')

    u, s, v = np.linalg.svd(F)
    F = u.dot(np.diag(s).dot(v))

    F = F/F[-1, -1]
    return F


def NormalizePts(x):
    mus = x[:, :2].mean(axis=0)
    sigma = x[:, :2].std()
    scale = np.sqrt(2.) / sigma

    transMat = np.array([[1, 0, mus[0]], [0, 1, mus[1]], [0, 0, 1]])
    scaleMat = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, 1]])

    T = scaleMat.dot(transMat)

    xNorm = T.dot(x.T).T

    return xNorm, T

# Normalized 8 pt algorithm


def EstimateFundamentalMatrixNormalized(x1, x2):
    if x1.shape[1] == 2:  # converting to homogenous coordinates if not already
        x1 = cv2.convertPointsToHomogeneous(x1)[:, 0, :]
        x2 = cv2.convertPointsToHomogeneous(x2)[:, 0, :]

    x1Norm, T1 = NormalizePts(x1)
    x2Norm, T2 = NormalizePts(x2)

    F = EstimateFundamentalMatrix(x1Norm, x2Norm)

    F = T1.T.dot(F.dot(T2))
    return F

# RANSAC algorithm


def EstimateFundamentalMatrixRANSAC(img1pts, img2pts, outlierThres, prob=None, iters=None):
    if img1pts.shape[1] == 2:  # converting to homogenous coordinates if not already
        img1pts = cv2.convertPointsToHomogeneous(img1pts)[:, 0, :]
        img2pts = cv2.convertPointsToHomogeneous(img2pts)[:, 0, :]

    bestInliers, bestF, bestmask = 0, None, None

    for i in range(iters):
        # Selecting 8 random points
        mask = np.random.randint(low=0, high=img1pts.shape[0], size=(8,))
        img1PtsIter = img1pts[mask]
        img2PtsIter = img2pts[mask]

        # Fitting fundamental matrix and evaluating error (using 8 points alg)
        F = EstimateFundamentalMatrixNormalized(img1PtsIter, img2PtsIter)
        # error = np.linalg.norm(F - img1pts.dot(img2pts.T))
        error = SampsonError(F, img1pts, img2pts)
        mask = error < outlierThres
        numInliers = np.sum(mask)

        # Updating the best measurements
        if bestInliers < numInliers:
            bestInliers = numInliers
            bestF = F
            bestmask = mask

    F = EstimateFundamentalMatrixNormalized(
        img1pts[bestmask], img2pts[bestmask])
    return F, bestmask

# Error used in the RANSAC algorithm


def SampsonError(F, x1, x2):
    num = np.sum(x1.dot(F) * x2, axis=-1)

    F_src = np.dot(F, x1.T)
    Ft_dst = np.dot(F.T, x2.T)

    dst_F_src = np.sum(x2 * F_src.T, axis=1)

    return np.abs(dst_F_src) / np.sqrt(F_src[0] ** 2 +
                                       F_src[1] ** 2 + Ft_dst[0] ** 2 + Ft_dst[1] ** 2)

# SECTION - Epipolar line calculations
# Epipolar lines are basically the line intersects of points on both image planes


def ComputeEpiline(pts, index, F):
    if pts.shape[1] == 2:  # converting to homogenous coordinates if not already
        pts = cv2.convertPointsToHomogeneous(pts)[:, 0, :]

    if index == 1:
        lines = F.dot(pts.T)
    elif index == 2:
        lines = F.T.dot(pts.T)

    return lines.T

# Extracting the camera position from the array provided.
# I need to look into how the camera position is provided initallly.

# SECTION - Calculating camera poses


def ExtractCameraPoses(E):
    u, d, v = np.linalg.svd(E)
    W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    Rs, Cs = np.zeros((4, 3, 3)), np.zeros((4, 3))

    t = u[:, -1]
    R1 = u.dot(W.dot(v))
    R2 = u.dot(W.T.dot(v))

    if np.linalg.det(R1) < 0:
        R1 = R1 * -1

    if np.linalg.det(R2) < 0:
        R2 = R2 * -1

    return R1, R2, t

# Triangulating points using Direct Linear Transformation

# SECTION - Trianulating points based on camera poses
# NOTE - I do not fully understand the camera poses sections


def GetTriangulatedPts(img1pts, img2pts, K, R, t, triangulateFunc, Rbase=None, tbase=None):
    img1ptsHom = cv2.convertPointsToHomogeneous(img1pts)[:, 0, :]
    img2ptsHom = cv2.convertPointsToHomogeneous(img2pts)[:, 0, :]

    img1ptsNorm = (np.linalg.inv(K).dot(img1ptsHom.T)).T
    img2ptsNorm = (np.linalg.inv(K).dot(img2ptsHom.T)).T

    img1ptsNorm = cv2.convertPointsFromHomogeneous(img1ptsNorm)[:, 0, :]
    img2ptsNorm = cv2.convertPointsFromHomogeneous(img2ptsNorm)[:, 0, :]

    pts4d = triangulateFunc(np.eye(3, 4), np.hstack(
        (R, t)), img1ptsNorm.T, img2ptsNorm.T)
    pts3d = cv2.convertPointsFromHomogeneous(pts4d.T)[:, 0, :]

    return pts3d

# This is a homebrew Direct Linear Transformation triangulation function
# This is intended to replate  cv2.triangulatePoints


def Triangulate(P1, P2, img1pts, img2pts):
    img1pts, img2pts = img1pts.T, img2pts.T
    if img1pts.shape[1] == 2:
        # converting to homogenous coordinates if not already
        img1pts = cv2.convertPointsToHomogeneous(img1pts)[:, 0, :]
        img2pts = cv2.convertPointsToHomogeneous(img2pts)[:, 0, :]

    out = np.zeros((img1pts.shape[0], 4))

    for i, (img1pt, img2pt) in enumerate(zip(img1pts, img2pts)):
        img1pt_cross, img2pt_cross = Vec2Skew(img1pt), Vec2Skew(img2pt)

        A = []
        A.append(img1pt_cross.dot(P1))
        A.append(img2pt_cross.dot(P2))

        A = np.concatenate(A, axis=0)

        u, s, v = np.linalg.svd(A)
        out[i, :] = v[-1, :]

    return out.T

# Helper Function for triangulate


def Vec2Skew(vec):
    return np.array([[0, -vec[2], vec[1]], [vec[2], 0, -vec[0]], [-vec[1], vec[0], 0]])


# SECTION - Choosing best camera pose

# Selecting a single camera pose from a series of potential camera poses
def DisambiguateCameraPose(configSet):
    maxfrontpts = -1
    for R, t, pts3d in configSet:
        count = CountFrontOfBothCameras(pts3d, R, t)

        if count > maxfrontpts:
            maxfrontpts = count
            bestR, bestt = R, t

    return bestR, bestt, maxfrontpts
# Helper Function for disambiguate camera pose


def CountFrontOfBothCameras(X, R, t):
    isfrontcam1 = X[:, -1] > 0
    isfrontcam2 = TransformCoordPts(X, R, t)[:, -1] > 0

    return np.sum(isfrontcam1 & isfrontcam2)

# Helper Function for countfrontofbothcameras


def TransformCoordPts(X, R, t):
    return (R.dot(X.T)+t).T


# Section - Reprojection, verifying if our mappings have low error when mapped back down.
def ComputeReprojections(X, R, t, K):
    """
    X: (n,3) 3D triangulated points in world coordinate system
    R: (3,3) Rotation Matrix to convert from world to camera coordinate system
    t: (3,1) Translation vector (from camera's origin to world's origin)
    K: (3,3) Camera calibration matrix

    out: (n,2) Projected points into image plane"""
    outh = K.dot(R.dot(X.T)+t)
    out = cv2.convertPointsFromHomogeneous(outh.T)[:, 0, :]
    return out


def ComputeReprojectionError(x2d, x2dreproj):
    """
    x2d: (n,2) Ground truth indices of SIFT features
    x2dreproj: (n,2) Reprojected indices of triangulated points of SIFT features

    out: (scalar) Mean reprojection error of points"""
    return np.mean(np.sqrt(np.sum((x2d-x2dreproj)**2, axis=-1)))
