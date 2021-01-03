# Standard imports
import cv2
import numpy as np;
import glob

square_size = 0.22      # 正方形の1辺のサイズ[cm]
pattern_size = (11, 11)  # 交差ポイントの数

pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 ) #チェスボード（X,Y,Z）座標の指定 (Z=0)
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size
objpoints = []
imgpoints = []

def find_corners(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    params = cv2.SimpleBlobDetector_Params()
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 100000

    params.minDistBetweenBlobs = 100

    params.filterByColor = True
    params.filterByConvexity = True

    # tweak these as you see fit
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.8

    # # # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.15

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.15

    params.minThreshold = 0.45

    blobDetector = cv2.SimpleBlobDetector_create(params)
    ret, corners = cv2.findCirclesGrid(gray, (w, h), cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector, None)
    if ret:
        cv2.imshow('image', img)
        cv2.waitKey(1)
        cv2.cornerSubPix(gray, corners, (w, h), (-1, -1), criteria)
        return ret, corners
    return ret, None


w = 11
h = 10
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

images = glob.glob('*.bmp')

for fname in images:
    print("FindImg" + fname)
    # Read image
    img= cv2.imread(fname)

    ret, corners = find_corners(img)
    if ret:
        imgpoints.append(corners.reshape(-1, 2))
        objpoints.append(pattern_points)

print("calculating camera parameter...")
# 内部パラメータを計算
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 計算結果を保存
np.save("mtx", mtx) # カメラ行列
np.save("dist", dist.ravel()) # 歪みパラメータ
# 計算結果を表示
print("RMS = ", ret)
print("mtx = \n", mtx)
print("dist = ", dist.ravel())