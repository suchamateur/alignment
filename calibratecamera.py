import numpy as np
import cv2
import ConfigParser
width=640
height=480
# cap=cv2.VideoCapture(0)
# cap.set(3,width)
# cap.set(4,height)
# i=0
# while 1:
#     ret, frame = cap.read()
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('s'):
#         fn='c:\\tmp\\calibration\\sample%d.jpg'%i
#         cv2.imwrite(fn,frame)
#         i=i+1
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# cap.release()
criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
objp=np.zeros((8*5,3),np.float32)
objp[:,:2]=np.mgrid[0:5,0:8].T.reshape(-1,2)
objpoints=[]
imgpoints=[]
i=0
for fi in range(0,18):
    imgname='c:\\tmp\\calibration\\sample%d.jpg'%fi
    img = cv2.imread(imgname, 1)    
    print(imgname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners=cv2.findChessboardCorners(gray,(5,8),None)
    
    if ret==True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)
        
        cv2.drawChessboardCorners(img, (5,8), corners, ret)
        cv2.imshow('img',img)
        cv2.waitKey(100)
        
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print(mtx)
print(dist)
config = ConfigParser.SafeConfigParser()
config.read('c:\\tmp\\detector_surface.ini')
if config.has_section('CameraCalibration'):
    print('update camera calibration results')
    config.set('CameraCalibration', 'calibrated', 'true')
    config.set('CameraCalibration', 'fx', '%f'%mtx[0][0])
    config.set('CameraCalibration', 'fy', '%f'%mtx[1][1])
    config.set('CameraCalibration', 'cx', '%f'%mtx[0][2])
    config.set('CameraCalibration', 'cy', '%f'%mtx[1][2])
    config.set('CameraCalibration', 'k1', '%f'%dist[0][0])
    config.set('CameraCalibration', 'k2', '%f'%dist[0][1])
    config.set('CameraCalibration', 'k3', '%f'%dist[0][4])
    config.set('CameraCalibration', 'p1', '%f'%dist[0][2])
    config.set('CameraCalibration', 'p2', '%f'%dist[0][3])
    with open('c:\\tmp\\detector_surface.ini','w') as configfile:
        config.write(configfile)

newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(width,height),1,(width,height))
mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(width,height),5)


# cap2 = cv2.VideoCapture(0)
# while 1:
#     ret, frame = cap2.read()
#     cv2.imshow('frame',frame)
#     
#     undist = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)
#     cv2.imshow('undistort',undist)
# 
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# # cap.release()
# cap2.release()

cv2.destroyAllWindows()