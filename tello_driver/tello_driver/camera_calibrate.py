import numpy as np
import cv2
import glob
import yaml


class CameraCalibration:
    def __init__(self, width=None, height=None):
        self.width = width
        self.height = height

        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((6*7,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.
    
    def process_frame(self, img):
        if self.width is None or self.height is None:
            self.width, self.height = img.shape[:-1]

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            self.objpoints.append(self.objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
            self.imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        return img

    def calibrate(self, output="camera.yaml"):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, (self.width,self.height), None,None)
        with open(output, 'w+') as f:
            data = {
                "image_width": self.width,
                "image_height": self.height,
                "camera_name": "",
                "camera_matrix": {
                    "rows": mtx.shape[0],
                    "cols": mtx.shape[1],
                    "data": mtx.flatten().tolist(),
                },
                "distortion_model": "plumb_bob",
                "distortion_coefficients": {
                    "rows": dist.shape[0],
                    "cols": dist.shape[1],
                    "data": dist.flatten().tolist(),
                },
            }
            yaml.dump(data, f)


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    camera_calib = CameraCalibration(width, height)

    while True:
        ret, frame = cap.read()
        img = camera_calib.process_frame(frame)
        cv2.imshow('img', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    camera_calib.calibrate()
    cap.release()
    cv2.destroyAllWindows()