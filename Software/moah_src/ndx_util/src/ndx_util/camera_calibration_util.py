#!/usr/bin/env python

import numpy as np
import yaml
import cv2
import cv2.aruco as aruco
from math import pi
import time
import os


class CameraCalibration(object):

    def __init__(self):

        home = os.path.expanduser("~")

        # Define video capture
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 90)
        self.out = None

        # Folders and Files
        self.filename = os.path.join(home, 'config_files', 'aruco_2.avi')
        self.camera_cal_file = 'calibration.yaml'
        self.calibration_out = os.path.join(home, 'config_files', self.camera_cal_file)
        self.video_frame_store = os.path.join(home, 'config_files', 'out')
        # self.video_frame_store is None if output frames from Video not Required
        self.video_frame_store = None

        self.framestamp = 200

    def record_cal_video(self):
        # Video recording
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.filename, fourcc, 60.0, (1280, 720))

        while True:
            # Capture frames
            ret, frame = self.cap.read()

            # Convert frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.out.write(frame)

            # Display the resulting frame
            cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.shutdown()

    def calibrate_camera(self):

        source = cv2.VideoCapture(self.filename)
        # square_size = float(args.get('--square_size', 1.0))

        pattern_size = (8, 5)
        pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
        pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
        # pattern_points *= square_size

        obj_points = []
        img_points = []
        h, w = 0, 0
        i = -1
        while True:
            i += 1
            if isinstance(source, list):
                # glob
                if i == len(source):
                    break
                img = cv2.imread(source[i])
            else:
                # cv2.VideoCapture
                retval, img = source.read()
                if not retval:
                    break
                if i % self.framestamp != 0:
                    continue

            print('Searching for chessboard in frame ' + str(i) + '...'),
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            h, w = img.shape[:2]
            found, corners = cv2.findChessboardCorners(img, pattern_size, flags=cv2.CALIB_CB_FILTER_QUADS)
            if found:
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
                cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
            if self.video_frame_store:
                img_chess = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(img_chess, pattern_size, corners, found)
                cv2.imwrite(os.path.join(self.video_frame_store, '%04d.png' % i), img_chess)
            if not found:
                print('not found')
                continue
            img_points.append(corners.reshape(1, -1, 2))
            obj_points.append(pattern_points.reshape(1, -1, 3))

            print('OK')

        print('\nPerforming calibration...')
        rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
        print("RMS: {}".format(rms))
        print("camera matrix: {}".format(camera_matrix))
        print("distortion coefficients: {}".format(dist_coefs.ravel()))

        camera_matrix = np.reshape(camera_matrix, [1, camera_matrix.shape[0] * camera_matrix.shape[1]])
        calibration = {'rms': rms, 'camera_matrix': camera_matrix.tolist(), 'dist_coefs': dist_coefs.tolist()}
        with open(self.calibration_out, 'w') as fw:
            yaml.dump(calibration, fw)

    def shutdown(self):
        self.out.release()
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    c = CameraCalibration()
    c.record_cal_video()
    c.calibrate_camera()
