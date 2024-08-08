import os, time, glob
import numpy as np
import cv2 as cv
from datetime import datetime

class VisionPos:
    def __init__(self):
        self.mark_size = 7.5
        self.marker_id = 40
        self.scale = 80
        self.refPt = []
        self.cropping = False
        printer = 'http://192.168.88.106/webcam/?action=stream'
        self.cap = cv.VideoCapture(printer)
        self.template = cv.imread("selected_object.jpg", cv.IMREAD_GRAYSCALE)
        # cama_matrix = np.array([[772.7152, 0, 327.7170], # 640x480
        #                  [0, 806.6740, 236.6339],
        #                  [0, 0, 1]])
        # dist_coeffs = np.array([0.0439, -0.2461, 0.0037, 0.0006, 0])
        self.cam_matrix = np.array([[1064.0758, 0, 980.6331], #1920x1080
                         [0, 1032.9971, 535.9386],
                         [0, 0, 1]])
        self.dist_coeffs = np.array([0.0218, -0.0496, -0.0018, 0.0001, 0.0188])
        self.T = np.array([[9.05647898e-01, 3.77814000e-01, 9.54179307e-01, -1.22963166e+02],
                          [1.41144070e-01, 8.81989661e-01, 4.56316442e+00, -1.27373612e+03],
                          [2.11589354e-17, 4.00721123e-16, 9.73179870e-16, 5.00000000e+00],
                          [-3.55574532e-18, -1.69135539e-17, -4.33680869e-17, 1.00000000e+00]])
        self.linear_coeffs = np.array([
            [[-170.40, 194.77], [-176.20, 127.07], [100, 400]],
            [[10, 345], [288, 13], [0, 200]]])

    def gen_marker(self, marker_id=40, marker_size=200):
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        marker_image = cv.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        cv.imwrite('marker.png', marker_image)
        cv.imshow('Marker', marker_image)
        cv.waitKey(0)
        cv.destroyAllWindows()

    def resize_image(self, image, scale_percent):
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        return cv.resize(image, dim, interpolation=cv.INTER_AREA)

    def calibrate(self, primary_points, secondary_points):
        # assert primary_points.shape == secondary_points.shape, \
        #     'The shape of primary and secondary points must match'
        # assert primary_points.shape[0] >= 4, \
        #     'At least 4 points are required for calibration'

        num_points = primary_points.shape[0]
        prime_points_homogeneous = np.hstack([primary_points, np.ones((num_points, 1))])
        sec_points_homogeneous = np.hstack([secondary_points, np.ones((num_points, 1))])

        T, _, _, _ = np.linalg.lstsq(prime_points_homogeneous, sec_points_homogeneous, rcond=None)

        return T.T

    def trans_coords(self, x, points):
        points_homogeneous = np.hstack([points, np.ones((len(points), 1))])
        real_points_homogeneous = points_homogeneous @ x
        real_points = real_points_homogeneous[:, :3] / real_points_homogeneous[:, 3][:, np.newaxis]

        return real_points

    def interpol_coords(self, coords, toohead, cam):
        return [np.interp(coords[i], toohead[i], cam[i]) for i in range(len(coords))]

    def track(self):
        def detect_aruco_markers(frame, aruco_dict, aruco_params):
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # Filter
            # gray = cv.GaussianBlur(gray, (5, 5), 0)
            # gray = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
            corners, ids, rejected = cv.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            return corners, ids

        # ArUco params
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        aruco_params = cv.aruco.DetectorParameters()

        aruco_params.minMarkerPerimeterRate = 0.03
        aruco_params.maxMarkerPerimeterRate = 4.0
        aruco_params.polygonalApproxAccuracyRate = 0.05
        aruco_params.minCornerDistanceRate = 0.05
        aruco_params.minDistanceToBorder = 3
        aruco_params.minOtsuStdDev = 5.0
        aruco_params.adaptiveThreshWinSizeMin = 3
        aruco_params.adaptiveThreshWinSizeMax = 23
        aruco_params.adaptiveThreshWinSizeStep = 10

        # Run
        mean_clocker = [0, [0, 0, 0], []]
        while True:
            ret, frame = self.cap.read()
            if not ret:
                raise exit('No ret')

            # Коррекция искажений
            h, w = frame.shape[:2]
            cam_mtx, roi = cv.getOptimalNewCameraMatrix(self.cam_matrix, self.dist_coeffs, (w, h), 1, (w, h))
            frame = cv.undistort(frame, self.cam_matrix, self.dist_coeffs, None, cam_mtx)

            corners, ids = detect_aruco_markers(frame, aruco_dict, aruco_params)
            if ids is not None:
                for id in range(len(ids)):
                    if ids[id][0] == self.marker_id:
                        mean_clocker[0] += 1
                        cv.aruco.drawDetectedMarkers(frame, corners, ids)
                        # Оценка позы маркера
                        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(
                            corners[id], self.mark_size, self.cam_matrix, self.dist_coeffs)
                        cv.drawFrameAxes(frame, self.cam_matrix, self.dist_coeffs, rvec, tvec, self.mark_size * 2)
                        # cv.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1)

                        # Центр маркера
                        c = corners[id][0]
                        center = [int((c[0][0] + c[2][0]) / 2), int((c[0][1] + c[2][1]) / 2)]
                        cv.circle(frame, center, 5, (0, 0, 255), -1)
                        coords_text = f'{tvec[0][0][0]:.2f}, {tvec[0][0][1]:.2f}, {tvec[0][0][2]:.2f}'
                        real_coords = self.interpol_coords(np.array(tvec[0][0]), *self.linear_coeffs)
                        # real_coords = self.trans_coords(self.T, np.array([tvec[0][0]]))
                        real_coords_text = f'X{real_coords[0]:.2f}, Y{real_coords[1]:.2f}, Z{real_coords[2]:.2f}'

                        cv.putText(frame, coords_text, (center[0] + 25, center[1] - 30),
                                    cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
                        cv.putText(frame, real_coords_text, (center[0] + 25, center[1] - 10),
                                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)

                        # Mean data output
                        mean_clocker[2].append(real_coords)
                        if mean_clocker[0] == 25:
                            coords = np.mean(np.array(mean_clocker[2]), axis=0)
                            if any(float(abs(i - j)) > 2 for i, j in zip(coords, mean_clocker[1])):
                                current_time = datetime.now().strftime('%H:%M:%S')
                                print(f'Marker ID: {ids[id][0]} | Coords: X{coords[0]:.2f},'
                                      f' Y{coords[1]:.2f}, Z{coords[2]:.2f} | Time: {current_time}')
                                mean_clocker = [0, coords, []]
                            mean_clocker[0] = 0

            frame = self.resize_image(frame, 60)
            cv.imshow("Frame", frame)
            key = cv.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        self.cap.release()
        cv.destroyAllWindows()

klass = VisionPos()
klass.track()

# x, y = klass.linear_coeffs

# primary_coords = [9.12, 57.92, 301.15]
# x = klass.line_trans_coords(primary_coords, *klass.linear_coeffs)


# prime = np.array([[9.02, 81.50, 274.06],
#                  [9.46, -136.75, 360.06],
#                  [-125.49, -54.56, 332.46],
#                  [91.31, -54.28, 325.70]])
# sec = np.array([[177.50, 50, 5],
#                [177.50, 250, 5],
#                [60, 177.50, 5],
#                [250, 177.50, 5]])
# T = klass.calibrate(prime, sec)
# print(str(T))

# prime = np.array([[9.02, 81.50, 274.06]])
# sec = np.array([[177.50, 50, 5]])
# T = klass.calibrate(prime, sec)
# # print(str(T))
#
# # realpos = klass.trans_coords(T, [[0, 0, 5000]])
# realpos = _trans_coords(T, [[250, 177.50, 5]])
# print(str(realpos))
