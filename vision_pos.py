import os, time, glob
import numpy as np
import cv2, cv2.aruco as aruco

# ser = serial.Serial('/home/pi/printer_data/comms/klippy.serial', 115200)
# ser.write(f'SET_VELOCITY_LIMIT ACCEL=10000  \r\n'.encode('utf-8'))
# ser.write(f'G0 F1500000  \r\n'.encode('utf-8'))

# cap = cv2.VideoCapture(0)
# if not cap.isOpened():
#     print("Error: Could not open video capture")
#     exit(1)
# while True:
#     ret, frame = cap.read()
#     cv2.imshow('Camera', frame)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break
#
# cap.release()
# cv2.destroyAllWindows()


# old_vals = [0, 0]

def get_salad_mask(img):
    lower_salad = np.array([30, 40, 40])
    upper_salad = np.array([90, 255, 255])
    return cv2.inRange(img, lower_salad, upper_salad)

def get_green_mask(img):
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    return cv2.inRange(img, lower_green, upper_green)
def get_yellow_mask(img):
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    return cv2.inRange(img, lower_yellow, upper_yellow)

# min_area = 100
# max_area = 10000
#
# cap = cv2.VideoCapture(0)
#
# while cap.isOpened():
#     now = time.time()
#     ret, frame = cap.read()
#     # cv2.imshow('Camera', frame)
#     # cv2.waitKey()
#     if cv2.waitKey(1) & 0xFF == 27:
#         break
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     thresholded = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)[1]
#
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     masks = get_salad_mask(hsv), get_green_mask(hsv), get_yellow_mask(hsv)
#     contours = []
#     for mask in masks:
#         for cal in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
#             contours.append(cal)
#
#     result = frame.copy()
#     # for contour in contours:
#     #     # Аппроксимация контура, чтобы он был гладким
#     #     approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
#     #
#     #     # Проверка, является ли контур кругом
#     #     if len(approx) > 20:
#     #         # Нарисовать контур
#     #         cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
#
#     for contour in contours:
#         if min_area <= cv2.contourArea(contour) <= max_area:
#             cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
#
#     # print('cycle')
#     cv2.imshow('Camera', result)
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # thresholded = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)[1]
    #
    # canny = cv2.Canny(thresholded, 10, 250)
    # contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # vals = [-contours[0][0][0][0] // 5, contours[0][0][0][1] // 5]
    # if abs(vals[0] - old_vals[0]) > 1 or abs(vals[1] - old_vals[1]) > 1:
    #     # ser.write(f'G0 X{vals[0]} Y{vals[1]}  \r\n'.encode('utf-8'))
    #     print(f'G0 X{-contours[0][0][0][0] // 5} Y{contours[0][0][0][1] // 5}  \r\n', time.time() - t_s)
    # old_vals = vals.copy()

# cap.release()
# cv2.destroyAllWindows()
# ser.close()

# roi = None
# ref_point = []
# tracker = None
# initialized = False
#
# # Инициализация фильтра Калмана
# kalman = cv2.KalmanFilter(4, 2)
# kalman.transitionMatrix = np.array([[1, 0, 1, 0],
#                                     [0, 1, 0, 1],
#                                     [0, 0, 1, 0],
#                                     [0, 0, 0, 1]], np.float32)
# kalman.measurementMatrix = np.array([[1, 0, 0, 0],
#                                      [0, 1, 0, 0]], np.float32)
# kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
# kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
# kalman.errorCovPost = np.eye(4, dtype=np.float32)
# kalman.statePost = np.zeros((4, 1), np.float32)
#
# # Функция для обработки события мыши
# def click_and_crop(event, x, y, flags, param):
#     global ref_point, tracker, initialized, kalman
#     if event == cv2.EVENT_LBUTTONDOWN:
#         ref_point = [(x, y)]
#     elif event == cv2.EVENT_LBUTTONUP:
#         ref_point.append((x, y))
#         cv2.rectangle(temp_frame, ref_point[0], ref_point[1], (0, 255, 0), 2)
#         cv2.imshow("Frame", temp_frame)
#         roi = (min(ref_point[0][0], ref_point[1][0]), min(ref_point[0][1], ref_point[1][1]),
#                abs(ref_point[1][0] - ref_point[0][0]), abs(ref_point[1][1] - ref_point[0][1]))
#         if roi:
#             x, y, w, h = roi
#             bbox = (x, y, w, h)
#             tracker = cv2.TrackerCSRT_create()
#             tracker.init(frame, bbox)
#             initialized = True
#             kalman.statePost = np.array([[x + w / 2], [y + h / 2], [0], [0]], np.float32)
#
# printer = 'http://192.168.88.106/webcam/?action=stream'
# cap = cv2.VideoCapture(printer)
# cv2.namedWindow("Frame")
# cv2.setMouseCallback("Frame", click_and_crop)
#
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print('No ret')
#         break
#
#     temp_frame = frame.copy()
#     if initialized and tracker is not None:
#         success, bbox = tracker.update(frame)
#         if success:
#             x, y, w, h = [int(v) for v in bbox]
#             center = np.array([[x + w / 2], [y + h / 2]], np.float32)
#
#             kalman.correct(center)
#             prediction = kalman.predict()
#             pred_x, pred_y = int(prediction[0]), int(prediction[1])
#
#             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
#             cv2.circle(frame, (pred_x, pred_y), 5, (0, 0, 255), -1)
#             cv2.putText(frame, "Predicted position", (pred_x + 10, pred_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#         else:
#             print('Lost tracker')
#     cv2.imshow("Frame", frame)
#
#     key = cv2.waitKey(1) & 0xFF
#     if key == ord('q'):
#         break
#     elif key == ord('r'):
#         roi = None
#         ref_point = []
#         initialized = False
#         tracker = None
#
# cap.release()
# cv2.destroyAllWindows()

def __init__():
    global scale, refPt, cropping, cap, template
    scale = 80
    refPt = []
    cropping = False
    printer = 'http://192.168.88.106/webcam/?action=stream'
    cap = cv2.VideoCapture(printer)
    template = cv2.imread("selected_object.jpg", cv2.IMREAD_GRAYSCALE)

# def init_model():
#     global template
#     def click_and_crop(event, x, y, flags, param):
#         global refPt, cropping
#         # Начало выбора области
#         if event == cv2.EVENT_LBUTTONDOWN:
#             refPt = [(x, y)]
#             cropping = True
#         # Обновление текущего прямоугольника
#         elif event == cv2.EVENT_MOUSEMOVE and cropping:
#             image_copy = image.copy()
#             cv2.rectangle(image_copy, refPt[0], (x, y), (0, 255, 0), 2)
#             cv2.imshow("image", image_copy)
#         # Завершение выбора области
#         elif event == cv2.EVENT_LBUTTONUP:
#             refPt.append((x, y))
#             cropping = False
#             cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
#             cv2.imshow("image", image)
#     # Чтение первого кадра из камеры
#     ret, image = cap.read()
#     if not ret:
#         print("Не удалось захватить кадр")
#         cap.release()
#         exit()
#     # Копия изображения для отображения
#     clone = image.copy()
#     cv2.namedWindow("image")
#     cv2.setMouseCallback("image", click_and_crop)
#     # Отображение изображения и ожидание выбора области
#     while True:
#         cv2.imshow("image", image)
#         key = cv2.waitKey(1) & 0xFF
#         # Нажатие 'r' для сброса выбора
#         if key == ord("r"):
#             image = clone.copy()
#         # Нажатие 'c' для завершения выбора
#         elif key == ord("c"):
#             break
#     # Проверка выбора области
#     if len(refPt) == 2:
#         roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
#         cv2.imshow("ROI", roi)
#         cv2.waitKey(0)
#         cv2.imwrite("selected_object.jpg", roi)
#     else:
#         print("Область не выбрана")
#     # Завершение работы с камерой и окнами
#     # cap.release()
#     cv2.destroyAllWindows()
#     template = cv2.imread("selected_object.jpg", cv2.IMREAD_GRAYSCALE)

def resize_image(image, scale_percent):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)


# def find_object(frame, template):
#     # Инициализация ORB детектора
#     orb = cv2.ORB_create()
#     # Поиск ключевых точек и дескрипторов в текущем кадре и шаблоне
#     kp1, des1 = orb.detectAndCompute(template, None)
#     kp2, des2 = orb.detectAndCompute(frame, None)
#     # Инициализация BFMatcher для сопоставления ключевых точек
#     bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#     matches = bf.match(des1, des2)
#     # Сортировка совпадений по расстоянию
#     matches = sorted(matches, key=lambda x: x.distance)
#     # Отображение ключевых точек и совпадений
#     result = cv2.drawMatches(template, kp1, frame, kp2, matches[:10], None,
#                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
#     if len(matches) > 0:
#         # Вычисление координат объекта
#         src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 2)
#         dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 2)
#         # Вычисление гомографии
#         M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
#         if M is not None:
#             h, w = template.shape[:2]
#             pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
#             dst = cv2.perspectiveTransform(pts, M)
#
#             # Рисование границ объекта на текущем кадре
#             frame = cv2.polylines(frame, [np.int32(dst)], True, (0, 255, 0), 3, cv2.LINE_AA)
#
#             # Координаты центра объекта в плоскости XY
#             center_x = np.mean(dst[:, 0, 0])
#             center_y = np.mean(dst[:, 0, 1])
#             print(f"Object center: ({center_x:.2f}, {center_y:.2f})")
#         else:
#             print("Гомографию найти не удалось.")
#     else:
#         print("Совпадения не найдены.")
#
#     return frame, result
#
# def t():
#     global template, scale
#     template = resize_image(template, scale)
#
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         frame_resized = resize_image(gray, scale)
#
#         frame_with_object, result_with_matches = find_object(frame_resized, template)
#
#         frame_with_object_resized = resize_image(frame_with_object, scale)
#         result_with_matches_resized = resize_image(result_with_matches, scale)
#
#         # Отображение результатов
#         cv2.imshow("Frame with Object", frame_with_object_resized)
#         cv2.imshow("Matches", result_with_matches_resized)
#
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord("q"):
#             break
#
#     cap.release()
#     cv2.destroyAllWindows()

def init_model(marker_id=40, marker_size=200):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    cv2.imwrite('marker.png', marker_image)
    cv2.imshow('Marker', marker_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# def calibrate():
#     CHECKERBOARD = (6, 9)
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#     objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
#     objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
#
#     objpoints = []  # 3d points in real world space
#     imgpoints = []  # 2d points in image plane
#
#     images = glob.glob('calibration_images/*.jpg')
#
#     for fname in images:
#         img = cv2.imread(fname)
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
#         if ret:
#             objpoints.append(objp)
#             imgpoints.append(corners)
#             cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
#             cv2.imshow('img', img)
#             cv2.waitKey(100)
#     cv2.destroyAllWindows()
#
#     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#
#     # Сохранение калибровочных данных
#     np.savez('calibration.npz', mtx=mtx, dist=dist)
points = []
selecting = False
def calibrate():
    def select_points(event, x, y, flags, param):
        global points, selecting
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 4:
                points.append((x, y))
                if len(points) == 4:
                    cv2.line(img, points[0], points[1], (0, 255, 0), 2)
                    cv2.line(img, points[1], points[2], (0, 255, 0), 2)
                    cv2.line(img, points[2], points[3], (0, 255, 0), 2)
                    cv2.line(img, points[3], points[0], (0, 255, 0), 2)
                    cv2.imshow("Select Points", img)
            elif len(points) == 4 and not selecting:
                selecting = True
                cv2.destroyWindow("Select Points")

    ret, img = cap.read()
    if not ret:
        print("Не удалось захватить изображение.")
        cap.release()
        cv2.destroyAllWindows()
        exit()
    img_copy = img.copy()

    cv2.imshow("Select Points", img_copy)
    cv2.setMouseCallback("Select Points", select_points)

    while not selecting:
        cv2.waitKey(100)

    print("Выбрано 4 точки:", points)
    cap.release()

    # Определение исходных точек (реальные точки на изображении)
    src_points = np.array(points, dtype=np.float32)

    # Определение целевых точек прямоугольник для коррекции искажений
    dst_points = np.array([
        [0, 0],
        [100, 0],
        [100, 100],
        [0, 100]
    ], dtype=np.float32)

    # Вычисление матрицы преобразования
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    # Применение преобразования к изображению
    warped_img = cv2.warpPerspective(img, matrix, (img.shape[1], img.shape[0]))

    cv2.imshow("Original Image", img)
    cv2.imshow("Warped Image", warped_img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def track():
    def detect_aruco_markers(frame, aruco_dict, aruco_params):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Filter
        # gray = cv2.GaussianBlur(gray, (5, 5), 0)
        # gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        return corners, ids

    # Загрузка калибровочных данных
    calib_data = np.load('calibration.npz')
    mtx = calib_data['mtx']
    dist = calib_data['dist']

    # ArUco params
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    aruco_params = aruco.DetectorParameters()

    marker_length = 0.05  # Длина стороны маркера в метрах
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
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Коррекция искажений
        # h, w = frame.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

        corners, ids = detect_aruco_markers(frame, aruco_dict, aruco_params)
        if ids is not None:
            for i in range(len(ids)):
                # Рисование маркеров на кадре
                aruco.drawDetectedMarkers(frame, corners, ids)

                # Вычисление позы маркера
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_length, mtx, dist)
                # Отображение осей
                aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1)

                # Центр маркера
                c = corners[i][0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                print(f"Marker ID: {ids[i][0]} - Center: ({center_x}, {center_y})")

        frame = resize_image(frame, 50)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


__init__()
# init_model()
calibrate()
exit()
track()

# import pygame
# import pygame.camera
# import requests
# from pygame.locals import *
# import time
# import subprocess
#
# session = requests.Session()
# session.get(f'http://192.168.1.122:7125/printer/gcode/script?script=SET_VELOCITY_LIMIT ACCEL=10000')
# session.get(f'http://192.168.1.122:7125/printer/gcode/script?script=G0 F1500000')
#
# h_res = 320
# w_res = 240
#
# screen = pygame.display.set_mode((h_res, w_res))
# pygame.init()
# pygame.camera.init()
# camlist = pygame.camera.list_cameras()
# if camlist:
#     cam = pygame.camera.Camera(camlist[0],(h_res,w_res))
#     cam.start()
#
# marcker_color = [97, 53, 61]
# zone_color = [38, 83, 145]
#
# track_zone_coords = [[[0, 0], [0, 0]], [[0, 0], [0, 0]]]
# current_track_disp = [[1000, 1000], [1000, 1000]]
# image = cam.get_image()
#
# for x_zone in range(2):
#     for y_zone in range(2):
#         for x in range(x_zone * h_res // 2, (x_zone + 1) * h_res // 2):
#             for y in range(y_zone * w_res // 2, (y_zone + 1) * w_res // 2):
#                 pixel = image.get_at((x, y))
#
#                 new_track_disp = abs(pixel[0] - zone_color[0]) + abs(pixel[1] - zone_color[1])
#                 if new_track_disp < current_track_disp[x_zone][y_zone]:
#                     current_track_disp[x_zone][y_zone] = new_track_disp
#                     track_zone_coords[x_zone][y_zone] = [x, y]
#
# while True:
#     marcker_coords = [0, 0]
#     current_marcker_disp = 1000
#
#     image = cam.get_image()
#     readyFlag_2 = False
#     for x in range(h_res):
#         for y in range(w_res):
#             pixel = image.get_at((x, y))
#             new_marcker_disp = abs(pixel[2] - marcker_color[2]) + abs(pixel[1] - marcker_color[1])
#             if new_marcker_disp < current_marcker_disp:
#                 marcker_coords = [x, y]
#                 current_marcker_disp = new_marcker_disp
#             if new_marcker_disp < 5:
#                 readyFlag_2 = True
#                 break
#         if readyFlag_2:
#             break
#     screen.blit(image, (0,0))
#     for x_zone in range(2):
#         for y_zone in range(2):
#             pygame.draw.rect(screen, zone_color,
#                              (track_zone_coords[x_zone][y_zone][0] - 10, track_zone_coords[x_zone][y_zone][1] - 10, 20, 20))
#     pygame.draw.rect(screen, marcker_color,
#                      (marcker_coords[0] - 10, marcker_coords[1] - 10, 20, 20))
#     t_s = time.time()
#     subprocess.run([f'echo G0 X{marcker_coords[0] // 3} Y{marcker_coords[1] // 3} > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
#
#     pygame.display.flip()

