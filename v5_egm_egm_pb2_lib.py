#funkci verze_test_real_ok
import cv2
import numpy as np
from pypylon import pylon
from PIL import ImageGrab
from collections import deque
import egm_pb2 as abbegm
import socket

import time


# Reálné rozměry stolu
real_width = 996  # Šířka v reálném prostoru (mm)
real_lengh = 2016 # Délka v reálném prostoru (mm)
target_y = 560 #pixely v obraze, kde se nachazi cara, ktera simuluje y pozici robota
target_y_on = True #predikovany bod se bude nachazet na primce ktera je na nastavene y ose
robot_komunikace = True # vypnuti komunikace mezi pythonem a robotem pro testovani, True- funguje komunikace
camera_real = False # pripojeni realne kamery nebo nahravani obrazovky
robot_offs = 0 # souradnicovy system kamery je 0,0 v levem spodnim rohu, 
#point na robotovy je ale uprostred, hodnota udava jak moc posunoty je robot uprostred v X ose


ip_addr = "127.0.0.1"
#ip_addr = "192.168.125.1"
port = 6599

socketUDP = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
socketCMDS = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
sensor_msg = abbegm.EgmSensor()
robot_msg = abbegm.EgmRobot()

sensor_msg.header.seqno = 0
sensor_msg.header.mtype = abbegm.EgmHeader.MessageType.MSGTYPE_CORRECTION

if camera_real:
    # Připojení ke kameře Basler
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    # Nastavení rozsahu pro cervenou barvu u v HSV-realvideo

    lower_color = np.array([0, 67, 108])  #
    upper_color = np.array([179, 255, 255])  #cervena
    area_down = 1500
    area_up = 2500

    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        image = converter.Convert(grabResult)
        first_frame = image.GetArray()
        first_frame = cv2.rotate(first_frame, cv2.ROTATE_90_CLOCKWISE)
        grabResult.Release()
    else:
        print("Chyba při získávání prvního snímku.")
        exit()

else:
# Nastavení rozsahu pro černou a šedou barvu v HSV -robotstudio
    lower_color = np.array([0, 100, 100])  # Černá (nízký jas)
    upper_color = np.array([10, 255, 255])  # Šedá (střední jas a vyšší kontrast)

    area_down = 300
    area_up = 900
    # area_up = 6000
    first_frame = np.array(ImageGrab.grab())
    first_frame = cv2.cvtColor(first_frame, cv2.COLOR_RGB2BGR)

# Výběr oblasti zájmu (ROI)
cv2.namedWindow("Vyber oblast zájmu (ROI)", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Vyber oblast zájmu (ROI)", 1280, 720)
roi = cv2.selectROI("Vyber oblast zájmu (ROI)", first_frame, fromCenter=False, showCrosshair=True)
cv2.destroyWindow("Vyber oblast zájmu (ROI)")

# Historie pozic puku pro výpočet rychlosti
history = deque(maxlen=4)  # Počet snímků, ze kterých počítáme trajektorii

# Rozměry hrací plochy
roi_x, roi_y, roi_w, roi_h = map(int, roi)

# Uložení poslední vypočítané trajektorie
predicted_trajectory = []
prediction_active = False  # Zda je predikce aktivní
max_deviation = 40  # Maximální povolená odchylka puku od predikované dráhy
bounce_zone = 20 #real150
bounce_lock = False

# Počet snímků, kdy puk nebyl detekován
missed_frames = 0
max_missed_frames = 2  # Kolik snímků můžeme ztratit, než přepočítáme trajektorii REAL3

def send_to_rob(point):
    robot_y = 170 #realny robot 40, simulace 170
    if point is None or point[1] != target_y:#pokud neni predikovani point nebo pokud je ale neni na hranici robota
        #fejkovej_stred = int((roi[2] / 2 * real_width / roi[2])-robot_offs)
        fake_center = float(500-robot_offs)
        sensor_msg.header.seqno += 1
        sensor_msg.planned.cartesian.pos.x = fake_center
        sensor_msg.planned.cartesian.pos.y = robot_y
        sensor_msg.planned.cartesian.pos.z = 55
        sensor_msg.planned.cartesian.euler.x = 0
        sensor_msg.planned.cartesian.euler.y = 0
        sensor_msg.planned.cartesian.euler.z = 0 

        print(f"fake_center{fake_center}")
    elif point[1] == target_y:#predikovany bod je
        real_x = int((point[0] * real_width / roi[2])-robot_offs) 
        #if real_x > 600 - robot_offs:
        #    real_x = 600 - robot_offs
        #elif real_x < 400 - robot_offs:
        #    real_x = 400 - robot_offs
        if real_x > (650 - robot_offs) or real_x < (350-robot_offs):
            return
        sensor_msg.header.seqno = 1
        sensor_msg.planned.cartesian.pos.x = real_x
        sensor_msg.planned.cartesian.pos.y = robot_y
        sensor_msg.planned.cartesian.pos.z = 55
        sensor_msg.planned.cartesian.euler.x = 0
        sensor_msg.planned.cartesian.euler.y = 0
        sensor_msg.planned.cartesian.euler.z = 0 

        print(f"real_x{real_x},{robot_y}")

    socketUDP.sendto(sensor_msg.SerializeToString(), (ip_addr, port))
    #time.sleep(0.1)    


def rob_attack(center):
    goal_x = roi_w // 2
    goal_y = 0
    puck_pos_x,puck_pos_y = center
    real_puck_x = int((puck_pos_x * real_width / roi[2])-robot_offs) 
    if real_puck_x > (650 - robot_offs) or real_puck_x < (350 - robot_offs):
        return

    deflection_angle = np.arctan2((goal_y - puck_pos_y),(goal_x - puck_pos_x))
    deflected_x = puck_pos_x + np.cos(deflection_angle) * 50
    deflected_y = puck_pos_y + np.sin(deflection_angle) * 50

    deflected_x_real = int((deflected_x * real_width / roi[2])-robot_offs) 
    deflected_y_real = int(real_lengh - (deflected_y * real_lengh / roi[3]))
    if deflected_y_real > 500: #omezeni aby robot nejezdil vysoko
        return
    print(f"Attack x:{deflected_x_real}, y: {deflected_y_real}")
    cv2.circle(frame, (int(deflected_x),int(deflected_y)), 5, (255, 0, 0), -1)  

    sensor_msg.header.seqno = 1
    sensor_msg.planned.cartesian.pos.x = deflected_x_real
    sensor_msg.planned.cartesian.pos.y = deflected_y_real
    sensor_msg.planned.cartesian.pos.z = 0
    sensor_msg.planned.cartesian.euler.x = 0
    sensor_msg.planned.cartesian.euler.y = 0
    sensor_msg.planned.cartesian.euler.z = 0 
    socketUDP.sendto(sensor_msg.SerializeToString(), (ip_addr, port))
   


while True:
    if camera_real:
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            image = converter.Convert(grabResult)
            frame = image.GetArray()
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            grabResult.Release()

            # Výřez oblasti podle vybraného ROI
            frame = frame[int(roi[1]):int(roi[1] + roi[3]), int(roi[0]):int(roi[0] + roi[2])]

    else:
        screen = np.array(ImageGrab.grab(bbox=(roi_x, roi_y, roi_x + roi_w, roi_y + roi_h)))
        frame = cv2.cvtColor(screen, cv2.COLOR_RGB2BGR)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
    mask = cv2.inRange(blurred, lower_color, upper_color)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    detected_puck = False
    puck_center = None
    final_predicted_point = None

    if target_y_on:
        cv2.line(frame, (0, target_y), (roi[2], target_y), (0, 255, 0), 2)  # Zelená přímka

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < area_down or area > area_up:
            continue

        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)

        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * (area / (perimeter * perimeter))
        if circularity < 0.78:
            continue

        puck_radius = radius

        cv2.circle(frame, center, radius, (0, 255, 0), 2)
        cv2.circle(frame, center, 2, (0, 255, 0), -1)
        detected_puck = True
        puck_center = center

    if detected_puck and puck_center:
        missed_frames = 0  # Restartujeme počítání ztracených snímků
        history.append(puck_center)

        if len(history) >= 4:  # Počítáme predikci až po nasbírání dostatečného počtu snímků
            dx_total = sum(history[i + 1][0] - history[i][0] for i in range(len(history) - 1))
            dy_total = sum(history[i + 1][1] - history[i][1] for i in range(len(history) - 1))
            dt = len(history) - 1  # Počet snímků mezi výpočtem

            vx = dx_total / dt
            vy = dy_total / dt

            if vy < 0:  # Pokud puk jede nahoru, nevytváříme trajektorii
                prediction_active = False
                predicted_trajectory = []
                final_predicted_point = None
            else:
                deviation_detected = False
                if prediction_active and predicted_trajectory:
                    predicted_next_pos = predicted_trajectory[0]
                    distance = np.linalg.norm(np.array(puck_center) - np.array(predicted_next_pos))

                    if bounce_lock or (distance < bounce_zone):
                        deviation_detected = False
                    elif distance > max_deviation:
                        deviation_detected = True    

                if not prediction_active or deviation_detected:
                    pred_x, pred_y = puck_center
                    predicted_trajectory = [puck_center]

                    for _ in range(40):  # Předpověď na 40 kroků
                        pred_x += vx
                        pred_y += vy

                        if pred_x - puck_radius <= 0 or pred_x + puck_radius >= roi_w:
                            vx = -vx  # Odrážení na ose X

                        if pred_y >= target_y:
                            prev_x, prev_y = predicted_trajectory[-1]
                            if pred_y - prev_y != 0:
                                t = (target_y - prev_y) / (pred_y - prev_y)
                                final_predicted_x = int(prev_x + t * (pred_x - prev_x))
                                final_predicted_y = target_y
                                final_predicted_point = (final_predicted_x, final_predicted_y)
                            else:
                                final_predicted_point = (int(pred_x), target_y)
                            break

                        predicted_trajectory.append((int(pred_x), int(pred_y)))

                    prediction_active = True  

    else:
        missed_frames += 1
        if missed_frames >= max_missed_frames:
            print("Puk nebyl detekován několik snímků. Trajektorie resetována.")
            prediction_active = False
            predicted_trajectory = []
            final_predicted_point = None

    if prediction_active and predicted_trajectory:
        for i in range(1, len(predicted_trajectory)):
            cv2.line(frame, predicted_trajectory[i - 1], predicted_trajectory[i], (0, 0, 255), 2)
        cv2.circle(frame, final_predicted_point, 5, (255, 0, 0), -1)  

    cv2.namedWindow("Detekce + Maska", cv2.WINDOW_NORMAL)
    combined = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
    cv2.imshow("Detekce + Maska", combined)

    # Odeslání příkazů na konci cyklu
    if robot_komunikace:
        send_to_rob(final_predicted_point)
        if detected_puck and (abs(puck_center[1] - target_y) <= 100):  # real 300
            rob_attack(puck_center)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
socketUDP.close()

