#funkci verze_test na realu - nutno vyzkouset, tato verze ma upravene komunikace a opravu EGM cyklu
#test na realu probelh, sciprt je nachystan na DOD, parametry upraveny
#aruco znacky zacinaji od leveho horniho rohu
import cv2
import numpy as np
from pypylon import pylon
from PIL import ImageGrab
from collections import deque
from ABBRobotEGM import EGM
import cv2.aruco as aruco
import time


# --- KONFIGURACE ---
target_y_on = True #predikovany bod se bude nachazet na primce ktera je na nastavene y ose
robot_komunikace = True # vypnuti komunikace mezi pythonem a robotem pro testovani
camera_real = False # pripojeni realne kamery nebo nahravani obrazovky
robot_offs = 0 # posun x souradnice od praveho rohu smerem k levemu

# Kde robot chyta, souradnice v mm
robot_y = 40 #realny robot 40, simulace 170
max_robot_movement = 150 #kolik muze robot jezdit od stredu
target_y_offset_mm = 100 #posunuti zelene cary kde robot chyta

# Inicializace EGM
egm = EGM()

if camera_real:
    # Připojení ke kameře Basler
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    
    # Nastavení rozsahu pro barvu v HSV - realvideo
    lower_color = np.array([0, 67, 108])  
    upper_color = np.array([179, 255, 255]) 
    area_down = 1500
    area_up = 3500

    # Reálné rozměry stolu
    real_width = 1030  
    real_lengh = 1940 

    # Offset obrazu
    offset_x = 100   
    offset_y = 60  

    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        image = converter.Convert(grabResult)
        first_frame = image.GetArray()
        first_frame = cv2.rotate(first_frame, cv2.ROTATE_90_CLOCKWISE)
        grabResult.Release()
    else:
        print("Chyba při získávání prvního snímku.")
        exit()

else: # Virtuální kamera (screen)
    robot_y = 170 
    target_y_offset_mm = 40
    max_robot_movement = 250 

    # Nastavení rozsahu pro barvu v HSV - robotstudio
    lower_color = np.array([0, 100, 100])  
    upper_color = np.array([10, 255, 255])  

    offset_x = 50  
    offset_y = 30  

    real_width = 996  
    real_lengh = 2016 

    area_down = 300
    area_up = 6000
    first_frame = np.array(ImageGrab.grab())
    first_frame = cv2.cvtColor(first_frame, cv2.COLOR_RGB2BGR)

# --- Automatický výběr ROI pomocí ArUco značek ---
auto_roi = False  
M = None          
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)
gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
corners, ids, _ = detector.detectMarkers(gray)
print(ids)

def shrink_polygon(corners, offset_x, offset_y):
    center_x = np.mean([p[0] for p in corners])
    center_y = np.mean([p[1] for p in corners])
    new_corners = []
    for (x, y) in corners:
        direction_x = center_x - x
        direction_y = center_y - y
        length = np.sqrt(direction_x**2 + direction_y**2)
        new_x = x + (direction_x / length) * offset_x
        new_y = y + (direction_y / length) * offset_y
        new_corners.append((int(new_x), int(new_y)))
    return np.array(new_corners, dtype=np.int32)

if ids is not None and set(ids.flatten()).issuperset({0, 1, 2, 3}):
    marker_corners = {marker_id: corners[i][0] for i, marker_id in enumerate(ids.flatten())}
    if all(key in marker_corners for key in [0, 1, 2, 3]):
        auto_roi = True
        top_left     = marker_corners[0][0]
        top_right    = marker_corners[1][1]
        bottom_right = marker_corners[2][2]
        bottom_left  = marker_corners[3][3]

        vis_frame = first_frame.copy()
        pts_roi_offset = shrink_polygon([top_left, top_right, bottom_right, bottom_left], offset_x, offset_y)
        
        roi_w = int(np.linalg.norm(np.array(top_right) - np.array(top_left)))
        roi_h = int(np.linalg.norm(np.array(bottom_left) - np.array(top_left)))

        pts_src = np.array(pts_roi_offset, dtype="float32")
        pts_dst = np.array([[0, 0], [roi_w, 0], [roi_w, roi_h], [0, roi_h]], dtype="float32")
        M = cv2.getPerspectiveTransform(pts_src, pts_dst)

        first_frame = cv2.warpPerspective(first_frame, M, (roi_w, roi_h))
        roi = (0, 0, roi_w, roi_h)
    else:
        auto_roi = False
        print("Nedetekovany vsechny 4 Aruco znacky")

if not auto_roi:
    cv2.namedWindow("Vyber oblast zájmu (ROI)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Vyber oblast zájmu (ROI)", 1280, 720)
    roi = cv2.selectROI("Vyber oblast zájmu (ROI)", first_frame, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow("Vyber oblast zájmu (ROI)")
    roi = tuple(map(int, roi))

roi_x, roi_y, roi_w, roi_h = map(int, roi)

# --- Proměnné pro hru ---
history = deque(maxlen=4)
predicted_trajectory = []
prediction_active = False
max_deviation = 40  
bounce_zone = 20 
bounce_lock = False
missed_frames = 0
max_missed_frames = 3  
last_valid_predicted_point = None
last_predicted_time = 0
max_point_age = 0.15 

target_y_offset = int(target_y_offset_mm * roi[3] / real_lengh)
target_y = int(roi[3] - (robot_y * roi[3] / real_lengh)) - target_y_offset

stred_roi = int((roi[2] / 2 * real_width / roi[2]) - robot_offs)
max_move_x_left = stred_roi - max_robot_movement
max_move_x_right = stred_roi + max_robot_movement

failed_egm_count = 0
max_failed_egm = 3

# --- FUNKCE PRO KOMUNIKACI (Sjednocená logika) ---

def send_to_rob(target_pos_mm):
    """
    Odešle příkaz robotovi. 
    target_pos_mm: Tuple (x, y) v mm, nebo None (pro návrat do středu).
    Řeší 1x Receive a 1x Send.
    """
    global failed_egm_count, egm

    # 1. RECEIVE
    success, state = egm.receive_from_robot()
    if not success:
        failed_egm_count += 1
        print("Failed to receive from robot (count:", failed_egm_count, ")")
        if failed_egm_count >= max_failed_egm:
            print("Restartuji EGM spojení...")
            try:
                egm.close()
            except:
                pass
            egm = EGM()
            failed_egm_count = 0
        return
    else:
        failed_egm_count = 0
    
    current_orient = np.array([state.cartesian.orient.u0, state.cartesian.orient.u1,  state.cartesian.orient.u2,  state.cartesian.orient.u3])
    
    # 2. DECIDE POSITION
    if target_pos_mm is None:
        # Jedu do středu
        new_pos = np.array([stred_roi, robot_y, state.cartesian.pos.z])
    else:
        # Jedu na cílovou pozici
        tx, ty = target_pos_mm
        new_pos = np.array([tx, ty, state.cartesian.pos.z])
        
    # 3. SEND
    egm.send_to_robot(cartesian=(new_pos, current_orient))

def calculate_attack_pos(center):
    """
    Vypočítá souřadnice pro útok, ale NEKOMUNIKUJE s robotem.
    Vrací (real_x, real_y) nebo None.
    """
    goal_x = roi_w // 2
    goal_y = 0
    puck_pos_x, puck_pos_y = center
    
    # Kontrola, zda je puk v dosahu (osa X)
    real_puck_x = int((puck_pos_x * real_width / roi[2]) - robot_offs) 
    if real_puck_x > (max_move_x_right - robot_offs) or real_puck_x < (max_move_x_left - robot_offs):
        return None

    # Vypocet vektoru
    deflection_angle = np.arctan2((goal_y - puck_pos_y),(goal_x - puck_pos_x))
    deflected_x = puck_pos_x + np.cos(deflection_angle) * 50 
    deflected_y = puck_pos_y + np.sin(deflection_angle) * 50 

    # Prevod na mm
    deflected_x_real = int((deflected_x * real_width / roi[2]) - robot_offs) 
    deflected_y_real = int(real_lengh - (deflected_y * real_lengh / roi[3]))
    
    # Omezeni vysky utoku
    if deflected_y_real > 500: 
        return None
        
    return (deflected_x_real, deflected_y_real)


# --- HLAVNÍ SMYČKA ---
while True:
    if camera_real:
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            image = converter.Convert(grabResult)
            frame = image.GetArray()
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            grabResult.Release()
            if auto_roi:
                frame = cv2.warpPerspective(frame, M, (roi_w, roi_h))
            else:
                frame = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
    else:
        if auto_roi:
            screen = np.array(ImageGrab.grab())
            screen = cv2.cvtColor(screen, cv2.COLOR_RGB2BGR)
            frame = cv2.warpPerspective(screen, M, (roi_w, roi_h))
        else:
            screen = np.array(ImageGrab.grab(bbox=(roi_x, roi_y, roi_x+roi_w, roi_y+roi_h)))
            frame = cv2.cvtColor(screen, cv2.COLOR_RGB2BGR)

    # Zpracování obrazu
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
        cv2.line(frame, (0, target_y), (roi[2], target_y), (0, 255, 0), 2)

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

    # Predikce a historie
    if detected_puck and puck_center:
        missed_frames = 0
        history.append(puck_center)

        if len(history) >= 4:
            dx_total = sum(history[i + 1][0] - history[i][0] for i in range(len(history) - 1))
            dy_total = sum(history[i + 1][1] - history[i][1] for i in range(len(history) - 1))
            dt = len(history) - 1

            vx = dx_total / dt
            vy = dy_total / dt

            if vy < 0:
                prediction_active = False
                predicted_trajectory = []
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

                    for _ in range(40):
                        pred_x += vx
                        pred_y += vy
                        if pred_x - puck_radius <= 0 or pred_x + puck_radius >= roi_w:
                            vx = -vx 
                        if pred_y >= target_y:
                            prev_x, prev_y = predicted_trajectory[-1]
                            if pred_y - prev_y != 0:
                                t = (target_y - prev_y) / (pred_y - prev_y)
                                final_predicted_x = int(prev_x + t * (pred_x - prev_x))
                                final_predicted_y = target_y
                                final_predicted_point = (final_predicted_x, final_predicted_y)
                                last_valid_predicted_point = final_predicted_point
                                last_predicted_time = time.time()
                            else:
                                final_predicted_point = (int(pred_x), target_y)
                                last_valid_predicted_point = final_predicted_point
                                last_predicted_time = time.time()
                            break
                        predicted_trajectory.append((int(pred_x), int(pred_y)))
                    prediction_active = True  
    else:
        missed_frames += 1
        if missed_frames >= max_missed_frames:
            prediction_active = False
            predicted_trajectory = []

    # Vykreslení
    if prediction_active and predicted_trajectory:
        for i in range(1, len(predicted_trajectory)):
            cv2.line(frame, predicted_trajectory[i - 1], predicted_trajectory[i], (0, 0, 255), 2)
        if final_predicted_point:
            cv2.circle(frame, final_predicted_point, 5, (255, 0, 0), -1)  

    cv2.namedWindow("Detekce + Maska", cv2.WINDOW_NORMAL)
    combined = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
    cv2.imshow("Detekce + Maska", combined)

    # --- ODESLÁNÍ DO ROBOTA (Opravená logika) ---
    if robot_komunikace:
        final_robot_target = None # Default = None = Jedu domu

        # 1. PRIORITA: ÚTOK
        is_attacking = False
        if detected_puck and puck_center and (abs(puck_center[1] - target_y) <= 300): # real 300
            attack_coords = calculate_attack_pos(puck_center)
            if attack_coords is not None:
                final_robot_target = attack_coords
                is_attacking = True
                # Vizualizace útoku (červená tečka v místě puku)
                cv2.circle(frame, puck_center, 10, (0, 0, 255), -1)

        # 2. PRIORITA: OBRANA (pokud neútočíme)
        if not is_attacking:
            if last_valid_predicted_point and (time.time() - last_predicted_time <= max_point_age):
                # Musime prevest predikovany PIXEL bod na MM bod pro robota
                px, py = last_valid_predicted_point
                # Pokud je predikce na care (mela by byt, ale pro jistotu)
                if py == target_y: 
                    real_x = int((px * real_width / roi[2]) - robot_offs)
                    # Kontrola limitu pohybu
                    if (max_move_x_left - robot_offs) <= real_x <= (max_move_x_right - robot_offs):
                        final_robot_target = (real_x, robot_y)

        # 3. VOLÁNÍ KOMUNIKACE (Jednou za smyčku!)
        try:
            send_to_rob(final_robot_target)
        except Exception as e:
            print("Chyba komunikace:", e)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()