#funkci verze_test na realu - nutno vyzkouset, tato verze ma upravene komunikace a jeste zlepsenou predikci
#test na realu probelh, sciprt je nachystan na DOD, parametry upraveny, funguje to znatelene lepe, viz video
#aruco znacky zacinaji od leveho horniho rohu
import cv2
import numpy as np
from pypylon import pylon
from PIL import ImageGrab
from collections import deque
from ABBRobotEGM import EGM
import cv2.aruco as aruco
import time


# Reálné rozměry stolu
#real_width = 1030  # Šířka v reálném prostoru (mm) #real
#real_lengh = 1940 # Délka v reálném prostoru (mm)  #real
#real_width = 996  # Šířka v reálném prostoru (mm)
#real_lengh = 2016 # Délka v reálném prostoru (mm)

target_y_on = True #predikovany bod se bude nachazet na primce ktera je na nastavene y ose
robot_komunikace = True # vypnuti komunikace mezi pythonem a robotem pro testovani, True- funguje komunikace python-robot
camera_real = False # pripojeni realne kamery nebo nahravani obrazovky True - je pripojena basler kamera pres usb
robot_offs = 0 # souradnicovy system kamery je 0,0 v levem spodnim rohu, posun x souradnice od praveho rohu smrerem k levemu
#point na robotovy je ale uprostred, hodnota udava jak moc posunoty je robot uprostred v X ose

#

#kde robot chyta, souradnice v mm
robot_y = 40 #realny robot 40, simulace 170
max_robot_movement = 150 #kolik muze robot jezdit od stredu
target_y_offset_mm = 100 #posunuti zelene cary kde robot chyta, posouva se smeren nahoru sim 40



egm = EGM()

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
    area_up = 3500

    # Reálné rozměry stolu
    real_width = 1030  # Šířka v reálném prostoru (mm) #real
    real_lengh = 1940 # Délka v reálném prostoru (mm)  #real

    #ofest obrazu a hraci plochy podle aruco znacek
    offset_x = 100  # Posun dovnitř na ose X (pixely) real  
    offset_y = 60  # Posun dovnitř na ose Y (pixely) real

    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        image = converter.Convert(grabResult)
        first_frame = image.GetArray()
        first_frame = cv2.rotate(first_frame, cv2.ROTATE_90_CLOCKWISE)
        grabResult.Release()
    else:
        print("Chyba při získávání prvního snímku.")
        exit()

else: #virtualni kamera alias screen
    #nastaveni parametru robotav simulaci
    robot_y = 170 #realny robot 40, simulace 170
    target_y_offset_mm = 40
    max_robot_movement = 250 #kolik muze robot jezdit od stredu

# Nastavení rozsahu pro černou a šedou barvu v HSV -robotstudio
    lower_color = np.array([0, 100, 100])  # Černá (nízký jas)
    upper_color = np.array([10, 255, 255])  # Šedá (střední jas a vyšší kontrast)


    #ofest obrazu a hraci plochy podle aruco znacek
    offset_x = 50  # Posun dovnitř na ose X (pixely)
    offset_y = 30  # Posun dovnitř na ose Y (pixely)

    # Reálné rozměry stolu
    real_width = 996  # Šířka v reálném prostoru (mm)
    real_lengh = 2016 # Délka v reálném prostoru (mm)c

    area_down = 300
    #area_up = 900
    area_up = 6000
    first_frame = np.array(ImageGrab.grab())
    first_frame = cv2.cvtColor(first_frame, cv2.COLOR_RGB2BGR)

# --- Automatický výběr ROI pomocí ArUco značek ---
auto_roi = False  # příznak, zda jsme ROI určili automaticky
M = None          # transformační matice

# Inicializace ArUco slovníku a parametrů
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)
gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
corners, ids, _ = detector.detectMarkers(gray)
print(ids)

# Vypočítáme vektorové posunutí
def shrink_polygon(corners, offset_x, offset_y):
    """
    Posune body ROI směrem do středu rovnoměrně podle offsetu.
    """
    center_x = np.mean([p[0] for p in corners])  # Střed X
    center_y = np.mean([p[1] for p in corners])  # Střed Y
    
    new_corners = []
    for (x, y) in corners:
        direction_x = center_x - x
        direction_y = center_y - y
        length = np.sqrt(direction_x**2 + direction_y**2)

        # Normalizujeme vektor směru a aplikujeme offset
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

        # Vykreslíme původní oblast (zelená)
        vis_frame = first_frame.copy()
        pts_roi_original = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.int32)
        cv2.polylines(vis_frame, [pts_roi_original], isClosed=True, color=(0, 255, 0), thickness=2)

       

        # Vykreslíme novou posunutou oblast (červená)
        pts_roi_offset = shrink_polygon([top_left, top_right, bottom_right, bottom_left], offset_x, offset_y)
        cv2.polylines(vis_frame, [pts_roi_offset], isClosed=True, color=(0, 0, 255), thickness=2)
        cv2.aruco.drawDetectedMarkers(vis_frame,corners,ids)
        # Zobrazíme okno s původní a upravenou oblastí
        cv2.namedWindow("Detekce ArUco a ROI", cv2.WINDOW_NORMAL)
        cv2.imshow("Detekce ArUco a ROI", vis_frame)
        cv2.waitKey(0)
        cv2.destroyWindow("Detekce ArUco a ROI")

        # Výpočet šířky a výšky nové ROI
        roi_w = int(np.linalg.norm(np.array(top_right) - np.array(top_left)))
        roi_h = int(np.linalg.norm(np.array(bottom_left) - np.array(top_left)))

        # Definice bodů pro perspektivní transformaci
        pts_src = np.array(pts_roi_offset, dtype="float32")
        pts_dst = np.array([[0, 0], [roi_w, 0], [roi_w, roi_h], [0, roi_h]], dtype="float32")
        M = cv2.getPerspectiveTransform(pts_src, pts_dst)

        # Aplikujeme transformaci na první snímek
        first_frame = cv2.warpPerspective(first_frame, M, (roi_w, roi_h))
        roi = (0, 0, roi_w, roi_h)
    else:
        auto_roi = False
        print("Nedetekovany vsechny 4 Aruco znacky")

if not auto_roi:
    # --- Manuální výběr ROI, pokud nebyly nalezeny všechny ArUco značky ---
    cv2.namedWindow("Vyber oblast zájmu (ROI)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Vyber oblast zájmu (ROI)", 1280, 720)
    roi = cv2.selectROI("Vyber oblast zájmu (ROI)", first_frame, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow("Vyber oblast zájmu (ROI)")
    roi = tuple(map(int, roi))
    roi_x, roi_y, roi_w, roi_h = roi

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
max_missed_frames = 3  # Kolik snímků můžeme ztratit, než přepočítáme trajektorii REAL3

# Uchovává poslední predikovaný bod a kdy byl vytvořen
last_valid_predicted_point = None
last_predicted_time = 0
max_point_age = 0.15  # v sekundách



target_y_offset = int(target_y_offset_mm * roi[3] / real_lengh)
target_y = int(roi[3] - (robot_y * roi[3] / real_lengh)) - target_y_offset

stred_roi= int((roi[2] / 2 * real_width / roi[2])-robot_offs)
max_move_x_left = stred_roi - max_robot_movement
max_move_x_right = stred_roi + max_robot_movement

failed_egm_count = 0
max_failed_egm = 3

def send_to_rob(point):
    global failed_egm_count,egm


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
    
    # Get current orientation (maintain throughout motion)
    current_orient = np.array([state.cartesian.orient.u0, state.cartesian.orient.u1,  state.cartesian.orient.u2,  state.cartesian.orient.u3])
    if point is None or point[1] != target_y:#pokud neni predikovani point nebo pokud je ale neni na hranici robota
       
        #fake_center = float(500-robot_offs)
        new_pos = np.array([stred_roi,robot_y,state.cartesian.pos.z])
        #print(f"stred pozice: {stred_roi}")
    elif point[1] == target_y:#predikovany bod je
        real_x = int((point[0] * real_width / roi[2])-robot_offs) 

        if real_x > (max_move_x_right - robot_offs) or real_x < (max_move_x_left-robot_offs):
            return

        new_pos = np.array([real_x,robot_y,state.cartesian.pos.z])
        #print(f"real_x")
        
    egm.send_to_robot(cartesian=(new_pos, current_orient))

def rob_attack(center):
    goal_x = roi_w // 2
    goal_y = 0
    puck_pos_x,puck_pos_y = center
    real_puck_x = int((puck_pos_x * real_width / roi[2])-robot_offs) 
    if real_puck_x > (max_move_x_right - robot_offs) or real_puck_x < (max_move_x_left - robot_offs):
        return

    deflection_angle = np.arctan2((goal_y - puck_pos_y),(goal_x - puck_pos_x))
    deflected_x = puck_pos_x + np.cos(deflection_angle) * 50 # 50 real funguje
    deflected_y = puck_pos_y + np.sin(deflection_angle) * 50 # 50 real funguje

    deflected_x_real = int((deflected_x * real_width / roi[2])-robot_offs) 
    deflected_y_real = int(real_lengh - (deflected_y * real_lengh / roi[3]))
    if deflected_y_real > 500: #omezeni aby robot nejezdil vysoko
        return
    #print(f"Attack x:{deflected_x_real}, y: {deflected_y_real}")
    cv2.circle(frame, (int(deflected_x),int(deflected_y)), 5, (255, 0, 0), -1)  


    success, state = egm.receive_from_robot()
    if not success:
        print("Failed to receive from robot")
        return
    current_orient = np.array([state.cartesian.orient.u0, state.cartesian.orient.u1,  state.cartesian.orient.u2,  state.cartesian.orient.u3])
    new_pos = np.array([deflected_x_real,deflected_y_real,state.cartesian.pos.z])
    egm.send_to_robot(cartesian=(new_pos, current_orient))  



while True:
    if camera_real:
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            image = converter.Convert(grabResult)
            frame = image.GetArray()
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            grabResult.Release()
            if auto_roi:
                # Aplikujeme stejnou perspektivní transformaci na aktuální snímek
                frame = cv2.warpPerspective(frame, M, (roi_w, roi_h))
            else:
                frame = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

    else:
        if auto_roi:
            # Při automatickém režimu musíme zachytit celý snímek, aby se dala aplikovat warpPerspective
            screen = np.array(ImageGrab.grab())
            screen = cv2.cvtColor(screen, cv2.COLOR_RGB2BGR)
            frame = cv2.warpPerspective(screen, M, (roi_w, roi_h))
        else:
            screen = np.array(ImageGrab.grab(bbox=(roi_x, roi_y, roi_x+roi_w, roi_y+roi_h)))
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
                #final_predicted_point = None
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
            print("Puk nebyl detekován několik snímků. Trajektorie resetována.")
            prediction_active = False
            predicted_trajectory = []
            #final_predicted_point = None

    if prediction_active and predicted_trajectory:
        for i in range(1, len(predicted_trajectory)):
            cv2.line(frame, predicted_trajectory[i - 1], predicted_trajectory[i], (0, 0, 255), 2)
        cv2.circle(frame, final_predicted_point, 5, (255, 0, 0), -1)  

    cv2.namedWindow("Detekce + Maska", cv2.WINDOW_NORMAL)
    combined = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
    cv2.imshow("Detekce + Maska", combined)

    
    # Odeslání příkazů na konci cyklu
    if robot_komunikace:
        try:
            if last_valid_predicted_point and (time.time() - last_predicted_time <= max_point_age):
                send_to_rob(last_valid_predicted_point)
            else:
                send_to_rob(None)  # Robot se vrátí do středu
        except Exception as e:
            print("Chyba pri odesilani do robota",e)
        try:
            if detected_puck and (abs(puck_center[1] - target_y) <= 300):  # real 300
                rob_attack(puck_center)
        except Exception as e:
            print("Chyba pri odesilani utoku do robota",e)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()


