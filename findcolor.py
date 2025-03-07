import cvzone
import cv2
from cvzone.ColorModule import ColorFinder

# Vytvoření instance třídy ColorFinder s trackBar povoleným.
myColorFinder = ColorFinder(trackBar=True)

# Inicializace video capture.
cap = cv2.VideoCapture("test_video.mp4")

# Nastavení rozlišení kamery na 640x480.
cap.set(3, 640)
cap.set(4, 480)

# Nastavení pevné velikosti okna
window_name = "Image Stack"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # Povolit změnu velikosti okna
cv2.resizeWindow(window_name, 800, 600)  # Nastavit pevnou velikost okna

# Načtení prvního rámce z videa
success, img = cap.read()

# Pokud je rámec úspěšně načten
if success:
    while True:
        # Použití metody update z třídy ColorFinder pro detekci barvy

        imgOrange, mask = myColorFinder.update(img)

        # Sloučení originálního obrázku, maskované barvy a binární masky
        imgStack = cvzone.stackImages([img, imgOrange, mask], 3, 1)

        # Zobrazení obrázků v nastaveném okně
        cv2.imshow(window_name, imgStack)

        # Ukončení programu, pokud stiskneš klávesu 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
else:
    print("Nepodařilo se načíst video.")

# Uvolnění objektu video capture a zavření oken
cap.release()
cv2.destroyAllWindows()
