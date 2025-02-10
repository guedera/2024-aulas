import cv2
import numpy as np

class ProcessImage():
    def __init__(self):
        pass

    def run_image(self,bgr):
        self.bgr = bgr
        self.hsv = cv2.cvtColor(self.bgr, cv2.COLOR_RGB2HSV)

        low_b = np.array([14, 155, 102])
        high_b = np.array([160,255,120])

        low_g = np.array([37, 55, 30])
        high_g = np.array([65, 255, 255])

        low_r = np.array([0, 170, 130])
        high_r = np.array([165, 255, 255])

        self.mask_blue = cv2.inRange(self.hsv, low_b, high_b)
        self.mask_green = cv2.inRange(self.hsv, low_g, high_g)
        self.mask_red = cv2.inRange(self.hsv, low_r, high_r)
        self.mask = self.mask_green + self.mask_blue + self.mask_red

    def show_image(self):
        cv2.imshow("Original", self.bgr)
        cv2.imshow("Mascara", self.mask)
    

def main():
    webcam = cv2.VideoCapture(0)
    processa_imagem = ProcessImage()
    
    while(True):
        val, image = webcam.read()
        if val:
            processa_imagem.run_image(image)
            processa_imagem.show_image()
        if cv2.waitKey(1) == 27: # Aguarda 1 ms pela tecla 'ESC'
            break
    cv2.destroyAllWindows()
    webcam.release()

main()
