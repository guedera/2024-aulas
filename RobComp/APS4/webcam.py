import cv2
import time as t


class ProcessImage():

    def __init__(self):
        pass

    def run_image(self,bgr):
        self.bgr = bgr
        self.rgb = cv2.cvtColor(self.bgr, cv2.COLOR_RGB2BGR)
        self.rgb = self.rgb.transpose((0,1,2))
  
        
    def show_image(self):
        cv2.imshow("Transposta", self.rgb)
        cv2.waitKey()
        cv2.destroyAllWindows()

def main():
    webcam = cv2.VideoCapture(0)
    processa_imagem = ProcessImage()
    
    while(True):
        val, image = webcam.read()
        if val:
            processa_imagem.run_image(image)
            cv2.imshow("cam",image)
        if cv2.waitKey(1) == 27: # Aguarda 1 ms pela tecla 'ESC'
            break
    cv2.destroyAllWindows()
    webcam.release()

main()
