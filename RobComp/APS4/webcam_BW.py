import cv2

class ProcessImage():
    def __init__(self):
        pass

    def run_image(self,imagem):
        self.bgr = imagem
        self.gray = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2GRAY)
        self.gray[self.gray < 128] = 0
        self.gray[self.gray >= 128] = 255
        return (self.gray)
    
    def show_image(self):
        cv2.imshow("imagem", self.gray)

def main():
    webcam = cv2.VideoCapture(0)
    processado = ProcessImage()

    while(True):
        val, frame = webcam.read()
        if not val:
            break
        imagem_processada = processado.run_image(frame)
        cv2.imshow("imagem_processada", imagem_processada)
        if cv2.waitKey(1) == 27:
            break
    webcam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()