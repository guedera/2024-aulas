import cv2

class ProcessImage():
    def __init__(self):
        pass

    def run_image(self, caminho):
        imagem= cv2.imread(caminho)
        height, width, _ = imagem.shape

        #corte em baixo
        imagem[int(height/2):,:int(width/3)] = 0
        imagem[int(height/2):,int(2*width/3):] = 0

        #corte em cima
        imagem[:int(height/2),int(width/3):int(2*width/3)] = 0
        self.bgr = imagem
    
    def show_image(self):
        cv2.imshow("imagem cortada", self.bgr)

def main():
    processado = ProcessImage()


    processado.run_image("img/arara.jpg")
    processado.show_image()

    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()