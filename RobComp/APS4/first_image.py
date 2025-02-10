import cv2

class ProcessImage:
    def __init__(self):
        pass

    def load_image(self, caminho):
        self.bgr = cv2.imread(caminho)
        return self.bgr
    
    def show_image(self):
        cv2.imshow("imagem_carregada", self.bgr)

    
    def show_channels(self):
        imagem_b, imagem_g, imagem_r = cv2.split(self.bgr)
        cv2.imshow("Canal Vermelho", imagem_r)
        cv2.imshow("Canal Verde", imagem_g)
        cv2.imshow("Canal Azul", imagem_b)

def main():
    processado = ProcessImage()

    processado.load_image("img/rgb_cube.png")
    
    # Exibe os canais da imagem + PLUS a imagem inteira para comparar

    processado.show_channels()
    processado.show_image()

    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
