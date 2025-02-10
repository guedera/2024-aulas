import cv2
import time
from math import *
import numpy as np

class ProcessImage():

    def __init__(self):
        pass

    def filter_bw(self,img,menor,maior): #FUNCIONAL
        self.img = img
        img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(img_gray,menor,maior)
        

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contornos, self.arvore = cv2.findContours(mask_close.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        return contornos



    def filter_hsv(self,img,menor,maior):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(img_hsv,menor,maior)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contornos, self.arvore = cv2.findContours(mask_close.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        return contornos
    
    def get_center(self,lista_contornos):
        
        centros_massa = []
        
        for contorno in lista_contornos:
            M = cv2.moments(contorno)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centros_massa.append((cX,cY))
        return centros_massa
    
    def find_latinhas(self,imagem,contornos):

        contornos = sorted(contornos, key=cv2.contourArea,reverse=True)
        contornos_deinteresse = contornos[:4]

        cv2.drawContours(imagem, contornos_deinteresse, -1, [255, 0, 0], 2)

        return contornos_deinteresse

    def find_latinha_life(self, img_com, contornos):

        def distancia(a,b):
            x0 = a[0]
            y0 = a[1]
            x = b[0]
            y = b[1]

            dist = sqrt(pow(x-x0, 2) + pow(y-y0, 2))
            return dist 

        centro_latinhas = self.get_center(contornos)

        contorno_verde = self.filter_hsv(img_com, (35, 30, 30), (85, 255, 255))

        if len(contorno_verde) == 0:
            return
        
        verdao = max(contorno_verde, key= cv2.contourArea)
        centro_verdao = self.get_center([verdao])

        distancia_lista = []
        for c in centro_latinhas:
            dist = distancia(c,centro_verdao[0])
            distancia_lista.append(dist)
        
        index = np.argmin(distancia_lista)
        contorno_latinha_com_verde = contornos[index]
        cv2.drawContours(img_com, contorno_latinha_com_verde, -1, [0, 255, 0], 3)
        return img_com

    def run_image(self,imagem):
        contornos_das_latinhas = self.filter_bw(imagem,0,230)
        imagem_com_contorno = self.find_latinhas(imagem,contornos_das_latinhas)
        self.bgr = self.find_latinha_life(imagem,imagem_com_contorno)

    def show_image(self):
        cv2.imshow('imagem processada',self.bgr)
        cv2.waitKey()
        cv2.destroyAllWindows()

def main():
    imagem = cv2.imread('exerc_latinha/img/coke-cans.jpg')


    processado = ProcessImage()

    processado.run_image(imagem)

    processado.show_image()


if __name__ == "__main__":
    main()