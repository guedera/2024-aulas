import cv2
import time as t
import time


class ProcessImage():

    def __init__(self):
        contagem_e = 0
        contagem_c = 0
        self.laranjas = {'cesto_claro':contagem_c,'cesto_escuro':contagem_e}
        self.definiu = False
    


    def run_image(self,imagem):
        agora = time.time()
        
        imagem_hsv = cv2.cvtColor(imagem, cv2.COLOR_BGR2HSV)

        menor1 = (5, 110, 70)
        maior1 = (25, 220, 225)

        maskc = cv2.inRange(imagem_hsv, menor1, maior1)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        maskc = cv2.morphologyEx(maskc, cv2.MORPH_OPEN, kernel)
        maskc = cv2.morphologyEx(maskc, cv2.MORPH_CLOSE, kernel)

        #contornos cestas
        contornosc, _ = cv2.findContours(maskc.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contornosc = sorted(contornosc, key=cv2.contourArea, reverse=True)

        cesta1 = contornosc[0]
        cesta2 = contornosc[1]

        if self.definiu == False:
            self.alvo1 = agora + 3
            self.alvo2 = agora + 3

            x, y, w, h = cv2.boundingRect(cesta2)

            # Canto superior esquerdo 1
            self.top_left1 = (x, y)

            # Canto inferior direito 1
            self.bottom_right1 = (x + w, y + h)

            x, y, w, h = cv2.boundingRect(cesta1)

            # Canto superior esquerdo 2
            self.top_left2 = (x, y)

            # Canto inferior direito 2
            self.bottom_right2 = (x + w, y + h)

            self.definiu = True

            self.cesta1_fixo = cesta1
            self.cesta2_fixo = cesta2
            print('definiu')
            

        cv2.drawContours(imagem, self.cesta1_fixo, -1, [255, 200, 0], 2)
        cv2.drawContours(imagem, self.cesta2_fixo, -1, [255, 0, 0], 2)
        
        #contornoslaranja
        menorl = (2, 140, 150)
        maiorl = (25, 255, 255)

        maskl = cv2.inRange(imagem_hsv, menorl, maiorl)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        maskl = cv2.morphologyEx(maskl, cv2.MORPH_OPEN, kernel)
        maskl = cv2.morphologyEx(maskl, cv2.MORPH_CLOSE, kernel)

        contornosl, _ = cv2.findContours(maskl.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contornosl = sorted(contornosl, key=cv2.contourArea, reverse=True)
    
        laranjas_cont = contornosl[:15]

        cv2.drawContours(imagem, laranjas_cont, -1, [0, 255, 0], 2)

        
        centro_laranjas = self.get_center(laranjas_cont)
        for i in centro_laranjas:          
            if i[0]>self.top_left1[0] and i[0]<self.bottom_right1[0]:
                if i[1]>self.top_left1[1] and i[1]<self.bottom_right1[1]:
                        if agora >= self.alvo1:
                            print('adicionou')
                            self.laranjas['cesto_escuro'] += 1
                            self.alvo1 = agora + 4

            if i[0]>self.top_left2[0] and i[0]<self.bottom_right2[0]:
                if i[1]>self.top_left2[1] and i[1]<self.bottom_right2[1]:
                        if agora >= self.alvo2:
                            print('adicionou')
                            self.laranjas['cesto_claro'] += 1
                            self.alvo2  = agora + 4 

        return self.laranjas

    def get_center(self,lista_contornos):
        
        centros_massa = []
        
        for contorno in lista_contornos:
            M = cv2.moments(contorno)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centros_massa.append((cX,cY))
                
        return centros_massa

def main():
    video = cv2.VideoCapture("/home/guedes/colcon_ws/src/ai-guedera/q2/laranja3.mp4")
    processa_imagem = ProcessImage()
    
    while(True):
        val, image = video.read()
        if val:
            lista = processa_imagem.run_image(image)
            cv2.imshow("cam",image)
        if cv2.waitKey(1) == 27: # Aguarda 1 ms pela tecla 'ESC'
            break


    print(lista)

main()

