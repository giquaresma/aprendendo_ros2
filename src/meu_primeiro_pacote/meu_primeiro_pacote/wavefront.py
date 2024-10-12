import cv2
import numpy as np
from matplotlib import pyplot as plt

pgmf = open('src/my_map.pgm', 'rb')
image = plt.imread(pgmf)


image_copia = 1.0 * (image > 250)#mudando os valores de cinza para escala de preto e branco

goal = (250, 125) #define o meu objetivo
robo = (350, 50) #define onde ta meu robo

#para visualização inicial, os dois são pretos
image_copia[goal[0]][goal[1]] = 0 #objetivo é 0
image_copia[robo[0]][robo[1]] = 0 #robo é 0

fig = plt.figure()
fig.canvas.manager.set_window_title('Figura 1')

plt.imshow(image_copia, interpolation='nearest', cmap='gray')
plt.title('Imagem inicial')
plt.show()

#tentativa 3
image_copia[goal[0]][goal[1]] = 2 #objetivo é 2
image_copia[robo[0]][robo[1]] = 1 #robo é 1

soma = 2
ponto = goal
parar = False
coordenadas = list()

while(1):
    soma += 1
    for l in range (-1,2):
        for c in range (-1,2):
            try:
                if(image_copia[ponto[0]+l][ponto[1]+c] == 1):
                    image_copia[ponto[0]+ l][ponto[1]+c]= soma
                    coordenadas.append([ponto[0]+ l, ponto[1]+c])

                if(ponto[0]+l == robo[0] and ponto[1]+c == robo[1]):
                    parar = True
                    break
            except: 
                continue
        if(parar == True):
           break   
    if(parar == True):
           break   
            
    ponto = coordenadas.pop(0)

fig = plt.figure()
fig.canvas.manager.set_window_title('Figura 2')

plt.imshow(image_copia, interpolation='nearest', cmap='viridis')  # Usando viridis para ver os valores
plt.colorbar()
plt.title('Imagem colorida de proximidade')
plt.show()

#achando o melhor caminho para a tragetoria

ponto_inicial = robo
caminho = [list(robo)]
menor = image_copia[robo[0]][robo[1]] + 1
menor_posicao = robo
parar = False

while(1):

    for l in range (1,-2,-1):
        for c in range (1,-2,-1):
            try:
                if(image_copia[ponto_inicial[0]+l][ponto_inicial[1]+c] > 1 and image_copia[ponto_inicial[0]+l][ponto_inicial[1]+c] < menor):
                    #print("posicao: " , (menor_posicao[0]+l,menor_posicao[1]+c), "menor posicao ", menor_posicao, " valor ", image_copia[menor_posicao[0]+l][menor_posicao[1]+c], "menor valor ", menor)
                    #print(menor)
                    
                    menor = image_copia[ponto_inicial[0]+l][ponto_inicial[1]+c]
                    menor_posicao = (ponto_inicial[0]+l, ponto_inicial[1]+c)

                if(ponto_inicial[0]+l == goal[0] and ponto_inicial[1]+c == goal[1]):
                    parar = True
                    break
            except: 
                continue

        if(parar == True):
            break 

    ponto_inicial = menor_posicao
    caminho.append(menor_posicao)

    if(parar == True):
        break 

#colorindo caminho :)

image_com_caminho = image.copy()
image_com_caminho = cv2.cvtColor(image_com_caminho, cv2.COLOR_GRAY2RGB)

for i in caminho:

    image_com_caminho[i[0]][i[1]] = [0, 0, 255]

    
cv2.imshow('graycsale image',image_com_caminho)
# waitKey() waits for a key press to close the window and 0 specifies indefinite loop
cv2.waitKey(0)
 
# cv2.destroyAllWindows() simply destroys all the windows we created.
cv2.destroyAllWindows()