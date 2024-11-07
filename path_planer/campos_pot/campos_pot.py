import cv2
import numpy as np
from matplotlib import pyplot as plt

pgmf = open('my_map.pgm', 'rb')
image = plt.imread(pgmf)
image_copia = 1.0 * (image > 250)

goal = np.array([70, 300])
robo = np.array([300, 25])

image_copia[goal[0], goal[1]] = 0
image_copia[robo[0], robo[1]] = 0

ka = 2
kr = 300
p0 = 20
passo = 0.5  
ruido = 0.1

def interpolar_pontos(ponto1, ponto2):
    pontos = []
    dist = np.linalg.norm(ponto2 - ponto1)
    num_passos = int(np.ceil(dist))
    for i in range(1, num_passos + 1):
        ponto_interpolado = ponto1 + i * (ponto2 - ponto1) / num_passos
        pontos.append(ponto_interpolado.astype(int))
    return pontos

def forca_atrativa(pos, goal, ka=2):
    direcao = goal - pos
    distancia = np.linalg.norm(direcao)
    if distancia > 0:
        return ka * direcao / distancia
    return np.array([0, 0])

def forca_repulsiva(pos, obstaculos, kr=300, p0=20):
    forca_total = np.array([0.0, 0.0])
    for obs in obstaculos:
        direcao = pos - obs
        distancia = np.linalg.norm(direcao)
        if 0 < distancia < p0:
            repul = kr * ((1 / distancia) - (1 / p0)) * (direcao / (distancia**3))
            forca_total += repul
    return forca_total

obstaculos = np.argwhere(image_copia == 0)
caminho = [robo]

posicao_atual = robo.copy()
ultimo_passo = posicao_atual.copy()
tolerancia_movimento = 0.1

while np.linalg.norm(goal - posicao_atual) > 2:
    fa = forca_atrativa(posicao_atual, goal)
    fr = forca_repulsiva(posicao_atual, obstaculos)
    ft = fa + fr

    ruido_aleatorio = np.random.uniform(-ruido, ruido, 2)
    nova_posicao = posicao_atual + passo * (ft + ruido_aleatorio)

    pontos_interpolados = interpolar_pontos(posicao_atual, nova_posicao)
    caminho.extend(pontos_interpolados)

    posicao_atual = nova_posicao
    caminho.append(posicao_atual.astype(int))

    if np.linalg.norm(posicao_atual - ultimo_passo) < tolerancia_movimento:
        print("O robô está preso em um ponto. Tentando aplicar ruído para sair.")
        ultimo_passo += ruido_aleatorio

    if len(caminho) > 1200:
        print("O caminho é longo demais, verifique parâmetros.")
        break

image_com_caminho = cv2.cvtColor(image.copy(), cv2.COLOR_GRAY2RGB)
for ponto in caminho:
    if 0 <= ponto[0] < image_com_caminho.shape[0] and 0 <= ponto[1] < image_com_caminho.shape[1]:
        image_com_caminho[ponto[0], ponto[1]] = [0, 255, 0]

plt.imshow(image_com_caminho)
plt.title('Campos Potenciais (Caminho Suavizado)')
plt.show()
