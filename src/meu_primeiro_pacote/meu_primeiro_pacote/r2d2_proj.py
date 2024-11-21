import rclpy
import numpy
import tf_transformations
import time
import math

import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.colors import Normalize
from math import *

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug ('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.mestados = 0
        self.wait(0.5)
        self.dist = 0.0
        self.dr2d2 = 0.0
        self.angulo_robo = 0.0
        self.caminho = []
        
    
    def untillnine(self):
        obj = self.novoponto
        self.dist = math.dist((self.pose.position.x, self.pose.position.y), obj)
        #tentar 
        self.angulo_robo = math.atan2(obj[0] - self.pose.position.x, obj[1] - self.pose.position.y)


    def wait(self, max_seconds):
        start = time.time()
        count = 0
        while count < max_seconds:
            count = time.time() - start            
            rclpy.spin_once(self)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
       
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose
    
    def algoritmo_a_star(self):
        pgmf = open('src/meu_primeiro_pacote/meu_primeiro_pacote/my_map.pgm', 'rb')
        image = plt.imread(pgmf)

        image_copia = 1.0 * (image > 250)

        goal = (40, 300) 
        robo = (360, 80)

        image_copia[goal[0]][goal[1]] = 0 
        image_copia[robo[0]][robo[1]] = 0   

        def indice_menor_valor(lista_f, lista_c):
            if not lista_f or not lista_c :
                return None
            
            lista_f = np.array(lista_f)
            lista_c = np.array(lista_c)

            lista_ind = np.where(lista_f == min(lista_f))
            lista_ind = lista_ind[0]

            h_min = math.dist(lista_c[lista_ind[0]], goal)

            menor_indice = lista_ind[0]

            for i in lista_ind:
                h = math.dist(lista_c[i], goal)
                if (h < h_min):
                    h_min = h
                    menor_indice = i

            return menor_indice


        #algoritmo de busca pelo ponto
        image_copia[goal[0]][goal[1]] = 2 #objetivo é 2
        image_copia[robo[0]][robo[1]] = 1 #robo é 1

        menor_h = 1000
        ponto = robo
        parar = False
        coordenadas = list()
        caminho_f = list ()

        while(1):
            for l in range (-1,2):
                for c in range (-1,2):
                    try:
                        if(image_copia[ponto[0]+l][ponto[1]+c] == 1):
                            g = math.dist(robo, (ponto[0]+l,ponto[1]+c))
                            h = math.dist((ponto[0]+l,ponto[1]+c), goal)
                            f = g + h

                            image_copia[ponto[0]+ l][ponto[1]+c]= f
                            caminho_f.append(f)
                            coordenadas.append([ponto[0]+ l, ponto[1]+c])

                        if(ponto[0]+l == goal[0] and ponto[1]+c == goal[1]):
                            parar = True
                            break
                    except: 
                        continue
                if(parar == True):
                    break   
            if(parar == True):
                break   
            
            menor_ind = indice_menor_valor(caminho_f, coordenadas)
            ponto = coordenadas.pop(menor_ind)
            caminho_f.pop(menor_ind)

        #algoritmo de encontrar o caminho certo
        ponto_inicial = robo
        caminho = [robo]
        menor = image_copia[robo[0]][robo[1]] + 2
        menor_posicao = robo
        parar = False
        listafechada = list()

        while(1):
            for l in range (1,-2,-1):
                for c in range (1,-2,-1):
                    try:          
                        if(image_copia[ponto_inicial[0]+l][ponto_inicial[1]+c] > 1 and image_copia[ponto_inicial[0]+l][ponto_inicial[1]+c] < menor and ([ponto_inicial[0]+l],[ponto_inicial[1]+c]) not in listafechada and (([ponto_inicial[0]+l],[ponto_inicial[1]+c]) != ([ponto_inicial[0]], [ponto_inicial[1]]))):
                            menor = image_copia[ponto_inicial[0]+l][ponto_inicial[1]+c]
                            menor_posicao = (ponto_inicial[0]+l, ponto_inicial[1]+c)

                        if(ponto_inicial[0]+l == goal[0] and ponto_inicial[1]+c == goal[1]):
                            parar = True
                            break

                        listafechada.append(([ponto_inicial[0]+l],[ponto_inicial[1]+c]))
                    except: 
                        continue

                if(parar == True):
                    break 

            ponto_inicial = menor_posicao
            caminho.append(menor_posicao)
            menor = menor + 2

            if(parar == True):
                break 
        
        image_com_caminho = image.copy()
        image_com_caminho = cv2.cvtColor(image_com_caminho, cv2.COLOR_GRAY2RGB)

        for i in caminho:

            image_com_caminho[i[0]][i[1]] = [254, 0, 0]

        plt.imshow(image_com_caminho)

        self.caminho = caminho

    def transf_coord(self, caminho):
        caminho_simulacao = []
        for y, x in caminho:
            # Rotacionar 90 graus (invertendo as coordenadas e ajustando o sinal)
            # (y, x) -> (x, -y)
            
            # Ajustar a escala para a simulação (de 0-400 para -10 a 10)
            sim_x = (x / 400) * 20 - 10  # A coordenada x no mapa vai para o eixo y na simulação
            sim_y = (y / 400) * -20 + 10  # A coordenada y no mapa vai para o eixo x na simulação, mas com sinal invertido
            
            # Adicionar a coordenada transformada
            caminho_simulacao.append((sim_x, sim_y))
        self.caminho_simulacao = caminho_simulacao


    def run(self):
        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.5,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

        #self.get_logger().info ('Ordenando o robô: "ir para a frente"')
        #self.pub_cmd_vel.publish(self.ir_para_frente)
        rclpy.spin_once(self)

        self.get_logger().info ('Entrando no loop princial do nó.')
        
        self.algoritmo_a_star()
        self.transf_coord(self.caminho)
        self.novoponto = self.caminho_simulacao.pop(0)

        while(rclpy.ok):
            
            self.pose.orientation #orientation
            self.pose.position #posicao
            _, _, yaw = tf_transformations.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]) #aqui so precisa usar o yaw
        
            rclpy.spin_once(self)

            self.get_logger().debug ('Atualizando as distancias lidas pelo laser.')
            distancia_direita = numpy.array(self.laser[0:10]).mean()
            self.distancia_direita   = min((self.laser[  0: 80])) # -90 a -10 graus
            self.distancia_frente    = min((self.laser[ 80:100])) # -10 a  10 graus
            self.distancia_esquerda  = min((self.laser[100:180])) #  10 a  90 graus

            self.proxponto = self.caminho_simulacao[1]
            
            cmd = Twist()
            self.erro_ang = self.angulo_robo - yaw
            self.untillnine()
            self.get_logger().info (' estado = ' + str(self.mestados) + ' erro de angulo: ' + str(self.erro_ang) + ' dist ponto:' + str(self.dist))
            self.get_logger().info ( ' ponto atual: ' + str(self.novoponto)  + ' prox ponto:' + str(self.proxponto))

            #olhar para o ponto
            if self.mestados == 0:

                if(abs(self.erro_ang) >= 0.02): #se a angulacação dele for muito diferente do angulo do ponto
                    cmd.angular.z = 0.4 if self.erro_ang > 0 else -0.4 #giro esquerda
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('TENTANDO VIRAR PRO PONTO CALMA')
                
                else:
                    cmd.angular.z = 0.0
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('olhando para (9;9), dist=' + str(self.dist) + 'dr2d2=' + str(self.pose.position.x)+ str(self.pose.position.y))
                    #print(self.dist, self.distancia_frente)
                    self.mestados = 1
        
            elif self.mestados == 1: #andando para o ponto

                self.get_logger().info ('estado = 1, andando')
                cmd.linear.x = 0.5
                self.pub_cmd_vel.publish(cmd)
                self.get_logger().debug ("Distância para o ponto" + str(self.dist))
                 
                if(self.dist <= 0.2):
                    cmd.angular.z = 0.0
                    cmd.linear.x = 0.0
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('CHEGOUUUUU')
                    self.mestados = 2
                
                elif(self.dist > 1 or self.erro_ang >=0.9):
                    self.mestados = 0

            elif self.mestados == 2:
                self.novoponto = self.caminho_simulacao.pop(0)
                self.mestados = 0
                self.get_logger().debug ("caminho " + str(self.caminho_simulacao))

                

        self.get_logger().info ('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  

