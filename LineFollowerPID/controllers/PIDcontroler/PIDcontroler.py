from controller import Robot
import numpy as np

# Constantes
TIME_STEP = 32
VELOCIDADE_BASE = 3.0  # Velocidade base do robô
Kp = 0.4  # Ganho proporcional
Ki = 0.2  # Ganho integral
Kd = 0.1  # Ganho derivativo

# Inicializa o robô
robot = Robot()

# Configura a câmera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
width = camera.getWidth()
height = camera.getHeight()

# Configura os motores
left_front_motor = robot.getDevice("front left wheel")
right_front_motor = robot.getDevice("front right wheel")
left_back_motor = robot.getDevice("back left wheel")
right_back_motor = robot.getDevice("back right wheel")

# Lista de motores
motors = [left_front_motor, right_front_motor, left_back_motor, right_back_motor]

# Configura os motores para rotação contínua
for motor in motors:
    motor.setPosition(float('inf'))  # Rotação contínua
    motor.setVelocity(0.0)  # Inicializa os motores parados

# Variáveis do PID
integral = 0.0
previous_error = 0.0

# Função para encontrar a posição média da linha verde
def calcular_centro_verde(image, width, height):
    green_threshold = 60  # Limite para detectar verde
    line_position = []  # Armazena as posições da linha verde
    
    for x in range(width):
        # Pega o valor RGB do pixel (linha do meio da imagem)
        r = camera.imageGetRed(image, width, x, height // 2)
        g = camera.imageGetGreen(image, width, x, height // 2)
        b = camera.imageGetBlue(image, width, x, height // 2)
        
        # Verifica se é verde suficiente
        if g > green_threshold and g > r and g > b:
            line_position.append(x)

    # Calcula a média da posição da linha
    if line_position:
        return sum(line_position) / len(line_position)
    else:
        return None  # Nenhuma linha encontrada

# Função para pegar os valores RGB do pixel central
def pegar_cor_pixel_central(image, width, height):
    center_x = width // 2
    center_y = height // 2
    red = camera.imageGetRed(image, width, center_x, center_y)
    green = camera.imageGetGreen(image, width, center_x, center_y)
    blue = camera.imageGetBlue(image, width, center_x, center_y)
    return red, green, blue

# Loop principal
while robot.step(TIME_STEP) != -1:
    # Obtém a imagem da câmera
    image = camera.getImage()

    # Calcula a posição da linha verde
    line_center = calcular_centro_verde(image, width, height)

    # Captura os valores RGB do pixel central
    red, green, blue = pegar_cor_pixel_central(image, width, height)

    # Calcula o erro
    if line_center is not None:
        # O erro é positivo se a linha estiver à direita do centro e negativo se estiver à esquerda
        error = (width // 2) - line_center  # Erro: distância do centro da imagem
    else:
        error = 0  # Nenhuma linha encontrada, manter o curso

    # Controle PID
    integral += error
    derivative = error - previous_error
    pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative)
    previous_error = error

    # Ajusta as velocidades dos motores com base no erro
    left_speed = VELOCIDADE_BASE - pid_output  # Motor esquerdo corrige a direção
    right_speed = VELOCIDADE_BASE + pid_output  # Motor direito corrige a direção

    # Aplica limites de velocidade
    left_speed = np.clip(left_speed, -6.28, 6.28)
    right_speed = np.clip(right_speed, -6.28, 6.28)

    # Define velocidades nos motores
    left_front_motor.setVelocity(left_speed)
    left_back_motor.setVelocity(left_speed)
    right_front_motor.setVelocity(right_speed)
    right_back_motor.setVelocity(right_speed)

    # Imprime informações úteis para debug
    print(f"Erro: {error}, PID: {pid_output}")
    ##, Left Speed: {left_speed}, Right Speed: {right_speed}")
    ##print(f"Cor do pixel central: R={red}, G={green}, B={blue}")
