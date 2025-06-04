import pygame
import threading
import time
import random
import math
import sys
import os
import json
import socket
import paho.mqtt.client as mqtt
from pygame.locals import *

# Configurações da janela
WINDOW_WIDTH = 1024
WINDOW_HEIGHT = 768
TITLE = "Rover Simulator - BitDogLab (COMPLETO + JOYSTICK CORRIGIDO)"

# Configurações MQTT
MQTT_BROKER_IP = "192.168.2.107"
MQTT_BROKER_PORT = 1885
MQTT_CLIENT_ID = "rover_simulator_complete"
MQTT_USERNAME = "torlee"
MQTT_PASSWORD = "123456"

# Tópicos MQTT
TOPIC_COMMANDS = "rover/commands"
TOPIC_STATUS = "rover/status"
TOPIC_JOYSTICK = "rover/joystick"
TOPIC_CAPTURE = "rover/capture"
TOPIC_SCORE = "rover/score"
TOPIC_HELLO = "rover/hello"

# Constantes de simulação
TERRAIN_ROUGHNESS = 0.1
MAX_SPEED = 5.0
BATTERY_DRAIN_RATE = 0.01
TEMPERATURE_BASE = 25.0
TEMPERATURE_VARIANCE = 10.0
CAPTURE_DISTANCE = 50

# Modos de operação do rover
MODE_MANUAL = 0
MODE_SEMI_AUTO = 1
MODE_AUTONOMOUS = 2

class CompleteRoverSimulator:
    def __init__(self):
        # Inicializa o pygame
        pygame.init()
        pygame.display.set_caption(TITLE)
        
        # Configura a janela
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 18)
        self.big_font = pygame.font.SysFont('Arial', 24, bold=True)
        
        # Carrega imagens e recursos
        self.load_assets()
        
        # Inicializa o estado do rover
        self.rover_x = WINDOW_WIDTH // 2
        self.rover_y = WINDOW_HEIGHT // 2
        self.rover_angle = 0
        self.rover_speed = 0
        self.rover_steering = 0
        self.rover_battery = 100.0
        self.rover_temperature = TEMPERATURE_BASE
        self.rover_mode = MODE_MANUAL
        self.rover_lights = False
        self.rover_camera = False
        
        # Variáveis para o terreno
        self.terrain_offset_x = 0
        self.terrain_offset_y = 0
        
        # Trajetória do rover
        self.trajectory = []
        self.max_trajectory_points = 100
        
        # Obstáculos e pontos de interesse
        self.obstacles = self.generate_obstacles(15)
        self.poi = self.generate_poi(8)  # Mais pontos para começar
        self.captured_poi = []
        self.capture_requested = False
        self.capture_score = 0
        self.capture_animation_time = 0
        self.capture_animation_pos = None
        
        # Variáveis MQTT
        self.mqtt_client = None
        self.mqtt_connected = False
        self.last_message_time = time.time()
        self.message_log = []
        self.max_log_entries = 5
        
        # Controle de execução
        self.running = True
        self.paused = False
        
        # Variáveis para modo autônomo
        self.autonomous_target = None
        self.autonomous_path = []
        
        # Lock para thread safety
        self.mqtt_lock = threading.Lock()
        
        # Dados do joystick para debug
        self.joystick_data = {
            "raw_speed": 0,
            "raw_steering": 0,
            "final_speed": 0,
            "final_steering": 0,
            "last_update": 0,
            "message_count": 0,
            "is_moving": False
        }
        
        # Inicializa MQTT
        self.init_mqtt()
        
    def load_assets(self):
        """Carrega as imagens e recursos necessários"""
        try:
            self.rover_img = pygame.image.load('rover.png').convert_alpha()
            self.rover_img = pygame.transform.scale(self.rover_img, (64, 64))
        except:
            # Cria rover personalizado se não encontrar a imagem
            self.rover_img = pygame.Surface((64, 64), pygame.SRCALPHA)
            pygame.draw.rect(self.rover_img, (200, 50, 50), (10, 10, 44, 44))
            pygame.draw.rect(self.rover_img, (50, 50, 200), (20, 5, 24, 15))
            pygame.draw.circle(self.rover_img, (30, 30, 30), (20, 50), 10)
            pygame.draw.circle(self.rover_img, (30, 30, 30), (44, 50), 10)
        
        # Imagens para obstáculos e pontos de interesse
        self.rock_img = pygame.Surface((40, 40), pygame.SRCALPHA)
        pygame.draw.circle(self.rock_img, (120, 120, 120), (20, 20), 15)
        pygame.draw.circle(self.rock_img, (80, 80, 80), (20, 20), 10)
        
        self.poi_img = pygame.Surface((30, 30), pygame.SRCALPHA)
        pygame.draw.circle(self.poi_img, (50, 200, 50), (15, 15), 12)
        pygame.draw.circle(self.poi_img, (100, 255, 100), (15, 15), 8)
        
        self.captured_poi_img = pygame.Surface((30, 30), pygame.SRCALPHA)
        pygame.draw.circle(self.captured_poi_img, (100, 100, 100), (15, 15), 8)
        pygame.draw.circle(self.captured_poi_img, (200, 200, 200), (15, 15), 5)
        
        # Ícones para luzes
        self.light_on_img = pygame.Surface((20, 20), pygame.SRCALPHA)
        pygame.draw.circle(self.light_on_img, (255, 255, 100), (10, 10), 8)
        
        self.light_off_img = pygame.Surface((20, 20), pygame.SRCALPHA)
        pygame.draw.circle(self.light_off_img, (100, 100, 100), (10, 10), 8)
    
    def init_mqtt(self):
        """Inicializa o cliente MQTT com configurações otimizadas"""
        try:
            # Teste de conectividade primeiro
            print(f"🔍 Testando conectividade TCP para {MQTT_BROKER_IP}:{MQTT_BROKER_PORT}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex((MQTT_BROKER_IP, MQTT_BROKER_PORT))
            sock.close()
            
            if result != 0:
                print(f"❌ Falha na conectividade TCP. Erro: {result}")
                self.mqtt_connected = False
                return
            else:
                print("✅ Conectividade TCP OK!")
            
            # Cria cliente MQTT
            self.mqtt_client = mqtt.Client(
                client_id=MQTT_CLIENT_ID,
                protocol=mqtt.MQTTv311,
                clean_session=True
            )
            
            # Configura autenticação MQTT
            self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
            
            # Configura callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            # Configurações de conexão otimizadas
            self.mqtt_client.max_inflight_messages_set(10)
            self.mqtt_client.max_queued_messages_set(100)
            
            print(f"🔌 Conectando ao broker MQTT em {MQTT_BROKER_IP}:{MQTT_BROKER_PORT}")
            
            # Conecta ao broker
            self.mqtt_client.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT, 60)
            
            # Inicia o loop MQTT em thread separada
            self.mqtt_client.loop_start()
            
            print("✅ Cliente MQTT iniciado com sucesso!")
            
        except Exception as e:
            print(f"❌ Erro ao inicializar MQTT: {e}")
            self.mqtt_connected = False
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback chamado quando conecta ao broker MQTT"""
        if rc == 0:
            print("🎉 CONECTADO AO BROKER MQTT COM SUCESSO!")
            self.mqtt_connected = True
            
            # Lista de tópicos para inscrever
            topics = [
                TOPIC_JOYSTICK,
                TOPIC_STATUS, 
                TOPIC_CAPTURE,
                TOPIC_HELLO
            ]
            
            print("📬 Inscrevendo em tópicos MQTT...")
            for topic in topics:
                try:
                    result = client.subscribe(topic)
                    if result[0] == mqtt.MQTT_ERR_SUCCESS:
                        print(f"✅ Inscrito em: {topic}")
                    else:
                        print(f"⚠️ Falha ao inscrever em {topic}: {result}")
                except Exception as e:
                    print(f"❌ Erro ao inscrever em {topic}: {e}")
            
            self.add_to_message_log("MQTT: ✅ Conectado!")
            
        else:
            print(f"❌ FALHA NA CONEXÃO MQTT! Código: {rc}")
            self.mqtt_connected = False
            self.add_to_message_log(f"MQTT: ❌ Falha (código {rc})")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback chamado quando desconecta do broker MQTT"""
        print("📡 Desconectado do broker MQTT")
        self.mqtt_connected = False
        self.add_to_message_log("MQTT: Desconectado")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback chamado quando recebe uma mensagem MQTT"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            # Log básico
            print(f"📨 MQTT [{topic}]: {payload}")
            self.add_to_message_log(f"RX [{topic}]: {payload[:40]}...")
            
            self.last_message_time = time.time()
            
            # Processa mensagem baseada no tópico
            if topic == TOPIC_HELLO:
                self.handle_hello_message(payload)
            elif topic == TOPIC_JOYSTICK:
                self.handle_joystick_message(payload)
            elif topic == TOPIC_STATUS:
                self.handle_status_message(payload)
            elif topic == TOPIC_CAPTURE:
                self.handle_capture_message(payload)
            else:
                print(f"⚠️ Tópico não reconhecido: {topic}")
                
        except UnicodeDecodeError as e:
            print(f"❌ Erro ao decodificar mensagem MQTT: {e}")
        except Exception as e:
            print(f"❌ Erro ao processar mensagem MQTT: {e}")
    
    def handle_hello_message(self, payload):
        """Processa mensagem de hello do Pico"""
        print(f"👋 Hello recebido do Pico: {payload}")
        # Responde com uma mensagem de comando para confirmar conexão
        self.publish_command("hello_ack=true")
        print("✅ Hello confirmado para o Pico!")
    
    def handle_joystick_message(self, payload):
        """Processa dados do joystick - VERSÃO CORRIGIDA FUNCIONANDO"""
        try:
            self.joystick_data["message_count"] += 1
            self.joystick_data["last_update"] = time.time()
            
            if payload.startswith('{') and payload.endswith('}'):
                data = json.loads(payload)
                
                # Extrai valores brutos
                speed_raw = data.get('speed', 0.0)
                steering_raw = data.get('steering', 0.0)
                mode = data.get('mode', 0)
                
                # Armazena para debug
                self.joystick_data["raw_speed"] = speed_raw
                self.joystick_data["raw_steering"] = steering_raw
                
                # CONVERSÃO CORRIGIDA - A MESMA QUE FUNCIONOU
                self.rover_speed = (speed_raw / 100.0) * MAX_SPEED
                self.rover_steering = steering_raw / 100.0
                self.rover_mode = mode
                
                # Armazena valores finais
                self.joystick_data["final_speed"] = self.rover_speed
                self.joystick_data["final_steering"] = self.rover_steering
                
                # Detecta movimento
                self.joystick_data["is_moving"] = (abs(speed_raw) > 5 or abs(steering_raw) > 5)
                
                # Debug só quando há movimento significativo
                if self.joystick_data["is_moving"]:
                    print(f"🎮 Joystick: speed={speed_raw:.1f}→{self.rover_speed:.2f}, steering={steering_raw:.1f}→{self.rover_steering:.2f}")
                
                # Processa luzes/câmera se presente no JSON
                if 'lights' in data:
                    self.rover_lights = data['lights'] in ['on', 'true', True]
                if 'camera' in data:
                    self.rover_camera = data['camera'] in ['on', 'true', True]
                if 'capture' in data and data.get('capture', False):
                    print("🟢 Comando de captura recebido via joystick!")
                    self.capture_requested = True
                
            else:
                print(f"🎮 Joystick (texto): {payload}")
                
        except json.JSONDecodeError as e:
            print(f"❌ Erro JSON no joystick: {e}")
        except Exception as e:
            print(f"❌ Erro ao processar joystick: {e}")
    
    def handle_status_message(self, payload):
        """Processa mensagem de status do Pico"""
        try:
            if payload.startswith('{') and payload.endswith('}'):
                data = json.loads(payload)
                
                if 'lights' in data:
                    self.rover_lights = data['lights'] in ['on', 'true', True]
                    print(f"💡 Luzes: {self.rover_lights}")
                    
                if 'camera' in data:
                    self.rover_camera = data['camera'] in ['on', 'true', True]
                    print(f"📷 Câmera: {self.rover_camera}")
                
            else:
                # Fallback para formato texto
                if 'lights=on' in payload or 'lights":"on"' in payload:
                    self.rover_lights = True
                elif 'lights=off' in payload or 'lights":"off"' in payload:
                    self.rover_lights = False
                    
                if 'camera=on' in payload or 'camera":"on"' in payload:
                    self.rover_camera = True
                elif 'camera=off' in payload or 'camera":"off"' in payload:
                    self.rover_camera = False
                
        except json.JSONDecodeError as e:
            print(f"❌ Erro JSON no status: {e}")
        except Exception as e:
            print(f"❌ Erro ao processar status: {e}")
    
    def handle_capture_message(self, payload):
        """Processa mensagem de captura do Pico"""
        try:
            if payload.startswith('{') and payload.endswith('}'):
                data = json.loads(payload)
                
                if data.get('capture', False):
                    print("🟢 Comando de captura recebido via MQTT!")
                    self.capture_requested = True
                    
            else:
                if 'capture=true' in payload or 'capture=1' in payload:
                    print("🟢 Comando de captura recebido via MQTT!")
                    self.capture_requested = True
                    
        except json.JSONDecodeError as e:
            print(f"❌ Erro JSON na captura: {e}")
        except Exception as e:
            print(f"❌ Erro ao processar captura: {e}")
    
    def publish_command(self, command):
        """Publica um comando para o Pico"""
        if not self.mqtt_connected or not self.mqtt_client:
            return False
            
        try:
            result = self.mqtt_client.publish(TOPIC_COMMANDS, command)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.add_to_message_log(f"TX [commands]: {command}")
                return True
        except Exception as e:
            print(f"❌ Erro ao publicar comando: {e}")
        return False
    
    def publish_score(self, score):
        """Publica o score atualizado para o Pico"""
        if not self.mqtt_connected or not self.mqtt_client:
            return False
            
        try:
            score_msg = f"score={score}"
            result = self.mqtt_client.publish(TOPIC_SCORE, score_msg)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.add_to_message_log(f"TX [score]: {score_msg}")
                print(f"🏆 Score enviado: {score}")
                return True
        except Exception as e:
            print(f"❌ Erro ao publicar score: {e}")
        return False
    
    def add_to_message_log(self, message):
        """Adiciona uma mensagem ao log"""
        with self.mqtt_lock:
            self.message_log.append(message)
            if len(self.message_log) > self.max_log_entries:
                self.message_log.pop(0)
    
    def generate_obstacles(self, count):
        """Gera obstáculos aleatórios no mapa"""
        obstacles = []
        for _ in range(count):
            # Evita gerar muito perto do centro onde o rover começa
            while True:
                x = random.randint(100, WINDOW_WIDTH - 100)
                y = random.randint(100, WINDOW_HEIGHT - 100)
                
                # Verifica se não está muito perto do centro
                center_x, center_y = WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2
                if math.sqrt((x - center_x)**2 + (y - center_y)**2) > 150:
                    size = random.randint(25, 45)
                    obstacles.append((x, y, size))
                    break
        return obstacles
    
    def generate_poi(self, count):
        """Gera pontos de interesse aleatórios no mapa"""
        points = []
        for _ in range(count):
            attempts = 0
            while attempts < 50:  # Evita loop infinito
                x = random.randint(80, WINDOW_WIDTH - 80)
                y = random.randint(80, WINDOW_HEIGHT - 80)
                
                # Verifica se está longe dos obstáculos
                valid = True
                for ox, oy, size in self.obstacles:
                    if math.sqrt((ox - x)**2 + (oy - y)**2) < size + 60:
                        valid = False
                        break
                
                # Verifica se está longe de outros pontos
                for px, py in points:
                    if math.sqrt((px - x)**2 + (py - y)**2) < 80:
                        valid = False
                        break
                
                # Verifica se não está muito perto do rover inicial
                center_x, center_y = WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2
                if math.sqrt((x - center_x)**2 + (y - center_y)**2) < 120:
                    valid = False
                
                if valid:
                    points.append((x, y))
                    break
                
                attempts += 1
        
        return points
    
    def try_capture_poi(self):
        """Tenta capturar um ponto de interesse próximo ao rover"""
        captured_any = False
        
        for poi in self.poi[:]:
            if poi in self.captured_poi:
                continue
                
            dist = math.sqrt((poi[0] - self.rover_x)**2 + (poi[1] - self.rover_y)**2)
            
            if dist < CAPTURE_DISTANCE:
                self.captured_poi.append(poi)
                self.capture_score += 100
                captured_any = True
                
                # Inicia animação de captura
                self.capture_animation_time = time.time()
                self.capture_animation_pos = poi
                
                print(f"🟢 PONTO CAPTURADO! Posição: {poi}, Score: {self.capture_score}")
                
                # Publica novo score via MQTT
                self.publish_score(self.capture_score)
                
                # Som de captura (simulado)
                print("🎵 *BEEP* Ponto coletado!")
                
                break
        
        # Adiciona novos pontos se poucos restarem
        remaining = len(self.poi) - len(self.captured_poi)
        if remaining < 3:
            self.add_new_poi(3)
            
        return captured_any
    
    def add_new_poi(self, count):
        """Adiciona novos pontos de interesse ao mapa"""
        added = 0
        attempts = 0
        
        while added < count and attempts < 100:
            x = random.randint(100, WINDOW_WIDTH - 100)
            y = random.randint(100, WINDOW_HEIGHT - 100)
            new_poi = (x, y)
            
            # Verifica se está longe de obstáculos
            valid = True
            for ox, oy, size in self.obstacles:
                if math.sqrt((ox - x)**2 + (oy - y)**2) < size + 70:
                    valid = False
                    break
            
            # Verifica se está longe de outros pontos
            for px, py in self.poi:
                if math.sqrt((px - x)**2 + (py - y)**2) < 100:
                    valid = False
                    break
            
            # Verifica se está longe do rover
            if math.sqrt((x - self.rover_x)**2 + (y - self.rover_y)**2) < 150:
                valid = False
            
            if valid:
                self.poi.append(new_poi)
                print(f"✨ Novo ponto de interesse adicionado: {new_poi}")
                added += 1
            
            attempts += 1
    
    def update(self):
        """Atualiza o estado da simulação"""
        if self.paused:
            return
            
        # Processa pedido de captura
        if self.capture_requested:
            self.try_capture_poi()
            self.capture_requested = False
            
        # Atualiza posição do rover baseado no joystick
        if self.rover_mode == MODE_MANUAL:
            self.update_manual_mode()
        elif self.rover_mode == MODE_SEMI_AUTO:
            self.update_semi_auto_mode()
        elif self.rover_mode == MODE_AUTONOMOUS:
            self.update_autonomous_mode()
        
        # Atualiza movimento
        delta_angle = self.rover_steering * 2.5  # Sensibilidade da direção
        self.rover_angle += delta_angle
        
        # Normaliza ângulo
        self.rover_angle = self.rover_angle % 360
        
        rad_angle = math.radians(self.rover_angle)
        
        dx = math.sin(rad_angle) * self.rover_speed
        dy = -math.cos(rad_angle) * self.rover_speed
        
        new_x = self.rover_x + dx
        new_y = self.rover_y + dy
        
        # Verifica colisão antes de mover
        if not self.check_collision(new_x, new_y):
            self.rover_x = new_x
            self.rover_y = new_y
            
            # Mantém rover dentro da tela
            self.rover_x = max(32, min(self.rover_x, WINDOW_WIDTH - 32))
            self.rover_y = max(32, min(self.rover_y, WINDOW_HEIGHT - 32))
        
        # Atualiza trajetória apenas se houver movimento significativo
        if abs(self.rover_speed) > 0.1:
            self.trajectory.append((int(self.rover_x), int(self.rover_y)))
            if len(self.trajectory) > self.max_trajectory_points:
                self.trajectory.pop(0)
        
        # Atualiza bateria e temperatura
        self.rover_battery -= abs(self.rover_speed) * BATTERY_DRAIN_RATE
        self.rover_battery = max(0.0, min(self.rover_battery, 100.0))
        
        temp_change = (random.random() - 0.5) * 0.2
        temp_change += abs(self.rover_speed) * 0.02
        self.rover_temperature += temp_change
        self.rover_temperature = max(TEMPERATURE_BASE - 5, 
                                   min(self.rover_temperature, TEMPERATURE_BASE + TEMPERATURE_VARIANCE))
        
        # Atualiza animação de captura
        if self.capture_animation_time > 0:
            if time.time() - self.capture_animation_time > 2.0:
                self.capture_animation_time = 0
                self.capture_animation_pos = None
    
    def update_manual_mode(self):
        """Atualização para modo manual - controlado pelo joystick"""
        pass  # O movimento já é tratado pelos valores do joystick
    
    def update_semi_auto_mode(self):
        """Modo semi-autônomo com assistência de colisão"""
        # Detecta obstáculo próximo
        min_distance = float('inf')
        closest_obstacle = None
        
        for ox, oy, size in self.obstacles:
            dist = math.sqrt((ox - self.rover_x)**2 + (oy - self.rover_y)**2)
            if dist < min_distance:
                min_distance = dist
                closest_obstacle = (ox, oy, size)
        
        # Se há obstáculo próximo, ajusta direção automaticamente
        if min_distance < 120:
            ox, oy, _ = closest_obstacle
            angle_to_obstacle = math.degrees(math.atan2(ox - self.rover_x, -(oy - self.rover_y))) % 360
            angle_diff = (angle_to_obstacle - self.rover_angle) % 360
            if angle_diff > 180:
                angle_diff -= 360
            
            repulsion = 1.0 - min_distance / 120.0
            steering_adjust = -math.copysign(repulsion, angle_diff)
            self.rover_steering = max(-1.0, min(1.0, self.rover_steering + steering_adjust * 0.3))
            
            # Reduz velocidade perto de obstáculos
            self.rover_speed *= (0.7 + 0.3 * (min_distance / 120.0))
    
    def update_autonomous_mode(self):
        """Modo totalmente autônomo - busca pontos automaticamente"""
        # Escolhe alvo se não há um
        if not self.autonomous_target:
            min_distance = float('inf')
            closest_poi = None
            
            for poi in self.poi:
                if poi in self.captured_poi:
                    continue
                
                dist = math.sqrt((poi[0] - self.rover_x)**2 + (poi[1] - self.rover_y)**2)
                if dist < min_distance:
                    min_distance = dist
                    closest_poi = poi
            
            if closest_poi:
                self.autonomous_target = closest_poi
                print(f"🎯 Novo alvo autônomo: {closest_poi}")
        
        # Move em direção ao alvo
        if self.autonomous_target:
            tx, ty = self.autonomous_target
            dist = math.sqrt((tx - self.rover_x)**2 + (ty - self.rover_y)**2)
            
            # Se chegou perto, captura
            if dist < 40:
                if self.autonomous_target not in self.captured_poi:
                    self.capture_requested = True
                
                self.autonomous_target = None
                return
            
            # Calcula direção para o alvo
            target_angle = math.degrees(math.atan2(tx - self.rover_x, -(ty - self.rover_y))) % 360
            angle_diff = (target_angle - self.rover_angle) % 360
            if angle_diff > 180:
                angle_diff -= 360
            
            # Ajusta direção
            self.rover_steering = max(-1.0, min(1.0, angle_diff / 60.0))
            
            # Ajusta velocidade baseada no ângulo
            speed_factor = 1.0 - min(1.0, abs(angle_diff) / 90.0) * 0.6
            self.rover_speed = MAX_SPEED * speed_factor * 0.6
    
    def check_collision(self, x, y):
        """Verifica colisão com obstáculos"""
        rover_radius = 32
        
        for ox, oy, size in self.obstacles:
            obstacle_radius = size / 2
            dist = math.sqrt((ox - x)**2 + (oy - y)**2)
            
            if dist < (rover_radius + obstacle_radius):
                return True
        
        return False
    
    def draw(self):
        """Desenha a simulação completa"""
        # Limpa a tela com cor de fundo
        self.screen.fill((40, 60, 40))  # Verde escuro para simular terreno
        
        # Desenha grid sutil
        grid_size = 50
        for x in range(0, WINDOW_WIDTH, grid_size):
            pygame.draw.line(self.screen, (50, 70, 50), (x, 0), (x, WINDOW_HEIGHT), 1)
        for y in range(0, WINDOW_HEIGHT, grid_size):
            pygame.draw.line(self.screen, (50, 70, 50), (0, y), (WINDOW_WIDTH, y), 1)
        
        # Desenha trajetória do rover
        if len(self.trajectory) > 2:
            pygame.draw.lines(self.screen, (100, 150, 255), False, self.trajectory, 3)
        
        # Desenha obstáculos
        for x, y, size in self.obstacles:
            scaled_img = pygame.transform.scale(self.rock_img, (size, size))
            self.screen.blit(scaled_img, (x - size//2, y - size//2))
            
            # Sombra do obstáculo
            shadow_rect = pygame.Rect(x - size//2 + 3, y - size//2 + 3, size, size)
            pygame.draw.ellipse(self.screen, (20, 30, 20), shadow_rect)
        
        # Desenha pontos de interesse
        for x, y in self.poi:
            if (x, y) in self.captured_poi:
                # Ponto capturado
                self.screen.blit(self.captured_poi_img, (x - 15, y - 15))
            else:
                # Ponto disponível - com brilho
                self.screen.blit(self.poi_img, (x - 15, y - 15))
                
                # Efeito de brilho pulsante
                pulse = int(127 + 127 * math.sin(time.time() * 3))
                glow_surface = pygame.Surface((40, 40), pygame.SRCALPHA)
                pygame.draw.circle(glow_surface, (50, pulse, 50, 60), (20, 20), 20)
                self.screen.blit(glow_surface, (x - 20, y - 20))
            
            # Marca alvo autônomo
            if self.autonomous_target and (x, y) == self.autonomous_target:
                pygame.draw.circle(self.screen, (255, 100, 100), (x, y), 25, 3)
                
            # Área de captura para pontos não capturados
            if not (x, y) in self.captured_poi:
                dist = math.sqrt((x - self.rover_x)**2 + (y - self.rover_y)**2)
                if dist < CAPTURE_DISTANCE:
                    pygame.draw.circle(self.screen, (255, 255, 100, 80), (x, y), 30, 2)
        
        # Desenha o rover
        self.draw_rover()
        
        # Desenha área de captura do rover
        capture_surface = pygame.Surface((CAPTURE_DISTANCE*2, CAPTURE_DISTANCE*2), pygame.SRCALPHA)
        pygame.draw.circle(capture_surface, (100, 100, 250, 30), 
                          (CAPTURE_DISTANCE, CAPTURE_DISTANCE), CAPTURE_DISTANCE)
        self.screen.blit(capture_surface, 
                        (self.rover_x - CAPTURE_DISTANCE, self.rover_y - CAPTURE_DISTANCE))
        
        # Desenha luzes do rover
        if self.rover_lights:
            self.draw_rover_lights()
        
        # Desenha animação de captura
        if self.capture_animation_time > 0 and self.capture_animation_pos:
            self.draw_capture_animation()
        
        # Desenha painéis de informação
        self.draw_info_panel()
        self.draw_score_info()
        self.draw_message_log()
        
        # Tela de pausa
        if self.paused:
            self.draw_pause_overlay()
        
        pygame.display.flip()
    
    def draw_rover(self):
        """Desenha o rover com detalhes"""
        # Rotaciona a imagem do rover
        rotated_rover = pygame.transform.rotate(self.rover_img, -self.rover_angle)
        rover_rect = rotated_rover.get_rect(center=(self.rover_x, self.rover_y))
        self.screen.blit(rotated_rover, rover_rect.topleft)
        
        # Indicador de direção
        rad_angle = math.radians(self.rover_angle)
        arrow_length = 40
        end_x = self.rover_x + math.sin(rad_angle) * arrow_length
        end_y = self.rover_y - math.cos(rad_angle) * arrow_length
        
        pygame.draw.line(self.screen, (255, 255, 100), 
                        (self.rover_x, self.rover_y), (end_x, end_y), 4)
        pygame.draw.circle(self.screen, (255, 255, 100), (int(end_x), int(end_y)), 6)
    
    def draw_rover_lights(self):
        """Desenha as luzes do rover"""
        angle_rad = math.radians(self.rover_angle)
        light_dist = 45
        light_spread = 25
        
        # Duas luzes
        for spread in [-light_spread, light_spread]:
            light_angle = angle_rad + math.radians(spread)
            lx = self.rover_x + math.sin(light_angle) * light_dist
            ly = self.rover_y - math.cos(light_angle) * light_dist
            
            # Feixes de luz com diferentes intensidades
            for i in range(5, 120, 8):
                alpha = max(0, 200 - i * 1.8)
                radius = i / 6
                light_surf = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
                pygame.draw.circle(light_surf, (255, 255, 200, alpha), (radius, radius), radius)
                self.screen.blit(light_surf, (lx - radius, ly - radius))
    
    def draw_capture_animation(self):
        """Desenha animação de captura"""
        elapsed = time.time() - self.capture_animation_time
        x, y = self.capture_animation_pos
        
        if elapsed < 1.0:
            # Expansão
            size = int(20 + 30 * elapsed)
            for i in range(3):
                color_alpha = max(0, 255 - elapsed * 200 - i * 50)
                pygame.draw.circle(self.screen, (255, 255 - i*50, 0, color_alpha), 
                                 (x, y), size + i * 10, 3)
        else:
            # Contração
            progress = elapsed - 1.0
            size = max(5, int(50 - 50 * progress))
            pygame.draw.circle(self.screen, (255, 200, 0), (x, y), size, 4)
        
        # Texto de pontos
        if elapsed < 1.5:
            points_text = self.big_font.render("+100", True, (255, 255, 100))
            text_y = y - 50 - int(elapsed * 20)
            alpha = max(0, 255 - elapsed * 170)
            points_text.set_alpha(alpha)
            text_rect = points_text.get_rect(center=(x, text_y))
            self.screen.blit(points_text, text_rect)
    
    def draw_info_panel(self):
        """Desenha painel de informações do rover"""
        panel_rect = pygame.Rect(10, 10, 280, 240)
        pygame.draw.rect(self.screen, (30, 30, 50, 200), panel_rect)
        pygame.draw.rect(self.screen, (100, 100, 150), panel_rect, 2)
        
        title = self.big_font.render("ROVER STATUS", True, (255, 255, 255))
        self.screen.blit(title, (20, 20))
        
        pygame.draw.line(self.screen, (100, 100, 150), (20, 50), (270, 50), 2)
        
        y_pos = 60
        
        # Status MQTT com ícone
        mqtt_status = "🟢 Conectado" if self.mqtt_connected else "🔴 Desconectado"
        mqtt_color = (50, 255, 50) if self.mqtt_connected else (255, 50, 50)
        mqtt_text = self.font.render(f"MQTT: {mqtt_status}", True, mqtt_color)
        self.screen.blit(mqtt_text, (20, y_pos))
        y_pos += 25
        
        # Joystick status
        joystick_active = self.joystick_data["is_moving"]
        joy_status = "🎮 Ativo" if joystick_active else "🎮 Parado"
        joy_color = (100, 255, 100) if joystick_active else (150, 150, 150)
        joy_text = self.font.render(joy_status, True, joy_color)
        self.screen.blit(joy_text, (20, y_pos))
        y_pos += 25
        
        # Velocidade com barra
        speed_percent = abs(self.rover_speed/MAX_SPEED*100)
        speed_text = self.font.render(f"Velocidade: {speed_percent:.1f}%", True, (255, 255, 255))
        self.screen.blit(speed_text, (20, y_pos))
        
        # Barra de velocidade
        bar_rect = pygame.Rect(150, y_pos + 5, 100, 12)
        pygame.draw.rect(self.screen, (50, 50, 50), bar_rect)
        if speed_percent > 0:
            fill_width = int(speed_percent)
            fill_rect = pygame.Rect(150, y_pos + 5, fill_width, 12)
            bar_color = (0, 255, 0) if self.rover_speed >= 0 else (255, 150, 0)
            pygame.draw.rect(self.screen, bar_color, fill_rect)
        pygame.draw.rect(self.screen, (200, 200, 200), bar_rect, 1)
        y_pos += 30
        
        # Direção
        dir_percent = self.rover_steering*100
        dir_text = self.font.render(f"Direção: {dir_percent:+.1f}%", True, (255, 255, 255))
        self.screen.blit(dir_text, (20, y_pos))
        y_pos += 25
        
        # Bateria com barra colorida
        bat_text = self.font.render(f"Bateria: {self.rover_battery:.1f}%", True, (255, 255, 255))
        self.screen.blit(bat_text, (20, y_pos))
        
        bat_rect = pygame.Rect(130, y_pos + 5, 120, 12)
        bat_fill = pygame.Rect(130, y_pos + 5, int(self.rover_battery * 1.2), 12)
        
        # Cor da bateria baseada no nível
        if self.rover_battery > 60:
            bat_color = (0, 255, 0)
        elif self.rover_battery > 30:
            bat_color = (255, 255, 0)
        else:
            bat_color = (255, 0, 0)
        
        pygame.draw.rect(self.screen, (50, 50, 50), bat_rect)
        pygame.draw.rect(self.screen, bat_color, bat_fill)
        pygame.draw.rect(self.screen, (200, 200, 200), bat_rect, 1)
        y_pos += 30
        
        # Modo de operação
        mode_names = ["🕹️ Manual", "🤖 Semi-Auto", "🎯 Autônomo"]
        mode_text = self.font.render(f"Modo: {mode_names[self.rover_mode]}", True, (255, 255, 255))
        self.screen.blit(mode_text, (20, y_pos))
        y_pos += 25
        
        # Luzes e câmera com ícones visuais
        light_color = (255, 255, 100) if self.rover_lights else (100, 100, 100)
        light_icon = "💡" if self.rover_lights else "🔆"
        lights_text = self.font.render(f"{light_icon} Luzes: {'ON' if self.rover_lights else 'OFF'}", True, light_color)
        self.screen.blit(lights_text, (20, y_pos))
        
        camera_color = (255, 255, 100) if self.rover_camera else (100, 100, 100)
        camera_icon = "📹" if self.rover_camera else "📷"
        camera_text = self.font.render(f"{camera_icon} Câmera: {'ON' if self.rover_camera else 'OFF'}", True, camera_color)
        self.screen.blit(camera_text, (150, y_pos))
    
    def draw_score_info(self):
        """Desenha informações de pontuação"""
        # Painel de score
        score_rect = pygame.Rect(WINDOW_WIDTH - 220, 10, 200, 120)
        pygame.draw.rect(self.screen, (50, 30, 50, 200), score_rect)
        pygame.draw.rect(self.screen, (150, 100, 150), score_rect, 2)
        
        # Score principal
        score_text = self.big_font.render(f"SCORE: {self.capture_score}", True, (255, 255, 100))
        self.screen.blit(score_text, (WINDOW_WIDTH - 210, 20))
        
        # Estatísticas
        total_points = len(self.poi)
        captured_points = len(self.captured_poi)
        remaining_points = total_points - captured_points
        
        stats_y = 55
        
        captured_text = self.font.render(f"🟢 Coletados: {captured_points}", True, (100, 255, 100))
        self.screen.blit(captured_text, (WINDOW_WIDTH - 210, stats_y))
        
        remaining_text = self.font.render(f"🎯 Restantes: {remaining_points}", True, (255, 200, 100))
        self.screen.blit(remaining_text, (WINDOW_WIDTH - 210, stats_y + 20))
        
        # Progresso
        if total_points > 0:
            progress = (captured_points / total_points) * 100
            progress_text = self.font.render(f"📊 Progresso: {progress:.1f}%", True, (200, 200, 255))
            self.screen.blit(progress_text, (WINDOW_WIDTH - 210, stats_y + 40))
        
        # Instruções de controle
        if self.mqtt_connected:
            help_text = self.font.render("🎮 Use o joystick e botões do Pico W", True, (200, 200, 255))
            self.screen.blit(help_text, (WINDOW_WIDTH // 2 - 150, WINDOW_HEIGHT - 25))
        else:
            help_text = self.font.render("❌ MQTT desconectado - Verifique conexão", True, (255, 100, 100))
            self.screen.blit(help_text, (WINDOW_WIDTH // 2 - 180, WINDOW_HEIGHT - 25))
    
    def draw_message_log(self):
        """Desenha log de mensagens MQTT"""
        if not self.message_log:
            return
            
        with self.mqtt_lock:
            log_messages = self.message_log.copy()
        
        log_x = 10
        log_y = WINDOW_HEIGHT - len(log_messages) * 20 - 10
        
        # Fundo semi-transparente
        log_bg = pygame.Surface((500, len(log_messages) * 20 + 10), pygame.SRCALPHA)
        log_bg.fill((0, 0, 0, 120))
        self.screen.blit(log_bg, (log_x - 5, log_y - 5))
        
        for i, message in enumerate(log_messages):
            if message.startswith("TX"):
                color = (100, 255, 100)  # Verde para enviadas
            else:
                color = (255, 200, 100)  # Laranja para recebidas
            
            text = self.font.render(message, True, color)
            self.screen.blit(text, (log_x, log_y + i * 20))
    
    def draw_pause_overlay(self):
        """Desenha overlay de pausa"""
        overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 150))
        self.screen.blit(overlay, (0, 0))
        
        pause_text = self.big_font.render("⏸️ SIMULAÇÃO PAUSADA", True, (255, 255, 255))
        text_rect = pause_text.get_rect(center=(WINDOW_WIDTH//2, WINDOW_HEIGHT//2))
        self.screen.blit(pause_text, text_rect)
        
        resume_text = self.font.render("Pressione P para continuar", True, (200, 200, 200))
        resume_rect = resume_text.get_rect(center=(WINDOW_WIDTH//2, WINDOW_HEIGHT//2 + 40))
        self.screen.blit(resume_text, resume_rect)
    
    def handle_events(self):
        """Processa eventos do pygame"""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
                return False
            
            elif event.type == KEYDOWN:
                if event.key == K_p:
                    self.paused = not self.paused
                    print(f"Simulação {'pausada' if self.paused else 'retomada'}")
                
                elif event.key == K_r:
                    self.rover_battery = 100.0
                    print("🔋 Bateria recarregada!")
                
                elif event.key == K_ESCAPE:
                    self.running = False
                    return False
                
                elif event.key == K_SPACE:
                    print("🟢 Simulando captura manual")
                    self.capture_requested = True
                
                elif event.key == K_F1:
                    self.rover_mode = MODE_MANUAL
                    print("🕹️ Modo Manual ativado")
                elif event.key == K_F2:
                    self.rover_mode = MODE_SEMI_AUTO
                    print("🤖 Modo Semi-Autônomo ativado")
                elif event.key == K_F3:
                    self.rover_mode = MODE_AUTONOMOUS
                    print("🎯 Modo Autônomo ativado")
                
                elif event.key == K_n:
                    # Adiciona novos pontos manualmente
                    self.add_new_poi(3)
                    print("✨ Novos pontos adicionados!")
                
                elif event.key == K_c:
                    # Limpa trajetória
                    self.trajectory.clear()
                    print("🧹 Trajetória limpa!")
        
        return True
    
    def run(self):
        """Loop principal da simulação"""
        try:
            print("🚀 Iniciando simulação completa...")
            print("🎮 Controle o rover com o joystick do Pico W!")
            
            while self.running:
                if not self.handle_events():
                    break
                
                self.update()
                self.draw()
                self.clock.tick(60)  # 60 FPS
                
        finally:
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            pygame.quit()
            print("🏁 Simulação encerrada")

if __name__ == "__main__":
    print("🚀" + "="*60 + "🚀")
    print("🚀           ROVER SIMULATOR COMPLETO v3.0            🚀")
    print("🚀              COM JOYSTICK CORRIGIDO               🚀")
    print("🚀" + "="*60 + "🚀")
    print(f"🌐 Broker MQTT: {MQTT_BROKER_IP}:{MQTT_BROKER_PORT}")
    print(f"👤 Usuário: {MQTT_USERNAME}")
    print("")
    print("🎯 OBJETIVOS:")
    print("   • Controle o rover com joystick e botões do Pico W")
    print("   • Colete pontos verdes (🟢) espalhados pelo mapa")
    print("   • Evite obstáculos rochosos (🪨)")
    print("   • Acumule pontos e monitore seu progresso")
    print("")
    print("🎮 CONTROLES DO PICO W:")
    print("   • Joystick: Movimento (Y=frente/trás, X=esquerda/direita)")
    print("   • Botão A: Capturar pontos próximos")
    print("   • Botão B: Ligar/desligar luzes")
    print("   • Botão C: Ligar/desligar câmera")
    print("")
    print("⌨️ CONTROLES DO TECLADO:")
    print("   • P: Pausar/retomar • R: Recarregar bateria")
    print("   • ESPAÇO: Captura manual • F1/F2/F3: Mudar modo")
    print("   • N: Adicionar pontos • C: Limpar trajetória")
    print("   • ESC: Sair")
    print("")
    print("🔧 MODOS DE OPERAÇÃO:")
    print("   🕹️ Manual: Controle total via joystick")
    print("   🤖 Semi-Auto: Assistência anti-colisão")
    print("   🎯 Autônomo: Rover busca pontos automaticamente")
    print("="*64)
    
    try:
        simulator = CompleteRoverSimulator()
        simulator.run()
    except KeyboardInterrupt:
        print("\n👋 Simulador encerrado pelo usuário")
    except Exception as e:
        print(f"\n❌ Erro crítico no simulador: {e}")
        import traceback
        traceback.print_exc()