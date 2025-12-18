"""
Sistema de Visão Computacional para Medição e Classificação de Caixas
- Prioriza retângulos alinhados com os eixos da câmera (±10 graus)
- Funcionamento em tempo real com câmera ou vídeo
- Calibração automática usando objeto de referência
- Classificação em três tipos: 10x20, 20x20, 30x50 cm
- Saída via mapa de bits para comunicação com ARD
"""

import cv2
import numpy as np
from scipy.spatial.distance import euclidean
from imutils import perspective
from imutils import contours
import imutils
import time
from collections import deque
import math

class ARDOutputMap:
    def __init__(self):
        # Mapa de bits (8 bits para comunicação simples)
        # Bit 0: Câmera ligada e operante
        # Bit 1: Erro no sistema
        # Bit 2: Caixa detectada
        # Bits 3-4: Tipo de caixa (00=nada, 01=10x20, 10=20x20, 11=30x50)
        # Bits 5-7: Reservados
        self.output_byte = 0b00000000
        
        # Histórico para estabilização
        self.output_history = deque(maxlen=5)
        
        # Timer para verificação de funcionamento
        self.last_update_time = time.time()
        self.operational_timeout = 2.0  # 2 segundos sem atualização = erro
        
    def update_camera_status(self, is_operational):
        """Atualiza status da câmera"""
        if is_operational:
            self.output_byte |= 0b00000001  # Bit 0 = 1 (câmera operante)
        else:
            self.output_byte &= 0b11111110  # Bit 0 = 0 (câmera com problema)
    
    def update_error_status(self, has_error):
        """Atualiza status de erro"""
        if has_error:
            self.output_byte |= 0b00000010  # Bit 1 = 1 (erro)
        else:
            self.output_byte &= 0b11111101  # Bit 1 = 0 (sem erro)
    
    def update_box_detected(self, is_detected):
        """Atualiza status de detecção de caixa"""
        if is_detected:
            self.output_byte |= 0b00000100  # Bit 2 = 1 (caixa detectada)
        else:
            self.output_byte &= 0b11111011  # Bit 2 = 0 (nada detectado)
    
    def update_box_type(self, box_type):
        """Atualiza tipo da caixa detectada"""
        # Primeiro limpa os bits de tipo (bits 3 e 4)
        self.output_byte &= 0b11100111
        
        # Mapeia tipo para bits
        if box_type == "10x20":
            self.output_byte |= 0b00001000  # Bits 3-4 = 01
        elif box_type == "20x20":
            self.output_byte |= 0b00010000  # Bits 3-4 = 10
        elif box_type == "30x50":
            self.output_byte |= 0b00011000  # Bits 3-4 = 11
        else:
            # Nenhum tipo válido ou UNKNOWN
            self.output_byte |= 0b00000000  # Bits 3-4 = 00
    
    def check_operational_timeout(self):
        """Verifica se o sistema está respondendo"""
        current_time = time.time()
        if current_time - self.last_update_time > self.operational_timeout:
            return False
        return True
    
    def get_output_string(self):
        """Retorna string formatada do mapa de bits"""
        camera_status = "1" if (self.output_byte & 0b00000001) else "0"
        error_status = "1" if (self.output_byte & 0b00000010) else "0"
        detection_status = "1" if (self.output_byte & 0b00000100) else "0"
        
        # Determinar tipo da caixa
        type_bits = (self.output_byte & 0b00011000) >> 3
        if type_bits == 0b01:
            box_type_str = "10x20"
        elif type_bits == 0b10:
            box_type_str = "20x20"
        elif type_bits == 0b11:
            box_type_str = "30x50"
        else:
            box_type_str = "NONE"
        
        # Formato binário para debug
        binary_str = format(self.output_byte, '08b')
        
        return {
            'binary': binary_str,
            'camera': camera_status,
            'error': error_status,
            'detection': detection_status,
            'type': box_type_str
        }
    
    def send_to_ARD(self):
        """Simula envio do mapa de bits para ARD"""
        output_info = self.get_output_string()
        
        # Simulação: apenas imprime o estado
        print(f"[Arduino OUT] Bin: {output_info['binary']} | "
              f"Camera: {output_info['camera']} | Erro: {output_info['error']} | "
              f"Detecao: {output_info['detection']} | Tipo: {output_info['type']}")
        
        # Adicionar ao histórico
        self.output_history.append(self.output_byte)
        
        # Atualizar timestamp
        self.last_update_time = time.time()
        
        return output_info

class BoxMeasurementSystem:
    def __init__(self):
        # Configurações do sistema
        self.box_types = {
            "10x20": (10.0, 20.0),
            "20x20": (20.0, 20.0), 
            "30x50": (30.0, 50.0)
        }
        self.tolerance_cm = 3.0  # Tolerância de ±3 cm
        
        # Parâmetros de processamento
        self.gaussian_blur = (9, 9)
        self.canny_threshold1 = 30
        self.canny_threshold2 = 100
        self.min_contour_area = 1000
        
        # Parâmetros para detecção de retângulos alinhados
        self.min_rectangle_score = 0.1  # Score mínimo para considerar como retângulo
        self.max_angle_deviation = 10   # Máximo 10 graus de inclinação em relação aos eixos
        
        # Sistema de calibração
        self.pixels_per_cm = None
        self.calibrated = False
        self.reference_width_cm = 1  # Largura do objeto de referência (2x2 cm)
        self.reference_height_cm = 1  # Altura do objeto de referência
        
        # Estado do sistema
        self.camera = None
        self.frame = None
        self.processing = True
        self.show_debug = False
        self.video_source = None  # Pode ser índice da câmera ou caminho de vídeo
        self.is_video_file = False
        
        # Histórico para estabilização
        self.measurement_history = deque(maxlen=20)
        self.box_type_history = deque(maxlen=20)
        
        # Comunicação ARD
        self.ARD_connected = True
        self.ARD_output = ARDOutputMap()
        
        # Controle de erros
        self.system_errors = {
            'camera_error': False,
            'calibration_error': False,
            'processing_error': False,
            'timeout_error': False
        }
        
        # Contador para estabilização
        self.stable_detection_count = 0
        self.last_stable_type = None
        
        # Estatísticas
        self.total_boxes = 0
        self.valid_boxes = 0
        self.frames_processed = 0
        
    def initialize_video_source(self, source):
        """Inicializa a fonte de vídeo (câmera ou arquivo)"""
        self.video_source = source
        
        # Verificar se é um arquivo de vídeo
        if isinstance(source, str):
            if source.isdigit():
                source = int(source)
                self.is_video_file = False
            else:
                self.is_video_file = True
        
        # Se for índice de câmera
        if not self.is_video_file:
            if isinstance(source, str):
                source = int(source)
            self.camera = cv2.VideoCapture(source)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        else:
            self.camera = cv2.VideoCapture(source)
        
        # Testar se a fonte está funcionando
        ret, test_frame = self.camera.read()
        if not ret:
            print(f"Erro: Não foi possível acessar a fonte de vídeo: {source}")
            return False
        
        # Retornar o frame de teste à posição original se for vídeo
        if self.is_video_file:
            self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
        
        print(f"Fonte de vídeo inicializada: {source}")
        print(f"Resolução: {test_frame.shape[1]}x{test_frame.shape[0]}")
        return True
    
    def is_aligned_rectangle(self, contour):
        """Verifica se um contorno é um retângulo alinhado com os eixos da câmera"""
        try:
            # Obter retângulo de área mínima
            rect = cv2.minAreaRect(contour)
            (center, size, angle) = rect
            
            # Normalizar ângulo para 0-90 graus
            if angle < -45:
                angle = 90 + angle
            angle = abs(angle)
            
            # Verificar alinhamento com os eixos (máximo 10 graus de inclinação)
            if angle > self.max_angle_deviation and angle < (90 - self.max_angle_deviation):
                return False, None, None
            
            # Obter os pontos do retângulo
            box = cv2.boxPoints(rect)
            box = np.array(box).astype("int")
            box = perspective.order_points(box)
            
            # Calcular ângulos internos para verificar retangularidade
            angles = []
            for i in range(4):
                p1 = box[i]
                p2 = box[(i + 1) % 4]
                p3 = box[(i + 2) % 4]
                
                # Vetores
                v1 = p2 - p1
                v2 = p2 - p3
                
                # Calcular ângulo
                dot = np.dot(v1, v2)
                norm = np.linalg.norm(v1) * np.linalg.norm(v2)
                if norm == 0:
                    return False, None, None
                
                cos_angle = max(-1.0, min(1.0, dot / norm))
                angle_deg = math.degrees(math.acos(cos_angle))
                angles.append(angle_deg)
            
            # Verificar se os ângulos são próximos de 90 graus
            angle_score = sum(min(abs(a - 90), 180 - abs(a - 90)) for a in angles) / 4.0
            
            # Calcular relação de aspecto
            width = euclidean(box[0], box[1])
            height = euclidean(box[1], box[2])
            if width == 0 or height == 0:
                return False, None, None
            
            # Score do retângulo
            rectangle_score = 1.0 - min(angle_score / 90.0, 1.0)
            
            return rectangle_score >= self.min_rectangle_score, box, angle
            
        except Exception as e:
            return False, None, None
    
    def find_aligned_rectangles(self, edged):
        """Encontra retângulos alinhados com os eixos da câmera"""
        # Encontrar contornos
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        if len(cnts) == 0:
            return [], [], []
        
        # Ordenar contornos por área (do maior para o menor)
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        
        # Filtrar contornos retangulares alinhados
        aligned_rectangles = []
        aligned_boxes = []
        aligned_angles = []
        
        for cnt in cnts:
            # Verificar área mínima
            if cv2.contourArea(cnt) < self.min_contour_area:
                continue
            
            # Verificar se é retângulo alinhado
            is_aligned, box, angle = self.is_aligned_rectangle(cnt)
            if is_aligned:
                aligned_rectangles.append(cnt)
                aligned_boxes.append(box)
                aligned_angles.append(angle)
        
        # Ordenar da esquerda para a direita (para manter a referência no início)
        if aligned_boxes:
            # Calcular centro X de cada caixa
            centroids = [np.mean(box[:, 0]) for box in aligned_boxes]
            sorted_indices = np.argsort(centroids)
            
            aligned_rectangles = [aligned_rectangles[i] for i in sorted_indices]
            aligned_boxes = [aligned_boxes[i] for i in sorted_indices]
            aligned_angles = [aligned_angles[i] for i in sorted_indices]
        
        return aligned_rectangles, aligned_boxes, aligned_angles
    
    def preprocess_frame(self, frame):
        """Pré-processa o frame para detecção de bordas retangulares"""
        # Redimensionar se necessário para processamento mais rápido
        height, width = frame.shape[:2]
        if width > 1280:
            scale = 1280 / width
            new_width = 1280
            new_height = int(height * scale)
            frame = cv2.resize(frame, (new_width, new_height))
        
        # Converter para escala de cinza
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Aplicar blur para reduzir ruído
        blur = cv2.GaussianBlur(gray, self.gaussian_blur, 0)
        
        # Detecção de bordas com thresholds adaptativos
        v = np.median(blur)
        sigma = 0.33
        
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        
        edged = cv2.Canny(blur, lower, upper)
        
        # Operações morfológicas para melhorar detecção de retângulos
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edged = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
        edged = cv2.dilate(edged, kernel, iterations=1)
        
        return frame, gray, blur, edged
    
    def calibrate_with_reference(self, ref_contour):
        """Calibra o sistema usando o objeto de referência"""
        try:
            # Obter retângulo do objeto de referência
            rect = cv2.minAreaRect(ref_contour)
            box = cv2.boxPoints(rect)
            box = np.array(box, dtype="int")
            box = perspective.order_points(box)
            
            # Extrair pontos
            (tl, tr, br, bl) = box

            
            # Calcular distâncias em pixels
            width_pixels = euclidean(tl, tr)
            height_pixels = euclidean(tr, br)
            
            # Usar a menor dimensão para evitar problemas de perspectiva
            min_dimension = min(width_pixels, height_pixels)
            
            # Calcular pixels por cm
            self.pixels_per_cm = min_dimension / self.reference_width_cm - 28
        
            
            self.calibrated = True
            print(f"✓ Sistema calibrado: {self.pixels_per_cm:.2f} pixels/cm")
            
            # Salvar calibração em arquivo
            with open("calibracao.txt", "w") as f:
                f.write(f"PIXELS_PER_CM = {self.pixels_per_cm:.2f}\n")
                f.write(f"DATA_CALIBRACAO = {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            
            return box
        except Exception as e:
            print(f"✗ Erro na calibração: {e}")
            return None
    
    def measure_box(self, box):
        """Mede as dimensões de uma caixa em cm"""
        try:
            # Extrair pontos
            (tl, tr, br, bl) = box
            
            # Calcular dimensões em pixels
            width_pixels = euclidean(tl, tr)
            height_pixels = euclidean(tr, br)
            
            # Determinar largura e comprimento
            width_px = min(width_pixels, height_pixels)
            height_px = max(width_pixels, height_pixels)
            
            # Converter para cm se calibrado
            if self.calibrated and self.pixels_per_cm > 0:
                width_cm = width_px / self.pixels_per_cm
                height_cm = height_px / self.pixels_per_cm
                return width_cm, height_cm
            else:
                return None, None
                
        except Exception as e:
            return None, None
    
    def classify_box(self, width_cm, height_cm):
        """Classifica a caixa baseado nas dimensões"""
        if width_cm is None or height_cm is None:
            return "UNKNOWN", False
            
        for box_type, (ref_width, ref_height) in self.box_types.items():
            # Verificar se as medidas estão dentro da tolerância
            width_match = abs(width_cm - ref_width) <= self.tolerance_cm
            height_match = abs(height_cm - ref_height) <= self.tolerance_cm
            
            # Verificar também a orientação invertida
            width_match_swapped = abs(width_cm - ref_height) <= self.tolerance_cm
            height_match_swapped = abs(height_cm - ref_width) <= self.tolerance_cm
            
            if (width_match and height_match) or (width_match_swapped and height_match_swapped):
                return box_type, True
        
        return "UNKNOWN", False
    
    def update_output_map(self):
        """Atualiza o mapa de bits de saída"""
        # 1. Status da câmera (sempre operante se estiver rodando)
        camera_operational = self.camera is not None and self.camera.isOpened()
        self.ARD_output.update_camera_status(camera_operational)
        
        # 2. Verificar erros
        has_error = any(self.system_errors.values())
        self.ARD_output.update_error_status(has_error)
        
        # 3. Verificar se há caixa detectada
        box_detected = len(self.box_type_history) > 0 and self.last_stable_type is not None
        self.ARD_output.update_box_detected(box_detected)
        
        # 4. Atualizar tipo da caixa
        if box_detected and self.last_stable_type in self.box_types:
            self.ARD_output.update_box_type(self.last_stable_type)
        else:
            self.ARD_output.update_box_type("UNKNOWN")
        
        # 5. Enviar para ARD
        return self.ARD_output.send_to_ARD()
    
    def check_system_errors(self):
        """Verifica e atualiza status de erros do sistema"""
        # Erro de câmera
        self.system_errors['camera_error'] = not (self.camera and self.camera.isOpened())
        
        # Erro de timeout
        if not self.ARD_output.check_operational_timeout():
            self.system_errors['timeout_error'] = True
        else:
            self.system_errors['timeout_error'] = False
        
        # Erro de processamento (se muitos frames sem detecção válida)
        if self.frames_processed > 100 and self.valid_boxes == 0:
            self.system_errors['processing_error'] = True
        else:
            self.system_errors['processing_error'] = False
        
        # Retorna se há algum erro
        return any(self.system_errors.values())
    
    def draw_measurements(self, frame, box, width_cm, height_cm, box_type, is_valid, angle=None, is_reference=False):
        """Desenha medições e informações no frame"""
        # Desenhar contorno
        if is_reference:
            color = (0, 255, 255)  # Amarelo para referência
            thickness = 3
        elif is_valid:
            color = (0, 255, 0)  # Verde para válido
            thickness = 3
        else:
            color = (0, 0, 255)  # Vermelho para inválido
            thickness = 2
        
        cv2.drawContours(frame, [box.astype("int")], -1, color, thickness)
        
        # Desenhar vértices
        for point in box:
            cv2.circle(frame, tuple(map(int, point)), 6, (255, 0, 0), -1)
        
        # Calcular centro para texto
        center_x = int(np.mean(box[:, 0]))
        center_y = int(np.mean(box[:, 1]))
        
        if is_reference:
            # Objeto de referência
            label = f"REF: {self.reference_width_cm}x{self.reference_height_cm}cm"
            cv2.putText(frame, label, (center_x - 60, center_y - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            # Caixas medidas
            if angle is not None:
                angle_text = f"Angulo: {angle:.1f}"
                cv2.putText(frame, angle_text, (center_x - 40, center_y - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
            
            label = f"{box_type}" if is_valid else "REJEITADA"
            color_label = (0, 255, 0) if is_valid else (0, 0, 255)
            
            cv2.putText(frame, label, (center_x - 40, center_y - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_label, 2)
            
            # Medidas
            if width_cm is not None and height_cm is not None:
                measures_text = f"{width_cm:.1f}x{height_cm:.1f}cm"
                cv2.putText(frame, measures_text, (center_x - 50, center_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        return frame
    
    def draw_output_status(self, frame, output_info):
        """Desenha informações da saída ARD no frame"""
        # Posição para desenhar (canto superior direito)
        x_start = frame.shape[1] - 250
        y_start = 120
        
        # Fundo semi-transparente
        overlay = frame.copy()
        cv2.rectangle(overlay, (x_start-10, y_start-10), 
                     (frame.shape[1]-10, y_start+100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Título
        cv2.putText(frame, "SAIDA Arduino:", (x_start, y_start),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Informações
        y_offset = y_start + 25
        lines = [
            f"Binario: {output_info['binary']}",
            f"Camera: {'ON' if output_info['camera'] == '0' else 'ERR'}",
            f"Erro: {'SIM' if output_info['error'] == '0' else 'NAO'}",
            f"Detecao: {'SIM' if output_info['detection'] == '1' else 'NAO'}",
            f"Tipo: {output_info['type']}"
        ]
        
        for line in lines:
            cv2.putText(frame, line, (x_start, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            y_offset += 15
    
    def process_frame(self, frame):
        """Processa um frame completo"""
        self.frames_processed += 1
        
        # Verificar erros do sistema
        self.check_system_errors()
        
        # Pré-processamento
        frame_resized, gray, blur, edged = self.preprocess_frame(frame)
        
        # Encontrar retângulos alinhados
        cnts, boxes, angles = self.find_aligned_rectangles(edged) if len(self.find_aligned_rectangles(edged)) == 3  else ([], [], [])
        
        # Criar cópia para desenhar resultados
        result_frame = frame_resized.copy()
        
        # Processar objeto de referência (primeiro retângulo)
        reference_box = None
        if len(cnts) > 0:
            ref_contour = cnts[0]
            is_aligned, ref_box, ref_angle = self.is_aligned_rectangle(ref_contour)
            
            if is_aligned and ref_box is not None:
                # Calibrar se necessário
                if not self.calibrated:
                    reference_box = self.calibrate_with_reference(ref_contour)
                    if reference_box is not None:
                        ref_box = reference_box
                
                # Desenhar objeto de referência
                result_frame = self.draw_measurements(
                    result_frame, ref_box, None, None, 
                    "REF", True, ref_angle, is_reference=True
                )
        
        # Processar demais retângulos (caixas)
        valid_boxes_count = 0
        current_detections = []
        
        for i in range(1, len(cnts)):  # Pular o primeiro (referência)
            if not self.calibrated:
                continue
            
            box = boxes[i]
            angle = angles[i] if i < len(angles) else None
            
            # Medir caixa
            width_cm, height_cm = self.measure_box(box)
            
            if width_cm is None or height_cm is None:
                continue
            
            # Classificar caixa
            box_type, is_valid = self.classify_box(width_cm, height_cm)
            
            if is_valid:
                current_detections.append(box_type)
                self.box_type_history.append(box_type)
            
            # Desenhar medições
            result_frame = self.draw_measurements(
                result_frame, box, width_cm, height_cm, box_type, is_valid, angle
            )
            
            if is_valid:
                valid_boxes_count += 1
        
        # Atualizar estatísticas
        self.total_boxes = len(cnts) - 1 if len(cnts) > 1 else 0
        self.valid_boxes = valid_boxes_count
        
        # Determinar tipo estável
        if current_detections:
            from collections import Counter
            type_counter = Counter(current_detections)
            most_common = type_counter.most_common(1)
            if most_common:
                current_type = most_common[0][0]
                
                # Verificar estabilidade
                if current_type == self.last_stable_type:
                    self.stable_detection_count += 1
                else:
                    self.stable_detection_count = 1
                    self.last_stable_type = current_type
                
                # Considerar estável após 3 detecções consecutivas
                if self.stable_detection_count >= 3:
                    self.last_stable_type = current_type
        else:
            self.stable_detection_count = 0
            self.last_stable_type = None
        
        # Atualizar e enviar mapa de bits
        output_info = self.update_output_map()
        
        # Adicionar informações de saída no frame
        self.draw_output_status(result_frame, output_info)
        
        # Desenhar status
        self.draw_status(result_frame, cnts, valid_boxes_count)
        
        # Mostrar janela
        if self.show_debug:
            debug_frame = self.create_debug_view(gray, blur, edged, result_frame)
            cv2.imshow("Debug View", debug_frame)
        
        cv2.imshow("Sistema de Visão - Caixas Alinhadas", result_frame)
        
        return result_frame
    
    def create_debug_view(self, gray, blur, edged, result):
        """Cria uma visualização de debug com múltiplas imagens"""
        # Redimensionar imagens para terem o mesmo tamanho
        h, w = result.shape[:2]
        small_h, small_w = h // 2, w // 2
        
        gray_resized = cv2.resize(gray, (small_w, small_h))
        blur_resized = cv2.resize(blur, (small_w, small_h))
        edged_resized = cv2.resize(edged, (small_w, small_h))
        result_resized = cv2.resize(result, (small_w, small_h))
        
        # Converter para BGR para exibição
        gray_bgr = cv2.cvtColor(gray_resized, cv2.COLOR_GRAY2BGR)
        blur_bgr = cv2.cvtColor(blur_resized, cv2.COLOR_GRAY2BGR)
        edged_bgr = cv2.cvtColor(edged_resized, cv2.COLOR_GRAY2BGR)
        
        # Criar grades
        top_row = np.hstack([gray_bgr, blur_bgr])
        bottom_row = np.hstack([edged_bgr, result_resized])
        debug_frame = np.vstack([top_row, bottom_row])
        
        # Adicionar rótulos
        labels = ["Escala de Cinza", "Blur", "Bordas", "Resultado"]
        positions = [(10, 30), (small_w + 10, 30), 
                    (10, small_h + 30), (small_w + 10, small_h + 30)]
        
        for label, pos in zip(labels, positions):
            cv2.putText(debug_frame, label, pos,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return debug_frame
    
    def draw_status(self, frame, cnts, valid_count):
        """Desenha informações de status no frame"""
        # Fundo para texto
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 90), (0, 0, 0), -1)
        
        # Status do sistema
        status_cal = "CALIBRADO" if self.calibrated else "AGUARDANDO CALIBRAÇÃO"
        color_cal = (0, 255, 0) if self.calibrated else (0, 165, 255)
        
        cv2.putText(frame, f"Status: {status_cal}", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_cal, 2)
        
        if self.calibrated:
            cv2.putText(frame, f"Escala: {self.pixels_per_cm:.1f} px/cm", (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Contadores
        total_rect = len(cnts)
        cv2.putText(frame, f"Retangulos: {total_rect} | Validos: {valid_count}", 
                   (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Instruções
        cv2.putText(frame, "Q: Sair | D: Debug | C: Calibrar | S: Salvar | Espaco: Pausar",
                   (10, frame.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def send_stable_detections(self):
        """Envia detecções estáveis para o ARD"""
        if not self.ARD_connected:
            return
        
        # Verificar se temos uma detecção estável
        if self.last_stable_type and self.stable_detection_count >= 5:
            print(f"[ARD] Enviando tipo estável: {self.last_stable_type}")
            
            # Resetar contador após envio
            self.stable_detection_count = 0
    
    def simulate_ARD_output(self):
        """Simula diferentes estados para testar a saída ARD"""
        print("\n=== SIMULAÇÃO DE SAÍDA ARD ===")
        
        # Teste 1: Sistema operacional sem detecção
        self.ARD_output.update_camera_status(True)
        self.ARD_output.update_error_status(False)
        self.ARD_output.update_box_detected(False)
        self.ARD_output.update_box_type("UNKNOWN")
        print("Estado 1 - Operacional sem detecção:")
        self.ARD_output.send_to_ARD()
        
        # Teste 2: Sistema com erro
        self.ARD_output.update_camera_status(True)
        self.ARD_output.update_error_status(True)
        self.ARD_output.update_box_detected(False)
        self.ARD_output.update_box_type("UNKNOWN")
        print("\nEstado 2 - Sistema com erro:")
        self.ARD_output.send_to_ARD()
        
        # Teste 3: Detecção de caixa 10x20
        self.ARD_output.update_camera_status(True)
        self.ARD_output.update_error_status(False)
        self.ARD_output.update_box_detected(True)
        self.ARD_output.update_box_type("10x20")
        print("\nEstado 3 - Caixa 10x20 detectada:")
        self.ARD_output.send_to_ARD()
        
        # Teste 4: Detecção de caixa 20x20
        self.ARD_output.update_camera_status(True)
        self.ARD_output.update_error_status(False)
        self.ARD_output.update_box_detected(True)
        self.ARD_output.update_box_type("20x20")
        print("\nEstado 4 - Caixa 20x20 detectada:")
        self.ARD_output.send_to_ARD()
        
        # Teste 5: Detecção de caixa 30x50
        self.ARD_output.update_camera_status(True)
        self.ARD_output.update_error_status(False)
        self.ARD_output.update_box_detected(True)
        self.ARD_output.update_box_type("30x50")
        print("\nEstado 5 - Caixa 30x50 detectada:")
        self.ARD_output.send_to_ARD()
    
    def run(self):
        """Executa o sistema principal"""
        print("=" * 60)
        print("SISTEMA DE VISÃO COMPUTACIONAL PARA CAIXAS ALINHADAS")
        print("=" * 60)
        print("SAIDA VIA MAPA DE BITS (8 bits):")
        print("Bit 0: Câmera operante (1=sim, 0=não)")
        print("Bit 1: Erro no sistema (1=sim, 0=não)")
        print("Bit 2: Caixa detectada (1=sim, 0=não)")
        print("Bits 3-4: Tipo (00=nada, 01=10x20, 10=20x20, 11=30x50)")
        print("=" * 60)
        
        # Verificar se há uma calibração salva
        try:
            with open("calibracao.txt", "r") as f:
                for line in f:
                    if "PIXELS_PER_CM" in line:
                        value = float(line.split("=")[1].strip())
                        self.pixels_per_cm = value
                        self.calibrated = True
                        print(f"Calibração carregada: {self.pixels_per_cm:.2f} px/cm")
        except:
            print("Nenhuma calibração anterior encontrada")
        
        print("\nIniciando processamento...")
        print("Controles:")
        print("  Q: Sair do sistema")
        print("  D: Alternar modo debug")
        print("  C: Forçar recalibração")
        print("  S: Salvar frame atual")
        print("  Espaço: Pausar/continuar")
        print("  +: Aumentar tolerância de ângulo")
        print("  -: Diminuir tolerância de ângulo")
        
        # Criar janela sempre visível
        cv2.namedWindow("Sistema de Visão - Caixas Alinhadas", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Sistema de Visão - Caixas Alinhadas", 1024, 768)
        
        if self.show_debug:
            cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Debug View", 1024, 768)
        
        paused = False
        last_time = time.time()
        frame_count = 0
        ARD_update_interval = 0.5  # Atualizar ARD a cada 0.5 segundos
        last_ARD_update = time.time()
        
        try:
            while self.processing:
                if not paused:
                    # Capturar frame
                    ret, frame = self.camera.read()
                    
                    # Se for vídeo e chegou ao fim, voltar ao início
                    if not ret and self.is_video_file:
                        self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, frame = self.camera.read()
                    
                    if not ret:
                        print("Fim do vídeo ou erro na captura")
                        break
                    
                    # Processar frame
                    processed_frame = self.process_frame(frame)
                    frame_count += 1
                    
                    # Atualizar ARD em intervalos regulares
                    current_time = time.time()
                    if current_time - last_ARD_update >= ARD_update_interval:
                        self.update_output_map()
                        last_ARD_update = current_time
                    
                    # Calcular FPS
                    current_time = time.time()
                    if current_time - last_time >= 1.0:
                        fps = frame_count / (current_time - last_time)
                        print(f"FPS: {fps:.1f} | Frames processados: {self.frames_processed}")
                        frame_count = 0
                        last_time = current_time
                
                # Controles de teclado
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    paused = not paused
                    print(f"Processamento {'PAUSADO' if paused else 'CONTINUADO'}")
                elif key == ord('d'):
                    self.show_debug = not self.show_debug
                    if self.show_debug:
                        cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)
                        cv2.resizeWindow("Debug View", 1024, 768)
                    else:
                        cv2.destroyWindow("Debug View")
                    print(f"Modo debug: {'ATIVADO' if self.show_debug else 'DESATIVADO'}")
                elif key == ord('c'):
                    self.calibrated = False
                    print("Calibração resetada")
                elif key == ord('s'):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"captura_{timestamp}.jpg"
                    cv2.imwrite(filename, processed_frame if 'processed_frame' in locals() else frame)
                    print(f"Frame salvo como: {filename}")
                elif key == ord('+'):
                    self.max_angle_deviation = min(45, self.max_angle_deviation + 1)
                    print(f"Tolerância de ângulo aumentada para: {self.max_angle_deviation}°")
                elif key == ord('-'):
                    self.max_angle_deviation = max(1, self.max_angle_deviation - 1)
                    print(f"Tolerância de ângulo diminuída para: {self.max_angle_deviation}°")
                elif key == ord('r'):
                    # Recarregar vídeo
                    if self.is_video_file:
                        self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        print("Vídeo reiniciado")
        
        except KeyboardInterrupt:
            print("\nSistema interrompido pelo usuário")
        
        finally:
            # Liberar recursos
            if self.camera is not None:
                self.camera.release()
            cv2.destroyAllWindows()
            
            # Exibir estatísticas finais
            self.show_final_statistics()
    
    def show_final_statistics(self):
        """Exibe estatísticas finais do processamento"""
        print("\n" + "=" * 60)
        print("ESTATÍSTICAS FINAIS")
        print("=" * 60)
        print(f"Total de frames processados: {self.frames_processed}")
        print(f"Total de retângulos detectados: {self.total_boxes}")
        print(f"Caixas válidas classificadas: {self.valid_boxes}")
        
        if self.total_boxes > 0:
            taxa = (self.valid_boxes / self.total_boxes) * 100
            print(f"Taxa de aceitação: {taxa:.1f}%")
        
        print(f"Sistema calibrado: {self.calibrated}")
        if self.calibrated:
            print(f"Pixels por cm: {self.pixels_per_cm:.2f}")
        
        print(f"Tolerância de ângulo final: {self.max_angle_deviation}°")
        
        # Mostrar mapa de bits final
        final_output = self.ARD_output.get_output_string()
        print(f"\nMapa de bits final:")
        print(f"  Binário: {final_output['binary']}")
        print(f"  Câmera: {'ON' if final_output['camera'] == '1' else 'ERR'}")
        print(f"  Erro: {'SIM' if final_output['error'] == '1' else 'NÃO'}")
        print(f"  Detecção: {'SIM' if final_output['detection'] == '1' else 'NÃO'}")
        print(f"  Tipo: {final_output['type']}")
        print("=" * 60)

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Sistema de Visão Computacional para Caixas Alinhadas',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemplos de uso:
  python sistema_caixas.py                    # Usa câmera padrão
  python sistema_caixas.py --camera 1         # Usa câmera índice 1
  python sistema_caixas.py --video video.mp4  # Usa arquivo de vídeo
  python sistema_caixas.py --image foto.jpg   # Processa imagem única
  python sistema_caixas.py --reference 5.0    # Objeto de referência 5x5 cm
  python sistema_caixas.py --test-ARD         # Testa saída ARD
        """
    )
    
    parser.add_argument('--camera', type=str, default='0', 
                       help='Índice da câmera (padrão: 0)')
    parser.add_argument('--video', type=str, 
                       help='Caminho para arquivo de vídeo')
    parser.add_argument('--image', type=str, 
                       help='Caminho para imagem única')
    parser.add_argument('--reference', type=float, default=2.0,
                       help='Tamanho do objeto de referência em cm (padrão: 2.0)')
    parser.add_argument('--debug', action='store_true',
                       help='Iniciar em modo debug')
    parser.add_argument('--test-ARD', action='store_true',
                       help='Testar saída ARD')
    
    args = parser.parse_args()
    
    system = BoxMeasurementSystem()
    system.reference_width_cm = args.reference
    system.reference_height_cm = args.reference
    system.show_debug = args.debug
    
    # Testar saída ARD
    if args.test_ARD:
        system.simulate_ARD_output()
        return
    
    # Determinar fonte de vídeo
    source = None
    
    if args.image:
        # Processar imagem única
        print(f"Processando imagem: {args.image}")
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Erro ao carregar imagem: {args.image}")
            return
        
        # Processar uma vez
        result = system.process_frame(frame)
        cv2.imshow("Resultado", result)
        
        print("\nPressione qualquer tecla para sair...")
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"captura_{timestamp}.jpg"
        cv2.imwrite(filename, result)
        print(f"Frame salvo como: {filename}")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        
    elif args.video:
        # Usar arquivo de vídeo
        print(f"Usando vídeo: {args.video}")
        source = args.video
    else:
        # Usar câmera
        print(f"Usando câmera: {args.camera}")
        source = args.camera
    
    if not args.image:
        # Inicializar e executar sistema
        if system.initialize_video_source(source):
            system.run()

if __name__ == "__main__":
    # Instalar dependências necessárias se não estiverem instaladas
    try:
        import scipy
        import imutils
    except ImportError:
        print("Instalando dependências necessárias...")
        import subprocess
        import sys
        
        dependencies = ["opencv-python", "numpy", "scipy", "imutils"]
        for dep in dependencies:
            subprocess.check_call([sys.executable, "-m", "pip", "install", dep])
        
        print("Dependências instaladas. Execute o programa novamente.")
        sys.exit(0)
    
    main()