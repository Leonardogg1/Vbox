from collections import deque
import time

class PLCOutputMap:
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
        
        # Formato binário e decimal para debug
        binary_str = format(self.output_byte, '08b')
        decimal_val = self.output_byte
        
        return {
            'binary': binary_str,
            'decimal': decimal_val,
            'camera': camera_status,
            'error': error_status,
            'detection': detection_status,
            'type': box_type_str
        }
    
    def send_to_plc(self):
        """Simula envio do mapa de bits para PLC"""
        output_info = self.get_output_string()
        
        # Simulação: apenas imprime o estado
        print(f"[PLC OUT] Bin: {output_info['binary']} | Dec: {output_info['decimal']} | "
              f"Camera: {output_info['camera']} | Erro: {output_info['error']} | "
              f"Detecao: {output_info['detection']} | Tipo: {output_info['type']}")
        
        # Adicionar ao histórico
        self.output_history.append(self.output_byte)
        
        # Atualizar timestamp
        self.last_update_time = time.time()
        
        return output_info