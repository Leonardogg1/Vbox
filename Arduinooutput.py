"""
Sistema de Saída via Pinos Digitais para Arduino
"""
import serial
import time

class ArduinoOutput:
    def __init__(self, port='COM3', baudrate=9600, simulate=True):
        """
        Inicializa comunicação com Arduino
        
        Pinos utilizados:
        D2: Câmera operante (HIGH = operante, LOW = não operante)
        D3: Erro no sistema (HIGH = erro, LOW = sem erro)
        D4: Caixa detectada (HIGH = detectada, LOW = não detectada)
        D5, D6, D7: Código do tipo de caixa (3 bits)
            D5: Bit 0 (LSB) - 10x20
            D6: Bit 1        - 20x20  
            D7: Bit 2 (MSB)  - 30x50
        
        Exemplo:
            Caixa 10x20: D5=HIGH, D6=LOW, D7=LOW (001)
            Caixa 20x20: D5=LOW, D6=HIGH, D7=LOW (010)
            Caixa 30x50: D5=LOW, D6=LOW, D7=HIGH (100)
            Sem caixa:   D5=LOW, D6=LOW, D7=LOW (000)
        """
        self.simulate = simulate
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.last_output = None
        
        # Mapeamento de tipos para pinos
        self.pin_mapping = {
            'camera': 2,    # D2 - Status da câmera
            'error': 3,     # D3 - Status de erro
            'detection': 4, # D4 - Caixa detectada
            'type_bit0': 5, # D5 - Bit 0 do tipo (LSB)
            'type_bit1': 6, # D6 - Bit 1 do tipo
            'type_bit2': 7  # D7 - Bit 2 do tipo (MSB)
        }
        
        # Mapeamento de tipos para código binário
        self.type_codes = {
            "10x20": 0b001,  # D5=HIGH, D6=LOW, D7=LOW
            "20x20": 0b010,  # D5=LOW, D6=HIGH, D7=LOW
            "30x50": 0b100,  # D5=LOW, D6=LOW, D7=HIGH
            "NONE": 0b000,   # D5=LOW, D6=LOW, D7=LOW
            "UNKNOWN": 0b111  # Todos HIGH (erro/desconhecido)
        }
        
        self.last_update_time = time.time()
        
        if not self.simulate:
            self._connect_serial()
    
    def _connect_serial(self):
        """Conecta ao Arduino via serial"""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Aguardar Arduino reiniciar
            print(f"✅ Conectado ao Arduino na porta {self.port}")
            return True
        except Exception as e:
            print(f"❌ Erro ao conectar ao Arduino: {e}")
            print("⚠️  Modo simulação ativado")
            self.simulate = True
            return False
    
    def _send_to_arduino(self, command):
        """Envia comando para o Arduino"""
        if not self.simulate and self.serial_connection:
            try:
                self.serial_connection.write(command.encode())
                return True
            except Exception as e:
                print(f"❌ Erro ao enviar para Arduino: {e}")
                return False
        return False
    
    def update_status(self, camera_status, error_status, detection_status, box_type):
        """Atualiza todos os status e envia para Arduino"""
        # Preparar string de comando
        # Formato: C:E:D:T0:T1:T2\n
        # Onde: C=câmera, E=erro, D=detecção, T0-T2=bits do tipo
        
        # Converter tipo para código binário
        type_code = self.type_codes.get(box_type, 0b111)
        
        # Extrair bits individuais
        type_bit0 = (type_code >> 0) & 1  # Bit 0 (LSB) - D5
        type_bit1 = (type_code >> 1) & 1  # Bit 1 - D6
        type_bit2 = (type_code >> 2) & 1  # Bit 2 (MSB) - D7
        
        # Criar comando
        command = f"{int(camera_status)}:{int(error_status)}:{int(detection_status)}:{type_bit0}:{type_bit1}:{type_bit2}\n"
        
        # Enviar para Arduino
        success = self._send_to_arduino(command)
        
        # Salvar último estado
        self.last_output = {
            'camera': camera_status,
            'error': error_status,
            'detection': detection_status,
            'type': box_type,
            'type_code': type_code,
            'command': command.strip()
        }
        
        # Atualizar timestamp
        self.last_update_time = time.time()
        
        # Log
        self._log_status()
        
        return success
    
    def _log_status(self):
        """Registra status no console"""
        if self.last_output:
            output = self.last_output
            print(f"[ARDUINO] Câmera:{output['camera']} | Erro:{output['error']} | "
                  f"Detecção:{output['detection']} | Tipo:{output['type']} | "
                  f"Código:{output['type_code']:03b} | Comando:{output['command']}")
    
    def get_pin_states(self):
        """Retorna estado atual de todos os pinos"""
        if not self.last_output:
            return None
        
        output = self.last_output
        type_code = output['type_code']
        
        return {
            'D2': output['camera'],      # Câmera operante
            'D3': output['error'],       # Erro no sistema
            'D4': output['detection'],   # Caixa detectada
            'D5': (type_code >> 0) & 1,  # Bit 0 do tipo
            'D6': (type_code >> 1) & 1,  # Bit 1 do tipo
            'D7': (type_code >> 2) & 1   # Bit 2 do tipo
        }
    
    def send_stable_detection(self, box_type):
        """Envia detecção estável para Arduino"""
        if box_type in self.type_codes:
            return self.update_status(
                camera_status=True,
                error_status=False,
                detection_status=True,
                box_type=box_type
            )
        return False
    
    def send_error_state(self, error_message="Erro desconhecido"):
        """Envia estado de erro para Arduino"""
        print(f"⚠️  Enviando estado de erro para Arduino: {error_message}")
        return self.update_status(
            camera_status=False,
            error_status=True,
            detection_status=False,
            box_type="UNKNOWN"
        )
    
    def send_ready_state(self):
        """Envia estado pronto (sem erro, sem detecção)"""
        return self.update_status(
            camera_status=True,
            error_status=False,
            detection_status=False,
            box_type="NONE"
        )
    
    def close(self):
        """Fecha conexão serial"""
        if self.serial_connection:
            self.serial_connection.close()
            print("🔌 Conexão serial fechada")