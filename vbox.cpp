/*
 * Código Arduino para receber comandos do sistema de visão computacional
 * 
 * Pinos utilizados:
 * D2: Câmera operante (HIGH = operante, LOW = não operante)
 * D3: Erro no sistema (HIGH = erro, LOW = sem erro)
 * D4: Caixa detectada (HIGH = detectada, LOW = não detectada)
 * D5, D6, D7: Código do tipo de caixa (3 bits)
 * 
 * Formato do comando serial: C:E:D:T0:T1:T2\n
 * Onde:
 *   C = Status da câmera (0 ou 1)
 *   E = Status de erro (0 ou 1)
 *   D = Detecção de caixa (0 ou 1)
 *   T0 = Bit 0 do tipo (LSB)
 *   T1 = Bit 1 do tipo
 *   T2 = Bit 2 do tipo (MSB)
 * 
 * Exemplo: "1:0:1:0:0:1\n" = Câmera OK, sem erro, caixa detectada, tipo 100 (30x50)
 */

// Definição dos pinos
const int PIN_CAMERA = 2;
const int PIN_ERROR = 3;
const int PIN_DETECTION = 4;
const int PIN_TYPE_BIT0 = 5;  // LSB
const int PIN_TYPE_BIT1 = 6;
const int PIN_TYPE_BIT2 = 7;  // MSB

// Buffer para receber dados
char buffer[32];
int bufferIndex = 0;

void setup() {
  // Inicializar comunicação serial
  Serial.begin(9600);
  while (!Serial) {
    ; // Aguardar conexão serial (apenas para Leonardo/Micro)
  }
  
  // Configurar pinos como saída
  pinMode(PIN_CAMERA, OUTPUT);
  pinMode(PIN_ERROR, OUTPUT);
  pinMode(PIN_DETECTION, OUTPUT);
  pinMode(PIN_TYPE_BIT0, OUTPUT);
  pinMode(PIN_TYPE_BIT1, OUTPUT);
  pinMode(PIN_TYPE_BIT2, OUTPUT);
  
  // Estado inicial: todos LOW
  digitalWrite(PIN_CAMERA, LOW);
  digitalWrite(PIN_ERROR, LOW);
  digitalWrite(PIN_DETECTION, LOW);
  digitalWrite(PIN_TYPE_BIT0, LOW);
  digitalWrite(PIN_TYPE_BIT1, LOW);
  digitalWrite(PIN_TYPE_BIT2, LOW);
  
  // LED interno para indicação
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println(" Arduino pronto para receber comandos");
  Serial.println(" Aguardando dados no formato: C:E:D:T0:T1:T2");
}

void loop() {
  // Verificar se há dados disponíveis na serial
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    // Se for nova linha, processar o comando
    if (incomingChar == '\n') {
      buffer[bufferIndex] = '\0'; // Terminar string
      processCommand(buffer);
      bufferIndex = 0; // Resetar buffer
      
      // Piscar LED para indicar recebimento
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
    } 
    // Se não for nova linha, adicionar ao buffer
    else if (bufferIndex < 31) {
      buffer[bufferIndex] = incomingChar;
      bufferIndex++;
    }
  }
}

void processCommand(char* command) {
  // Variáveis para armazenar os valores
  int camera, error, detection, type0, type1, type2;
  
  // Parsing do comando no formato "C:E:D:T0:T1:T2"
  if (sscanf(command, "%d:%d:%d:%d:%d:%d", 
             &camera, &error, &detection, &type0, &type1, &type2) == 6) {
    
    // Validar valores (devem ser 0 ou 1)
    camera = constrain(camera, 0, 1);
    error = constrain(error, 0, 1);
    detection = constrain(detection, 0, 1);
    type0 = constrain(type0, 0, 1);
    type1 = constrain(type1, 0, 1);
    type2 = constrain(type2, 0, 1);
    
    // Atualizar pinos
    digitalWrite(PIN_CAMERA, camera ? HIGH : LOW);
    digitalWrite(PIN_ERROR, error ? HIGH : LOW);
    digitalWrite(PIN_DETECTION, detection ? HIGH : LOW);
    digitalWrite(PIN_TYPE_BIT0, type0 ? HIGH : LOW);
    digitalWrite(PIN_TYPE_BIT1, type1 ? HIGH : LOW);
    digitalWrite(PIN_TYPE_BIT2, type2 ? HIGH : LOW);
    
    // Log no monitor serial (opcional)
    Serial.print(" Estado: C=");
    Serial.print(camera);
    Serial.print(" E=");
    Serial.print(error);
    Serial.print(" D=");
    Serial.print(detection);
    Serial.print(" T=");
    Serial.print(type2);
    Serial.print(type1);
    Serial.print(type0);
    
    // Identificar tipo de caixa
    int typeCode = (type2 << 2) | (type1 << 1) | type0;
    Serial.print(" (");
    
    switch(typeCode) {
      case 0b001:
        Serial.print("10x20");
        break;
      case 0b010:
        Serial.print("20x20");
        break;
      case 0b100:
        Serial.print("30x50");
        break;
      case 0b000:
        Serial.print("Nenhuma");
        break;
      case 0b111:
        Serial.print("Desconhecido");
        break;
      default:
        Serial.print("Código inválido");
        break;
    }
    
    Serial.println(")");
  } else {
    // Comando inválido
    Serial.print(" Comando inválido: ");
    Serial.println(command);
    
    // Piscar LED de erro
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
}