# Controle Customizado para F1

## Jogo
**F1 23** - F1 23 é um jogo de simulação de corrida onde o jogador assume o controle de um carro de Fórmula 1, competindo em circuitos oficiais da temporada. O jogo conta com modos de carreira, desafios online e uma física aprimorada que influencia o desempenho na pista.

## Ideia do Controle
O controle será um conjunto de volante e pedais desenvolvido para facilitar a jogabilidade de F1. O volante terá um design ergonômico similar ao dos volantes de corrida, e contará com dois botões de câmbio (gear up e gear down) na parte traseira, além de um botão de overtake e outro para acionar o DRS (ambos na parte da frente). Já os pedais serão apenas de acelerador e freio, feitos em impressora 3d com um sistema simples de mola e potenciômetro. 

## Inputs e Outputs
### **Entradas (Inputs)**
- **Volante (IMU) - sensor MUP6050** Virar o carro para direita ou esquerda.
- **Pedal do acelerador (analógico) - potenciômetro** Acelerar.
- **Pedal do freio (analógico) - potenciômetro** Frear.
- **4x Entradas Digitais:**
  - Gear up - sobe marcha.
  - Gear down - desce marcha.
  - Overtake - aumenta potência para ultrapassagem.
  - DRS - ativa DRS.

### **Saídas (Outputs)**
- **LED indicador de conexão**

## Protocolo Utilizado
- **UART (Universal Asynchronous Receiver-Transmitter)** para comunicação entre o controle e o computador.
- **GPIO Interrupts** para os botões e entradas digitais.

## Diagrama de Blocos Explicativo do Firmware
### **Estrutura Geral**
---

![Estrutura](/img/diagrama.jpg)

---

#### **Principais Componentes do RTOS**
- **Tasks:**
  - Task de leitura de entradas (volante, pedais, botões)
  - Task de envio de comandos via UART
  - Task de atualização do LED indicador

- **Filas:**
  - Fila de eventos de entrada
  - Fila de comandos para o jogo

- **Semáforos:**
  - Verificação do estado de conexão

- **Interrupts:**
  - Callbacks para os botões e pedais

## Imagens do Controle
### **Proposta Inicial**
---

![Proposta](/img/PropostaControle.png)

---
