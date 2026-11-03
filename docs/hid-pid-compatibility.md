# Correções USB HID PID — OpenFFBoard v1.17.1

Esta alteração parte de `e3cdaa369c6e560d086bc5f5cda99398d8b20f58`, preservando o limitador de variação de torque, os diagnósticos e a seleção opcional da velocidade do ODrive do PR #2. Não muda ganhos, filtros, limite de torque, configuração CAN ou parâmetros do ODrive.

## Correções

- Normaliza relatórios de saída recebidos por SET_REPORT no canal de controle USB. O TinyUSB entrega esses dados sem o ID; o endpoint OUT entrega o ID junto dos dados. Os dois caminhos passam a produzir a mesma entrada para o manipulador HID e a interface de comandos.
- Valida tipo, ID e comprimento antes de interpretar relatórios; limita GET_REPORT ao espaço solicitado. A consulta INPUT PID State passa a retornar o estado agregado atual.
- Calcula a memória disponível a partir dos blocos realmente alocados. Liberação duplicada ou inválida não altera o pool. Reset libera todos os blocos; falha de alocação não reutiliza um índice de sucesso anterior. Tipos não anunciados no descritor retornam erro.
- Stop All Effects para os efeitos sem desabilitar os atuadores. Atualizar parâmetros não desfaz Disable Actuators ou Pause. Continue respeita a desativação explícita. Pause/Continue desloca os horários dos efeitos pelo intervalo pausado, inclusive para efeitos iniciados durante a pausa. Device Reset limpa a pausa, os efeitos e a desativação; a limpeza usada no desligamento USB continua sem ativar a força.
- Rejeita operações inválidas e tentativas de iniciar blocos não alocados.
- Decodifica Parameter Block Offset conforme o descritor de um ou dois eixos e rejeita índices fora do intervalo, evitando escrita fora das condições do efeito.
- Corrige o intervalo lógico dos bits Axes Enable/Direction Enable do descritor de um eixo para 0–1, nas variantes de 16 e 32 bits. Corrige também o nome da constante de tamanho do descritor de um eixo/32 bits.

A saturação zero continua sendo zero. A direção e a intensidade das forças não receberam os atalhos encontrados no Odrive-Wheel. Os IDs e os tamanhos dos relatórios USB permanecem iguais.

## Validação automatizada

`python3 tests/test_diagnostics.py` mantém os testes anteriores de torque, velocidade e timestamps.

`python3 tests/test_hid_pid.py` compila a classe HidFFB real, o callback USB real e os métodos reais de alocação/liberação, substituindo apenas as dependências de hardware e filtros. Usa AddressSanitizer e UndefinedBehaviorSanitizer e cobre sete grupos: pool, controle, pausa, condições, transporte, pacotes truncados e GET_REPORT. O teste de condições percorre os 256 valores do byte de offset para um e dois eixos. Outro teste compila e interpreta os quatro descritores (1/2 eixos, 16/32 bits), conferindo tamanhos, campos de habilitação e offsets.

Em ambientes que bloqueiam a inspeção de processos pelo LeakSanitizer, executar com `ASAN_OPTIONS=detect_leaks=0`; a verificação de acesso à memória e comportamento indefinido permanece ativa. O CI executa os testes normalmente e compila F407VG, F407VG_DISCO e F411RE.

## Validação física pendente

Os testes automatizados não comprovam compatibilidade com todos os jogos ou drivers. No F407VG + ODrive 3.6 FW 0.5.6, verificar enumeração USB, configuração pelo configurador, força constante e efeitos condicionais, parada/pausa/retomada, troca de sessão/jogo e reconexão USB. Comparar a sensação e os diagnósticos com a versão anterior usando as mesmas configurações e potência reduzida no primeiro teste.

O bit Actuator Power permanece uma indicação lógica do protocolo, não uma medição do motor. Esta alteração não implementa recursos PID ausentes, como reprodução por gatilho, repetição por loopCount ou força personalizada, nem revisa toda a temporização do calculador de efeitos. Os testes de pausa verificam os comandos e horários armazenados; a continuidade física da força ainda precisa ser avaliada no volante.

## Referências da revisão

- [USB PID 1.0](https://www.usb.org/sites/default/files/documents/pid1_01.pdf): Parameter Block Offset, saturação, PID Device Control e PID State.
- TinyUSB embarcado: `Firmware/FFBoard/USB/class/hid/hid_device.c`, caminhos SET_REPORT e endpoint OUT.
- [Odrive-Wheel](https://github.com/eagabriel/Odrive-Wheel/tree/a2beb5cfc9e42a103f42e0aa1cb5e2b8daf5c6a2): comparação de implementação, sem importação indiscriminada das alterações de força.
