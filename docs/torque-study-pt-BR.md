# Torque, temporização CAN, velocidade e perfis persistentes

Implementação para OpenFFBoard v1.17.1/F407VG e ODrive CANSimple 0.5.6.
Inclui as correções anteriores de HID PID, limitador após clipping com `dt` real
e persistência de `encvel`/`maxtorquerate`. Não altera a escala `odrv.maxtorque`,
`power`, filtros, ganhos, corrente, tensão ou modo passthrough do ODrive.

## O que foi acrescentado

| Pedido | Implementação |
|---|---|
| Comando × corrente | Histórico de cada quadro Get_Iq recebido, com Iq solicitado/medido, último comando CAN aceito, velocidade e Vbus, incluindo idades e estado/erros do eixo |
| Temporização | `axis.N.diag2?` e `odrv.N.diag2?`, com timestamps em µs, intervalos reais, média, desvio padrão, mínimo, máximo e contagem da janela |
| Continuidade da velocidade | Validade por idade, estado/erros e heartbeat; dois quadros recentes para retorno; transição de fonte de 5 ms sem derivar a transição como aceleração física |
| Perfis de slew | Três slots editáveis por eixo; valores e seleção persistentes; gravação verificada e transação em dois bancos |
| Estudo elétrico | Captura CSV e análise de erro de corrente por faixa de RPM, sentido e motorização/regeneração; resultados dependem das capturas físicas |

## Uso no configurador atualizado

1. Na aba do eixo, abra **Advanced axis settings → Persistent slew profiles**.
2. Grave a taxa desejada em cada slot. A unidade continua **counts/ms**; zero
   desativa o limitador. Os slots começam vazios, sem valores presumidos.
3. Selecione um slot e use **Activate and verify**. Editar o slot que já está
   ativo também aplica sua nova taxa. **Custom** mantém a taxa atual.
4. Para telemetria leve, abra **Live diagnostics / CSV** e inclua a instância
   correta do ODrive. Os novos campos de temporização aparecem quando suportados;
   firmwares anteriores continuam usando `diag` versão 1.
5. Para estudar corrente, abra **Torque / current study**. Informe o `Kt`
   realmente configurado no ODrive; o valor inicial é 1,11 Nm/A. Escolha a
   instância e um rótulo, depois **Start new capture**.
6. **Stop → Export trace CSV** guarda os dados. **Analyze / export JSON** agrupa
   as observações por RPM, sentido e quadrante. O gráfico mostra comando CAN,
   Iq solicitado × Kt e Iq medido × Kt, em coordenadas do motor.

O início descarta o histórico anterior ao clique, sem escrever na placa.
A captura não envia torque, efeitos, comandos de movimento ou ajustes ao ODrive.
Não é um gerador automático de testes de motor.

### Se não aparecer corrente

O firmware recebe passivamente o quadro CANSimple `Get_Iq` (0x14). A captura
depende de o ODrive já transmiti-lo. Consulte no odrivetool o
`odrv0.axis0.config.can.iq_rate_ms` do eixo realmente utilizado. Zero desativa
a transmissão periódica. Um período de 10 ms é um ponto de partida para
observação a 100 Hz; a ferramenta não muda esse parâmetro nem aumenta o tráfego
CAN por conta própria. Se já houver uma configuração adequada, mantenha-a.
Para usar a velocidade CAN, os quadros de encoder devem chegar com intervalos
de até 5 ms; Iq e encoder têm períodos independentes.

Protocolo conferido no [código oficial ODrive 0.5.6](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.6/Firmware/communication/can/can_simple.cpp).

## Persistência e migração

- O firmware importa os valores gravados pelo firmware de persistência anterior.
  A importação não grava flash durante a inicialização e não ativa slots.
- A primeira alteração salva o novo formato: taxa ativa, preferência `encvel`,
  três taxas, máscara dos slots inicializados e seleção ativa.
- Os dados são escritos e lidos de volta no banco inativo antes de confirmar o
  seletor. Falha retorna `ERR` antes de mudar os valores ativos na RAM.
- SET repetido sem mudança não regrava flash. A gravação acontece somente nos
  comandos de configuração, nunca no loop FFB ou nas amostras CAN.
- Os 19 endereços novos por eixo constam no mapa de preservação/exportação.
  O registro antigo permanece intacto; firmware antigo não interpreta os slots.
  Ao voltar para firmware antigo, confira novamente os valores ativos.
- Se o firmware instalado ainda for `aba4956`, que não persistia esses valores,
  anote os valores em RAM antes da atualização e reaplique-os depois. Atualizar
  o firmware não pode recuperar configurações que nunca foram gravadas.

Exemplo de terminal, somente quando quiser aplicar essa taxa:

```text
axis.0.slewslot=10922?1
axis.0.slewslot?1
axis.0.slewprofile=1
axis.0.slewprofile?
axis.0.maxtorquerate?
axis.0.slewvalid?
```

`slewvalid` é uma máscara: bit 0 corresponde ao slot 1, bit 1 ao 2, bit 2 ao 3.
Selecionar um slot não inicializado retorna erro. Alterar `maxtorquerate` para
outra taxa seleciona Custom; os slots guardados continuam disponíveis.
Com `odrv.maxtorque=3000` (30 Nm), 10922 counts/ms equivalem aproximadamente a
10 Nm/ms **no comando**: `taxa × 30 / 32767`. Isso não comprova 10 Nm/ms físicos.

## Validade e transição da velocidade

`encvel=0` mantém o cálculo pela posição. Em `encvel=1`, a estimativa CAN só é
elegível com valores finitos, dois quadros consecutivos separados por até 5 ms,
amostra atual com idade de até 5 ms, heartbeat recente (até 2 s), estado 8 e
erros de eixo iguais a zero. Na falta dessas condições, usa o cálculo pela posição.
Quadro inválido, erro CAN ou perda de continuidade exige nova qualificação.

Na troca de fonte, um deslocamento inicial conserva a velocidade anterior e
decai em 5 ms. Variações da nova fonte continuam passando; não há extrapolação
da velocidade CAN vencida. A derivada da remoção desse deslocamento não entra
na aceleração usada pela inércia. `velocity_transition` e
`velocity_source_switches` tornam essas trocas observáveis. Os filtros existentes,
inclusive o perfil 1 (70/55 Hz, Q 0,55/0,30), não são retunados.

## Temporização e limites de interpretação

- `diag` continua versão 1. `diag2` usa versão 2 e acrescenta campos; clientes
  devem verificar versão e quantidade de campos. As listas de campos estão em
  `tools/capture_odrive_diagnostics.py` e `tools/torque_study.py`.
- Intervalos são registrados no firmware, inclusive os que acontecem entre duas
  consultas do PC. As estatísticas usam janelas sucessivas de até 1024 intervalos;
  a janela atual reinicia após 1024. Não se deve tirar média simples de snapshots
  repetidos nem tratá-los como um histograma completo de uma sessão.
- O timestamp de RX inclui atraso de atendimento da interrupção. Não é o instante
  de amostragem interno do ODrive. A atualização do tick e o reconhecimento do
  overflow ficam atômicos para evitar intervalos falsos quando o CAN interrompe o tick.
- Timestamp de TX significa aceitação pelo driver CAN local, não confirmação de
  transmissão no barramento ou de torque produzido. Intervalos TX podem ser longos
  se o comando permanecer constante; isso não significa perda da malha de 1 kHz.
- O relógio em µs e as sequências têm 32 bits e podem dar volta. O coletor trata
  rollover e marca reinicialização/descontinuidade; lacunas longas não permitem
  reconstruir múltiplas voltas do relógio.
- O buffer contém 64 observações, cerca de 4 KiB por instância de ODrive. Uma
  resposta entrega até 16. O produtor nunca espera pelo PC. Perdas aparecem
  explicitamente; não há garantia de exportar cada quadro sob carga elevada.
- A interface guarda 12000 amostras e informa quantas foram descartadas por esse
  limite. O gráfico mostra até 600 amostras contíguas e não liga pontos através
  de perdas conhecidas. Para sessões longas, use o coletor CLI, que grava em disco.
- Iq solicitado e Iq medido estão no mesmo quadro, mas os outros sinais são os
  últimos disponíveis, com suas idades. Um erro comando × Iq não demonstra sozinho
  saturação elétrica, atraso físico ou limitação da malha.
- `Iq × Kt` estima torque eletromagnético; atrito, perdas, temperatura e erro de Kt
  não são medidos. A análise rejeita estado fora de 8, erros e velocidade inválida;
  tensão com mais de 2 s não entra nas estatísticas de Vbus.

## Coleta e análise pela linha de comando

Feche o configurador para liberar a porta CDC. Os arquivos são criados sem
sobrescrever resultados existentes.

```sh
python -m pip install pyserial
python tools/capture_torque_trace.py COM5 --seconds 60 --kt 1.11 --label baseline --output baseline_trace.csv
python tools/analyze_torque_trace.py baseline_trace.csv --rpm-bin 30 --output baseline_study.json
```

O primeiro comando também gera `baseline_trace_timing.csv`, com snapshots dos
intervalos de loop e CAN. A taxa de pedidos do PC não é a taxa de amostragem Iq.

## Roteiro do estudo experimental

1. Registre firmware, Kt, tensão medida, período CAN de Iq/encoder, taxa de slew,
   `encvel`, filtros e configurações do ODrive. Mantenha esses dados junto do CSV.
2. Faça uma captura de referência durante o uso normal, incluindo repouso e as
   faixas de rotação que já ocorrem no seu volante. Não use este software para
   gerar rotação livre ou um degrau de força no motor.
3. Repita condições semelhantes mudando apenas um fator: por exemplo, slot de
   slew ou `encvel`. Faça uma nova captura para cada condição e mantenha os demais
   ajustes, incluindo alimentação e ganhos, iguais.
4. Compare RMSE/P95 do erro entre Iq solicitado e medido nas mesmas faixas de RPM,
   sentido e motorização/regeneração. Observe Vbus, erros, validade e perdas; grupos
   com poucas amostras ou cargas distintas não sustentam uma conclusão causal.
5. Se o erro aumentar com a rotação, use isso como evidência para investigar.
   Confirmar saturação de tensão ou largura de banda requer medições adicionais
   no controlador, com sincronização e taxa apropriadas. Não inferimos esses
   limites físicos apenas deste CSV.

Este documento é o protocolo de ensaio. Não contém resultados físicos inventados.

## Validação automatizada

```sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

Os testes compilam corpos reais dos métodos Axis e ODrive com ASan/UBSan e duplos
de hardware. Cobrem segurança do limitador, persistência/migração/cortes de energia,
validade/reaquisição CAN, transição sem impulso de inércia, estatísticas, histórico
limitado, sequência/tempo com rollover, CSV e análise. A CI compila F407VG,
F407VG_DISCO e F411RE depois dos testes. Testes físicos continuam necessários.
