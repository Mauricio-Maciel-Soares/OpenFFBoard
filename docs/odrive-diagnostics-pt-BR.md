# Diagnóstico ODrive e velocidade opcional — OpenFFBoard v1.17.1

Base: `bae3c527a80d14c21f167189f43a0b84cc63cbcd`, fork Mauricio-Maciel-Soares/OpenFFBoard.
Alvo do usuário: OpenFFBoard F407VG + ODrive 3.6 FW 0.5.6 + CAN 1 Mbps + servo 130ST-M15025.

## O que muda

- `axis.0.diag?`: amostra do cálculo do eixo, com torque dos efeitos, pedido antes do clipping,
  alvo após clipping, saída após rate limiter, contadores de clipping/slew e intervalos do loop.
- `odrv.0.diag?`: Iq solicitado/medido, velocidade do encoder, idade e sequência de cada
  amostra, último torque aceito pela fila CAN e falhas ao enfileirar esse torque.
- `axis.0.encvel=1`: opção em RAM para usar a velocidade estimada da ODrive, já recebida
  na mesma mensagem CAN de posição. Conversão de voltas/s para graus/s, redução mecânica
  e inversão iguais às da posição. O padrão é `0`, inclusive depois de reiniciar.
- As amostras de velocidade com mais de 5 ms são rejeitadas; nesse caso volta-se à derivada
  da posição. A troca de fonte não é diferenciada como se fosse aceleração física.
- Mantêm-se os biquads existentes de velocidade/aceleração. Não há equalizador novo.
  O limitador de torque mantém a correção anterior, seus valores e a base temporal original.
  Medir `dt` aqui não significa utilizá-lo para alterar `maxtorquerate`.
- A recepção de telemetria rejeita frames de outro barramento/nó, IDs estendidos, RTR,
  tamanhos incorretos e pares de floats não finitos. Comandos existentes mantêm seus IDs.

Os snapshots são formatados fora do loop FFB. A leitura copia o estado sob uma seção
crítica curta; o firmware não faz polling adicional de Iq e não escreve configuração na ODrive.
Não se alteram calibração, corrente máxima, torque_constant, ganhos de efeitos ou flash.

## Captura inicial (velocidade original)

1. Usar o firmware **F407VG desta branch** no OpenFFBoard. O arquivo não é firmware para
   gravar na ODrive. A compilação e os testes de software não substituem teste no hardware.
2. No console **odrivetool**, conectado à ODrive, habilitar temporariamente a transmissão
   periódica de Iq (o nome do objeto pode ser `dev0` ou `odrv0` no seu console):

   ```python
   old_iq_rate = dev0.axis0.config.can.iq_rate_ms
   dev0.axis0.config.can.iq_rate_ms = 10
   ```

   Isso habilita 100 mensagens/s. Não é necessário recalibrar nem executar
   `save_configuration()`. Não mudar CAN ID, baudrate ou os demais períodos.
3. No terminal USB do **OpenFFBoard**:

   ```text
   axis.0.encvel=0
   axis.0.diag?
   odrv.0.diag?
   ```

4. Fechar o configurador/terminal para liberar a porta CDC e executar no PC:

   ```text
   python -m pip install pyserial
   python tools/capture_odrive_diagnostics.py COM5 --seconds 30 --kt 1.11 --output baseline.csv
   ```

   Trocar `COM5` pela porta do OpenFFBoard. O script apenas consulta `diag?`; não liga motor,
   aplica efeitos de teste nem muda configuração. O arquivo existente não é sobrescrito.
   A captura padrão é de 20 consultas/s; o firmware conta todas as iterações do loop.
5. Comparar, com o mesmo jogo/carro/ganhos e força inicialmente reduzida, habilitando
   `axis.0.encvel=1` no terminal antes de uma segunda captura para `encoder_velocity.csv`.
   Usar `axis.0.encvel=0` para voltar imediatamente. Evitar trocar a fonte com volante carregado.
6. Ao terminar, restaurar no console ODrive:

   ```python
   dev0.axis0.config.can.iq_rate_ms = old_iq_rate
   ```

## Interpretar os dados

`estimated_torque_nm = iq_measured_a × Kt` estima torque eletromagnético. Para o valor
configurado de `Kt = 1,11 Nm/A`, 10 A correspondem a 11,1 Nm estimados. Não é uma leitura
de célula de carga nem garantia de torque no aro. O valor de Kt deve coincidir com a ODrive.

O CSV mantém separadamente:

- torque solicitado pelo eixo, incluindo os efeitos/limitadores locais, em counts;
- torque aceito pela fila CAN, em Nm e nas coordenadas do motor;
- Iq_setpoint e Iq_measured da ODrive, em A; e sua diferença multiplicada por Kt.

`effect_counts` é o resultado agregado do EffectsCalculator, antes do processamento do
eixo, não um pacote HID bruto. Ele pode ter sinal diferente do torque final após inversão.
`output_counts` é o resultado do cálculo: pode não ter sido enviado se o motor não estava
pronto ou se o valor não mudou. `queued_torque_nm` indica aceitação na fila TX, não ACK da
ODrive; `tx_age_ms` alto é normal se a força permanece constante. `tx_errors` conta falhas
ao enfileirar; não detecta toda perda de comunicação posterior.

O contador `clip_count` registra alterações no primeiro clipping de saída (não o clipping
interno do jogo). `slew_count` registra atuação do slew, antes do segundo clipping de proteção.
Os contadores acumulam desde a inicialização; o script calcula eventos/s por diferenças.
`loop_hz` usa a diferença de sequências e timestamps de duas consultas; não é a taxa de
pacotes HID. `dt_us/min_dt_us/max_dt_us` usam uma leitura coerente do tick HAL e do
timer de microssegundos (`diagnosticMicros()`, somente para diagnóstico) e
medem intervalos entre `prepareForUpdate()`, não latência USB → torque físico. Pausas e
mudanças de frequência entram nos mínimos/máximos acumulados desde o boot.

A leitura compensa uma virada do timer com interrupção ainda pendente e repete
o snapshot se o contador virar entre as leituras dos registradores. Isso evita
que uma leitura temporal inconsistente contamine os extremos com valores como
4294967295. O relógio dos efeitos e o rate limiter permanecem inalterados.
Não há descarte de intervalos longos nem substituição por valores nominais.
Após instalar a correção, `axis.0.diag?` mostra extremos medidos a partir do novo
boot; eles não precisam ser 1000 exatos, pois incluem pausas e jitter.

Os snapshots do eixo e da ODrive são **assíncronos**. Iq e encoder têm suas próprias idades
e sequências; sequência zero significa que ainda não houve amostra. O script deixa torque
estimado em branco quando Iq tem mais de 50 ms (configurável). Ele não declara que a diferença
entre o último comando CAN e Iq medido seja erro instantâneo sincronizado.

Esta captura é útil para tendências, saturação, ruído de baixa frequência e comparação A/B.
**Não mede com validade bandwidth de 200/400 Hz, slew em Nm/ms ou latência submilissegundo.**
Para isso é necessária captura local sincronizada na ODrive, com taxa adequada e excitação
controlada. Não escolher `current_control_bandwidth` apenas a partir deste CSV.

## Formato dos snapshots CSV v1

`axis.N.diag?`:

```text
version,sequence,time_us,dt_us,min_dt_us,max_dt_us,clip_count,slew_count,effect_counts,requested_counts,clipped_counts,output_counts,position_speed_deg_s,encoder_speed_deg_s,speed_deg_s,accel_deg_s2,encoder_valid,encoder_used,driver_ready
```

`odrv.N.diag?`:

```text
version,time_ms,iq_sequence,iq_age_ms,iq_setpoint_a,iq_measured_a,encoder_sequence,encoder_age_ms,velocity_turns_s,tx_sequence,tx_age_ms,queued_torque_nm,tx_errors
```

Os campos `age_ms` valem 4294967295 quando nunca houve amostra. Timestamps e contadores
são uint32 e podem dar a volta. As duas velocidades no snapshot do eixo estão em graus/s
e no mesmo referencial, para comparação direta. `encoder_valid=1` não significa que a
opção esteja ativa: verificar `encoder_used`. A posição HID continua vindo da posição.

## Validação

Executar `python3 tests/test_diagnostics.py` (Python 3 e g++ C++20). Os testes compilam
os corpos reais de `Axis::updateMetrics`/`updateTorque` com substitutos do hardware e
filtros passantes; verificam progressão até saturação, reversão, redução de potência,
shutdown fora da faixa, escolha de velocidade, unidades, inversão, fallback, troca de fonte,
formato CAN, NaN/Inf, freshness, rollover e parser CDC fragmentado. Não simulam FOC nem
estabilidade mecânica. O workflow existente compila os três alvos e executa esses testes.

Referência do protocolo verificada no código ODrive **fw-v0.5.6**:
[CANSimple](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.6/Firmware/communication/can/can_simple.cpp)
(`get_iq_callback`, `get_encoder_estimates_callback`, `service_stack`).
Inspiração para usar a velocidade estimada:
[Odrive-Wheel](https://github.com/eagabriel/Odrive-Wheel/blob/main/Odrive-Wheel/src/ffb_task.cpp).
