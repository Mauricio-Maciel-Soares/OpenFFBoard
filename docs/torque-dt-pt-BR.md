# Limitador de variação de torque com dt real

Base: OpenFFBoard v1.17.1/F407VG, a partir de `c1f0a0ab3d5c1a057cdd090a22ddbf4eaeea50d0` (PR #3). Inclui os diagnósticos, a opção de velocidade do encoder da ODrive e as correções HID PID anteriores.

## Comportamento

`axis.0.maxtorquerate` continua em **counts/ms**. O valor zero continua desativando completamente o limitador, sem ler seu relógio ou acumular crédito. A inicialização continua em zero; esta alteração não ativa 10 Nm/ms nem altera configurações da ODrive.

O limite por atualização passa a ser:

```
passo máximo [counts] = maxtorquerate [counts/ms] × dt [µs] / 1000
```

O alvo é limitado por `power` antes do slew. A referência é o último comando passado a `MotorDriver::turn`, guardado separadamente das métricas de posição/velocidade. Após o slew, outro clipping garante que uma redução de `power` seja aplicada imediatamente. As frações inferiores a um count são preservadas apenas enquanto a saída continua avançando na mesma direção; não há crédito acumulado quando o alvo é atingido, após reversões ou após um corte pelo limite final de potência.

| Taxa do loop | dt nominal | Passo para 1000 counts/ms | Maior dt admitido |
|---|---:|---:|---:|
| 1000 Hz | 1 ms | 1000 counts | 1,5 ms |
| 500 Hz | 2 ms | 2000 counts | 3 ms |
| 250 Hz | 4 ms | 4000 counts | 6 ms |
| 125 Hz | 8 ms | 8000 counts | 12 ms |

O dt é medido novamente no caminho de saída, pelo relógio coerente em microssegundos usado nos três alvos STM32 compilados. A subtração sem sinal trata o rollover de 32 bits. O relógio original dos efeitos e as fórmulas de velocidade, aceleração, damping, filtros, expo e maxspeed não foram alterados.

## Inicialização, atrasos e desligamento

- O primeiro cálculo com o limitador ativo apenas estabelece o instante de referência e mantém o último comando. Depois de uma reinicialização do driver, essa referência é zero.
- Chamadas no mesmo microssegundo não permitem avanço. Não existe um dt mínimo artificial que acelere a rampa.
- O dt máximo é 1,5 período nominal. Um atraso maior descarta o excedente e a fração anterior: um loop parado não acumula autorização para um salto arbitrariamente grande.
- Mudanças de taxa do loop ou de `maxtorquerate` reinicializam o histórico temporal/fração, mantendo o comando anterior como referência. O primeiro ciclo com a nova configuração mantém esse comando, salvo clipping de segurança.
- Suspensão/reconexão USB, emergência e troca de driver reinicializam o estado. Se o driver não estiver pronto, nenhum torque calculado avança a referência. Ao recuperar, a saída recomeça em zero quando o limitador está ativo.
- `outOfBounds`, os caminhos de desligamento USB/emergência e a redução do limite de potência não esperam a rampa para reduzir a força.

O teto de dt é uma política para limitar o passo de retorno, não um watchdog que consiga desligar o motor enquanto o processador estiver travado. Também não diagnostica nem corrige por si só a desconexão USB relatada.

## Relação com Nm/ms

A conversão depende de `odrv.0.maxtorque`, mantendo a escala existente da ODriveCAN:

```
Nm/ms ≈ maxtorquerate × torque máximo da ODrive [Nm] / 32767
```

Com escala de 30 Nm, aproximadamente 10922 counts/ms correspondem a 10 Nm/ms no comando: passos de cerca de 10 Nm a 1 kHz ou 20 Nm a 500 Hz, sujeitos ao alvo e ao clipping por `power`. Isso não mede a rapidez física do motor nem garante que um quadro CAN foi recebido. A API `turn()` não retorna confirmação; a referência significa comando entregue ao driver. O diagnóstico CAN existente continua sendo a fonte para identificar falhas de transmissão.

## Validação

`python3 tests/test_diagnostics.py` compila o limitador real e os métodos reais de atualização do eixo, usando um relógio controlado e um driver simulado. Cobre 1000/500/250/125 Hz, jitter, taxas fracionárias, inversão, saturação, redução de potência, pausa longa, zero dt, zero rate, rollover, valores extremos, reinicialização de métricas, driver indisponível, mudança de prontidão durante o envio, USB, emergência e mudança de frequência. As verificações anteriores de diagnóstico e velocidade permanecem.

`python3 tests/test_hid_pid.py` mantém as regressões dos relatórios USB HID PID e dos quatro descritores. Os testes C++ executam com AddressSanitizer e UndefinedBehaviorSanitizer. No ambiente local restrito, somente LeakSanitizer é desativado com `ASAN_OPTIONS=detect_leaks=0`; no GitHub Actions os sanitizadores executam normalmente.

O workflow compila F407VG, F407VG_DISCO e F411RE. A validação física do `.hex` e da regularidade de saída no conjunto F407VG + ODrive 3.6 FW0.5.6 permanece pendente. A implementação mantém o limite desativado por padrão, conforme combinado.
