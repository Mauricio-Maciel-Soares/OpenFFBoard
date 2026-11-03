# Slew rate assimétrico por magnitude

O modo simétrico atual continua sendo o padrão e preserva o comportamento legado. O modo assimétrico é opcional e não altera o pipeline quando está desativado.

## Taxas e comandos

- `maxtorquerate`: taxa simétrica legada e valor de fallback.
- `slewmode=0`: modo simétrico legado.
- `slewmode=1`: modo assimétrico por magnitude.
- `slewup`: limite para aumentar a magnitude do torque, em counts/ms.
- `slewdown`: limite para reduzir a magnitude do torque, em counts/ms.
- Zero em `slewup` ou `slewdown` herda `maxtorquerate`.
- Para desativar o limitador por completo, use `maxtorquerate=0`, `slewup=0`, `slewdown=0` e `slewmode=0`.

Exemplo (não é recomendação de taxa para o motor):

```text
axis.0.maxtorquerate=10922
axis.0.slewmode=1
axis.0.slewup=12000
axis.0.slewdown=6000
```

O cálculo usa o mesmo intervalo real em microssegundos, teto de 1,5 período nominal e preservação de frações sub-count do limitador atual.

## Reversão de sinal

No modo assimétrico, uma transição de torque positivo para negativo (ou vice-versa) não atravessa zero no mesmo quadro. A magnitude existente primeiro é reduzida usando `slewdown`; somente em uma atualização posterior a magnitude oposta começa a crescer usando `slewup`.

A escolha entre subida e descida é feita pela magnitude absoluta do torque, não pela ordem numérica dos valores. Isso evita criar uma preferência artificial de força entre o lado positivo e o negativo do volante.

## Diagnóstico

`axis.0.slewstats?` retorna CSV v1:

`version,power_clip_updates,slew_limited_updates,slew_delta_counts`

- `power_clip_updates`: atualizações em que o limite de potência alterou o alvo ou a saída final.
- `slew_limited_updates`: atualizações em que o slew limiter alterou o alvo.
- `slew_delta_counts`: soma saturante da diferença absoluta, em counts, entre o alvo após clipping de potência e o torque após o slew.

`axis.0.slewstatsreset` solicita a limpeza no próximo ciclo FFB. O comando não altera o histórico do limitador, a saída de torque ou o driver.

### Snapshot integrado diag3

`axis.0.diag3?` preserva os campos de `diag2` e acrescenta
`slew_mode,slew_up_counts_ms,slew_down_counts_ms,slew_fallback_counts_ms,
slew_power_clip_updates,slew_limited_updates,slew_delta_counts`.
Configuração, fallback e estatísticas são copiados na mesma seção crítica,
para que o PC receba um snapshot coerente. Os comandos antigos `diag`,
`diag2` e `slewstats` continuam disponíveis e seus formatos não mudam.
O novo ID de comando foi acrescentado ao fim do enum para preservar os IDs
já publicados.

## Persistência

As taxas e o modo são armazenados por eixo em 11 endereços novos e reservados, separados do registro de perfis existente. O armazenamento de extensão usa dois bancos e um seletor de commit; uma gravação interrompida antes do seletor mantém o banco anterior. Os endereços e dados legados não são remapeados.

Os testes simulam interrupções durante as gravações, mas isso não substitui a validação física de desligamento e reinicialização da placa.
