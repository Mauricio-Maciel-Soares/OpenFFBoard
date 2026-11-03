# Persistência automática dos ajustes do eixo

`axis.0.encvel` e `axis.0.maxtorquerate` são gravados automaticamente na memória
não volátil do OpenFFBoard quando um comando SET válido é confirmado. Os valores
são restaurados ao reiniciar, inclusive depois de desligar e ligar. Não é necessário
executar `save`. O comando global `save` também inclui os dois ajustes.

- `encvel`: aceita `0` ou `1`; a seleção da fonte de velocidade continua igual.
- `maxtorquerate`: aceita de `0` a `4294967295` counts/ms, preservando os 32 bits.
  Zero continua desativando o limitador. O cálculo com dt real permanece igual.
- Sem registro válido salvo, os dois valores começam em zero. Endereços antigos
  de `MAX_ACCEL` não são reinterpretados como uma configuração nova.
- Consultas e reenvios dos mesmos valores já salvos não escrevem novamente na flash.
  Alterações são comandos de configuração; não há gravação periódica no loop FFB.
- A gravação é verificada por leitura antes de confirmar e aplicar o valor em RAM.
  Se ela falhar, o comando retorna erro e mantém o ajuste em uso.

Para conferir após reiniciar, consultar no terminal do OpenFFBoard:

```text
axis.0.encvel?
axis.0.maxtorquerate?
```

Os mesmos recursos estão disponíveis nos outros eixos, com armazenamento independente.
Nenhuma configuração é gravada na ODrive.

## Formato e validação

São usados sete endereços virtuais novos por eixo: dois bancos com a parte baixa
e alta de `maxtorquerate` e um campo identificado para `encvel`, mais um seletor de
banco identificado. O banco inativo é gravado e verificado antes de mudar o seletor.
Uma interrupção antes da confirmação mantém o último par completo selecionado.
A atomicidade de cada palavra e a recuperação das páginas continuam a cargo do
backend de EEPROM do firmware. Os endereços existentes não são remapeados; os
novos constam nas tabelas de preservação e exportação.

`python3 tests/test_axis_persistence.py` compila o código real de armazenamento,
SET/GET e save/restore com uma memória simulada. Cobre reinicialização, 32 bits,
valores inválidos, valores iguais sem regravação, separação dos eixos, falhas de
gravação, interrupções entre palavras, tentativas posteriores e dados incompletos.
Também verifica o mapa e os arquivos gerados. Os testes anteriores de dt,
diagnósticos e USB HID PID permanecem no workflow, assim como as compilações dos
três alvos. A simulação não substitui a validação de desligamento físico na placa.
