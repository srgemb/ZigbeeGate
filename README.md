# ZigbeeGate
Шлюз сети Zigbee.

Предназначен для приема и отправки сообщений [терминалам](https://github.com/srgemb/WaterControl) сети.

Управление терминалами выполняется с помощью консольных команд:
``` bash
dtime [sync]                     - вывод текущих значений дата/время (синхронизация на терминалах).
valve num_dev                    - вывод состояния электроприводов
valve num_dev [cold/hot opn/cls] - управление электроприводами
water num_dev [N]                - вывод показаний расхода воды
wtlog num_dev num_logs           - запрос данных из журнала событий
dev [N]                          - вывод списка терминалов зарегестрированных в сети
```
Общие консольные команды управления:
``` bash
stat                             - Statistics.
task                             - List task statuses, time statistics.
flash                            - FLASH config HEX dump.
zb [res/init/net/save/cfg/chk]   - ZigBee module control.
config                           - Display of configuration parameters.
config save                      - Save configuration settings.
config uart xxxxx                - Setting the speed baud (600 - 115200).
config panid 0x0000 - 0xFFFE     - Network PANID (HEX format without 0x).
config netgrp 1-99               - Network group number.
config netkey XXXX....           - Network key (HEX format without 0x).
config devnumb 0x0001 - 0xFFFF   - Device number on the network (HEX format without 0x).
config gate 0x0000- 0xFFF8       - Gateway address (HEX format without 0x).
version                          - Displays the version number and date.
reset                            - Reset controller.
?                                - Help.
```