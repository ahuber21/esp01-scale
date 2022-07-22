# ESP01s scale

## Wiring

| GPIO name | Arduino name | Pin number (on chip) | connected to |
|-----------|--------------|----------------------|--------------|
| 1         | 1            | 26                   | HX711 SCK    |
| 2         | 2            | 14                   | LED          |
| 3         | 3            | 25                   | HX711 D0     |


## MQTT

Topics

* `/scale` for all outbound messages
* `/scale/e` for error messages
* `/scale/in` for inbound messages


## Calibrate

