## Инерциальный датчик MPU6000 (сконфигурирован)

#### `SPI_1`
Модуль SPI1 используется для взаимодействия с инерциальным датчиком MPU6000, расположенным на плате

- `MOSI` - `SDA2/PMA9/RP100/RF4`
- `MISO` - `PMCS1/RPI75/RD11`
- `CLK` - `SCL2/PMA8/RP101/RF5`
- `CS` for `MPU6000` - `RB5/AN5/C1IN1+/VBUSON/VBUSST/RPI37`

## Отладочный пакет данных для SerialPlot (сконфигурирован)

#### `UART3_Tx`

- `UART3_Tx` - `AN28/PWM3L/PMD4/RP84/RE4`
- `UART3_Rx` - `AN8/PMA6/RPI40/RB8`

#### `DMA3` for `UART3_Tx`


## `LRMC_left_right_motor_control` - программный модуль для управления правым и левым электродвигателем на балансирующем роботе. 
Управление осуществляется с помощью команд, которые передаются в два контроллера векторного регулирования через интерфейс UART2

#### `UART2` - transmit and receive

 - `UART2_Tx with DMA2`
 - `UART2_Rx` - not used

#### `DMA2` for `UART2_Tx`