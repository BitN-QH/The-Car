# STM32 UART Protocol Draft

The STM32 side only receives serial data. It does not send odometry or status back
to RDK X5 in the current design.

## Motion Commands

RDK sends ASCII lines:

- `(1,0,0)` forward
- `(-1,0,0)` backward
- `(0,1,0)` left/right direction 1
- `(0,-1,0)` left/right direction -1
- `(0,0,1)` rotate direction 1
- `(0,0,-1)` rotate direction -1
- `(0,0,0)` stop

Only one field may be non-zero at the same time. Non-zero values must be `1` or
`-1`.

## Fork Commands

- `up`
- `down`

Fork commands must not be sent too frequently. The RDK node enforces
`serial_min_interval_sec`, default `0.3` seconds.

## RDK Safety

- Serial mode is disabled by default.
- RDK only opens serial when `serial_write_commands:=true` and `serial_port` is set.
- `/cmd_vel` is discretized so at most one of x/y/z is non-zero.
- `/fork/cmd` accepts `up`, `down`, and `stop`; `stop` sends `(0,0,0)`.
- All commands are newline-terminated ASCII.
