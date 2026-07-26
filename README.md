# Lite6 Chess Robot

A UFactory xArm **Lite6** robot arm that physically plays chess against you.
Moves are made on a Chessnut e-board, processed by the Fritz chess engine on
Windows, and executed on the board by the robot arm running in a ROS2 / MoveIt
Docker container.

```
┌─────────────── Windows host ───────────────┐   ┌──────── Docker (ROS2 Humble) ────────┐
│ Chessnut board -> Fritz GUI                │   │  RabbitMQ subscriber (chess_robot)   │
│   └─ clipboard PGN ─ chess_fritz scraper   │   │    └─ MoveIt motion planning         │
│        └─ python-chess parse/diff          │   │        └─ xArm Lite6 + vacuum gripper│
│             └─ RabbitMQ publish ───────────┼───┼──> queue: robot_moves                │
└────────────────────────────────────────────┘   └──────────────────────────────────────┘
```

Moves cross the boundary as JSON `{"from_square": "e2", "to_square": "e4"}`.
Captures are executed as a two-phase sequence: first the captured piece is
cleared to a capture zone (`{"to_square": "xx"}` sentinel), then the capturing
piece is moved. Castling arrives as a king move followed by a rook move.
En passant clears the pawn from its actual square, not the arrival square.

## Repo layout

```
src/chess_common/   Shared config + logging (both sides)
src/chess_fritz/    Windows side: Fritz window scraping, PGN diffing, publisher
src/chess_robot/    Container side: RabbitMQ subscriber, MoveIt control, RViz viz
config/             YAML configuration (board geometry, messaging, logging, fritz)
tests/              pytest suite (pure-logic modules; no ROS/Windows needed)
```

## Known limitations

- **Promotions:** the robot has no spare queen to place. A promotion is
  executed as a plain pawn move and logged with a warning — swap the piece
  manually.
- The robot plays one fixed color per session (`--color`).
- Physical board geometry is calibrated in `config/board_config.yaml`; the
  numeric values there match a specific table setup.

## Prerequisites

- [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/) (Windows, WSL2 backend)
- An X server, e.g. [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (disable access control) — for RViz
- Fritz 15+ with the [Chessnut plugins](https://goneill.co.nz/chess.php#chessnut)
- [RabbitMQ](https://www.rabbitmq.com/docs/install-windows#downloads) running on the Windows host
- Python 3.10+

## Setup — Windows side

```powershell
git clone <this repo> && cd Lite6_chess
pip install -e ".[windows,dev]"
pytest                     # verify: all tests should pass
```

## Setup — robot container

```bash
docker build -t lite6-chess .
docker run -it --env DISPLAY=host.docker.internal:0.0 --network=host \
    --add-host=host.docker.internal:host-gateway \
    -v <path-to-this-repo>:/home/dev_ws/chess \
    --ipc=host --rm --name lite6chess lite6-chess
```

Inside the container:

```bash
# Simulation:
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py add_vacuum_gripper:=true
# Real arm:
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.175 add_vacuum_gripper:=true
```

In a second terminal into the same container (`docker exec -it lite6chess bash`):

```bash
cd /home/dev_ws/chess
pip3 install -e .
CHESS_SIM_MODE=1 chess-robot        # 1 = simulation, 0 = real arm
```

Optional: in RViz, Add → MarkerArray → topic `/chess_board_visualization` to
see the board overlay.

### Environment variables

| Variable           | Default            | Purpose                                  |
|--------------------|--------------------|------------------------------------------|
| `CHESS_CONFIG_DIR` | `<repo>/config`    | Where YAML configs are loaded from       |
| `CHESS_LOG_DIR`    | `<repo>/logs`      | Log files + JSON metrics exports         |
| `CHESS_SIM_MODE`   | prompt (TTY only)  | `1` sim / `0` hardware; required headless|

## Run — Windows side

Start Fritz, connect the Chessnut board (DGT board in the toolbar), then:

```powershell
chess-fritz --color white
```

The scraper polls Fritz's game notation, publishes new moves for the robot's
color to RabbitMQ, and recovers automatically from broker restarts, Fritz
window loss, and new-game restarts.

## Tests

```bash
pip install -e ".[dev]"
pytest
```

The suite covers the PGN move decomposition (captures, en passant, castling,
promotions), new-game detection, publisher retry logic (mocked broker),
board-coordinate math, and the metrics logger (including thread-safety).
ROS2- and pywinauto-dependent modules are exercised on the real rig instead.

## Reliability notes

- Publisher: delivery confirms, bounded reconnect/retry; a failed publish is
  logged and never crashes the poll loop.
- Subscriber: at-least-once delivery — a move is acked only after the robot
  completes it (`prefetch_count=1` gives natural backpressure). Malformed
  messages are dropped without requeue; a move that fails to execute is
  requeued once (for a transient planning failure) and dropped on the second
  attempt so it can't poison the queue. A broker lost mid-game is retried
  until it returns, rather than silently killing the consumer.
- New game: the Fritz side detects a restarted game and publishes a `reset`
  control message; the robot clears its capture-zone allocator before the
  replayed moves run.
- All pika operations run on the connection's owner thread; cross-thread
  signalling uses `add_callback_threadsafe` only.
