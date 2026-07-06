# Remote visualization — Foxglove over WiFi (no ethernet)

Visualise the on-Jetson navigation stack (point clouds, TF, paths, costmaps, markers)
**on a laptop over WiFi**, with **no ethernet cable** and without fighting DDS across
the network. A single `foxglove_bridge` node on the Jetson serves everything over one
**WebSocket (TCP `:8765`)**; Foxglove Studio on the laptop connects to it.

!!! danger "Do not run heavy remote viz while the SONIC controller is balancing"
    On the 6-core **Orin Nano**, running the full nav stack **plus** the Foxglove bridge
    **plus** the SONIC walking controller at once can **starve the controller's LowState
    DDS thread** → `Lost LowState data connection` → safety-stop → **the robot falls**
    (this actually happened during bring-up: Foxglove Studio subscribed to `/livox/lidar`
    + point clouds, load hit ~3.7, and the controller lost LowState ~20 s later).
    Foxglove only streams topics a **connected client subscribes to**, so:

    - Prefer remote viz when the robot is **not** under active balance control
      (perception/mapping/planner debugging, robot hoisted & limp, or controller off).
    - If you must view while balancing, subscribe **only to light topics**
      (`/dlio/odom_node/odom`, `/global_path`, `/tf`, costmaps) — **never raw
      `/livox/lidar` or the DLIO point clouds** — and watch `uptime` load on the Jetson.

---

## Quick start

**On the Jetson** (native stack; a terminal per line):

```bash
# 1) the nav stack as usual (see SONIC_REAL_BRINGUP.md for the controller ordering)
cd ~/Navigation/ros2_ws && GAIT=sonic ./autonomy.sh        # USE_RVIZ=0 is implied on the Nano

# 2) the Foxglove bridge (separate terminal). Auto-detects the WiFi IP and prints the URL.
cd ~/Navigation/ros2_ws && ./start_foxglove.sh
# >> On the laptop, open Foxglove Studio and connect to:  ws://10.251.101.176:8765
```

**On the laptop:**

1. Open **Foxglove Studio** (`foxglove-studio`, or the web app at <https://app.foxglove.dev>).
2. *Open connection → Foxglove WebSocket →* `ws://<jetson-wifi-ip>:8765`.
   Deep link from a terminal:
   ```bash
   foxglove-studio "foxglove://open?ds=foxglove-websocket&ds.url=ws://10.251.101.176:8765"
   ```
3. Add a **3D** panel, set **Fixed frame** to `odom` (or `map`), and enable the topics you
   want (start light: `/global_path`, `/dlio/odom_node/odom`, `/tf`, `/global_planner/costmap`).

!!! note "The Jetson's WiFi IP can change (DHCP)"
    `start_foxglove.sh` prints the current IP. If it's wrong (multiple wireless NICs),
    run `JETSON_WIFI_IP=<ip> ./start_foxglove.sh`. Find it on the Jetson with
    `ip -br addr | grep -E 'wl'`.

---

## How it works

```
 Jetson (native, ROS_DOMAIN_ID=42)                         Laptop
 ┌───────────────────────────────────────────┐
 │ DLIO / g1_local_map / A* / MPC  ──DDS──►   │
 │ foxglove_bridge  ──subscribes locally──┐   │
 │                                        ▼   │            ┌──────────────────┐
 │                            WebSocket server │  1 × TCP   │ Foxglove Studio  │
 │                            0.0.0.0:8765 ────┼───────────►│  ws://jetson:8765│
 └───────────────────────────────────────────┘   (WiFi)   └──────────────────┘
```

- DDS stays **local to the Jetson** (bridge ↔ nav nodes on the same host) — the part
  that is reliable. Nothing about the ROS graph crosses the WiFi.
- The **only** thing on the WiFi is one TCP WebSocket, which enterprise APs pass fine
  (unlike DDS's multicast + many-port UDP).
- `foxglove_bridge` is already installed on the Jetson (`ros-humble-foxglove-bridge`).
  `start_foxglove.sh` also sources `ros2_ws/install` so **custom** message types
  (livox `CustomMsg`, planner msgs) advertise correctly.

### Why not native RViz over DDS?

Attempted first; it does not work on this campus WiFi. For the record, three things break it:

| Symptom | Cause |
|---|---|
| No topics discovered at all | AP **blocks DDS multicast**; worked around with unicast `Peers`. |
| Participants discovered, but `ros2 topic list` stays empty (no SEDP) | Every Jetson node also advertises the **unroutable robot-eth locator** `192.168.123.x`, poisoning endpoint matching. Humble's CycloneDDS ignores the newer `<Interfaces>` block — you must pin the NIC with the deprecated `<NetworkInterfaceAddress>` (see `docker/config/cyclonedds_jetson.xml`). |
| Even pinned, SEDP never completes over WiFi | Small SPDP packets pass but larger/fragmented **SEDP UDP is dropped** by the AP; plus the laptop runs **ROS 2 Jazzy** vs the Jetson's **Humble**, adding cross-version fragility. |

Foxglove's single WebSocket sidesteps all of it. If you ever *do* want native RViz, the
laptop helper `rviz_client.sh` and `docker/config/cyclonedds_jetson.xml` capture the
WiFi-only DDS config — but it is not the recommended path here.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `start_foxglove.sh`: *could not auto-detect a WiFi interface* | Pass `JETSON_WIFI_IP=<ip>`; check `ip -br addr`. |
| Bridge dies: *Failed to find a free participant index for domain 42* | The nav stack already claims the low DDS indices. The script sets `ParticipantIndex=none`; if you launch the bridge by hand, use that config (`CYCLONEDDS_URI`). |
| Foxglove connects but **no topics** | The bridge only sees nav topics it can discover locally — make sure the nav stack is up **on the same `ROS_DOMAIN_ID` (42)** and started before/with the bridge. Check `ss -ltn | grep 8765` on the Jetson and `grep 'Advertising new channel' /tmp/foxglove.log`. |
| Laptop can't reach `:8765` | Confirm WiFi reachability: `ping <jetson-ip>`; the WebSocket binds `0.0.0.0` so no firewall on the Jetson by default. |
| 3D panel empty / *Missing transform `odom`→`base_link`* | Set the **Fixed frame** to a frame that exists (`odom`), and enable `/tf` + `/tf_static`. `base_link` needs the robot's TF to be published. |
| Robot **fell** right after connecting Foxglove | LowState starvation from Jetson overload — see the danger box at the top. Unsubscribe heavy topics; don't run heavy viz while balancing. |

## Sync note

The Jetson runs the stack from a git checkout (`~/Navigation`); pull changes with
`git -C ~/Navigation pull`. `start_foxglove.sh` lives in `ros2_ws/` so it travels with
the repo — no per-Jetson editing needed (it auto-detects the IP at runtime).
