#!/usr/bin/env bash
# LAPTOP: put the navigation stack next to KILVO on the X2's PC2, in ~/talos_nav_ws (nothing is started).
#
#   x2_onboard_deploy.sh [--target run@10.0.1.41] [--key ~/.ssh/x2_robot] [--no-build] [--no-casadi]
#
# 1. rsync a_star_mpc_planner, g1_local_map, x2_box_pnp, x2_bringup into PC2 ~/talos_nav_ws/src/<pkg>/
#    (each package dir separately, --delete only inside it: ~/talos_nav_ws/src/kilvo -- the KILVO copy -- is
#    never touched, nor is ~/kilvo_ws or anything else outside ~/talos_nav_ws);
# 2. casadi, offline: the pinned manylinux aarch64 cp310 wheel is downloaded HERE (cached, sha256-checked),
#    scp'd and `pip install --no-deps --target ~/talos_nav_ws/pydeps` (PC2 has no internet; numpy 1.26.1 and
#    scipy 1.8.0 are PC2's own -- casadi's wheel needs only `numpy`, no C-API pin; scipy is the planner's
#    ndimage/interpolate, present in 1.8);
# 3. colcon build on PC2 of those four packages (nice -n 10, --parallel-workers 1), refusing while another
#    colcon build (e.g. the KILVO one) runs there;
# 4. the planner's / x2_bringup's / x2_box_pnp's / g1_local_map's pure-python pytest on PC2 (none of them
#    imports rclpy; the rclpy-dependent test is skipped), and an `import casadi` check;
# 5. run_nav_on_pc2.sh to ~/talos_nav_ws/.
# It never runs a ROS node, launch, ros2 CLI, or a python that imports rclpy on PC2.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
target=run@10.0.1.41
key="$HOME/.ssh/x2_robot"
build=1
casadi=1
while [ "$#" -gt 0 ]; do
  case "$1" in
    --target) target="$2"; shift 2 ;;
    --key) key="$2"; shift 2 ;;
    --no-build) build=0; shift ;;
    --no-casadi) casadi=0; shift ;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
SSH=(ssh -i "$key" -o BatchMode=yes "$target")
RSYNC_SSH="ssh -i $key -o BatchMode=yes"
PKGS=(a_star_mpc_planner g1_local_map x2_box_pnp x2_bringup)
WS='$HOME/talos_nav_ws'   # expanded on PC2

CASADI_VERSION=3.8.1   # what the laptop tests ran with
CASADI_WHEEL="casadi-${CASADI_VERSION}-cp310-none-manylinux2014_aarch64.whl"
CASADI_URL="https://files.pythonhosted.org/packages/7f/67/4d685192eb7ed780bae72bfc90415b8241ce5e675365a367441e04a1a38b/$CASADI_WHEEL"
CASADI_SHA256=e3ce6e5b9d38917ae0b538c0db839f4016f51aef739cf92ca621a2067e413413
CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/x2_onboard"

say() { echo "[x2_onboard_deploy] $*"; }

"${SSH[@]}" "test -d $WS/src/kilvo" || { say "FAILED: no $WS/src/kilvo on PC2 (the KILVO copy is the first step, not this script's)"; exit 1; }
if "${SSH[@]}" "pgrep -f '^[^ ]*python3 .*(x2_onboard_nav|ros2 launch x2_bringup)'" >/dev/null 2>&1; then
  say "FAILED: the navigation stack seems to be running on PC2; stop it first (run_nav_on_pc2.sh stop)"; exit 1
fi

say "1/5 rsync packages"
"${SSH[@]}" "mkdir -p $WS/src $WS/logs"
for p in "${PKGS[@]}"; do
  rsync -a --delete --exclude __pycache__ --exclude .pytest_cache --exclude '*.pyc' \
    -e "$RSYNC_SSH" "$HERE/src/$p/" "$target:talos_nav_ws/src/$p/"
done

if [ "$casadi" = 1 ]; then
  say "2/5 casadi $CASADI_VERSION (offline wheel)"
  mkdir -p "$CACHE"
  if [ ! -f "$CACHE/$CASADI_WHEEL" ]; then
    curl -fsSL -o "$CACHE/$CASADI_WHEEL.part" "$CASADI_URL"
    mv "$CACHE/$CASADI_WHEEL.part" "$CACHE/$CASADI_WHEEL"
  fi
  echo "$CASADI_SHA256  $CACHE/$CASADI_WHEEL" | sha256sum -c - >/dev/null || { say "FAILED: wheel checksum"; exit 1; }
  scp -q -i "$key" "$CACHE/$CASADI_WHEEL" "$target:talos_nav_ws/$CASADI_WHEEL"
  # --no-deps: numpy / scipy are PC2's own. --target: a private folder (run_nav_on_pc2.sh puts it on PYTHONPATH).
  "${SSH[@]}" "cd $WS && python3 -m pip install --no-deps --no-index --upgrade --target pydeps $CASADI_WHEEL 2>&1 | tail -2"
fi

if [ "$build" = 1 ]; then
  say "3/5 colcon build on PC2 (nice 10, one worker)"
  "${SSH[@]}" "if pgrep -f '^/usr/bin/python3 /usr/bin/colcon' >/dev/null; then echo 'FAILED: a colcon build is already running on PC2'; exit 1; fi
    cd $WS && set +u && source /opt/ros/humble/setup.bash && source \$HOME/lx2501*/install/setup.bash && set -u &&
    nice -n 10 colcon build --packages-select ${PKGS[*]} --parallel-workers 1 2>&1 | tail -15"
fi

say "4/5 pure-python tests on PC2 (no rclpy)"
"${SSH[@]}" "cd $WS &&
  export PYTHONPATH=\$PWD/pydeps:\$PWD/src/a_star_mpc_planner:\$PWD/src/g1_local_map:\$PWD/src/x2_box_pnp:\$PWD/src/x2_bringup OPENBLAS_NUM_THREADS=1 &&
  python3 -c 'import casadi, numpy, scipy; print(\"casadi\", casadi.__version__, \"numpy\", numpy.__version__, \"scipy\", scipy.__version__)' &&
  nice -n 10 python3 -m pytest -q -p no:cacheprovider src/a_star_mpc_planner/test --ignore=src/a_star_mpc_planner/test/test_slam_map_fusion.py \
     src/g1_local_map/test src/x2_box_pnp/test src/x2_bringup/test/test_frames.py src/x2_bringup/test/test_onboard.py src/x2_bringup/test/test_nav_relay.py src/x2_bringup/test/test_nav_image.py 2>&1 | tail -8"

say "5/5 run script"
scp -q -i "$key" "$HERE/x2_onboard_run_on_pc2.sh" "$target:talos_nav_ws/run_nav_on_pc2.sh"
"${SSH[@]}" "chmod +x $WS/run_nav_on_pc2.sh"
say "done. Nothing started. On PC2: ~/talos_nav_ws/run_nav_on_pc2.sh start   (see its header)"
