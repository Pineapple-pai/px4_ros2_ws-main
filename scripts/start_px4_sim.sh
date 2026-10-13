#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-/home/p/PX4-Autopilot}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(dirname -- "${SCRIPT_DIR}")}"
QGC_DIR="${QGC_DIR:-/home/p/下载/PX4/qgc-daily-root/squashfs-root}"
VEHICLE_PROFILE="${VEHICLE_PROFILE:-}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-}"
PX4_PARAM_FILE="${PX4_PARAM_FILE:-}"

if [[ -n "${VEHICLE_PROFILE}" ]]; then
  if [[ ! "${VEHICLE_PROFILE}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "Invalid VEHICLE_PROFILE=${VEHICLE_PROFILE}"
    exit 1
  fi
  PROFILE_FILE="${WS_DIR}/sim/vehicles/${VEHICLE_PROFILE}.env"
  if [[ ! -f "${PROFILE_FILE}" ]]; then
    echo "Vehicle profile not found: ${PROFILE_FILE}"
    exit 1
  fi
  # shellcheck disable=SC1090
  source "${PROFILE_FILE}"
  PX4_MODEL="${PX4_MODEL:-${MODEL_NAME:-}}"
fi

PX4_MODEL="${PX4_MODEL:-iris}"
if [[ -n "${PX4_PARAM_FILE}" && "${PX4_PARAM_FILE}" != /* ]]; then
  PX4_PARAM_FILE="${WS_DIR}/${PX4_PARAM_FILE}"
fi
AGENT_PORT="${AGENT_PORT:-8888}"
TERMINAL_LAYOUT="${TERMINAL_LAYOUT:-windows}"
KEEP_TERMINALS_OPEN="${KEEP_TERMINALS_OPEN:-true}"

# ============ 仿真配置 ============
# 仿真后端: gazebo-classic 或 gz（新版 Gazebo Ignition）
SIM_BACKEND="${SIM_BACKEND:-gazebo-classic}"
# PX4 机型模型
PX4_MODEL="${PX4_MODEL:-iris}"
MODEL_SDF="${MODEL_SDF:-sim/models/${PX4_MODEL}/${PX4_MODEL}.sdf}"
if [[ "${MODEL_SDF}" != /* ]]; then
  MODEL_SDF="${WS_DIR}/${MODEL_SDF}"
fi
# Gazebo 世界。默认保留障碍物几何体，供外接/仿真近场测距节点验证。
PX4_WORLD="${PX4_WORLD:-room_obstacles}"

# PX4 家位置（可选，留空则使用 PX4 默认）
PX4_HOME_LAT="${PX4_HOME_LAT:-}"
PX4_HOME_LON="${PX4_HOME_LON:-}"
PX4_HOME_ALT="${PX4_HOME_ALT:-}"
PX4_HOME_YAW="${PX4_HOME_YAW:-}"
USE_SYSTEM_GEOLOCATION="${USE_SYSTEM_GEOLOCATION:-false}"
PX4_DISABLE_MAG_CHECKS="${PX4_DISABLE_MAG_CHECKS:-false}"
PX4_DISABLE_HEALTH_FAILSAFE_CHECKS="${PX4_DISABLE_HEALTH_FAILSAFE_CHECKS:-true}"
PX4_DISABLE_LOW_BATTERY_FAILSAFE="${PX4_DISABLE_LOW_BATTERY_FAILSAFE:-true}"

ENABLE_GAZEBO_ROS_SENSORS="${ENABLE_GAZEBO_ROS_SENSORS:-true}"

# ---------- 预处理检查 ----------
case "${TERMINAL_LAYOUT}" in
  windows|tabs|tmux|headless) ;;
  *)
    echo "Unsupported TERMINAL_LAYOUT=${TERMINAL_LAYOUT}. Use windows, tabs, tmux, or headless."
    exit 1
    ;;
esac

if [[ "${TERMINAL_LAYOUT}" == "windows" || "${TERMINAL_LAYOUT}" == "tabs" ]]; then
  if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal not found. Use TERMINAL_LAYOUT=tmux or TERMINAL_LAYOUT=headless."
    exit 1
  fi
fi

if [[ "${TERMINAL_LAYOUT}" == "tmux" ]] && ! command -v tmux >/dev/null 2>&1; then
  echo "tmux not found. Use TERMINAL_LAYOUT=windows/tabs/headless or install tmux."
  exit 1
fi


if [[ ! -d "${PX4_DIR}" ]]; then
  echo "PX4 directory not found: ${PX4_DIR}"
  exit 1
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "ROS 2 workspace not built: ${WS_DIR}/install/setup.bash missing"
  exit 1
fi

if [[ ! -x "${QGC_DIR}/AppRun" ]]; then
  echo "QGroundControl Daily not found: ${QGC_DIR}/AppRun"
  exit 1
fi

if [[ -n "${PX4_PARAM_FILE}" && ! -f "${PX4_PARAM_FILE}" ]]; then
  echo "PX4 parameter file not found: ${PX4_PARAM_FILE}"
  exit 1
fi

if [[ -n "${VEHICLE_PROFILE}" && ! -f "${MODEL_SDF}" ]]; then
  echo "Profile model SDF not found: ${MODEL_SDF}"
  exit 1
fi

# ============ 辅助函数 ============

PX4_GZ_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
WS_WORLDS_DIR="${WS_DIR}/sim/worlds"
WS_MODELS_DIR="${WS_DIR}/sim/models"
PX4_GAZEBO_CLASSIC_MODELS_DIR="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
PX4_GAZEBO_CLASSIC_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"

check_gazebo_classic_model_dependencies() {
  local root_model_name="$1"
  local models_dir="$2"

  if [[ -z "${root_model_name}" || ! -d "${models_dir}" ]]; then
    return 0
  fi

  local -a queue=("${root_model_name}")
  local -A visited=()

  while ((${#queue[@]} > 0)); do
    local model_name="${queue[0]}"
    queue=("${queue[@]:1}")

    if [[ -n "${visited[${model_name}]:-}" ]]; then
      continue
    fi
    visited["${model_name}"]=1

    local model_dir="${models_dir}/${model_name}"
    local sdf_path=""

    if [[ ! -d "${model_dir}" ]]; then
      echo "Missing Gazebo Classic model dependency: ${model_name}"
      return 1
    fi

    if [[ -f "${model_dir}/model.config" ]]; then
      local sdf_rel
      sdf_rel="$(sed -n 's:.*<sdf[^>]*>\([^<]*\)</sdf>.*:\1:p' "${model_dir}/model.config" | head -1)"
      if [[ -n "${sdf_rel}" && -f "${model_dir}/${sdf_rel}" ]]; then
        sdf_path="${model_dir}/${sdf_rel}"
      fi
    fi

    if [[ -z "${sdf_path}" ]]; then
      sdf_path="$(find "${model_dir}" -maxdepth 1 -name '*.sdf' | head -1)"
    fi

    if [[ -z "${sdf_path}" || ! -f "${sdf_path}" ]]; then
      echo "Unable to resolve SDF for Gazebo Classic model: ${model_name}"
      return 1
    fi

    while IFS= read -r dependency_name; do
      [[ -z "${dependency_name}" ]] && continue
      if [[ -z "${visited[${dependency_name}]:-}" ]]; then
        queue+=("${dependency_name}")
      fi
    done < <(
      sed -n 's|.*<uri>model://\([^<[:space:]]*\)</uri>.*|\1|p' "${sdf_path}" \
        | sed 's|/.*||' \
        | sort -u
    )
  done
}

install_custom_worlds() {
  if [[ ! -d "${WS_WORLDS_DIR}" ]]; then
    return 0
  fi

  if [[ -d "${PX4_GZ_WORLDS_DIR}" ]]; then
    if compgen -G "${WS_WORLDS_DIR}/*.sdf" >/dev/null; then
      cp -u "${WS_WORLDS_DIR}"/*.sdf "${PX4_GZ_WORLDS_DIR}/"
    fi
  elif [[ "${SIM_BACKEND}" == "gz" ]]; then
    echo "PX4 Gazebo worlds directory not found: ${PX4_GZ_WORLDS_DIR}"
  fi

  if [[ -d "${PX4_GAZEBO_CLASSIC_WORLDS_DIR}" ]]; then
    local world_file
    for world_file in "${WS_WORLDS_DIR}"/*.sdf; do
      [[ -f "${world_file}" ]] || continue
      cp -u "${world_file}" \
        "${PX4_GAZEBO_CLASSIC_WORLDS_DIR}/$(basename "${world_file}" .sdf).world"
    done
  else
    echo "PX4 Gazebo Classic worlds directory not found: ${PX4_GAZEBO_CLASSIC_WORLDS_DIR}"
  fi
}

install_custom_models() {
  if [[ ! -d "${WS_MODELS_DIR}" || ! -d "${PX4_GAZEBO_CLASSIC_MODELS_DIR}" ]]; then
    return 0
  fi

  local model_dir
  for model_dir in "${WS_MODELS_DIR}"/*; do
    [[ -d "${model_dir}" ]] || continue
    rm -rf "${PX4_GAZEBO_CLASSIC_MODELS_DIR}/$(basename "${model_dir}")"
    cp -r "${model_dir}" "${PX4_GAZEBO_CLASSIC_MODELS_DIR}/"
  done
}

resolve_system_geolocation() {
  if [[ "${USE_SYSTEM_GEOLOCATION}" != "true" ]]; then
    return 0
  fi

  if [[ -n "${PX4_HOME_LAT}" && -n "${PX4_HOME_LON}" ]]; then
    return 0
  fi

  if ! command -v curl >/dev/null 2>&1; then
    echo "curl not found, skipping system geolocation lookup"
    return 0
  fi

  local geo_json lat lon
  geo_json="$(curl -s --max-time 5 https://ipapi.co/json/ || true)"
  if [[ -z "${geo_json}" ]]; then
    echo "System geolocation lookup failed, continuing without PX4_HOME_* override"
    return 0
  fi

  lat="$(printf '%s' "${geo_json}" | sed -n 's/.*"latitude":[[:space:]]*\([-0-9.]*\).*/\1/p' | head -1)"
  lon="$(printf '%s' "${geo_json}" | sed -n 's/.*"longitude":[[:space:]]*\([-0-9.]*\).*/\1/p' | head -1)"

  if [[ -n "${lat}" && -n "${lon}" ]]; then
    PX4_HOME_LAT="${PX4_HOME_LAT:-${lat}}"
    PX4_HOME_LON="${PX4_HOME_LON:-${lon}}"
    PX4_HOME_ALT="${PX4_HOME_ALT:-20}"
    echo "Using system geolocation estimate: lat=${PX4_HOME_LAT}, lon=${PX4_HOME_LON}, alt=${PX4_HOME_ALT}"
  else
    echo "System geolocation lookup returned no coordinates, continuing without PX4_HOME_* override"
  fi
}

# ============ 执行预处理 ============
resolve_system_geolocation
echo ">>> 预处理: install_custom_worlds"
install_custom_worlds
echo ">>> 预处理: install_custom_models"
install_custom_models

if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  echo ">>> 准备 Gazebo Classic 启动参数"
  if [[ -n "${PX4_BUILD_TARGET:-}" ]]; then
    CLASSIC_MAKE_TARGET="${PX4_BUILD_TARGET}"
  else
    CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}"
    case "${PX4_WORLD}" in
      empty|none)
        CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}"
        ;;
      baylands|ksql_airport|mcmillan_airfield|ramped_up_wind|sonoma_raceway|warehouse|windy|yosemite)
        CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}__${PX4_WORLD}"
        ;;
      *)
        CLASSIC_MAKE_TARGET="gazebo-classic_${PX4_MODEL}"
        ;;
    esac
  fi

  if ! check_gazebo_classic_model_dependencies "${PX4_MODEL}" "${PX4_GAZEBO_CLASSIC_MODELS_DIR}"; then
    echo "Gazebo Classic model dependency check failed for PX4_MODEL=${PX4_MODEL}"
    echo "Please fix the missing model(s) under: ${PX4_GAZEBO_CLASSIC_MODELS_DIR}"
    exit 1
  fi
fi

mkdir -p "${WS_DIR}/.runtime"
mkdir -p "${WS_DIR}/.runtime/logs"
echo ">>> 已创建运行时目录: ${WS_DIR}/.runtime"

# ============ 生成 PX4 SITL 启动脚本 ============
cat > "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "${PX4_DIR}"

# 设置 PX4 SIM 模型和环境变量
if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  export PX4_SIM_MODEL="gazebo-classic_${PX4_MODEL}"
else
  export PX4_SIM_MODEL="${PX4_MODEL}"
fi
export PX4_GZ_MODEL="${PX4_MODEL}"
export PX4_SIMULATOR="${SIM_BACKEND}"

# 家位置（可选）
EOF

if [[ -n "${PX4_SYS_AUTOSTART}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART}"
EOF
fi

if [[ -n "${PX4_PARAM_FILE}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF
PX4_PARAM_FILE="${PX4_PARAM_FILE}"
while IFS= read -r line || [[ -n "\$line" ]]; do
  [[ -z "\$line" || "\$line" =~ ^[[:space:]]*# ]] && continue
  read -r param value _ <<< "\$line"
  [[ "\$param" =~ ^[A-Z][A-Z0-9_]*\$ && "\$value" =~ ^-?[0-9]+([.][0-9]+)?\$ ]] || continue
  export "PX4_PARAM_\${param}=\${value}"
done < "\${PX4_PARAM_FILE}"
EOF
fi

# 添加家位置设置
if [[ -n "${PX4_HOME_LAT}" && -n "${PX4_HOME_LON}" && -n "${PX4_HOME_ALT}" ]]; then
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF
export PX4_HOME_LAT="${PX4_HOME_LAT}"
export PX4_HOME_LON="${PX4_HOME_LON}"
export PX4_HOME_ALT="${PX4_HOME_ALT}"
export PX4_HOME_YAW="${PX4_HOME_YAW}"
EOF
fi

# Gazebo-Classic 需要额外设置环境变量
if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF

# 先 source ROS2 环境，这样 gazebo_ros 插件才能正常初始化
# ROS2 setup.bash 中可能引用未定义变量（如 AMENT_TRACE_SETUP_FILES），
# 需要用 set +u 避免 set -euo pipefail 报错
set +u
source /opt/ros/humble/setup.bash
set -u

# Gazebo-Classic 环境设置
# 先给变量赋默认空值，防止 setup_gazebo.bash 引用未绑定变量时报错退出
export GAZEBO_PLUGIN_PATH="\${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="\${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:-}"
source "${PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" "${PX4_DIR}" "${PX4_DIR}/build/px4_sitl_default"
export GAZEBO_MODEL_PATH="${WS_MODELS_DIR}:\${GAZEBO_MODEL_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""

if [[ "${PX4_WORLD}" != "empty" && "${PX4_WORLD}" != "none" ]]; then
  export PX4_SIM_WORLD="${PX4_WORLD}"
  export PX4_SITL_WORLD="${PX4_GAZEBO_CLASSIC_WORLDS_DIR}/${PX4_WORLD}.world"
fi

# 恢复 DISPLAY 和 XAUTHORITY 环境变量
# gnome-terminal -- bash -lc ... 创建的 login shell 不会继承父环境的 DISPLAY 和 XAUTHORITY，
# 导致 gzclient 找不到 X11 显示，Gazebo 图形界面无法出现。
export DISPLAY="\${DISPLAY:-${DISPLAY:-:0}}"
export XAUTHORITY="\${XAUTHORITY:-${XAUTHORITY:-}}"

# 检查 DISPLAY 是否可用，如果不可用则强制 HEADLESS 模式
if ! timeout 1 xdpyinfo -display "\$DISPLAY" >/dev/null 2>&1; then
  echo "WARNING: DISPLAY=\$DISPLAY is not accessible. Gazebo GUI will not appear."
  echo "Set HEADLESS=1 or check your X11 configuration."
  export HEADLESS=1
fi

if [[ "${ENABLE_GAZEBO_ROS_SENSORS}" != "true" ]]; then
  # source /opt/ros/humble/setup.bash 会设置 ROS_VERSION=2，
  # 这会导致 sitl_run.sh 检测到 ROS2 并让 gzserver 加载全局 ROS2 插件
  # (-s libgazebo_ros_init.so -s libgazebo_ros_factory.so)。
  # 当我们只需要 Gazebo 原生插件时，unset ROS_VERSION 能避免额外阻塞。
  unset ROS_VERSION
fi

# 不依赖 PX4 在后台隐式拉起 gzclient。
# 改为由本脚本在单独终端显式启动 GUI，避免 GUI 进程静默退出但终端不可见。
export HEADLESS=1
export PX4_NO_FOLLOW_MODE=1

cleanup() {
  local exit_code=\$?
  if [[ -n "\${SPAWN_PID:-}" ]]; then
    kill "\${SPAWN_PID}" >/dev/null 2>&1 || true
    wait "\${SPAWN_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "\${SIM_PID:-}" ]]; then
    kill "\${SIM_PID}" >/dev/null 2>&1 || true
    wait "\${SIM_PID}" >/dev/null 2>&1 || true
  fi
  exit "\${exit_code}"
}
trap cleanup EXIT INT TERM

SITL_BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
SITL_ROOTFS="\${SITL_BUILD_DIR}/rootfs"
SITL_BIN="\${SITL_BUILD_DIR}/bin/px4"
if [[ ! -x "\${SITL_BIN}" ]]; then
  export DONT_RUN=1
  make px4_sitl ${CLASSIC_MAKE_TARGET}
  unset DONT_RUN
else
  echo "Reusing existing PX4 SITL binary: \${SITL_BIN}"
fi
mkdir -p "\${SITL_ROOTFS}"

if [[ -n "\${PX4_SITL_WORLD:-}" ]]; then
  if [[ -f "${PX4_GAZEBO_CLASSIC_WORLDS_DIR}/\${PX4_SITL_WORLD}.world" ]]; then
    WORLD_PATH="${PX4_GAZEBO_CLASSIC_WORLDS_DIR}/\${PX4_SITL_WORLD}.world"
  else
    WORLD_PATH="\${PX4_SITL_WORLD}"
  fi
elif [[ "${PX4_WORLD}" == "none" ]]; then
  if [[ -f "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/${PX4_MODEL}.world" ]]; then
    WORLD_PATH="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/${PX4_MODEL}.world"
  else
    WORLD_PATH="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world"
  fi
else
  WORLD_PATH="${PX4_GAZEBO_CLASSIC_WORLDS_DIR}/${PX4_WORLD}.world"
fi

ROS_ARGS=""
if [[ -n "\${ROS_VERSION:-}" && "\${ROS_VERSION}" == "2" ]]; then
  ROS_ARGS="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
fi

echo "Starting gzserver: \${WORLD_PATH}"
gzserver \${ROS_ARGS} "\${WORLD_PATH}" &
SIM_PID=\$!

deadline=\$((SECONDS + 30))
until timeout 2 gz topic -l >/dev/null 2>&1; do
  if ((SECONDS >= deadline)); then
    echo "gzserver did not become ready within 30s"
    exit 1
  fi
  sleep 1
done

MODEL_SDF="${MODEL_SDF}"
if [[ ! -f "\${MODEL_SDF}" ]]; then
  MODEL_SDF="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${PX4_MODEL}/${PX4_MODEL}.sdf"
fi
if [[ ! -f "\${MODEL_SDF}" ]]; then
  echo "Unable to resolve model SDF for ${PX4_MODEL}"
  exit 1
fi

echo "Spawning model via gazebo_ros spawn_entity: \${MODEL_SDF}"
nohup bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1 && ros2 run gazebo_ros spawn_entity.py -file "\$1" -entity "\$2" -x 1.01 -y 0.98 -z 0.83 -R 0 -P 0 -Y 0 -timeout 30' _ "\${MODEL_SDF}" "${PX4_MODEL}" > "${WS_DIR}/.runtime/logs/gz_spawn.log" 2>&1 &
SPAWN_PID=\$!
echo "Background spawn pid: \${SPAWN_PID}"
sleep 3
if ! kill -0 "\${SPAWN_PID}" >/dev/null 2>&1; then
  wait "\${SPAWN_PID}" >/dev/null 2>&1 || true
  unset SPAWN_PID
fi

echo "Launching PX4 binary directly"
cd "\${SITL_ROOTFS}"
export PX4_PARAM_SYS_HAS_MAG=1
if [[ "${PX4_DISABLE_MAG_CHECKS}" == "true" ]]; then
  export PX4_PARAM_EKF2_MAG_CHECK=0
  export PX4_PARAM_COM_ARM_MAG_ANG=-1
  export PX4_PARAM_COM_ARM_MAG_STR=0
fi
if [[ "${PX4_DISABLE_HEALTH_FAILSAFE_CHECKS}" == "true" ]]; then
  export PX4_PARAM_COM_ARM_HFLT_CHK=0
fi
if [[ "${PX4_DISABLE_LOW_BATTERY_FAILSAFE}" == "true" ]]; then
  export PX4_PARAM_COM_LOW_BAT_ACT=0
fi

exec "\${SITL_BIN}" "\${SITL_BUILD_DIR}/etc"
EOF
else
  # 新版 Gazebo (gz)
  cat >> "${WS_DIR}/.runtime/start_px4_sitl.sh" <<EOF

# 新版 Gazebo (Ignition)
export GZ_SIM_SYSTEM_PLUGIN_PATH="\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:${PX4_DIR}/build/px4_sitl_default/build_gz/lib"

exec make px4_sitl gz_${PX4_MODEL}
EOF
fi

chmod +x "${WS_DIR}/.runtime/start_px4_sitl.sh"

if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  cat > "${WS_DIR}/.runtime/start_gazebo_gui.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "${PX4_DIR}"

set +u
source /opt/ros/humble/setup.bash
set -u

export GAZEBO_PLUGIN_PATH="\${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="\${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:-}"
source "${PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" "${PX4_DIR}" "${PX4_DIR}/build/px4_sitl_default"
export GAZEBO_MODEL_PATH="${WS_MODELS_DIR}:\${GAZEBO_MODEL_PATH}"

export DISPLAY="\${DISPLAY:-${DISPLAY:-:0}}"
export XAUTHORITY="\${XAUTHORITY:-${XAUTHORITY:-}}"

if ! command -v gzclient >/dev/null 2>&1; then
  echo "gzclient not found. Please install gazebo-classic GUI components."
  exit 1
fi

if ! timeout 1 xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
  echo "DISPLAY=$DISPLAY is not accessible, cannot launch Gazebo GUI."
  exit 1
fi

echo "Waiting for gzserver to become ready..."
until timeout 2 gz topic -l >/dev/null 2>&1; do
  sleep 1
done

echo "Starting Gazebo GUI (gzclient)..."
exec nice -n 10 gzclient --verbose
EOF
  chmod +x "${WS_DIR}/.runtime/start_gazebo_gui.sh"
fi

# ============ 生成 MicroXRCEAgent 启动脚本 ============
cat > "${WS_DIR}/.runtime/start_agent.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
exec MicroXRCEAgent udp4 -p ${AGENT_PORT}
EOF
chmod +x "${WS_DIR}/.runtime/start_agent.sh"

# ============ 生成 QGC 启动脚本 ============
cat > "${WS_DIR}/.runtime/start_qgc.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
if pgrep -f "QGroundControl|${QGC_DIR}/AppRun" >/dev/null 2>&1; then
  echo "QGroundControl is already running; not starting a second instance."
  exit 0
fi
cd "${QGC_DIR}"
exec env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY ./AppRun
EOF
chmod +x "${WS_DIR}/.runtime/start_qgc.sh"

cat > "${WS_DIR}/.runtime/wait_for_qgc_start.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
echo "Waiting a few seconds for PX4 MAVLink to come up before starting QGC..."
sleep 10
exec "${WS_DIR}/.runtime/start_qgc.sh"
EOF
chmod +x "${WS_DIR}/.runtime/wait_for_qgc_start.sh"

# ============ 启动终端 ============

terminal_total=3
if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  terminal_total=4
fi
component_titles=("PX4 SITL + Gazebo")
component_scripts=("${WS_DIR}/.runtime/start_px4_sitl.sh")
component_logs=("${WS_DIR}/.runtime/logs/px4.log")

if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then
  component_titles+=("Gazebo GUI")
  component_scripts+=("${WS_DIR}/.runtime/start_gazebo_gui.sh")
  component_logs+=("${WS_DIR}/.runtime/logs/gzclient.log")
fi

component_titles+=("MicroXRCEAgent")
component_scripts+=("${WS_DIR}/.runtime/start_agent.sh")
component_logs+=("${WS_DIR}/.runtime/logs/agent.log")

component_titles+=("QGroundControl")
component_scripts+=("${WS_DIR}/.runtime/wait_for_qgc_start.sh")
component_logs+=("${WS_DIR}/.runtime/logs/qgc.log")

terminal_command() {
  local script_path="$1"
  local log_path="$2"
  if [[ "${KEEP_TERMINALS_OPEN}" == "true" ]]; then
    printf '"%s" 2>&1 | tee "%s"; exec bash' "${script_path}" "${log_path}"
  else
    printf '"%s" 2>&1 | tee "%s"' "${script_path}" "${log_path}"
  fi
}

run_in_terminal() {
  local title="$1"
  local script_path="$2"
  local log_path="$3"
  gnome-terminal --title="${title}" -- bash -c "$(terminal_command "${script_path}" "${log_path}")"
}

append_terminal_tab() {
  local -n _args_ref="$1"
  local title="$2"
  local script_path="$3"
  local log_path="$4"
  _args_ref+=(--tab --title="${title}" -- bash -c "$(terminal_command "${script_path}" "${log_path}")")
}

run_in_tmux() {
  local session_name="px4-sim"
  if tmux has-session -t "${session_name}" 2>/dev/null; then
    tmux kill-session -t "${session_name}"
  fi

  tmux new-session -d -s "${session_name}" -n "PX4" \
    "bash -lc '\"${component_scripts[0]}\" 2>&1 | tee \"${component_logs[0]}\"'"

  local i
  for ((i = 1; i < ${#component_titles[@]}; ++i)); do
    tmux new-window -t "${session_name}" -n "${component_titles[$i]}" \
      "bash -lc '\"${component_scripts[$i]}\" 2>&1 | tee \"${component_logs[$i]}\"'"
    sleep 1
  done

  echo ">>> tmux session started: ${session_name}"
  echo ">>> Attach with: tmux attach -t ${session_name}"
}

run_headless() {
  local i
  if ! command -v script >/dev/null 2>&1; then
    echo "script(1) not found. Headless PX4 needs a pseudo-terminal; install util-linux or use TERMINAL_LAYOUT=windows/tabs."
    exit 1
  fi

  for ((i = 0; i < ${#component_titles[@]}; ++i)); do
    if [[ "${component_titles[$i]}" == "Gazebo GUI" || "${component_titles[$i]}" == "QGroundControl" ]]; then
      echo ">>> 跳过 GUI 组件（headless）: ${component_titles[$i]}"
      continue
    fi
    echo ">>> 后台启动: ${component_titles[$i]}"
    if [[ "${component_titles[$i]}" == "PX4 SITL + Gazebo" ]]; then
      nohup setsid script -q -f -c "${component_scripts[$i]}" "${component_logs[$i]}" >/dev/null 2>&1 &
      local bg_pid="$!"
    else
      nohup setsid "${component_scripts[$i]}" > "${component_logs[$i]}" 2>&1 &
      local bg_pid="$!"
    fi
    echo "${bg_pid}" > "${component_logs[$i]}.pid"
    sleep 1
    if [[ "${component_titles[$i]}" == "PX4 SITL + Gazebo" ]] && ! kill -0 "${bg_pid}" 2>/dev/null; then
      echo "Headless component exited immediately: ${component_titles[$i]}"
      [[ -f "${component_logs[$i]}" ]] && tail -n 80 "${component_logs[$i]}"
      exit 1
    fi
  done
}

echo ">>> 启动 ${terminal_total} 个组件，终端布局: ${TERMINAL_LAYOUT}"

if [[ "${TERMINAL_LAYOUT}" == "windows" ]]; then
  for ((i = 0; i < ${#component_titles[@]}; ++i)); do
    echo ">>> 启动窗口 $((i + 1))/${#component_titles[@]}: ${component_titles[$i]}"
    run_in_terminal "${component_titles[$i]}" "${component_scripts[$i]}" "${component_logs[$i]}"
    sleep 2
  done
elif [[ "${TERMINAL_LAYOUT}" == "tabs" ]]; then
  terminal_args=()
  for ((i = 0; i < ${#component_titles[@]}; ++i)); do
    append_terminal_tab terminal_args "${component_titles[$i]}" "${component_scripts[$i]}" "${component_logs[$i]}"
  done

  echo ">>> 启动单个 GNOME Terminal 窗口，并在其中创建 ${terminal_total} 个标签页"
  gnome-terminal "${terminal_args[@]}"
elif [[ "${TERMINAL_LAYOUT}" == "tmux" ]]; then
  run_in_tmux
else
  run_headless
fi

# ============ 输出提示 ============
cat <<MSG

================================================================
  PX4 仿真堆栈已启动！
================================================================

组件:
  标签页/终端 - PX4 SITL:        $(basename ${WS_DIR}/.runtime/logs/px4.log)
$(if [[ "${SIM_BACKEND}" == "gazebo-classic" ]]; then echo "  标签页/终端 - Gazebo GUI:      $(basename ${WS_DIR}/.runtime/logs/gzclient.log)"; fi)
  标签页/终端 - MicroXRCEAgent:  $(basename ${WS_DIR}/.runtime/logs/agent.log)
  标签页/终端 - QGroundControl:  $(basename ${WS_DIR}/.runtime/logs/qgc.log)
  ROS2 局部自主模式:            请单独运行 scripts/start_local_autonomy.sh

仿真配置:
  - 机型 profile: ${VEHICLE_PROFILE:-默认}
  - 后端:        ${SIM_BACKEND}
  - 模型:        ${PX4_MODEL}
  - 世界:        ${PX4_WORLD}
  - 当前主链:    PX4 本地位置 + 单局部目标
  - PX4 本地位置: 光流融合由 PX4 负责
  - 自主模式:    单独启动，使用单个局部目标

终端布局:       ${TERMINAL_LAYOUT}（windows=多窗口，tabs=单窗口多标签，tmux/headless=无桌面验证）
Gazebo 界面:    $(if [[ "${TERMINAL_LAYOUT}" == "headless" ]]; then echo "headless 模式跳过 GUI，仅启动 gzserver"; else echo "自动启动（gzserver + 独立标签页/终端中的 gzclient）"; fi)
QGC 界面:       $(if [[ "${TERMINAL_LAYOUT}" == "headless" ]]; then echo "headless 模式跳过"; else echo "自动启动"; fi)

等待 Gazebo 加载完成后:
  1. 在 QGC 中连接（自动检测）
  2. 切换到 Position 模式 → Arm → Takeoff
  3. 人工起飞并确认 PX4 本地位置有效
  4. 运行 ./scripts/start_local_autonomy.sh，切入 ROS2 Autonomy，再发布局部目标

主链仿真启动:
  ./scripts/start_px4_sim.sh

圆圆机自定义模型:
  VEHICLE_PROFILE=yuanyuan_quad ./scripts/start_px4_sim.sh

  # 无桌面窗口验证
  TERMINAL_LAYOUT=headless ./scripts/start_px4_sim.sh

停止: 关闭终端窗口即可
================================================================
MSG
