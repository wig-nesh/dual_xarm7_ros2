#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_name="dual_xarm7_humble"
container_name="dual_xarm_dev"

if [ "${1:-}" = "--rebuild" ] || ! sudo docker image inspect "${image_name}" >/dev/null 2>&1; then
  echo "building image ${image_name}"
  sudo docker build -t "${image_name}" "${project_root}/docker"
fi

display="${DISPLAY:-}"
if [ -z "${display}" ]; then
  # terminals inside wayland sessions sometimes lack DISPLAY; Xwayland still
  # exists, so fall back to the highest-numbered X11 socket
  for socket in /tmp/.X11-unix/X[0-9]*; do
    [ -e "${socket}" ] && display=":${socket##*X}"
  done
fi
if [ -z "${display}" ]; then
  echo "warning: no DISPLAY and no X11 socket found, GUIs will not open" >&2
fi

docker_args=(
  --name "${container_name}"
  --network host
  --ipc host
  --privileged
  --env DISPLAY="${display}"
  --env QT_X11_NO_MITSHM=1
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
  --volume "${project_root}/workspace:/colcon_ws:rw"
  --volume "${project_root}:/project:ro"
  --rm -it
)

# The nvidia container toolkit is not configured on this host. The NixOS driver
# tree is a symlink farm into /nix/store, so both must be mounted for the
# container to resolve the host NVIDIA libraries.
if [ -d /run/opengl-driver ]; then
  docker_args+=(
    --volume /run/opengl-driver:/run/opengl-driver:ro
    --volume /nix/store:/nix/store:ro
    --env LD_LIBRARY_PATH=/run/opengl-driver/lib
    --env VK_ICD_FILENAMES=/run/opengl-driver/share/vulkan/icd.d/nvidia_icd.x86_64.json
  )
else
  echo "warning: /run/opengl-driver not found, the container will have no GPU" >&2
fi

if command -v xhost >/dev/null 2>&1; then
  DISPLAY="${display}" xhost +local: >/dev/null 2>&1 || true
else
  echo "note: xhost not found, GUIs may fail to open the display" >&2
fi

sudo docker run "${docker_args[@]}" "${image_name}" bash
