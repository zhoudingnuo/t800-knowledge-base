#!/bin/bash
# Download EngineAI repos as tarballs via ghfast.top mirror (fallback: direct codeload).
BASE=/home/admin2/t800
OUT="$BASE/repos"
TMP="$BASE/.kb_tmp"
mkdir -p "$OUT" "$TMP/logs"

dl() {
  name=$1; branch=$2
  gh="https://github.com/engineai-robotics/$name/archive/refs/heads/$branch.tar.gz"
  dest="$OUT/$name"
  mkdir -p "$dest"
  ok=0
  for attempt in 1 2; do
    if wget -q --tries=2 --timeout=180 -O "$TMP/$name.tar.gz" "https://ghfast.top/$gh"; then ok=1; break; fi
    echo "RETRY-MIRROR $name (attempt $attempt)"
  done
  if [ $ok -eq 0 ]; then
    echo "FALLBACK-DIRECT $name"
    if ! wget -q --tries=2 --timeout=180 -O "$TMP/$name.tar.gz" "https://codeload.github.com/engineai-robotics/$name/tar.gz/refs/heads/$branch"; then
      echo "DLFAIL  $name"; return
    fi
  fi
  if tar -xzf "$TMP/$name.tar.gz" -C "$dest" --strip-components=1 2>/dev/null; then
    sha=$(sha256sum "$TMP/$name.tar.gz" | cut -d' ' -f1)
    echo "OK      $name $(du -sh "$dest" | cut -f1) sha256=$sha"
    echo -e "$name\t$branch\t$sha" >> "$TMP/sha256.tsv"
    rm -f "$TMP/$name.tar.gz"
  else
    echo "TARFAIL $name"
  fi
}
export -f dl
export BASE OUT TMP

printf '%s\n' \
  "engineai_rl_lab main" \
  "engineai_ros2_workspace community" \
  "engineai_robotics_native_sdk main" \
  "engineai_robot_simulation_runtime main" \
  "engineai_legged_gym master" \
  "engineai_humanoid master" \
  "engineai_amp main" \
  "engineai_robotics_description master" \
  "engineai_navigation master" \
  "GMR main" \
| xargs -P 5 -n 2 bash -c 'dl "$0" "$1"'
echo "ALL_DONE"
