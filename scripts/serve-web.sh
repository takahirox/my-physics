#!/usr/bin/env bash
set -euo pipefail
repository_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repository_dir/web"
python3 -m http.server "${PORT:-8080}"
