#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
PYTHONPATH="$project_root/src" python -m unittest discover -s "$project_root/tests" -v
