#!/usr/bin/env bash
set -euo pipefail

ENV_FILE=".env"

uid="$(id -u)"
gid="$(id -g)"
xauth="${XAUTHORITY:-$HOME/.Xauthority}"

if [[ -z "${OPENAI_API_KEY:-}" ]]; then
  read -r -p "OPENAI_API_KEY: " OPENAI_API_KEY
fi

cat > "${ENV_FILE}" <<EOF
OPENAI_API_KEY=${OPENAI_API_KEY}
UID=${uid}
GID=${gid}
XAUTHORITY=${xauth}
EOF

echo "Wrote ${ENV_FILE}"
