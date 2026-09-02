#!/usr/bin/env bash
set -e

# root inside the container requires --no-sandbox for electron, and the
# container has no GPU stack wired for this app, so software rendering it is
exec /project/ufactory_studio/ufactory-studio-client --no-sandbox --disable-gpu "$@"
