# =============================================================================
# Benji Overview Presentation Container
# =============================================================================
# Container for building the Reveal.js/Svelte presentation.
# Uses Node.js with npm for dependency management and Vite for building.
#
# Usage:
#   docker run --rm -v $PWD:/workspace benji-pres:latest
# =============================================================================

FROM node:20-slim

WORKDIR /workspace

# Default command: install dependencies and build
ENTRYPOINT ["/bin/sh", "-c", "npm ci && npm run build"]
