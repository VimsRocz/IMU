# Use the latest Ubuntu LTS
FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

# Install Python and dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3 python3-pip python3-venv build-essential \
    && pip3 install --no-cache-dir filterpy pandas numpy matplotlib

# Short installation note for filterpy
# On Ubuntu, install via:
#   sudo apt update && sudo apt install python3-pip
#   pip3 install filterpy
