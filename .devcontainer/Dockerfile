# Use the official VS Code Python devcontainer image
FROM mcr.microsoft.com/vscode/devcontainers/python:0-3.10

# [Optional] If you need system packages (e.g. for Proj/GDAL), install here
# RUN apt-get update && apt-get install -y proj-bin

# Set working directory
WORKDIR /workspace

# Copy your dependency lists
COPY requirements.txt /workspace/

# Install Python deps
RUN pip install --no-cache-dir -r requirements.txt

# Ensure your scripts are executable
RUN chmod +x /workspace/*.py
