# Use a lightweight Python base image
FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    libatlas-base-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user for Codespaces
ARG USER=vscode
ARG UID=1000
RUN useradd -m --uid $UID $USER \
    && mkdir -p /home/$USER/.local \
    && chown -R $USER:$USER /home/$USER

USER $USER
WORKDIR /workspace

COPY --chown=vscode:vscode requirements.txt .
RUN pip install --user --upgrade pip setuptools wheel \
    && pip install --user -r requirements.txt

ENV PATH=/home/$USER/.local/bin:$PATH
