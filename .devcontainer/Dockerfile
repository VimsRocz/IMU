# Use an official Python runtime as a parent image
FROM python:3.10-slim

# Install any system dependencies here (e.g. for plotting)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy only requirements first to leverage Docker cache
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the code
COPY . .

# Default to bash
CMD [ "bash" ]
