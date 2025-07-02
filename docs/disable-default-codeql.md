# Disable the Default CodeQL Setup

This repository already runs CodeQL via a custom GitHub Actions workflow. To avoid duplicate scans, administrators should disable GitHub's automatic setup:

1. Open **Settings → Security → Code security and analysis → Code scanning**.
2. Under **Default setup**, click **Disable**.

Disabling the default scanner ensures only the custom workflow executes.
