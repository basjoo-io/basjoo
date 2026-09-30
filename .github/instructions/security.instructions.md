---
applyTo: "**/*"
---
# Security

Never hardcode secrets (API keys, tokens, passwords, private URLs with embedded credentials) in source code or tracked configuration.

Use environment variables or approved secret-management systems. Do not commit `.env` files containing real credentials.
