# Security

- **Never** embed API keys, tokens, passwords, or private URLs directly in source code.
- **Never** commit `.env` files containing real secrets.
- Read secrets from **environment variables** or approved secret managers at runtime.
- Keep credentials out of logs and doctest output.
