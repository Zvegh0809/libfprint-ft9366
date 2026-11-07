# Security policy

This policy covers the **FT9366 driver and its tools** in this fork
(`libfprint/drivers/ft9366/`, `tools/ft9366/`). For the rest of libfprint,
follow the upstream project's process.

## Supported versions

Only the latest commit of the `ft9366` branch (and the `libfprint-ft9366-git`
AUR package built from it) receives fixes.

## Reporting a vulnerability

Please **do not open a public issue**. Report it privately through
[GitHub's private vulnerability reporting](https://github.com/Zvegh0809/libfprint-ft9366/security/advisories/new).

Include a description, the affected commit, and steps or a proof of concept.
**Do not send real fingerprint images or enrolled templates**; synthetic data
is enough.

This is a hobby project maintained by one person: expect an acknowledgement
within about a week, and a fix or mitigation as soon as practical. You will
be credited in the advisory unless you prefer otherwise.

## In scope

- Template encryption at rest: AES-256-GCM sealing, authentication, key
  loading and the TPM-sealed systemd credential set up by `ft9366-setup-key`.
- Leaks of fingerprint images or templates to disk, logs or freed memory.
- Memory-safety bugs in the driver, the matcher or template parsing.
- Matching flaws that make a different finger or person accepted.

## Known limitations (not vulnerabilities)

These are documented in [tools/ft9366/README.md](../tools/ft9366/README.md)
and [TEMPLATE-ENCRYPTION.md](../tools/ft9366/TEMPLATE-ENCRYPTION.md):

- The USB link between the sensor and the host is neither encrypted nor
  authenticated, and there is no liveness detection; someone with physical
  access could replay a captured image.
- On a running system, root can read both the key and decrypted templates;
  the encryption protects templates at rest only.
- Images saved by the development tools are not encrypted.
