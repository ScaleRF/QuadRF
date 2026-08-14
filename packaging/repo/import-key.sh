#!/bin/bash
# Import the packaging signing key from QUADRF_GPG_PRIVATE_KEY.
# Used by GitHub Actions so every release is signed with the same key.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
export GNUPGHOME="${GNUPGHOME:-${here}/.gnupg}"

if [ -z "${QUADRF_GPG_PRIVATE_KEY:-}" ]; then
    echo "QUADRF_GPG_PRIVATE_KEY is not set" >&2
    exit 1
fi

mkdir -p "${GNUPGHOME}"
chmod 700 "${GNUPGHOME}"
printf 'allow-loopback-pinentry\n' > "${GNUPGHOME}/gpg-agent.conf"
printf 'pinentry-mode loopback\n' > "${GNUPGHOME}/gpg.conf"

keyfile="$(mktemp)"
trap 'rm -f "${keyfile}"' EXIT
printf '%s\n' "${QUADRF_GPG_PRIVATE_KEY}" > "${keyfile}"

passphrase="${QUADRF_GPG_PASSPHRASE:-}"
gpg --batch --pinentry-mode loopback --passphrase "${passphrase}" --import "${keyfile}"

gpg --list-secret-keys --keyid-format long
echo "imported signing key into ${GNUPGHOME}"
