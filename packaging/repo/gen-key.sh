#!/bin/bash
# Create a local packaging signing key used by reprepro when
# QUADRF_GPG_PRIVATE_KEY is not set (maintainer machines only).
# Key material stays under packaging/repo/.gnupg and is not published.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
export GNUPGHOME="${GNUPGHOME:-${here}/.gnupg}"
name="${QUADRF_KEY_NAME:-QuadRF Packaging}"
email="${QUADRF_KEY_EMAIL:-packages@quadrf.local}"

mkdir -p "${GNUPGHOME}"
chmod 700 "${GNUPGHOME}"
printf 'allow-loopback-pinentry\n' > "${GNUPGHOME}/gpg-agent.conf"
printf 'pinentry-mode loopback\n' > "${GNUPGHOME}/gpg.conf"

if gpg --list-secret-keys --with-colons | grep -q '^sec:'; then
    echo "signing key already present in ${GNUPGHOME}"
    gpg --list-secret-keys --keyid-format long
    exit 0
fi

gpg --batch --pinentry-mode loopback --passphrase '' \
    --quick-generate-key "${name} <${email}>" default default 0

gpg --list-secret-keys --keyid-format long
echo "created signing key in ${GNUPGHOME}"
