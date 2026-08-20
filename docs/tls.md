# HTTPS setup

QuadRF creates its own local certificate authority during package
installation. No certificate server, factory enrollment or Internet connection
is involved. Install that QuadRF's public root certificate once on a phone or
computer, then use:

```text
https://quadrf.local/
```

The initial setup page remains available over HTTP because the browser cannot
trust HTTPS until the root is installed:

```text
http://quadrf.local/setup/security/
```

That HTTP page intentionally remains available and will always be labeled
**Not secure**, even after the root is trusted. After installation, use the
prominent **Open QuadRF over HTTPS** link on the setup page or type the
complete `https://quadrf.local/` address. The page does not redirect
automatically because doing so before installation would send the user into a
certificate-warning screen.

The setup page detects and moves the instructions for iOS, Android, macOS,
Windows, Linux or ChromeOS to the top. It also displays the root certificate's
SHA-256 fingerprint and shows browser-specific help for desktop Firefox or a
compatible Chrome/Chromium HTTP fallback.

Keep using `quadrf.local` when possible; the stable name lets an open page
survive connection changes. If a client cannot resolve mDNS, use the fixed
address for the current direct connection instead: `192.168.44.1` for the
access point, `10.55.1.1` for direct Ethernet, or `10.55.0.1` for USB. The
setup page keeps that fixed IP when it opens HTTPS, and the certificate covers
all three addresses.

## iPhone and iPad

1. Connect to the QuadRF or to the same LAN as the QuadRF.
2. In **Safari**, open `http://quadrf.local/setup/security/`.
3. Tap **Download iPhone/iPad Profile**, then tap **Allow**.
4. Within eight minutes, open **Settings** and tap **Profile Downloaded**. If
   it is not shown, open **General → VPN & Device Management**.
5. Tap **Install**, enter the device passcode, review the warning, and tap
   **Install** again.
6. Open **Settings → General → About → Certificate Trust Settings**.
7. Under **Enable Full Trust for Root Certificates**, enable the entry
   beginning with **QuadRF Local Root CA**, then confirm.
8. Return to Safari and open `https://quadrf.local/`.
9. Launch Spatial RF Vision and tap **Allow** when Safari requests camera
   permission.

Apple removes a downloaded profile if it is not installed within eight
minutes. Stolen Device Protection can also prevent profile installation away
from a familiar location; follow the Settings prompt and re-enable the
protection afterward. Apple documents the current profile and manual-trust
steps in [Install a configuration profile](https://support.apple.com/102400)
and [Trust manually installed certificate profiles](https://support.apple.com/102390).

The profile is unsigned because it is generated locally and there is no online
signing service. iOS therefore displays an installation warning. The profile
contains only the public root certificate, never either private key.

## Android

1. Connect to the QuadRF or to the same LAN as the QuadRF.
2. In Chrome, open `http://quadrf.local/setup/security/`.
3. Tap **Download Android Certificate**.
4. Open Android **Settings** and search for **Install a certificate**.
5. Choose **CA certificate**. Do not choose **VPN and app certificate** or
   **Wi-Fi certificate**.
6. Review Android's network-monitoring warning, continue, and select
   `quadrf-root-ca.cer`. Set a screen lock if Android requires one.
7. Return to Chrome and open `https://quadrf.local/`.
8. Launch Spatial RF Vision and allow camera access.

Common settings paths are:

- Pixel: **Security & privacy → More security settings → Encryption &
  credentials → Install a certificate → CA certificate**.
- Samsung: **Security and privacy → More security settings → Install from
  device storage → CA certificate**.

Names differ by Android release and manufacturer. Google documents the general
certificate settings in [Android Help](https://support.google.com/android/answer/9654714)
and [Pixel certificate help](https://support.google.com/pixelphone/answer/2844832).
Chrome 131 and later use user-added Android TLS roots by default. A managed
device can disable that behavior with the
[`CAPlatformIntegrationEnabled`](https://chromeenterprise.google/policies/ca-platform-integration-enabled/)
policy.

If Android reports that a private key is required, the wrong certificate type
was selected. Go back and choose **CA certificate**; the downloadable `.cer`
intentionally contains no private key.

## macOS

1. Open `http://quadrf.local/setup/security/` and download the Mac
   certificate.
2. Open **Keychain Access**, select the **System** keychain and choose
   **File → Import Items**.
3. Import `quadrf-root-ca.cer`.
4. Double-click the **QuadRF Local Root CA…** entry, expand **Trust**, and set
   **When using this certificate** to **Always Trust**.
5. Authenticate, fully quit and reopen the browser, then open
   `https://quadrf.local/`.

The System keychain trust applies to Safari, Chrome and Edge. Firefox can use
its own certificate store; follow the Firefox instructions below if it still
shows a certificate warning.

## Windows with Chrome or Edge

1. Open `http://quadrf.local/setup/security/` and download the Windows
   certificate.
2. Open `quadrf-root-ca.cer` from Downloads and click **Install Certificate**.
3. Select **Current User**, then **Place all certificates in the following
   store**.
4. Click **Browse → Trusted Root Certification Authorities → OK → Next →
   Finish**, then accept the security warning.
5. Fully quit and reopen Chrome or Edge, then open
   `https://quadrf.local/`.

Use **Local Machine** instead of **Current User** only when an administrator
wants all Windows accounts on that computer to trust the QuadRF. Chrome and
Edge both consider roots explicitly installed in the Windows trust stores.
See Chromium's [local trust-store behavior](https://chromium.googlesource.com/chromium/src/+/main/net/data/ssl/chrome_root_store/faq.md#how-does-the-chrome-certificate-verifier-integrate-with-platform-trust-stores-for-local-trust-decisions)
and Microsoft's [Edge certificate-verification guidance](https://learn.microsoft.com/en-us/deployedge/microsoft-edge-security-cert-verification).

## Linux with Chrome, Chromium or Edge

1. Open `http://quadrf.local/setup/security/` and download the Linux
   certificate.
2. Open **Settings → Privacy and security → Security → Manage certificates**.
   Chrome 134 and later also provides `chrome://certificate-manager`.
3. Under locally installed or custom certificates, import
   `quadrf-root-ca.cer` and trust it for identifying websites/TLS servers.
4. Fully quit and reopen the browser, then open `https://quadrf.local/`.

If the built-in manager is unavailable, install `libnss3-tools` on
Debian/Ubuntu or `nss-tools` on Fedora and use Chromium's NSS database:

```bash
quadrf_nss_db="$HOME/.local/share/pki/nssdb"
[ -d "$HOME/.pki/nssdb" ] && quadrf_nss_db="$HOME/.pki/nssdb"
mkdir -p "$quadrf_nss_db"
[ -f "$quadrf_nss_db/cert9.db" ] || \
  certutil -d "sql:$quadrf_nss_db" -N --empty-password
certutil -d "sql:$quadrf_nss_db" -A -t "C,," \
  -n "QuadRF Local Root" -i "$HOME/Downloads/quadrf-root-ca.cer"
```

Chromium 146 and later defaults to `~/.local/share/pki/nssdb` unless the old
`~/.pki/nssdb` exists. Chromium documents both the browser manager and NSS
fallback in [Linux certificate management](https://chromium.googlesource.com/chromium/src/+/master/docs/linux/cert_management.md).

## ChromeOS

1. Download the ChromeOS certificate from the local setup page.
2. Open a new tab and paste `chrome://certificate-manager`.
3. Open the local/custom certificates section, import
   `quadrf-root-ca.cer`, and trust it for identifying websites.
4. Return to the setup page and open QuadRF over HTTPS.

A managed Chromebook may prevent user-installed roots. Its administrator must
deploy the certificate in that case. Chrome 134 and later documents the unified
manager at `chrome://certificate-manager` in the
[Chrome Root Store FAQ](https://chromium.googlesource.com/chromium/src/+/main/net/data/ssl/chrome_root_store/faq.md#whats-the-chrome-certificate-manager).

## Firefox on desktop

If Firefox still shows a certificate warning after the operating-system steps,
open **Settings → Privacy & Security → Certificates → View Certificates →
Authorities → Import**. Select `quadrf-root-ca.cer` and enable trust for
identifying websites. The local setup page detects desktop Firefox and shows
this browser-specific note automatically.

## Restarting desktop Chrome

After importing the root, desktop Chrome/Chromium may need a complete browser
restart before the new trust setting is applied. Save unfinished browser work,
open a new tab, paste the following internal address, and press Enter:

```text
chrome://restart
```

Chrome normally restores the open tabs. The HTTP setup page detects desktop
Chrome/Chromium and shows this as a copyable value. After Chrome reopens, click
**Open QuadRF over HTTPS** or explicitly open
`https://quadrf.local/`; returning to the HTTP address will still say **Not
secure**. Chromium defines this internal endpoint as
[`chrome://restart/`](https://chromium.googlesource.com/chromium/src/+/lkgr/chrome/common/webui_url_constants.h).

## Confirm HTTPS and camera access

After opening the HTTPS address, the browser must show no certificate warning.
Modern browsers do not necessarily display a green lock. In developer tools,
these expressions should return `true` and `"function"`:

```javascript
window.isSecureContext
typeof navigator.mediaDevices?.getUserMedia
```

Spatial RF Vision opens `/AR/` relative to the controls page, so launching it
from the HTTPS controls page keeps the AR tab on HTTPS and uses a secure
WebSocket.

## What installation creates

The `quadrf-network` package configuration runs the TLS setup before nginx is
started. This happens during `apt install`/package configuration, not on the
first browser visit or the first reboot. The reboot starts the services using
the identity that installation already created. The setup creates:

- A per-install 3072-bit RSA root CA valid for ten years.
- A separate 2048-bit RSA server key and certificate valid for at most 825
  days, with SHA-256, `serverAuth` EKU and SAN entries for
  `quadrf.local`, `quadrfd.local`, `quadrf-desktop.local`, `192.168.44.1`,
  `10.55.1.1` and `10.55.0.1`.
- An iOS configuration profile, a standard DER root certificate for Android,
  macOS, Windows, Linux, ChromeOS and Firefox, and a fingerprint text file,
  all served directly by the QuadRF.

The root is constrained to `.local` DNS names and the three fixed QuadRF
direct-connect networks. Its private key is mode `0600`, stays on the QuadRF
and is never exposed by nginx. The server certificate is renewed with the same
root when `quadrf apply` runs with fewer than 30 days remaining.

Normal boots, package upgrades and a package removal without purge preserve the
root. A fresh image, package purge or the following command creates a new root
and requires every client to remove the old certificate and repeat setup:

```bash
sudo quadrf tls local-ca
```

Changing `QUADRF_HOSTNAME` and running `sudo quadrf apply` issues a new server
certificate from the existing root, so clients that already trust that root do
not reinstall it.

## Imported certificates

An administrator can replace the local identity with an externally managed
certificate. Put the matching files in a directory as `fullchain.pem` and
`privkey.pem`, then run:

```bash
sudo quadrf tls import /path/to/certificate-directory
```

Import validates that the certificate and private key match. It removes the
local root download artifacts so users are not offered a root that does not
sign the active server certificate. Run `sudo quadrf tls local-ca` to return to
the locally generated setup.

## Temporary Chrome HTTP workaround

Android and desktop Chrome/Chromium can experimentally treat one exact HTTP
origin as secure with
`chrome://flags/#unsafely-treat-insecure-origin-as-secure`. When a compatible
browser opens the HTTP setup page, a collapsed section provides copyable
flag and origin values. The same fallback remains on the AR error screen.

This does not encrypt or authenticate the connection and should be used only
as a temporary camera workaround. Installing the local root and using HTTPS is
the supported workflow. The option is not shown for Edge, other Chromium-based
browsers, or any iOS browser.

## Remove trust

- iPhone/iPad: **Settings → General → VPN & Device Management → QuadRF Local
  HTTPS → Remove Profile**.
- Android: open **Encryption & credentials → User credentials** and remove the
  QuadRF entry. Paths vary by manufacturer.
- Mac: delete **QuadRF Local Root CA…** from the System keychain in Keychain
  Access.
- Windows: run `certmgr.msc` and delete the QuadRF entry under **Trusted Root
  Certification Authorities → Certificates**.
- Linux/ChromeOS: remove the root in the browser certificate manager. NSS users
  can delete the `QuadRF Local Root` nickname with `certutil`.
- Firefox: remove the QuadRF root under **View Certificates → Authorities**.
