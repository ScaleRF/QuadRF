const $ = (id) => document.getElementById(id);
const RATES = ['6', '9', '12', '18', '24', '36', '48', '54'];
const MOD_CODING = {
    '6': 'BPSK 1/2',
    '9': 'BPSK 3/4',
    '12': 'QPSK 1/2',
    '18': 'QPSK 3/4',
    '24': '16-QAM 1/2',
    '36': '16-QAM 3/4',
    '48': '64-QAM 2/3',
    '54': '64-QAM 3/4',
};
const CH = ['#b8c4b8', '#e6b45a', '#8fa7c4', '#c98f86'];
const GRID = '#232823';
const TICK = 'rgba(184,196,184,0.65)';
const CLIP = 39;  // int8 RX samples saturate near 39 counts

let ws = null;
let st = null;
let lastViz = null;

// ------------------------------------------------------------------ socket & tab lifecycle

const dataClient = (window.crypto && crypto.randomUUID)
    ? crypto.randomUUID()
    : String(Date.now()) + '-' + Math.random().toString(16).slice(2);
let dataLeaving = false;
let attachTimer = null;
let attachCtrl = null;

function dataAttach() {
    if (dataLeaving) return;
    if (attachCtrl) {
        try { attachCtrl.abort(); } catch (e) {}
    }
    attachCtrl = typeof AbortController !== 'undefined' ? new AbortController() : null;
    fetch('/api/apps/attach?app=data&client=' + encodeURIComponent(dataClient), {
        method: 'POST',
        keepalive: true,
        signal: attachCtrl ? attachCtrl.signal : undefined
    }).catch(() => {});
}

function dataDetach() {
    if (dataLeaving) return;
    dataLeaving = true;
    if (attachTimer) {
        clearInterval(attachTimer);
        attachTimer = null;
    }
    if (attachCtrl) {
        try { attachCtrl.abort(); } catch (e) {}
        attachCtrl = null;
    }
    if (ws) {
        try { ws.close(); } catch (e) {}
    }
    const url = '/api/apps/detach?app=data&client=' + encodeURIComponent(dataClient);
    try {
        if (navigator.sendBeacon) navigator.sendBeacon(url);
    } catch (e) {}
    fetch(url, { method: 'POST', keepalive: true }).catch(() => {});
}

function dataResume() {
    if (!dataLeaving && attachTimer) return;
    dataLeaving = false;
    if (!attachTimer)
        attachTimer = setInterval(dataAttach, 1000);
    dataAttach();
    if (!ws || ws.readyState > 1) connect();
}

dataAttach();
attachTimer = setInterval(dataAttach, 1000);

// Detach when the tab/page is closed or navigated away, but do NOT detach on
// visibilitychange (tab hidden) so multiple Quads can be tested across tabs.
window.addEventListener('pagehide', dataDetach);
window.addEventListener('beforeunload', dataDetach);
window.addEventListener('freeze', dataDetach);
window.addEventListener('pageshow', dataResume);
window.addEventListener('resume', dataResume);
document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible') dataAttach();
});

function connect() {
    const proto = (location.protocol === 'https:' ? 'wss://' : 'ws://');
    const base = location.pathname.endsWith('/') ? location.pathname : location.pathname + '/';
    const wsUrl = proto + location.host + (base === '/' ? '/ws' : base + 'ws');
    ws = new WebSocket(wsUrl);
    ws.onopen = () => {
        setStatus('CONNECTED', '');
        send({ t: 'hello' });
        send({ t: 'viz', on: true });
    };
    ws.onclose = () => {
        setStatus('CONNECTING...', 'warn');
        if (!dataLeaving) setTimeout(connect, 1000);
    };
    ws.onmessage = (ev) => {
        let m;
        try { m = JSON.parse(ev.data); } catch (e) { return; }
        if (m.t === 'st') onStatus(m);
        else if (m.t === 'chat') onChat(m.msgs);
        else if (m.t === 'viz') onViz(m);
    };
}
function send(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }

function setStatus(txt, cls) {
    const el = $('status');
    el.textContent = txt;
    el.className = cls || '';
}

// ------------------------------------------------------------------ tooltips

const tip = $('tip');
let tipFor = null;
function showTip(el) {
    tipFor = el;
    tip.textContent = el.dataset.tip;
    tip.style.display = 'block';
    const r = el.getBoundingClientRect(), tw = tip.offsetWidth, th = tip.offsetHeight;
    let x = Math.min(Math.max(8, r.left), window.innerWidth - tw - 8);
    let y = r.bottom + 6;
    if (y + th > window.innerHeight - 8) y = Math.max(8, r.top - th - 6);
    tip.style.left = x + 'px';
    tip.style.top = y + 'px';
}
function hideTip() { tipFor = null; tip.style.display = 'none'; }
document.addEventListener('pointerover', (e) => {
    const t = e.target.closest && e.target.closest('[data-tip]');
    if (t) { if (t !== tipFor) showTip(t); } else if (e.pointerType === 'mouse') hideTip();
});
document.addEventListener('pointerdown', (e) => {
    const t = e.target.closest && e.target.closest('[data-tip]');
    if (t) showTip(t); else hideTip();
});
document.addEventListener('scroll', hideTip, true);

// ------------------------------------------------------------------ formatting

const f1 = (v) => (v == null || !isFinite(v)) ? '-' : v.toFixed(1);
const bits = (m) => [0, 1, 2, 3].filter((k) => m >> k & 1);
function fmtBytes(b) {
    if (b == null || !isFinite(b)) return '0 B';
    if (b >= 1e6) return (b / 1e6).toFixed(2) + ' MB';
    if (b >= 1e3) return (b / 1e3).toFixed(1) + ' kB';
    return b + ' B';
}

// ------------------------------------------------------------------ settings

const TIPS = {
    freq: 'RF center frequency in MHz (4900–6000 MHz C-band). Both peers must be tuned to the same carrier.',
    tx_gain: 'MAX2850 transmit gain (0–40 dB). Typical bench setting for 1–3 m links is 30–36 dB.',
    rx_gain: 'MAX2851 + FPGA receiver gain (0–63 dB). Adjust so incoming burst peaks sit around 15–30 ADC counts below the 39-count clip ceiling.',
    tx_ant: 'Active transmit antenna elements on MAX2850. Digital baseband scales by 1/√N_tx to maintain constant total array power.',
    rx_ant: 'Active receive antenna channels on MAX2851. Multiple enabled channels are coherently combined in the FPGA.',
    pa_mute: 'Mute SE5003L 1W power amplifiers during receive and unmute strictly around the DSI VSYNC transmit burst window.',
    tun: 'Enable Linux tun0 virtual network interface for IPv4 packet routing over the 802.11 OFDM link.',
    auto_ping: 'Continuously send ICMP Echo Requests to the peer tun0 IP every 1.5 s.',
    viz: 'Stream real-time 20 Hz DSP stage waveforms and constellation snapshots from the receiver.',
};

let curRateIdx = 0;

const settings = [
    { k: 'freq', group: 'RADIO & PHY', label: 'CARRIER FREQUENCY', type: 'range', lo: 5100, hi: 5900, step: 5, unit: 'MHz', v: 5860, def: 5860 },
    { k: 'tx_gain', group: 'RADIO & PHY', label: 'TX GAIN', type: 'range', lo: 0, hi: 40, step: 1, unit: 'dB', v: 34, def: 34 },
    { k: 'rx_gain', group: 'RADIO & PHY', label: 'RX GAIN', type: 'range', lo: 0, hi: 63, step: 1, unit: 'dB', v: 30, def: 30 },
    { k: 'tx_ant', group: 'RADIO & PHY', label: 'TX ELEMENTS', type: 'mask', v: 1, def: 1 },
    { k: 'rx_ant', group: 'RADIO & PHY', label: 'RX CHANNELS', type: 'mask', v: 1, def: 1 },
    { k: 'pa_mute', group: 'TDD & TUNNEL', label: 'FAST TDD PA MUTE', type: 'bool', v: 1, def: 1 },
    { k: 'tun', group: 'TDD & TUNNEL', label: 'LINUX TUN0 INTERFACE', type: 'bool', v: 1, def: 1 },
    { k: 'auto_ping', group: 'TDD & TUNNEL', label: 'AUTO PING PEER (1.5 s)', type: 'bool', v: 0, def: 0 },
    { k: 'viz', group: 'DIAGNOSTICS', label: 'LIVE DSP STAGE PLOTS', type: 'bool', v: 1, def: 1 },
];

const setting = (k) => settings.find((s) => s.k === k);

function fmtVal(s, v) {
    if (s.type === 'sel') return (s.opts.split(',')[v] || v) + ' Mb/s';
    if (s.type === 'mask') return bits(v).map((k) => 'EL' + k).join(' ') || 'NONE';
    const d = s.step >= 1 ? 0 : s.step >= 0.1 ? 1 : 2;
    return Number(v).toFixed(d) + (s.unit ? ' ' + s.unit : '');
}

function pushSetting(k, v) {
    if (k === 'rate') {
        const rateStr = RATES[v] || '6';
        send({ t: 'set', k: 'rate', v: rateStr });
    } else if (k === 'pa_mute' || k === 'tun') {
        send({ t: 'set', k, v: !!v });
    } else if (k === 'auto_ping') {
        send({ t: 'auto_ping', v: !!v });
    } else if (k === 'viz') {
        send({ t: 'viz', on: !!v });
    } else {
        send({ t: 'set', k, v: Number(v) });
    }
}

const maskButtons = (k) => [0, 1, 2, 3].map((b) => `<button class="eb" data-m="${k}" data-b="${b}">EL${b}</button>`).join('');

function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

function buildSettings() {
    const body = $('set-body');
    body.innerHTML = '';
    let grp = '';
    for (const s of settings) {
        if (s.group !== grp) {
            grp = s.group;
            const h = document.createElement('div');
            h.className = 'sect';
            h.textContent = grp;
            body.appendChild(h);
        }
        const tipAttr = TIPS[s.k] ? ` data-tip="${esc(TIPS[s.k])}"` : '';
        let row;
        if (s.type === 'bool') {
            row = document.createElement('div');
            row.className = 'tgl';
            row.innerHTML = `<span${tipAttr}>${s.label}</span><div class="sw2"></div>`;
            row.onclick = () => { s.v = s.v ? 0 : 1; pushSetting(s.k, s.v); syncSettings(); };
        } else if (s.type === 'sel') {
            row = document.createElement('div');
            row.className = 'srow';
            const opts = s.opts.split(',');
            row.innerHTML = `<label><em${tipAttr} style="font-style:normal">${s.label}</em> <span></span></label><div class="tabs">` +
                opts.map((o, i) => `<button class="tab" data-i="${i}">${o}</button>`).join('') + '</div>';
            row.querySelectorAll('.tab').forEach((b) => {
                b.onclick = () => { s.v = +b.dataset.i; pushSetting(s.k, s.v); syncSettings(); };
            });
        } else if (s.type === 'mask') {
            row = document.createElement('div');
            row.className = 'srow';
            row.innerHTML = `<label><em${tipAttr} style="font-style:normal">${s.label}</em> <span></span></label><div class="eg">${maskButtons(s.k)}</div>`;
        } else {
            row = document.createElement('div');
            row.className = 'srow';
            row.innerHTML = `<label><em${tipAttr} style="font-style:normal">${s.label}</em> <span></span></label><input type="range">`;
            const r = row.querySelector('input');
            r.min = s.lo; r.max = s.hi; r.step = s.step; r.value = s.v;
            r.oninput = () => { s.v = +r.value; row.querySelector('label span').textContent = fmtVal(s, s.v); pushSetting(s.k, s.v); };
            r.ondblclick = () => { s.v = s.def; pushSetting(s.k, s.v); syncSettings(); };
        }
        row.dataset.k = s.k;
        body.appendChild(row);
    }
    $('r-modtabs').innerHTML = RATES.map((o, i) => `<button class="tab" data-i="${i}">${o}</button>`).join('');
    $('r-modtabs').querySelectorAll('.tab').forEach((b) => {
        b.onclick = () => { curRateIdx = +b.dataset.i; pushSetting('rate', curRateIdx); syncSettings(); };
    });
}

document.addEventListener('click', (e) => {
    const b = e.target.closest && e.target.closest('.eb');
    if (!b) return;
    const s = setting(b.dataset.m);
    if (!s) return;
    const v = s.v ^ (1 << +b.dataset.b);
    if (!(v & 15)) return;  // keep at least 1 element enabled
    s.v = v;
    pushSetting(s.k, v);
    syncSettings();
});

function syncSettings() {
    for (const s of settings) {
        const row = document.querySelector(`#set-body [data-k="${s.k}"]`);
        if (!row) continue;
        if (s.type === 'bool') row.classList.toggle('on', !!s.v);
        else if (s.type === 'sel') {
            row.querySelector('label span').textContent = fmtVal(s, s.v);
            row.querySelectorAll('.tab').forEach((b) => b.classList.toggle('on', +b.dataset.i === s.v));
        } else if (s.type === 'mask') {
            row.querySelector('label span').textContent = fmtVal(s, s.v);
        } else {
            const r = row.querySelector('input');
            if (document.activeElement !== r) r.value = s.v;
            row.querySelector('label span').textContent = fmtVal(s, s.v);
        }
    }
    document.querySelectorAll('.eb').forEach((b) => {
        const s = setting(b.dataset.m);
        if (!s) return;
        const on = !!(s.v >> +b.dataset.b & 1);
        b.classList.toggle('on', on);
        if (b.closest('#eg')) b.textContent = on ? 'ON' : 'OFF';
    });
    $('r-modtabs').querySelectorAll('.tab').forEach((b) => b.classList.toggle('on', +b.dataset.i === curRateIdx));
}

$('btn-set').onclick = () => {
    const p = $('set-pop');
    p.classList.toggle('open');
    $('btn-set').classList.toggle('on', p.classList.contains('open'));
};
$('s-defaults').onclick = () => send({ t: 'defaults' });
const help = (on) => { $('help').classList.toggle('open', on); $('btn-help').classList.toggle('on', on); };
$('btn-help').onclick = () => help(!$('help').classList.contains('open'));
$('help-x').onclick = () => help(false);
$('help').addEventListener('click', (e) => { if (e.target === $('help')) help(false); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') help(false); });
document.addEventListener('pointerdown', (e) => {
    const p = $('set-pop');
    if (p.classList.contains('open') && !p.contains(e.target) && e.target !== $('btn-set')) {
        p.classList.remove('open');
        $('btn-set').classList.remove('on');
    }
});

$('btn-ping').onclick = () => send({ t: 'ping' });
$('btn-autoping').onclick = () => {
    const ap = setting('auto_ping');
    if (!ap) return;
    ap.v = ap.v ? 0 : 1;
    pushSetting('auto_ping', ap.v);
    syncSettings();
};

// ------------------------------------------------------------------ status & channel summary

const snrHist = [];
const gpHist = [];

// Subcarrier lookup for k in [-26..-1, 1..26] from either 52-bin active array or 64-bin FFT array
function scVal(arr, k) {
    if (!arr || k === 0 || k < -26 || k > 26) return null;
    if (arr.length === 52) return arr[k < 0 ? k + 26 : k + 25];
    if (arr.length >= 64) return arr[k + 32];
    return null;
}

function hasSubcarriers(arr) {
    return Array.isArray(arr) && (arr.length === 52 || arr.length >= 64);
}

function onStatus(m) {
    st = m;
    $('host').textContent = (m.host || 'QUADRF').toUpperCase();
    setStatus(m.link_up ? 'LINK' : 'LISTEN', m.link_up ? '' : 'warn');

    const rateKey = String(parseInt(m.rate, 10) || 6);
    curRateIdx = Math.max(0, RATES.indexOf(rateKey));
    if (m.freq != null) setting('freq').v = m.freq;
    if (m.tx_gain != null) setting('tx_gain').v = m.tx_gain;
    if (m.rx_gain != null) setting('rx_gain').v = m.rx_gain;
    if (m.tx_elements != null) setting('tx_ant').v = m.tx_elements;
    if (m.rx_elements != null) setting('rx_ant').v = m.rx_elements;
    if (m.pa_mute != null) setting('pa_mute').v = m.pa_mute ? 1 : 0;
    if (m.tun_up != null) setting('tun').v = m.tun_up ? 1 : 0;
    if (m.auto_ping != null) setting('auto_ping').v = m.auto_ping ? 1 : 0;
    if (m.viz_on != null) setting('viz').v = m.viz_on ? 1 : 0;
    syncSettings();

    $('btn-autoping').classList.toggle('on', !!m.auto_ping);
    $('btn-autoping').textContent = m.auto_ping ? 'AUTO PING: ON' : 'AUTO PING';

    const clip = (m.peak_adc || 0) >= CLIP;
    $('r-call').textContent = (m.host || '-').toUpperCase();
    $('r-freq').textContent = `${f1(m.freq)} MHz / 20 MHz`;
    $('r-tx').textContent = `${f1(m.tx_gain)} dB / ${m.pa_mute ? 'TDD MUTE' : 'CONT TX'}`;
    $('r-rx').textContent = `${f1(m.rx_gain)} dB`;
    $('r-lvl').textContent = `${m.peak_adc || 0} / ${CLIP}${clip ? '  CLIPPING' : ''}`;
    $('r-lvl').classList.toggle('warn', clip);

    const peerName = (m.peer_host && m.peer_host !== 'peer')
        ? m.peer_host.toUpperCase()
        : (m.peer_ip === '10.99.0.2' ? 'QUADRF-2' : m.peer_ip === '10.99.0.3' ? 'QUADRF-3' : 'PEER');
    $('r-pcall').textContent = peerName;
    const shortMac = (mac) => mac && mac.length >= 17 ? mac.slice(0, 5) + '..' + mac.slice(12) : (mac || '-');
    $('r-peer').textContent = `${m.peer_ip || '-'} · ${shortMac(m.peer_mac)}`;
    $('r-peer').title = `${m.peer_ip || '-'} / ${m.peer_mac || '-'}`;
    $('r-ping').textContent = (m.ping_status || 'Idle').replace(' (avg: ', ' (avg ');
    $('r-ping').classList.toggle('warn', m.ping_status === 'Timeout / Loss');

    const gpRx = m.gp_rx != null ? m.gp_rx : (m.goodput_kbps || 0);
    const gpTx = m.gp_tx != null ? m.gp_tx : 0;
    const gpTot = m.goodput_kbps != null ? m.goodput_kbps : (gpRx + gpTx);
    $('gp-rx').textContent = f1(gpRx);
    $('gp-tx').textContent = f1(gpTx);
    $('se').textContent = rateKey;

    const dataPct = Math.min(100, Math.max(0, (gpTot / 1000) * 100));
    $('bar-data').style.width = dataPct + '%';
    $('bar-data-v').textContent = f1(gpTot) + ' kb/s';
    const deliv = m.delivery != null ? m.delivery : 0;
    $('bar-bw').style.width = Math.min(100, Math.max(0, deliv)) + '%';
    $('bar-bw-v').textContent = f1(deliv) + ' %';

    $('lobe').textContent = MOD_CODING[rateKey] || '-';
    $('phy').textContent = `${m.my_ip || '-'} / 576 B`;
    const nRxEl = bits(m.rx_elements || 1).length;
    const nTxEl = bits(m.tx_elements || 1).length;
    $('streams').textContent = `${nRxEl} / ${nTxEl}`;

    // Elements table
    const txScaleDb = nTxEl > 1 ? -10 * Math.log10(nTxEl) : 0;
    const rows = [];
    for (let k = 0; k < 4; k++) {
        const rxOn = !!((m.rx_elements || 1) >> k & 1);
        const txOn = !!((m.tx_elements || 1) >> k & 1);
        const rxg = rxOn ? m.rx_gain : null;
        const txg = txOn ? Math.max(0, m.tx_gain + txScaleDb) : null;
        const bar = (v, maxV, cls) => `<span class="sbar${cls || ''}"><i style="width:${v == null ? 0 : Math.max(0, Math.min(100, (v / maxV) * 100))}%"></i></span>${v == null ? '-' : f1(v) + ' dB'}`;
        rows.push(
            `<tr><td>${k}</td>` +
            `<td class="mod${rxOn ? '' : ' off'}">${rxOn ? 'ON' : 'OFF'}</td>` +
            `<td>${bar(rxg, 63, '')}</td>` +
            `<td class="pdiv mod${txOn ? '' : ' off'}">${txOn ? 'ON' : 'OFF'}</td>` +
            `<td>${bar(txg, 40, ' peer')}</td></tr>`
        );
    }
    $('el-body').innerHTML = rows.join('');

    // Frames & ARQ
    const pct = (a, b) => b > 0 ? ` (${(100 * a / b).toFixed(0)}%)` : '';
    $('frames').textContent = `${m.fcs_ok || 0} / ${m.fcs_fail || 0} / ${m.rx_frames || 0}`;
    $('blocks').textContent = `${fmtBytes(m.rx_bytes || 0)} / ${fmtBytes(m.tx_bytes || 0)}`;
    const coarseCfo = lastViz && lastViz.coarse_cfo != null ? `${(lastViz.coarse_cfo / 1e3).toFixed(1)}` : '-';
    $('cfo').textContent = `${((m.cfo || 0) / 1e3).toFixed(1)} / ${coarseCfo} kHz`;
    $('tper').textContent = lastViz && lastViz.pkt_len ? `${lastViz.pkt_len} B` : '-';
    $('fsent').textContent = `${m.tx_packets || 0}`;
    $('rxc').textContent = `${m.tun_up ? 'UP' : 'DOWN'} · ${shortMac(m.my_mac)}`;
    $('rxc').title = `${m.tun_up ? 'UP' : 'DOWN'} / ${m.my_mac || '-'}`;

    $('pblocks').textContent = `${m.ping_sent || 0} / ${m.ping_rcvd || 0}${pct(m.ping_rcvd || 0, m.ping_sent || 0)}`;
    $('p-rtt').textContent = m.ping_rcvd > 0
        ? `${Math.round(m.ping_min)} / ${Math.round(m.ping_avg)} / ${Math.round(m.ping_max)} ms`
        : '-';

    const nowSec = performance.now() / 1000;
    if (m.fcs_ok > 0 && isFinite(m.snr)) {
        snrHist.push([nowSec, m.snr]);
    }
    while (snrHist.length && snrHist[0][0] < nowSec - 60) snrHist.shift();

    gpHist.push([nowSec, gpRx, gpTx]);
    while (gpHist.length && gpHist[0][0] < nowSec - 60) gpHist.shift();

    $('sep-n').textContent = m.fcs_ok > 0 ? f1(m.snr) : '-';
    $('sep-w').textContent = m.fcs_ok > 0 ? f1(m.evm) : '-';
    drawChanSummary(nowSec);
}

function drawChanSummary(nowSec) {
    const hMag = lastViz && hasSubcarriers(lastViz.h_mag) ? lastViz.h_mag : null;
    const hPh = lastViz && hasSubcarriers(lastViz.h_phase) ? lastViz.h_phase : null;

    if (!st || !st.fcs_ok) {
        $('sep-note').textContent = 'WAITING FOR INCOMING 802.11 OFDM BURSTS';
    } else {
        const nRx = bits(st.rx_elements || 1).length;
        const nTx = bits(st.tx_elements || 1).length;
        const rk = String(parseInt(st.rate, 10) || 6);
        $('sep-note').textContent = `ACTIVE: ${nRx} RX CH / ${nTx} TX EL · ${MOD_CODING[rk] || ''}`;
    }

    // Subcarrier group strength bars in #sep-sv (4 quartiles of active subcarriers)
    let P = prep($('sep-sv'));
    if (P) {
        const { ctx, w, h } = P;
        ctx.font = '9px ui-monospace, Menlo, monospace';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = TICK;
        ctx.textAlign = 'left';
        ctx.fillText('SUBBAND GAIN', 0, 5);
        const groups = [
            { label: 'B0', idx: [-26, -14] },
            { label: 'B1', idx: [-13, -1] },
            { label: 'B2', idx: [1, 13] },
            { label: 'B3', idx: [14, 26] },
        ];
        const vals = hMag ? groups.map((g) => {
            let sum = 0, cnt = 0;
            for (let k = g.idx[0]; k <= g.idx[1]; k++) {
                const v = scVal(hMag, k);
                if (v != null && v > -90) { sum += v; cnt++; }
            }
            return cnt > 0 ? sum / cnt : -90;
        }) : [0, 0, 0, 0];
        const mx = hMag ? Math.max(...vals, -80) : 0;
        const rowh = Math.min(24, (h - 12) / 4);
        const L = 22, R = 44, bw = Math.max(16, w - L - R);
        const sbTxt = [];
        for (let i = 0; i < 4; i++) {
            const relDb = hMag ? (vals[i] - mx) : 0;
            const y = 13 + i * rowh;
            const frac = hMag ? Math.max(0, Math.min(1, 1 + relDb / 20)) : 0;
            ctx.fillStyle = TICK;
            ctx.textAlign = 'left';
            ctx.fillText(groups[i].label, 1, y + rowh / 2 - 1);
            ctx.strokeStyle = GRID;
            ctx.strokeRect(L + 0.5, y + 2.5, bw, rowh - 6);
            if (hMag) {
                ctx.fillStyle = CH[0];
                ctx.globalAlpha = 0.75;
                ctx.fillRect(L + 1, y + 3, (bw - 1) * frac, rowh - 7);
                ctx.globalAlpha = 1;
            }
            ctx.fillStyle = '#e8ece8';
            ctx.textAlign = 'right';
            ctx.fillText(hMag ? relDb.toFixed(1) + ' dB' : '-', w - 1, y + rowh / 2 - 1);
            if (hMag) sbTxt.push(relDb.toFixed(1));
        }
        if ($('el-sb')) $('el-sb').textContent = hMag ? sbTxt.join('/') + ' dB' : '-';
    }

    // Subband phase coherence matrix in #sep-ang
    P = prep($('sep-ang'));
    if (P) {
        const { ctx, w, h } = P;
        ctx.font = '9px ui-monospace, Menlo, monospace';
        ctx.fillStyle = TICK;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText('SUBBAND ∠H, °', 0, 5);
        const groups = [[-26, -14], [-13, -1], [1, 13], [14, 26]];
        const phs = hPh ? groups.map((g) => {
            let re = 0, im = 0;
            for (let k = g[0]; k <= g[1]; k++) {
                const rad = (scVal(hPh, k) || 0) * Math.PI / 180;
                re += Math.cos(rad);
                im += Math.sin(rad);
            }
            return Math.atan2(im, re) * 180 / Math.PI;
        }) : [0, 0, 0, 0];
        const n = 4;
        const L = 20, T = 22, cs = Math.min((w - L - 2) / n, (h - T - 2) / n);
        ctx.textAlign = 'center';
        for (let i = 0; i < n; i++) {
            ctx.fillStyle = TICK;
            ctx.fillText('B' + i, L + (i + 0.5) * cs, T - 6);
            ctx.fillText('B' + i, 9, T + (i + 0.5) * cs);
            for (let j = 0; j < n; j++) {
                const x = L + j * cs, y = T + i * cs;
                if (i === j || !hPh) {
                    ctx.strokeStyle = GRID;
                    ctx.strokeRect(x + 1.5, y + 1.5, cs - 3, cs - 3);
                    continue;
                }
                let diff = Math.abs(phs[i] - phs[j]);
                if (diff > 180) diff = 360 - diff;
                const rel = 1 - Math.min(1, diff / 180);
                ctx.fillStyle = `rgba(184,196,184,${0.06 + 0.6 * rel})`;
                ctx.fillRect(x + 1, y + 1, cs - 2, cs - 2);
                ctx.fillStyle = rel > 0.6 ? '#0c110c' : '#e8ece8';
                ctx.fillText(diff.toFixed(0) + '°', x + cs / 2, y + cs / 2);
            }
        }
    }

    const tNow = nowSec || (performance.now() / 1000);
    // SNR history over last 60s
    plot($('sep-hist'), {
        xr: [-60, 0], yr: [0, 35], xl: 's', yl: '', compact: true,
        series: [{ y: snrHist.map((e) => e[1]), x: snrHist.map((e) => e[0] - tNow), c: CH[0] }],
    });
    // Goodput history over last 60s
    if ($('gp-hist')) {
        const maxGp = Math.max(20, ...gpHist.map((e) => Math.max(e[1], e[2])));
        plot($('gp-hist'), {
            xr: [-60, 0], yr: [0, Math.ceil(maxGp / 10) * 10], xl: 's', yl: '', compact: true,
            series: [
                { y: gpHist.map((e) => e[1]), x: gpHist.map((e) => e[0] - tNow), c: CH[0] },
                { y: gpHist.map((e) => e[2]), x: gpHist.map((e) => e[0] - tNow), c: CH[2] },
            ],
        });
    }
}

// ------------------------------------------------------------------ chat

function onChat(msgs) {
    if (!msgs) return;
    const log = $('chat-log');
    const atEnd = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
    log.innerHTML = msgs.map((m) => {
        const who = m.mine ? 'LOCAL' : (m.who ? m.who.toUpperCase() : 'PEER');
        const stt = m.mine ? '<span class="st">SENT</span>' : '';
        return `<div class="m"><span class="who">${esc(who)}</span><span class="tx">${esc(m.text)}</span>${stt}</div>`;
    }).join('');
    if (atEnd) log.scrollTop = log.scrollHeight;
}

function sendChat() {
    const i = $('chat-in');
    const v = i.value.trim();
    if (!v) return;
    send({ t: 'chat', text: v });
    i.value = '';
}
$('chat-send').onclick = sendChat;
$('chat-in').addEventListener('keydown', (e) => { if (e.key === 'Enter') sendChat(); });

// ------------------------------------------------------------------ plots

function ticks(a, b, n) {
    const span = b - a;
    if (!(span > 0)) return [a];
    const raw = span / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 5, 10].map((m) => m * mag).find((s) => s >= raw) || raw;
    const out = [];
    for (let v = Math.ceil(a / step) * step; v <= b + step * 1e-6; v += step) out.push(Math.abs(v) < step * 1e-6 ? 0 : v);
    return out;
}
function tickTxt(v) {
    const a = Math.abs(v);
    if (a >= 10 || a === 0) return v.toFixed(0);
    if (a >= 1) return v.toFixed(1).replace(/\.0$/, '');
    return v.toFixed(2).replace(/0$/, '');
}

function prep(cv) {
    if (!cv) return null;
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (w < 4 || h < 4) return null;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
        cv.width = Math.round(w * dpr);
        cv.height = Math.round(h * dpr);
    }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
}

function plot(cv, o) {
    const P = prep(cv);
    if (!P) return;
    const { ctx, w, h } = P;
    const L = o.compact ? 22 : 36, R = 6, T = o.compact ? 4 : 14, B = o.compact ? 12 : 16;
    const pw = w - L - R, ph = h - T - B;
    const [x0, x1] = o.xr, [y0, y1] = o.yr;
    const X = (x) => L + (x - x0) / (x1 - x0) * pw;
    const Y = (y) => T + (1 - (y - y0) / (y1 - y0)) * ph;
    ctx.font = '9px ui-monospace, Menlo, monospace';
    ctx.lineWidth = 1;
    ctx.strokeStyle = GRID;
    ctx.fillStyle = TICK;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const xu = ctx.measureText(o.xl || '').width;
    for (const t of ticks(x0, x1, Math.max(3, Math.floor(pw / 60)))) {
        const x = Math.round(X(t)) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
        const lab = tickTxt(t);
        if (x + ctx.measureText(lab).width / 2 < w - xu - 8) ctx.fillText(lab, x, T + ph + 2);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const t of ticks(y0, y1, o.yn || Math.max(1, Math.floor(ph / 28)))) {
        const y = Math.round(Y(t)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke();
        ctx.fillText(tickTxt(t), L - 4, y);
    }
    ctx.strokeStyle = '#3a423a';
    ctx.strokeRect(L + 0.5, T + 0.5, pw, ph);
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(o.yl || '', 2, 0);
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.fillText(o.xl || '', w - 2, h);

    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    for (const l of o.hl || []) {
        ctx.strokeStyle = l.c || '#e6b45a';
        ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(L, Y(l.y)); ctx.lineTo(L + pw, Y(l.y)); ctx.stroke();
    }
    for (const l of o.vl || []) {
        ctx.strokeStyle = l.c || '#e6b45a';
        ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(X(l.x), T); ctx.lineTo(X(l.x), T + ph); ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const s of o.series) {
        const y = s.y;
        if (!y || !y.length) continue;
        ctx.strokeStyle = s.c;
        ctx.fillStyle = s.c;
        ctx.globalAlpha = s.a || 0.9;
        if (s.pts) {
            for (let i = 0; i < y.length; i++) {
                const xv = s.x ? s.x[i] : s.x0 + i * s.dx;
                if (y[i] == null || !isFinite(y[i])) continue;
                ctx.fillRect(X(xv) - 1, Y(y[i]) - 1, 2, 2);
            }
        } else {
            ctx.beginPath();
            let pen = false;
            let prevY = null;
            for (let i = 0; i < y.length; i++) {
                const xv = s.x ? s.x[i] : s.x0 + i * s.dx;
                if (y[i] == null || !isFinite(y[i])) { pen = false; prevY = null; continue; }
                if (s.wrapJump && prevY != null && Math.abs(y[i] - prevY) > s.wrapJump) {
                    pen = false;
                }
                if (!pen) { ctx.moveTo(X(xv), Y(y[i])); pen = true; } else ctx.lineTo(X(xv), Y(y[i]));
                prevY = y[i];
            }
            ctx.stroke();
        }
    }
    ctx.globalAlpha = 1;
    if (o.note) {
        ctx.font = '10px ui-monospace, Menlo, monospace';
        ctx.fillStyle = '#e6b45a';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(o.note, L + pw / 2, T + 4);
    }
    ctx.restore();
}

function constel(cv, pts, label, note) {
    const P = prep(cv);
    if (!P) return;
    const { ctx, w, h } = P;
    const L = 6, R = 6, T = 4, B = 4;
    const pw = w - L - R, ph = h - T - B;
    const s = Math.min(pw, ph), cx = L + pw / 2, cy = T + ph / 2, lim = 1.6;
    const Xp = (v) => cx + v / lim * s / 2, Yp = (v) => cy - v / lim * s / 2;
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(L, Math.round(cy) + 0.5); ctx.lineTo(L + pw, Math.round(cy) + 0.5);
    ctx.moveTo(Math.round(cx) + 0.5, T); ctx.lineTo(Math.round(cx) + 0.5, T + ph);
    ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, s / 2 / lim, 0, 2 * Math.PI); ctx.stroke();
    ctx.strokeStyle = '#3a423a';
    ctx.strokeRect(L + 0.5, T + 0.5, pw, ph);
    if (pts && pts.length >= 2) {
        ctx.fillStyle = 'rgba(184,196,184,0.8)';
        for (let i = 0; i + 1 < pts.length; i += 2) {
            ctx.fillRect(Xp(pts[i]) - 1, Yp(pts[i + 1]) - 1, 2, 2);
        }
    }
    ctx.font = '9px ui-monospace, Menlo, monospace';
    ctx.fillStyle = TICK;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    if (label) ctx.fillText(label, L + 5, T + 4);
    if (note) {
        ctx.font = '10px ui-monospace, Menlo, monospace';
        ctx.fillStyle = '#e6b45a';
        ctx.textAlign = 'center';
        ctx.fillText(note, cx, T + 6);
    }
}

function computeCirFromH(hMag, hPh) {
    if (!hasSubcarriers(hMag) || !hasSubcarriers(hPh)) return null;
    const Hre = new Float64Array(64);
    const Him = new Float64Array(64);
    for (let k = -26; k <= 26; k++) {
        if (k === 0) continue;
        const db = scVal(hMag, k);
        const deg = scVal(hPh, k);
        if (db == null || deg == null) continue;
        const mag = Math.pow(10, db / 20);
        const rad = deg * Math.PI / 180;
        const bin = (k + 64) & 63;
        Hre[bin] = mag * Math.cos(rad);
        Him[bin] = mag * Math.sin(rad);
    }
    // Interpolate DC bin from k = ±1
    Hre[0] = 0.5 * (Hre[1] + Hre[63]);
    Him[0] = 0.5 * (Him[1] + Him[63]);

    // 64-point IDFT over lag n in [-8..+23] (dt = 0.05 us at 20 MS/s)
    const lags = [];
    const pwr = [];
    let maxP = 1e-12;
    for (let n = -8; n <= 23; n++) {
        let re = 0, im = 0;
        for (let b = 0; b < 64; b++) {
            const ang = 2 * Math.PI * b * n / 64;
            const c = Math.cos(ang), s = Math.sin(ang);
            re += Hre[b] * c - Him[b] * s;
            im += Hre[b] * s + Him[b] * c;
        }
        const p = (re * re + im * im) / (64 * 64);
        lags.push(n * 0.05);  // us
        pwr.push(p);
        if (p > maxP) maxP = p;
    }

    // RMS delay spread inside the 16-tap window [-0.20..+0.60 us]
    let sumW = 0, sumT = 0, sumT2 = 0;
    for (let i = 0; i < lags.length; i++) {
        const tau = lags[i];
        if (tau < -0.201 || tau > 0.601) continue;
        const relP = pwr[i] / maxP;
        if (relP < 0.01) continue;  // -20 dB threshold
        sumW += pwr[i];
        sumT += pwr[i] * tau;
        sumT2 += pwr[i] * tau * tau;
    }
    const meanT = sumW > 0 ? sumT / sumW : 0;
    const rmsNs = sumW > 0 ? Math.sqrt(Math.max(0, sumT2 / sumW - meanT * meanT)) * 1000 : 0;
    const db = pwr.map((p) => Math.max(-45, 10 * Math.log10(p / maxP + 1e-9)));
    return { lags, db, rmsNs };
}

function onViz(m) {
    if (m) lastViz = m;
    const v = lastViz || {};
    const waitNote = (!v.h_mag || !v.h_mag.length) ? 'WAITING FOR BURST' : '';

    // 1. ADC Spectrum (64 bins across ±10 MHz)
    if (v.adc_psd && v.adc_psd.length) {
        const n = v.adc_psd.length;
        const fsM = 20.0;
        const pk = Math.max(...v.adc_psd);
        plot($('p-adc'), {
            xr: [-fsM / 2, fsM / 2], yr: [-75, 5], xl: 'MHz from LO, fs 20.0 MS/s', yl: 'dBFS',
            series: [{ y: v.adc_psd, x0: -fsM / 2 + fsM / n / 2, dx: fsM / n, c: CH[0] }],
            vl: [{ x: 0, c: '#3a423a' }],
        });
        $('r-adc').textContent = `PEAK ${pk.toFixed(1)} dBFS`;
    } else {
        plot($('p-adc'), { xr: [-10, 10], yr: [-75, 5], xl: 'MHz from LO, fs 20.0 MS/s', yl: 'dBFS', series: [] });
    }

    // 2. ADC I/Q Time-domain
    if (v.adc_iq && v.adc_iq.length >= 2) {
        const I = [], Q = [];
        let pk = 0;
        for (let i = 0; i + 1 < v.adc_iq.length; i += 2) {
            const iv = v.adc_iq[i], qv = v.adc_iq[i + 1];
            I.push(iv); Q.push(qv);
            pk = Math.max(pk, Math.abs(iv), Math.abs(qv));
        }
        const dtu = 1.0 / 20.0; // 20 MS/s -> 0.05 us/sample
        plot($('p-iq'), {
            xr: [0, I.length * dtu], yr: [-48, 48], xl: 'µs', yl: 'counts', yn: 6,
            series: [{ y: I, x0: 0, dx: dtu, c: CH[0] }, { y: Q, x0: 0, dx: dtu, c: '#e8ece8', a: 0.5 }],
            hl: [{ y: CLIP, c: '#d05545' }, { y: -CLIP, c: '#d05545' }],
        });
        $('r-iq').textContent = `I/Q PEAK ${pk.toFixed(0)} CTS`;
    } else {
        plot($('p-iq'), { xr: [0, 10], yr: [-48, 48], xl: 'µs', yl: 'counts', yn: 6, series: [] });
    }

    // 3. Preamble I/Q & LTF sync
    if (v.sts_iq && v.sts_iq.length >= 2) {
        const I = [], Q = [];
        for (let i = 0; i + 1 < v.sts_iq.length; i += 2) {
            I.push(v.sts_iq[i]); Q.push(v.sts_iq[i + 1]);
        }
        const dtu = 1.0 / 20.0;
        plot($('p-sts'), {
            xr: [0, I.length * dtu], yr: [-0.45, 0.45], xl: 'µs from burst start', yl: 'norm',
            series: [{ y: I, x0: 0, dx: dtu, c: CH[0] }, { y: Q, x0: 0, dx: dtu, c: CH[2], a: 0.65 }],
            vl: [{ x: 8.0, c: '#e6b45a' }, { x: 16.0, c: '#8fa7c4' }],
        });
        const ltfP = v.ltf_peak != null ? v.ltf_peak.toFixed(2) : '-';
        $('r-sts').textContent = `LTF CORR ${ltfP}`;
    } else {
        plot($('p-sts'), { xr: [0, 12.8], yr: [-0.45, 0.45], xl: 'µs from burst start', yl: 'norm', note: waitNote, series: [] });
    }

    // 4. Channel Magnitude |H(f)| across active subcarriers -26..+26
    if (hasSubcarriers(v.h_mag)) {
        const xs = [], ys = [];
        let hi = -100, lo = 100;
        for (let k = -26; k <= 26; k++) {
            xs.push(k);
            if (k === 0) {
                ys.push(null);
            } else {
                const val = scVal(v.h_mag, k);
                ys.push(val);
                if (val != null && val > -90) { hi = Math.max(hi, val); lo = Math.min(lo, val); }
            }
        }
        const yMin = isFinite(lo) ? Math.floor((lo - 4) / 5) * 5 : -45;
        const yMax = isFinite(hi) ? Math.ceil((hi + 4) / 5) * 5 : 0;
        plot($('p-hmag'), {
            xr: [-28, 28], yr: [yMin, Math.max(yMin + 15, yMax)], xl: 'subcarrier k', yl: 'dB',
            series: [{ y: ys, x: xs, c: CH[0] }],
            vl: [{ x: 0, c: '#3a423a' }],
        });
        const span = (isFinite(hi) && isFinite(lo)) ? (hi - lo) : null;
        $('r-hmag').textContent = span != null ? `SPAN ${span.toFixed(1)} dB` : '';
        $('cond').textContent = span != null ? `${span.toFixed(1)} dB` : '-';
        $('model').textContent = `${v.ltf_peak != null ? v.ltf_peak.toFixed(2) : '-'} / ${st && st.fcs_ok ? f1(st.evm) + ' dB' : '-'}`;
    } else {
        plot($('p-hmag'), { xr: [-28, 28], yr: [-35, 0], xl: 'subcarrier k', yl: 'dB', note: waitNote, series: [] });
    }

    // 5. Channel Phase ∠H(f)
    if (hasSubcarriers(v.h_phase)) {
        const xs = [], ys = [];
        for (let k = -26; k <= 26; k++) {
            xs.push(k);
            ys.push(k === 0 ? null : scVal(v.h_phase, k));
        }
        plot($('p-hph'), {
            xr: [-28, 28], yr: [-180, 180], xl: 'subcarrier k', yl: 'deg', yn: 4,
            series: [{ y: ys, x: xs, c: CH[2], wrapJump: 180 }],
            hl: [{ y: 0, c: '#3a423a' }],
            vl: [{ x: 0, c: '#3a423a' }],
        });
        $('r-hph').textContent = '52 SUBCARRIERS';
    } else {
        plot($('p-hph'), { xr: [-28, 28], yr: [-180, 180], xl: 'subcarrier k', yl: 'deg', yn: 4, note: waitNote, series: [] });
    }

    // 6. Channel Impulse Response |h(τ)|
    const cir = computeCirFromH(v.h_mag, v.h_phase);
    if (cir) {
        plot($('p-cir'), {
            xr: [-0.4, 1.15], yr: [-40, 2], xl: 'delay τ (µs)', yl: 'dB',
            series: [{ y: cir.db, x: cir.lags, c: CH[1] }],
            vl: [{ x: -0.20, c: '#8fa7c4' }, { x: 0.60, c: '#8fa7c4' }],
        });
        $('r-cir').textContent = `τ_RMS ${cir.rmsNs.toFixed(0)} ns`;
        if ($('el-tau')) $('el-tau').textContent = `${cir.rmsNs.toFixed(0)} ns / 800 ns`;
    } else {
        plot($('p-cir'), { xr: [-0.4, 1.15], yr: [-40, 2], xl: 'delay τ (µs)', yl: 'dB', note: waitNote, series: [] });
    }

    // 7. Pilot Phase Tracking (CPE in degrees)
    if (v.phase_cpe && v.phase_cpe.length) {
        const deg = v.phase_cpe.map((r) => r * 180 / Math.PI);
        const n = deg.length;
        const mn = Math.min(...deg), mx = Math.max(...deg);
        const pad = Math.max(8, (mx - mn) * 0.15);
        let sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (let i = 0; i < n; i++) { sx += i; sy += deg[i]; sxx += i * i; sxy += i * deg[i]; }
        const slope = (n * sxy - sx * sy) / Math.max(n * sxx - sx * sx, 1);
        const hz = slope / 360 / 4.0e-6;  // 4 us per OFDM symbol
        plot($('p-cpe'), {
            xr: [0, Math.max(8, n - 1)], yr: [mn - pad, mx + pad], xl: 'OFDM symbol #', yl: 'deg',
            series: [{ y: deg, x0: 0, dx: 1, c: CH[0] }],
            hl: [{ y: 0, c: '#3a423a' }],
        });
        $('r-cpe').textContent = `SPAN ${(mx - mn).toFixed(1)}°  RES ${hz >= 0 ? '+' : ''}${hz.toFixed(0)} Hz`;
    } else {
        plot($('p-cpe'), { xr: [0, 12], yr: [-45, 45], xl: 'OFDM symbol #', yl: 'deg', note: waitNote, series: [] });
    }

    // 8. Equalized Constellation
    if (v.constel && v.constel.length >= 2) {
        const rateStr = st ? String(parseInt(st.rate, 10) || 6) : '6';
        const modName = MOD_CODING[rateStr] || (rateStr + ' Mb/s');
        const evmTxt = st && st.fcs_ok ? `  EVM ${f1(st.evm)} dB` : '';
        constel($('p-cons'), v.constel, `${modName}${evmTxt}`);
        $('r-cons').textContent = `${v.constel.length / 2} SYMS`;
    } else {
        constel($('p-cons'), [], 'CONSTELLATION', waitNote);
    }

    drawChanSummary();
}

buildSettings();
syncSettings();
onViz(null);
window.addEventListener('resize', () => {
    drawChanSummary();
    onViz(lastViz);
});
connect();
