#include <iostream>
#include <vector>
#include <atomic>
#include <cstring>
#include <csignal>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <cmath>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <sys/stat.h>

#include <zmq.h>
#include "csi_uapi.hpp"
#include "NEON.hpp"
#include "Farrow.hpp"

static volatile int keep_running = 1;
void sig_handler(int) { keep_running = 0; }

// --- Lock-Free Memory Pool ---
class LockFreeBufferPool {
private:
    size_t bufferSize_;
    size_t poolSize_;
    void* memorySlab_;
    std::vector<void*> freeNodes_;
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
    size_t mask_;

public:
    LockFreeBufferPool(size_t numBuffers, size_t bufferSizeBytes) : bufferSize_(bufferSizeBytes) {
        poolSize_ = 1;
        while (poolSize_ < numBuffers) poolSize_ <<= 1;
        mask_ = poolSize_ - 1;
        freeNodes_.resize(poolSize_, nullptr);

        if (posix_memalign(&memorySlab_, 64, poolSize_ * bufferSize_) != 0) {
            throw std::bad_alloc();
        }

        for (size_t i = 0; i < poolSize_; ++i) {
            freeNodes_[i] = static_cast<char*>(memorySlab_) + (i * bufferSize_);
        }
        tail_.store(poolSize_, std::memory_order_release);
    }
    ~LockFreeBufferPool() { free(memorySlab_); }

    void* pop() {
        size_t currentHead = head_.load(std::memory_order_relaxed);
        if (currentHead == tail_.load(std::memory_order_acquire)) return nullptr; 
        void* ptr = freeNodes_[currentHead & mask_];
        head_.store(currentHead + 1, std::memory_order_release);
        return ptr;
    }

    void push(void* ptr) {
        size_t currentTail = tail_.fetch_add(1, std::memory_order_acq_rel);
        freeNodes_[currentTail & mask_] = ptr;
    }
};

void zmq_pool_free(void *data, void *hint) {
    static_cast<LockFreeBufferPool*>(hint)->push(data);
}

// --- RX Controller ---
class CsiHardwareController {
private:
    int fdRx_ = -1;
    void* rxRing_ = nullptr;
    size_t rxRingSize_ = 0;
    size_t rxMapLen_ = 0;

    size_t page_align(size_t n) {
        size_t page = sysconf(_SC_PAGESIZE);
        return (n + page - 1) & ~(page - 1);
    }
    int xpoll(int timeoutMs) {
        struct pollfd pfd{fdRx_, POLLIN, 0};
        int rc = ::poll(&pfd, 1, timeoutMs);
        return (rc <= 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) ? -1 : rc;
    }

public:
     CsiHardwareController(const char* devPath, uint32_t bpl, uint32_t lines) {
        fdRx_ = ::open(devPath, O_RDONLY | O_NONBLOCK);
        if (fdRx_ < 0) {
            throw std::runtime_error("Failed to open CSI: " + std::string(strerror(errno)));
        }

        // NOTE: Removed RESET, SET_GEOMETRY, and SET_FILTER IOCTLs.
        // We defer to the Raspberry Pi 5 Device Tree/v4l2 to configure the ECP5 link.

        csi_ring_info ri{};
        if (::ioctl(fdRx_, CSI_IOC_GET_RING_INFO, &ri) != 0) {
            throw std::runtime_error("CSI GET_RING_INFO failed: " + std::string(strerror(errno)));
        }
        
        rxRingSize_ = ri.ring_size;
        rxMapLen_ = page_align(rxRingSize_);
        rxRing_ = ::mmap(nullptr, rxMapLen_, PROT_READ, MAP_SHARED, fdRx_, 0);
        
        if (rxRing_ == MAP_FAILED) {
            throw std::runtime_error("Failed to mmap CSI ring buffer: " + std::string(strerror(errno)));
        }
        
        std::cout << "Hardware Link Initialized. Ring size: " << rxRingSize_ << " bytes." << std::endl;
    }
    ~CsiHardwareController() {
        if (rxRing_ && rxRing_ != MAP_FAILED) ::munmap(rxRing_, rxMapLen_);
        if (fdRx_ >= 0) ::close(fdRx_);
    }

    struct ChunkView {
        const int8_t* p1 = nullptr;
        size_t n1 = 0; // bytes
        const int8_t* p2 = nullptr;
        size_t n2 = 0; // bytes
        size_t total() const { return n1 + n2; }
    };

    // Wrap-aware, zero-copy view into the mmap ring.
    // Consumes bytes immediately (same semantics as pullChunk()).
    ssize_t pullChunkView(ChunkView& v, size_t maxBytes, long timeoutUs)
    {
        v = ChunkView{};

        if (xpoll((int)(timeoutUs / 1000)) <= 0) return -EAGAIN;

        csi_ring_info ri{};
        if (ioctl(fdRx_, CSI_IOC_GET_RING_INFO, &ri) != 0) return -EIO;

        // bytes available
        size_t used = (ri.head >= ri.tail)
            ? (ri.head - ri.tail)
            : (rxRingSize_ - (ri.tail - ri.head));

        if (!used) return -EAGAIN;

        size_t want = std::min(used, maxBytes);

        // Ensure even number of bytes (CS8 IQ is 2 bytes per complex)
        want &= ~size_t(1);
        if (want == 0) return -EAGAIN;

        const size_t tail = (size_t)ri.tail;

        // segment 1: tail -> end
        size_t seg1 = std::min(want, rxRingSize_ - tail);
        seg1 &= ~size_t(1);
        v.p1 = reinterpret_cast<const int8_t*>(rxRing_) + tail;
        v.n1 = seg1;

        // segment 2: wrap
        size_t rem = want - seg1;
        rem &= ~size_t(1);
        if (rem) {
            v.p2 = reinterpret_cast<const int8_t*>(rxRing_);
            v.n2 = rem;
        }

        size_t total = v.n1 + v.n2;
        if (total == 0) return -EAGAIN;

        __u32 cons = (__u32)total;
        if (ioctl(fdRx_, CSI_IOC_CONSUME_BYTES, &cons) != 0) return -EIO;

        return (ssize_t)total;
    }

    ssize_t pullChunk(int8_t* dst, size_t bytes, long timeoutUs) {
        if (xpoll(timeoutUs / 1000) <= 0) return -EAGAIN;
        csi_ring_info ri{};
        if (ioctl(fdRx_, CSI_IOC_GET_RING_INFO, &ri) != 0) return -EIO;

        size_t used = (ri.head >= ri.tail) ? (ri.head - ri.tail) : (rxRingSize_ - (ri.tail - ri.head));
        if (!used) return -EAGAIN;

        size_t want = std::min(std::min(used, rxRingSize_ - ri.tail), bytes);
        std::memcpy(dst, static_cast<const uint8_t*>(rxRing_) + ri.tail, want);

        __u32 cons = (__u32)want;
        ioctl(fdRx_, CSI_IOC_CONSUME_BYTES, &cons);
        return (ssize_t)want;
    }
};


static inline void convert_CS8_to_CF32_NEON_2seg(
    const int8_t* p1, size_t n1_bytes,
    const int8_t* p2, size_t n2_bytes,
    float* out_cf32)
{
    // CS8 IQ interleaved: 2 bytes per complex sample
    const size_t n1_elems = n1_bytes / 2;
    const size_t n2_elems = n2_bytes / 2;

    if (n1_elems) {
        convert_CS8_to_CF32_NEON(p1, out_cf32, n1_elems);
    }
    if (n2_elems) {
        convert_CS8_to_CF32_NEON(p2, out_cf32 + (n1_elems * 2), n2_elems);
    }
}

// --- TX Controller ---
class DsiHardwareController {
private:
    int fdTx_ = -1;
    void* txStaging_ = nullptr;
    size_t txFbBytes_ = 0;
    uint32_t txFbCount_ = 0;
    size_t txMapLen_ = 0;
    size_t txHeadIndex_ = 0;
    size_t txHeadOff_ = 0;

    size_t page_align(size_t n) {
        size_t page = sysconf(_SC_PAGESIZE);
        return (n + page - 1) & ~(page - 1);
    }

public:
    DsiHardwareController(const char* devPath) {
        fdTx_ = ::open(devPath, O_RDWR | O_NONBLOCK);
        if (fdTx_ < 0) throw std::runtime_error("Failed to open DSI");

        dsi_fb_info info{};
        if (::ioctl(fdTx_, DSI_IOC_GET_FB_INFO, &info) == 0 && info.fb_bytes && info.fb_count) {
            txFbBytes_ = info.fb_bytes;
            txFbCount_ = info.fb_count;
        } else throw std::runtime_error("DSI GET_FB_INFO failed");

        txMapLen_ = page_align(txFbBytes_ * txFbCount_);
        txStaging_ = ::mmap(nullptr, txMapLen_, PROT_WRITE, MAP_SHARED, fdTx_, 0);
    }
    ~DsiHardwareController() {
        if (txStaging_ && txStaging_ != MAP_FAILED) ::munmap(txStaging_, txMapLen_);
        if (fdTx_ >= 0) ::close(fdTx_);
    }

    ssize_t pushChunk(const int8_t* src, size_t bytes) {
        const uint8_t *p = reinterpret_cast<const uint8_t*>(src);
        size_t remaining = bytes, written = 0;

        while (remaining) {
            if (txHeadOff_ == 0) {
                dsi_fb_info info{};
                if (::ioctl(fdTx_, DSI_IOC_GET_FB_INFO, &info) == 0) txHeadIndex_ = info.head;
            }

            size_t toCopy = std::min(remaining, txFbBytes_ - txHeadOff_);
            uint8_t *dst = static_cast<uint8_t*>(txStaging_) + (txHeadIndex_ % txFbCount_) * txFbBytes_ + txHeadOff_;
            
            std::memcpy(dst, p, toCopy);
            p += toCopy; remaining -= toCopy; written += toCopy; txHeadOff_ += toCopy;

            if (txHeadOff_ == txFbBytes_) {
                if (::ioctl(fdTx_, DSI_IOC_QUEUE_NEXT) != 0) return written ? written : -EIO;
                txHeadIndex_ = (txHeadIndex_ + 1) % txFbCount_;
                txHeadOff_ = 0;
            }
        }
        return written;
    }
};

// --- TX Thread ---
void tx_thread_func(void* zmq_ctx) {
    void *tx_puller = zmq_socket(zmq_ctx, ZMQ_PULL);
    zmq_bind(tx_puller, "tcp://*:5556");

    DsiHardwareController hw_tx("/dev/dsi_stream0");

    double hostRate = 40.0e6;
    double txLineRate = 80.95076e6;
    DSP::FarrowResampler txResampler;
    double txSampleRateRatio = hostRate / txLineRate;

    if (std::abs(hostRate - txLineRate) > 1.0e3 && hostRate > 0.0) {
        txResampler.setEnabled(true);
    }

    std::vector<float> txInFloat;
    size_t txInOffPairs = 0;
    const size_t MAX_TX_OUT_SAMPLES = 8192;
    std::vector<float> cf32_out_buffer(MAX_TX_OUT_SAMPLES * 2);
    std::vector<int8_t> cs8_out_buffer(MAX_TX_OUT_SAMPLES * 2);

    int linger = 0;
    zmq_setsockopt(tx_puller, ZMQ_LINGER, &linger, sizeof(linger));

    while (keep_running) {
        // --- Poll with 100ms timeout so thread can exit on Ctrl+C ---
        zmq_pollitem_t items[] = { { tx_puller, 0, ZMQ_POLLIN, 0 } };
        if (zmq_poll(items, 1, 100) <= 0) {
            continue; // Timeout, loop around and check keep_running
        }

        zmq_msg_t msg;
        zmq_msg_init(&msg);
        if (zmq_msg_recv(&msg, tx_puller, ZMQ_DONTWAIT) == -1) { 
            zmq_msg_close(&msg); 
            continue; 
        }

        float* inbound_cf32 = static_cast<float*>(zmq_msg_data(&msg));
        size_t num_samples = zmq_msg_size(&msg) / (2 * sizeof(float));

        if (num_samples > 0) {
            if (txResampler.isEnabled()) {
                size_t currentSize = txInFloat.size();
                txInFloat.resize(currentSize + (num_samples * 2));
                std::memcpy(txInFloat.data() + currentSize, inbound_cf32, num_samples * 2 * sizeof(float));

                for (;;) {
                    size_t activePairs = (txInFloat.size() / 2) > txInOffPairs ? (txInFloat.size() / 2) - txInOffPairs : 0;
                    if (activePairs < 4) break; 

                    int inConsumed = 0;
                    int produced = txResampler.processConsume(
                        txInFloat.data() + (txInOffPairs * 2), activePairs, 
                        cf32_out_buffer.data(), MAX_TX_OUT_SAMPLES, txSampleRateRatio, inConsumed
                    );
                    if (produced <= 0) break;

                    convert_CF32_to_CS8_NEON(cf32_out_buffer.data(), cs8_out_buffer.data(), produced);
                    hw_tx.pushChunk(cs8_out_buffer.data(), produced * 2);
                    txInOffPairs += inConsumed;
                }

                size_t remainPairs = (txInFloat.size() / 2) > txInOffPairs ? (txInFloat.size() / 2) - txInOffPairs : 0;
                if (txInOffPairs > 4096 && (txInOffPairs > remainPairs + 1024)) {
                    if (remainPairs > 0) std::memmove(txInFloat.data(), txInFloat.data() + (txInOffPairs * 2), remainPairs * 2 * sizeof(float));
                    txInFloat.resize(remainPairs * 2);
                    txInOffPairs = 0;
                }
            } else {
                size_t samples_processed = 0;
                while (samples_processed < num_samples) {
                    size_t chunk = std::min(num_samples - samples_processed, MAX_TX_OUT_SAMPLES);
                    convert_CF32_to_CS8_NEON(inbound_cf32 + (samples_processed * 2), cs8_out_buffer.data(), chunk);
                    hw_tx.pushChunk(cs8_out_buffer.data(), chunk * 2);
                    samples_processed += chunk;
                }
            }
        }
        zmq_msg_close(&msg);
    }
    zmq_close(tx_puller);
}

// --- Main Daemon ---
int main(int argc, char* argv[]) {
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    // ---------------- ZMQ PUB (RX stream) ----------------
    void* zmq_ctx = zmq_ctx_new();
    if (!zmq_ctx) {
        std::cerr << "[Daemon] FATAL: zmq_ctx_new failed";
        return 1;
    }

    void* publisher = zmq_socket(zmq_ctx, ZMQ_PUB);
    if (!publisher) {
        std::cerr << "[Daemon] FATAL: zmq_socket(PUB) failed: " << zmq_strerror(zmq_errno()) << "";
        zmq_ctx_term(zmq_ctx);
        return 1;
    }

    // Stream tuning: prefer low-latency + drop under load.
    int sndhwm = 2;
    zmq_setsockopt(publisher, ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));

    int immediate = 1;
    zmq_setsockopt(publisher, ZMQ_IMMEDIATE, &immediate, sizeof(immediate));

    int linger = 0;
    zmq_setsockopt(publisher, ZMQ_LINGER, &linger, sizeof(linger));

    #ifdef ZMQ_TCP_NODELAY
    int tcp_nodelay = 1;
    zmq_setsockopt(publisher, ZMQ_TCP_NODELAY, &tcp_nodelay, sizeof(tcp_nodelay));
    #endif

    // Bind to TCP for remote and (optional) abstract-namespace IPC for local.
    // Abstract IPC avoids filesystem permissions and does not require sudo.
    // NOTE: ZMQ allows binding a single socket to multiple endpoints.
    if (zmq_bind(publisher, "tcp://*:5555") != 0) {
        std::cerr << "[Daemon] FATAL: zmq_bind tcp://*:5555 failed: " << zmq_strerror(zmq_errno()) << "";
        zmq_close(publisher);
        zmq_ctx_term(zmq_ctx);
        return 1;
    }
    // Local abstract IPC endpoint (no file created):
    // clients connect with: ipc://@mipi_sdr_cf32
    if (zmq_bind(publisher, "ipc://@mipi_sdr_cf32") != 0) {
        std::cerr << "[Daemon] WARN: zmq_bind ipc://@mipi_sdr_cf32 failed: " << zmq_strerror(zmq_errno())
                  << " (continuing with TCP only)";
    }

    // ---------------- ZMQ REP (RPC) ----------------
    void* rpc_server = zmq_socket(zmq_ctx, ZMQ_REP);
    if (!rpc_server) {
        std::cerr << "[Daemon] FATAL: zmq_socket(REP) failed: " << zmq_strerror(zmq_errno()) << "";
        zmq_close(publisher);
        zmq_ctx_term(zmq_ctx);
        return 1;
    }
    zmq_setsockopt(rpc_server, ZMQ_LINGER, &linger, sizeof(linger));

    if (zmq_bind(rpc_server, "tcp://*:5557") != 0) {
        std::cerr << "[Daemon] FATAL: zmq_bind tcp://*:5557 failed: " << zmq_strerror(zmq_errno()) << "";
        zmq_close(rpc_server);
        zmq_close(publisher);
        zmq_ctx_term(zmq_ctx);
        return 1;
    }

    // ---------------- UDP (optional) ----------------
    int udp_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        std::cerr << "[Daemon] WARN: UDP socket() failed: " << strerror(errno) << " (continuing without UDP)";
    } else {
        int flags = fcntl(udp_sock, F_GETFL, 0);
        if (flags >= 0) fcntl(udp_sock, F_SETFL, flags | O_NONBLOCK);
    }

    struct sockaddr_in target_addr;
    std::memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(6666);
    inet_pton(AF_INET, "192.168.1.100", &target_addr.sin_addr);

    // ---------------- Rates / Resampler ----------------
    double target_rate = (argc > 1) ? std::stod(argv[1]) : 80.95076e6;
    const double kFsIn = 80.95076e6;
    double sampleRateRatio = kFsIn / target_rate;

    DSP::FarrowResampler rxResampler;
    rxResampler.setEnabled(std::abs(target_rate - kFsIn) > 1.0e3 && target_rate > 0.0);

    // ---------------- Chunk sizing ----------------
    // IMPORTANT PERFORMANCE NOTE:
    // With 8192, at ~80.95 Msps, message rate is ~9880 msgs/sec and ZMQ/TCP overhead dominates.
    // Raise chunk size to cut msgs/sec and CPU. You can override via argv[2].
    size_t MAX_SAMPLES_PER_CHUNK = 65536;
    if (argc > 2) {
        MAX_SAMPLES_PER_CHUNK = std::max<size_t>(4096, (size_t)std::stoul(argv[2]));
    }

    // ---------------- Hardware RX ----------------
    CsiHardwareController hw("/dev/csi_stream0", 1024, 1024);

    // Pool holds CF32 frames (IQ interleaved): 2 floats per complex sample
    // Increase pool count moderately to tolerate subscriber jitter.
    LockFreeBufferPool bufferPool(64, MAX_SAMPLES_PER_CHUNK * 2 * sizeof(float));

    // Scratch only needed when resampler enabled (but can be enabled by RPC later)
    float* scratch_cf32_buffer = (float*)std::aligned_alloc(64, MAX_SAMPLES_PER_CHUNK * 2 * sizeof(float));
    if (!scratch_cf32_buffer) {
        std::cerr << "[Daemon] FATAL: scratch_cf32_buffer alloc failed";
        if (udp_sock >= 0) ::close(udp_sock);
        zmq_close(rpc_server);
        zmq_close(publisher);
        zmq_ctx_term(zmq_ctx);
        return 1;
    }

    bool four_channel_mode = false;

    // Launch TX Thread
    std::thread tx_worker(tx_thread_func, zmq_ctx);

    int empty_polls = 0;
    uint64_t chunks_pushed = 0;
    int loop_counter = 0;

    while (keep_running) {
        // ---------------- RPC Check ----------------
        // Only check the RPC socket once every ~100 loops to reduce overhead.
        if (++loop_counter % 100 == 0) {
            char rpc_buf[128];
            int rc = zmq_recv(rpc_server, rpc_buf, sizeof(rpc_buf) - 1, ZMQ_DONTWAIT);
            if (rc > 0) {
                rpc_buf[rc] = ' ';
                std::string cmd(rpc_buf);

                try {
                    if (cmd.rfind("RATE ", 0) == 0) {
                        target_rate = std::stod(cmd.substr(5));
                        sampleRateRatio = kFsIn / target_rate;

                        rxResampler.setEnabled(std::abs(target_rate - kFsIn) > 1.0e3 && target_rate > 0.0);
                        zmq_send(rpc_server, "OK", 2, 0);
                    }
                    else if (cmd == "MODE 4CH") {
                        four_channel_mode = true;
                        zmq_send(rpc_server, "OK", 2, 0);
                    }
                    else if (cmd == "MODE 2CH") {
                        four_channel_mode = false;
                        zmq_send(rpc_server, "OK", 2, 0);
                    }
                    else if (cmd.rfind("GAIN ", 0) == 0) {
                        double gain = std::stod(cmd.substr(5));
                        std::cout << "[Daemon] RPC: Setting Gain to " << gain << " dB";
                        zmq_send(rpc_server, "OK", 2, 0);
                    }
                    else if (cmd.rfind("FREQ ", 0) == 0) {
                        double freq = std::stod(cmd.substr(5));
                        std::cout << "[Daemon] RPC: Setting Frequency to " << freq << " Hz";
                        zmq_send(rpc_server, "OK", 2, 0);
                    }
                    else if (cmd.rfind("ANTENNA ", 0) == 0) {
                        std::string ant = cmd.substr(8);
                        std::cout << "[Daemon] RPC: Switching Antenna to " << ant << "";
                        zmq_send(rpc_server, "OK", 2, 0);
                    }
                    else if (cmd.rfind("SET bypass_iir ", 0) == 0) {
                        std::string val = cmd.substr(15);
                        bool bypass = (val == "true");
                        std::cout << "[Daemon] RPC: Setting Hardware IIR Bypass to " << (bypass ? "true" : "false") << "";
                        zmq_send(rpc_server, "OK", 2, 0);
                    }
                    else {
                        std::cerr << "[Daemon] RPC: Unknown command '" << cmd << "'";
                        zmq_send(rpc_server, "ERR", 3, 0);
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "[Daemon] RPC Parsing Error: " << e.what() << " for command: " << cmd << "";
                    zmq_send(rpc_server, "ERR", 3, 0);
                }
            }
        }

        // ---------------- RX Data Path (zero-copy view into mmap ring) ----------------
        CsiHardwareController::ChunkView view;
        ssize_t gotBytes = hw.pullChunkView(view, MAX_SAMPLES_PER_CHUNK * 2, 100000);

        if (gotBytes <= 0) {
            empty_polls++;
            if (empty_polls % 50 == 0) {
                std::cout << "[Daemon] Waiting for FPGA data... (poll timeout)";
            }
            continue;
        }
        empty_polls = 0;

        // Ensure even byte count (CS8 IQ is 2 bytes/sample)
        gotBytes &= ~ssize_t(1);

        const size_t samples_received = (size_t)gotBytes / 2;

        float* final_cf32_buffer = static_cast<float*>(bufferPool.pop());
        if (!final_cf32_buffer) {
            std::cerr << "[Daemon] OVERFLOW: Dropping frame (pool empty)";
            continue;
        }

        size_t bytes_to_send = 0;

        if (four_channel_mode) {
            // TODO: 4-channel phased array path
            bufferPool.push(final_cf32_buffer);
            continue;
        }

        if (!rxResampler.isEnabled()) {
            // FAST PATH:
            // Convert directly from mmap ring segments into pool buffer.
            convert_CS8_to_CF32_NEON_2seg(view.p1, view.n1, view.p2, view.n2, final_cf32_buffer);
            bytes_to_send = samples_received * 2 * sizeof(float);
        } else {
            // Resampler path:
            convert_CS8_to_CF32_NEON_2seg(view.p1, view.n1, view.p2, view.n2, scratch_cf32_buffer);

            int produced = rxResampler.process(
                scratch_cf32_buffer,
                (int)samples_received,
                final_cf32_buffer,
                (int)MAX_SAMPLES_PER_CHUNK,
                sampleRateRatio
            );
            if (produced > 0) {
                bytes_to_send = (size_t)produced * 2 * sizeof(float);
            } else {
                bytes_to_send = 0;
            }
        }

        if (bytes_to_send > 0) {
            // UDP best-effort (kernel will copy)
            if (udp_sock >= 0) {
                (void)::sendto(
                    udp_sock,
                    final_cf32_buffer,
                    bytes_to_send,
                    MSG_DONTWAIT,
                    (struct sockaddr*)&target_addr,
                    sizeof(target_addr)
                );
            }

            // ZMQ zero-copy: buffer returned to pool by zmq_pool_free callback.
            zmq_msg_t msg;
            zmq_msg_init_data(&msg, final_cf32_buffer, bytes_to_send, zmq_pool_free, &bufferPool);

            if (zmq_msg_send(&msg, publisher, ZMQ_DONTWAIT) == -1) {
                // On failure, close to trigger free callback immediately.
                zmq_msg_close(&msg);
            } else {
                zmq_msg_close(&msg);
            }

            chunks_pushed++;
            if ((chunks_pushed % 1000) == 0) {
                std::cout << "[Daemon] Pushed " << chunks_pushed
                          << " chunks, chunk_elems=" << samples_received
                          << " resamp=" << (rxResampler.isEnabled() ? "on" : "off") << "";
            }
        } else {
            // return unused buffer
            bufferPool.push(final_cf32_buffer);
        }
    }

    tx_worker.join();

    std::free(scratch_cf32_buffer);
    if (udp_sock >= 0) ::close(udp_sock);

    zmq_close(rpc_server);
    zmq_close(publisher);
    zmq_ctx_destroy(zmq_ctx);

    return 0;
}

