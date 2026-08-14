// g++ -std=c++17 rx_test.cpp -o rx_test -lSoapySDR
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <csignal>
#include <cmath>
#include <getopt.h>
#include <fstream>
#include <atomic>
#include <algorithm>

// Global flag to handle Ctrl+C cleanly
std::atomic<bool> running(true);

void sigIntHandler(int) {
    running = false;
}

void print_usage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [options]\n"
              << "Options:\n"
              << "  -r, --rate <hz>     Set sample rate (default: 40000000)\n"
              << "  -c, --channels <n>  Number of channels (default: 1, max: 4)\n"
              << "  -f, --file <path>   Output file path (default: none/discard)\n"
              << "  -h, --help          Show this help message\n";
}

int main(int argc, char** argv)
{
    // Defaults
    double sampleRate = 40e6;
    size_t numChannels = 1;
    std::string outputFilename = "";

    // Parse arguments
    int opt;
    struct option long_options[] = {
        {"rate",     required_argument, 0, 'r'},
        {"channels", required_argument, 0, 'c'},
        {"file",     required_argument, 0, 'f'},
        {"help",     no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "r:c:f:h", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'r': sampleRate = std::stod(optarg); break;
            case 'c': numChannels = std::stoul(optarg); break;
            case 'f': outputFilename = optarg; break;
            case 'h': print_usage(argv[0]); return 0;
            default:  print_usage(argv[0]); return 1;
        }
    }

    signal(SIGINT, sigIntHandler);

    // Setup Device
    SoapySDR::Kwargs args;
    args["driver"] = "mipi";

    std::cout << "Making device..." << std::endl;
    auto dev = SoapySDR::Device::make(args);
    if (!dev) {
        std::cerr << "Error: Device::make failed" << std::endl;
        return 1;
    }

    dev->setSampleRate(SOAPY_SDR_RX, 0, sampleRate);
    double actualRate = dev->getSampleRate(SOAPY_SDR_RX, 0);
    std::cout << "Actual sample rate: " << actualRate / 1e6 << " MSPS" << std::endl;

    std::vector<size_t> channels;
    for (size_t i = 0; i < numChannels; i++) channels.push_back(i);

    auto stream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS8, channels);
    if (!stream) {
        std::cerr << "Error: setupStream failed" << std::endl;
        return 1;
    }

    size_t mtu = dev->getStreamMTU(stream);
    if (mtu == 0) mtu = 1024 * 1024; 

    // Buffers
    std::vector<std::vector<int8_t>> channelBuffers(numChannels, std::vector<int8_t>(mtu * 2)); 
    std::vector<void*> buffs(numChannels);
    for (size_t i = 0; i < numChannels; i++) buffs[i] = channelBuffers[i].data();

    std::ofstream outFile;
    if (!outputFilename.empty()) {
        outFile.open(outputFilename, std::ios::binary);
        if (!outFile.is_open()) {
            std::cerr << "Error: Could not open output file." << std::endl;
            return 1;
        }
    }

    std::cout << "Stream configured. MTU: " << mtu << " samples. Press Ctrl+C to stop.\n" << std::endl;

    dev->activateStream(stream);

    // Stats
    long long totalSamples = 0;
    long long totalBytes = 0;
    long long overflowCount = 0;
    
    // Timing Stats
    auto startTime = std::chrono::steady_clock::now();
    auto lastReportTime = startTime;
    auto lastReadTime = startTime;
    double maxLatencyUs = 0;

    while (running)
    {
        int flags = 0;
        long long timeNs = 0;
        
        auto before = std::chrono::steady_clock::now();
        int ret = dev->readStream(stream, buffs.data(), mtu, flags, timeNs, 100000);
        auto after = std::chrono::steady_clock::now();

        // Calculate loop latency (jitter)
        std::chrono::duration<double, std::micro> loopDur = after - before;
        double currentLatency = loopDur.count();
        if (currentLatency > maxLatencyUs) maxLatencyUs = currentLatency;

        if (ret > 0) {
            totalSamples += ret;
            totalBytes += (long long)ret * 2 * numChannels;
            if (outFile.is_open()) outFile.write(reinterpret_cast<const char*>(buffs[0]), ret * 2);
        }
        else if (ret == SOAPY_SDR_OVERFLOW) {
            overflowCount++;
        }
        else if (ret != SOAPY_SDR_TIMEOUT) {
            std::cerr << "\nRead Error: " << ret << " " << SoapySDR::errToStr(ret) << std::endl;
            break;
        }

        // Report every 1.0s
        std::chrono::duration<double> diff = after - lastReportTime;
        if (diff.count() >= 1.0) {
            double msps = (double(totalSamples) / diff.count()) / 1e6;
            double mibs = (double(totalBytes) / diff.count()) / (1024.0 * 1024.0);
            
            // Clear line
            printf("\rRate: %6.2f MSPS | BW: %6.2f MiB/s | Ovf: %lld | Jitter: %6.1f us (Max)", 
                   msps, mibs, overflowCount, maxLatencyUs);
            fflush(stdout);

            totalSamples = 0;
            totalBytes = 0;
            maxLatencyUs = 0; // Reset max jitter for next window
            lastReportTime = after;
        }
    }

    std::cout << "\n\nStopping stream..." << std::endl;
    dev->deactivateStream(stream);
    dev->closeStream(stream);
    SoapySDR::Device::unmake(dev);
    return 0;
}