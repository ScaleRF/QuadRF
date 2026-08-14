#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <cstdint>
#include <string>
#include <csignal>

// g++ -std=c++17 tx_test.cpp -o tx_test -lSoapySDR

bool running = true;
void sigIntHandler(int) {
    running = false;
}

void usage(const char* progName) {
    std::cout << "Usage: " << progName << " [options]\n\n"
              << "Options:\n"
              << "  -r <rate>      Sample rate in Hz (default: 40000000)\n"
              << "  -t <freq>      Tone frequency in Hz (default: 1000000)\n"
              << "  -f <format>    Output format, CS8 or CF32 (default: CS8)\n"
              << "  -n <frames>    Number of frames to send, 0 for infinite (default: 60)\n"
              << "  -h             Show this help message\n";
}

int main(int argc, char** argv)
{
    // Default parameters
    double sampleRate = 40e6;      // 40 Msps
    double toneFreq = 1e6;         // 1 MHz tone
    std::string format = SOAPY_SDR_CS8; 
    int numFrames = 60;            // Number of MTU chunks/frames to send (0 = infinite)
    double amplitude = 0.8;        // 80% of full scale

    // Command line parsing
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-r" && i + 1 < argc) {
            sampleRate = std::stod(argv[++i]);
        } else if (arg == "-t" && i + 1 < argc) {
            toneFreq = std::stod(argv[++i]);
        } else if (arg == "-f" && i + 1 < argc) {
            format = argv[++i];
        } else if (arg == "-n" && i + 1 < argc) {
            numFrames = std::stoi(argv[++i]);
        } else if (arg == "-h") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n\n";
            usage(argv[0]);
            return 1;
        }
    }

    // Register signal handler for clean exit on Ctrl+C
    std::signal(SIGINT, sigIntHandler);

    SoapySDR::Kwargs args;
    args["driver"] = "mipi";

    std::cout << "Initializing MIPI SDR Device...\n";
    auto dev = SoapySDR::Device::make(args);
    if (!dev) { 
        std::cerr << "Device::make failed\n"; 
        return 1; 
    }

    // Configure the Sample Rate 
    dev->setSampleRate(SOAPY_SDR_TX, 0, sampleRate);
    double actualRate = dev->getSampleRate(SOAPY_SDR_TX, 0);
    std::cout << "TX Sample Rate: " << (actualRate / 1e6) << " MHz\n";

    // Setup the TX Stream
    auto stream = dev->setupStream(SOAPY_SDR_TX, format);
    if (!stream) {
        std::cerr << "Failed to setup TX stream with format: " << format << "\n";
        SoapySDR::Device::unmake(dev);
        return 1;
    }

    // Determine optimal buffer size based on the driver's MTU
    size_t mtuElems = dev->getStreamMTU(stream);
    std::cout << "Stream MTU: " << mtuElems << " complex elements\n";

    // Allocate buffers depending on the chosen format
    std::vector<std::complex<int8_t>> buffCS8;
    std::vector<std::complex<float>> buffCF32;
    void* buffs[1] = { nullptr };

    if (format == SOAPY_SDR_CF32) {
        buffCF32.resize(mtuElems);
        buffs[0] = buffCF32.data();
    } else {
        buffCS8.resize(mtuElems);
        buffs[0] = buffCS8.data();
    }

    dev->activateStream(stream);
    std::cout << "Starting Transmission. Sending " 
              << (numFrames > 0 ? std::to_string(numFrames) : "infinite") 
              << " frames...\n";

    // Tone generation variables
    double phase = 0.0;
    // Calculate phase increment: 2 * PI * (f_tone / f_sample)
    double phaseInc = 2.0 * M_PI * toneFreq / actualRate;

    int framesSent = 0;
    while (running && (numFrames <= 0 || framesSent < numFrames))
    {
        // Fill the buffer with the tone
        for (size_t i = 0; i < mtuElems; i++) {
            // Complex tone: e^(j*phase)
            float i_val = amplitude * std::cos(phase);
            float q_val = amplitude * std::sin(phase);

            if (format == SOAPY_SDR_CF32) {
                buffCF32[i] = std::complex<float>(i_val, q_val);
            } else {
                // Scale to int8 range (-127 to +127) for CS8
                buffCS8[i] = std::complex<int8_t>(
                    static_cast<int8_t>(i_val * 127.0f),
                    static_cast<int8_t>(q_val * 127.0f)
                );
            }

            phase += phaseInc;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        }

        int flags = 0;
        // Write stream to SDR
        int ret = dev->writeStream(stream, (const void* const*)buffs, mtuElems, flags, 0 /*timeNs*/, 1000000);
        
        if (ret < 0) {
            std::cerr << "writeStream error ret=" << ret << "\n";
            break;
        } else if (static_cast<size_t>(ret) != mtuElems) {
            std::cerr << "writeStream short write: " << ret << " / " << mtuElems << "\n";
        }

        framesSent++;
    }

    std::cout << "\nCleaning up...\n";
    dev->deactivateStream(stream, 0, 0);
    dev->closeStream(stream);
    SoapySDR::Device::unmake(dev);
    
    std::cout << "Done TX (" << framesSent << " frames sent)\n";
    return 0;
}