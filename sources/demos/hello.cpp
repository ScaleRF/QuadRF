#include <fpga_csi.h>

#include <SoapySDR/Device.hpp>

#include <iostream>

int main() {
    std::cout << "fpga_csi.h OK (CSI_IOC_MAGIC=" << CSI_IOC_MAGIC << ")\n";
    for (const auto& d : SoapySDR::Device::enumerate()) {
        auto it = d.find("driver");
        if (it != d.end())
            std::cout << "SoapySDR driver: " << it->second << "\n";
    }
    return 0;
}
