/*
 * Copyright 2017 by Lukas Lao Beyer <lukas@electronics.kitchen>
 *
 * This file is part of libfreesrp.
 *
 * libfreesrp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * libfreesrp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with libfreesrp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "adsdr.hpp"
#include <fstream>
#include <vector>
#include "libusb.h"

#define GET_LSW(v) ((unsigned short)((v) & 0xFFFF))
#define GET_MSW(v) ((unsigned short)((v) >> 16))
#define MAX_WRITE_SIZE (2 * 1024)

using namespace ADSDR;
using namespace std;

void ram_write(libusb_device_handle *fx3_handle, unsigned char *buf, unsigned int ramAddress, int len)
{
    int index = 0;

    while(len > 0)
    {
        int size = (len > MAX_WRITE_SIZE) ? MAX_WRITE_SIZE : len;
        int r = libusb_control_transfer(fx3_handle, 0x40, 0xA0, GET_LSW(ramAddress), GET_MSW(ramAddress), &buf[index], size, ADSDR_USB_TIMEOUT);
        if(r != size)
        {
            throw ConnectionError("FX3 firmware write via libusb control transfer failed: " + std::to_string(r));
        }

        ramAddress += size;
        index += size;
        len -= size;
    }
}

int ADSDR::Util::get_device_count()
{
    libusb_context* ctx = nullptr;
    libusb_device_handle* fx3_handle = nullptr;
    int dev_count = 0;

    int ret = libusb_init(&ctx);
    if(ret < 0)
    {
	cerr << "libusb init error: error " << std::to_string(ret) << endl;
        return ret;
    }

    libusb_device **devs;
    int num_devs = (int) libusb_get_device_list(ctx, &devs);
    if(num_devs < 0)
    {
        cerr << "libusb device list retrieval error" << endl;
        return ret;
    }

    for(int i = 0; i < num_devs; i++)
    {
        libusb_device_descriptor desc;
        int ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0)
        {
            cerr << "libusb error getting device descriptor: error " << std::to_string(ret);
        }

        if(desc.idVendor == FX3_VENDOR_ID && (desc.idProduct == FX3_PRODUCT_BOOT_ID || desc.idProduct == ADSDR_PRODUCT_ID))
		dev_count++;
    }

    return dev_count;
}

int32_t ADSDR::Util::find_fx3() {
    libusb_context *ctx = nullptr;
    int32_t retv = -1;

    try {
        int ret = libusb_init(&ctx);
        if (ret < 0) {
            throw ConnectionError("libusb init error: error " + std::to_string(ret));
        }

        libusb_device **devs;
        int num_devs = (int) libusb_get_device_list(ctx, &devs);
        if (num_devs < 0) {
            throw ConnectionError("libusb device list retrieval error");
        }

        for (int i = 0; i < num_devs; i++) {
            libusb_device_descriptor desc;
            int ret = libusb_get_device_descriptor(devs[i], &desc);
            if (ret < 0) {
                throw ConnectionError("libusb error getting device descriptor: error " + std::to_string(ret));
            }

            if (desc.idVendor == FX3_VENDOR_ID &&
                (desc.idProduct == FX3_PRODUCT_BOOT_ID || desc.idProduct == ADSDR_PRODUCT_ID)) {
                retv = desc.idProduct;
            }
        }
    } catch (runtime_error &e) {
        if(ctx != nullptr)
        {
            libusb_exit(ctx);
        }

        throw e;
    }

    if(ctx != nullptr)
    {
        libusb_exit(ctx);
    }
    return retv;
}

void ADSDR::Util::reset_fx3() {
    libusb_context *ctx = nullptr;
    libusb_device_handle *handle = nullptr;

    try {
        int ret = libusb_init(&ctx);
        if (ret < 0) {
            throw ConnectionError("libusb init error: error " + std::to_string(ret));
        }

        libusb_device **devs;
        int num_devs = (int) libusb_get_device_list(ctx, &devs);
        if (num_devs < 0) {
            throw ConnectionError("libusb device list retrieval error");
        }

        for (int i = 0; i < num_devs; i++) {
            libusb_device_descriptor desc;
            int ret = libusb_get_device_descriptor(devs[i], &desc);
            if (ret < 0) {
                throw ConnectionError("libusb error getting device descriptor: error " + std::to_string(ret));
            }

            if (desc.idVendor == FX3_VENDOR_ID && desc.idProduct == ADSDR_PRODUCT_ID) {
                ret = libusb_open(devs[i], &handle);
                if(ret < 0) {
                    throw ConnectionError("libusb error opening device: error " + std::to_string(ret));
                }
                uint8_t data[3] = {0xFF, 0xFF, 0xFF};
                ret = libusb_control_transfer(handle, LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, 0xB3, 0, 0, data, 3, ADSDR_USB_TIMEOUT);
                if(ret < 0) {
                    throw ConnectionError("libusb error resetting device: error " + std::to_string(ret));
                }
            }
        }
    } catch (runtime_error &e) {
        if(handle != nullptr)
        {
            libusb_release_interface(handle, 0);
            libusb_close(handle);
        }

        if(ctx != nullptr)
        {
            libusb_exit(ctx);
        }

        throw e;
    }

    if(handle != nullptr)
    {
        libusb_release_interface(handle, 0);
        libusb_close(handle);
    }

    if(ctx != nullptr)
    {
        libusb_exit(ctx);
    }
}

void ADSDR::Util::flash_fx3(std::string filename) {
    libusb_context *ctx = nullptr;
    libusb_device_handle *handle = nullptr;

    try {
        int ret = libusb_init(&ctx);
        if (ret < 0) {
            throw ConnectionError("libusb init error: error " + std::to_string(ret));
        }

        libusb_device **devs;
        int num_devs = (int) libusb_get_device_list(ctx, &devs);
        if (num_devs < 0) {
            throw ConnectionError("libusb device list retrieval error");
        }

        for (int i = 0; i < num_devs; i++) {
            libusb_device_descriptor desc;
            int ret = libusb_get_device_descriptor(devs[i], &desc);
            if (ret < 0) {
                throw ConnectionError("libusb error getting device descriptor: error " + std::to_string(ret));
            }

            if (desc.idVendor == FX3_VENDOR_ID &&desc.idProduct == FX3_PRODUCT_BOOT_ID) {
                ret = libusb_open(devs[i], &handle);
                if(ret < 0) {
                    throw ConnectionError("libusb error opening device: error " + std::to_string(ret));
                }
                // Open ifstream for firmware file
                std::ifstream stream;
                stream.exceptions(std::ios::failbit | std::ios::badbit);
                stream.open(filename, std::ios::binary | std::ios::ate);
                std::streamsize size = stream.tellg();
                stream.seekg(0, std::ios::beg);

                // Read firmware into vector
                std::vector<char> firmware_buffer((size_t) size);
                if(!stream.read(firmware_buffer.data(), size))
                {
                    // Could not load file into buffer
                    throw std::runtime_error("Could not load FX3 firmware file into buffer!");
                }

                uint32_t filesize = static_cast<uint32_t>(size);

                // Write firmware to RAM
                uint32_t index = 4;
                uint32_t checksum = 0;
                while(index < filesize)
                {
                    unsigned int *data_p  = (unsigned int *)(firmware_buffer.data() + index);
                    int length = data_p[0];
                    int address = data_p[1];
                    if(length != 0)
                    {
                        for (int i = 0; i < length; i++)
                        {
                            checksum += data_p[2+i];
                        }

                        ram_write(handle, (unsigned char *) firmware_buffer.data() + index + 8, (unsigned int) address, length * 4);
                    }
                    else
                    {
                        if(checksum != data_p[2])
                        {
                            throw std::runtime_error("Checksum error in firmware binary");
                        }
                        int r = libusb_control_transfer(handle, 0x40, 0xA0, GET_LSW(address), GET_MSW(address), 0, 0, ADSDR_USB_TIMEOUT);
                        if(r != 0 && r != -4)
                        {
                            // Ignore this, somehow this error's but still works (??): cerr << "Error in control transfer: " << r << endl;
                        }
                        break;
                    }

                    index += (8 + length * 4);
                }
            }
        }
    } catch (runtime_error &e) {
        if(handle != nullptr)
        {
            libusb_release_interface(handle, 0);
            libusb_close(handle);
        }

        if(ctx != nullptr)
        {
            libusb_exit(ctx);
        }

        throw e;
    }

    if(handle != nullptr)
    {
        libusb_release_interface(handle, 0);
        libusb_close(handle);
    }

    if(ctx != nullptr)
    {
        libusb_exit(ctx);
    }
}

//bool ADSDR::Util::find_fx3(bool upload_firmware, std::string filename, bool reset)
//{
//    // TODO: This is UGLY! Clean up
//
//    bool success = false;
//
//    // Check for FX3
//    libusb_context *ctx = nullptr;
//    libusb_device_handle *fx3_handle = nullptr;
//
//    try
//    {
//        int ret = libusb_init(&ctx);
//        if(ret < 0)
//        {
//            throw ConnectionError("libusb init error: error " + std::to_string(ret));
//        }
//
//        libusb_device **devs;
//        int num_devs = (int) libusb_get_device_list(ctx, &devs);
//        if(num_devs < 0)
//        {
//            throw ConnectionError("libusb device list retrieval error");
//        }
//
//        for(int i = 0; i < num_devs; i++)
//        {
//            libusb_device_descriptor desc;
//            int ret = libusb_get_device_descriptor(devs[i], &desc);
//            if(ret < 0)
//            {
//                throw ConnectionError("libusb error getting device descriptor: error " + std::to_string(ret));
//            }
//
//            if(desc.idVendor == FX3_VENDOR_ID && desc.idProduct == FX3_PRODUCT_BOOT_ID)
//            {
//                if(upload_firmware)
//                {
//                    int ret = libusb_open(devs[i], &fx3_handle);
//                    if(ret != 0)
//                    {
//                        throw ConnectionError("libusb could not open FX3 device: error " + std::to_string(ret));
//                    }
//                }
//                else
//                {
//                    success = true;
//                }
//            }
//        }
//
//        if(upload_firmware && fx3_handle == nullptr)
//        {
//            throw ConnectionError("No Cypress EZ-USB FX3 in bootloader mode found.");
//        }
//        else if(upload_firmware)
//        {
//            // Open ifstream for firmware file
//            std::ifstream stream;
//            stream.exceptions(std::ios::failbit | std::ios::badbit);
//            stream.open(filename, std::ios::binary | std::ios::ate);
//            std::streamsize size = stream.tellg();
//            stream.seekg(0, std::ios::beg);
//
//            // Read firmware into vector
//            std::vector<char> firmware_buffer((size_t) size);
//            if(!stream.read(firmware_buffer.data(), size))
//            {
//                // Could not load file into buffer
//                throw std::runtime_error("Could not load FX3 firmware file into buffer!");
//            }
//
//            uint32_t filesize = static_cast<uint32_t>(size);
//
//            // Write firmware to RAM
//            uint32_t index = 4;
//            uint32_t checksum = 0;
//            while(index < filesize)
//            {
//                unsigned int *data_p  = (unsigned int *)(firmware_buffer.data() + index);
//                int length = data_p[0];
//                int address = data_p[1];
//                if(length != 0)
//                {
//                    for (int i = 0; i < length; i++)
//                    {
//                        checksum += data_p[2+i];
//                    }
//
//                    ram_write(fx3_handle, (unsigned char *) firmware_buffer.data() + index + 8, (unsigned int) address, length * 4);
//                }
//                else
//                {
//                    if(checksum != data_p[2])
//                    {
//                        throw std::runtime_error("Checksum error in firmware binary");
//                    }
//                    int r = libusb_control_transfer(fx3_handle, 0x40, 0xA0, GET_LSW(address), GET_MSW(address), 0, 0, ADSDR_USB_TIMEOUT);
//                    if(r != 0 && r != -4)
//                    {
//                        // Ignore this, somehow this error's but still works (??): cerr << "Error in control transfer: " << r << endl;
//                    }
//                    break;
//                }
//
//                index += (8 + length * 4);
//            }
//
//            success = true;
//        }
//    }
//    catch(const std::runtime_error &e)
//    {
//        if(fx3_handle != nullptr)
//        {
//            libusb_release_interface(fx3_handle, 0);
//            libusb_close(fx3_handle);
//        }
//
//        if(ctx != nullptr)
//        {
//            libusb_exit(ctx);
//        }
//
//        throw e;
//    }
//
//    if(fx3_handle != nullptr)
//    {
//        libusb_release_interface(fx3_handle, 0);
//        libusb_close(fx3_handle);
//    }
//
//    if(ctx != nullptr)
//    {
//        libusb_exit(ctx);
//    }
//
//    return success;
//}
