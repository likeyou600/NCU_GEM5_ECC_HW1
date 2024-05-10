/*
 * Copyright (c) 2017 Jason Lowe-Power
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ncu_lab1_src/ecc_memobj.hh"

#include "base/trace.hh"
#include "debug/ECCMemobj.hh"
#include <chrono>
#include <random>
namespace gem5
{
    int i = 0;
    int j = 0;
    ECCMemobj::ECCMemobj(const ECCMemobjParams& params) :
        SimObject(params),
        instPort(params.name + ".inst_port", this),
        dataPort(params.name + ".data_port", this),
        memPort(params.name + ".mem_side", this),
        blocked(false)
    {
    }

    Port&
        ECCMemobj::getPort(const std::string& if_name, PortID idx)
    {
        panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

        // This is the name from the Python SimObject declaration (ECCMemobj.py)
        if (if_name == "mem_side") {
            return memPort;
        }
        else if (if_name == "inst_port") {
            return instPort;
        }
        else if (if_name == "data_port") {
            return dataPort;
        }
        else {
            // pass it along to our super class
            return SimObject::getPort(if_name, idx);
        }
    }

    void
        ECCMemobj::CPUSidePort::sendPacket(PacketPtr pkt)
    {
        // Note: This flow control is very simple since the memobj is blocking.

        panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

        // If we can't send the packet across the port, store it for later.
        if (!sendTimingResp(pkt)) {
            blockedPacket = pkt;
        }
    }

    AddrRangeList
        ECCMemobj::CPUSidePort::getAddrRanges() const
    {
        return owner->getAddrRanges();
    }

    void
        ECCMemobj::CPUSidePort::trySendRetry()
    {
        if (needRetry && blockedPacket == nullptr) {
            // Only send a retry if the port is now completely free
            needRetry = false;
            DPRINTF(ECCMemobj, "Sending retry req for %d\n", id);
            sendRetryReq();
        }
    }

    void
        ECCMemobj::CPUSidePort::recvFunctional(PacketPtr pkt)
    {
        // Just forward to the memobj.
        return owner->handleFunctional(pkt);
    }

    bool
        ECCMemobj::CPUSidePort::recvTimingReq(PacketPtr pkt)
    {
        // Just forward to the memobj.
        if (!owner->handleRequest(pkt)) {
            needRetry = true;
            return false;
        }
        else {
            return true;
        }
    }

    void
        ECCMemobj::CPUSidePort::recvRespRetry()
    {
        // We should have a blocked packet if this function is called.
        assert(blockedPacket != nullptr);

        // Grab the blocked packet.
        PacketPtr pkt = blockedPacket;
        blockedPacket = nullptr;

        // Try to resend it. It's possible that it fails again.
        sendPacket(pkt);
    }

    void
        ECCMemobj::MemSidePort::sendPacket(PacketPtr pkt)
    {
        // Note: This flow control is very simple since the memobj is blocking.

        panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

        // If we can't send the packet across the port, store it for later.
        if (!sendTimingReq(pkt)) {
            blockedPacket = pkt;
        }
    }

    bool
        ECCMemobj::MemSidePort::recvTimingResp(PacketPtr pkt)
    {
        // Just forward to the memobj.
        return owner->handleResponse(pkt);
    }

    void
        ECCMemobj::MemSidePort::recvReqRetry()
    {
        // We should have a blocked packet if this function is called.
        assert(blockedPacket != nullptr);

        // Grab the blocked packet.
        PacketPtr pkt = blockedPacket;
        blockedPacket = nullptr;

        // Try to resend it. It's possible that it fails again.
        sendPacket(pkt);
    }

    void
        ECCMemobj::MemSidePort::recvRangeChange()
    {
        owner->sendRangeChange();
    }

    bool
        ECCMemobj::handleRequest(PacketPtr pkt)
    {
        if (blocked) {
            // There is currently an outstanding request. Stall.
            return false;
        }
        //----------------------------------------------------------------Lab1 cpu->mem修改區域--------------------------------------------------------------------

        if (pkt->isWrite()) {

            //pkt基本資訊---------------------------------------------------------------------------------------------
            DPRINTF(ECCMemobj, "cpu->mem Got request for addr %#x\n", pkt->getAddr());
            DPRINTF(ECCMemobj, "size %#x\n", pkt->getSize());
            uint8_t* data = pkt->getPtr<uint8_t>();
            size_t size = pkt->getSize();
            int n_bits_data = sizeof(uint8_t) * size;
            int m_bits_parity = static_cast<int>(std::log2(n_bits_data) + 1);

            //pkt基本資訊---------------------------------------------------------------------------------------------


            //輸出hex_data---------------------------------------------------------------------------------------------
            std::string output_hex;
            for (size_t i = 0; i < size; i++) {
                char hexByte[3];
                snprintf(hexByte, sizeof(hexByte), "%02X", data[i]);
                output_hex += hexByte;
            }
            DPRINTF(ECCMemobj, "data_hex : %s \n", output_hex.c_str());
            //輸出hex_data----------------------------------------------------------------------------------------------



            //輸出binary_data---------------------------------------------------------------------------------------------
            size_t index = 0;
            std::vector<int> data_vector(8 * size);
            for (size_t i = 0; i < size; i++) {
                std::bitset<8> binary(data[i]);
                for (int j = 7; j >= 0; j--) { // 從高位到低位逐位複製
                    data_vector[index++] = static_cast<int>(binary[j]); // 將二進制位轉換為字符
                }
            }
            DPRINTF(ECCMemobj, "data : %s \n", vector_int_to_string(data_vector).c_str());
            //輸出binary_data---------------------------------------------------------------------------------------------


            //產生hamming code---------------------------------------------------------------------------------------------

            int parity = generateHammingCode(data_vector, n_bits_data, m_bits_parity);
            hammingCodeMap[pkt->getAddr()].first = parity;
            hammingCodeMap[pkt->getAddr()].second = size;
            DPRINTF(ECCMemobj, "hamming code : %d \n", parity);
            //產生hamming code---------------------------------------------------------------------------------------------


            //反轉某一個bit 存入Pkt---------------------------------------------------------------------------------------------
            if (error_time % 10 == 0) {
                std::random_device rd;
                std::mt19937 gen(rd());

                auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                gen.seed(seed);


                int min = 0;
                int max = n_bits_data - 1;

                int x = rand() % (max - min + 1) + min;

                DPRINTF(ECCMemobj, "Enter Error Bit index: %d \n", x + 1);

                size_t byte_index = x / 8;
                int bit_index = x % 8;

                data[byte_index] ^= (1 << 7 - bit_index);

                //輸出flip_binary_data---------------------------------------------------------------------------------------------
                index = 0;
                data_vector.resize(8 * size, 0);
                for (size_t i = 0; i < size; i++) {
                    std::bitset<8> binary(data[i]);
                    for (int j = 7; j >= 0; j--) { // 從高位到低位逐位複製
                        data_vector[index++] = static_cast<int>(binary[j]); // 將二進制位轉換為字符
                    }
                }
                DPRINTF(ECCMemobj, "flip data : %s \n", vector_int_to_string(data_vector).c_str());
                //輸出flip_binary_data---------------------------------------------------------------------------------------------

            }
            ++error_time;
            //反轉某一個bit 存入Pkt---------------------------------------------------------------------------------------------



            DPRINTF(ECCMemobj, "-------------------------------------\n");
        }



        //----------------------------------------------------------------Lab1 cpu->mem修改區域--------------------------------------------------------------------

        // This memobj is now blocked waiting for the response to this packet.
        blocked = true;

        // Simply forward to the memory port
        memPort.sendPacket(pkt);

        return true;
    }

    bool
        ECCMemobj::handleResponse(PacketPtr pkt)
    {
        assert(blocked);
        //----------------------------------------------------------------Lab1 mem->cpu修改區域--------------------------------------------------------------------

        if (pkt->isRead() &&
            hammingCodeMap.find(pkt->getAddr()) != hammingCodeMap.end() &&
            hammingCodeMap[pkt->getAddr()].second == pkt->getSize())
        {

            //pkt基本資訊---------------------------------------------------------------------------------------------
            DPRINTF(ECCMemobj, "mem->cpu Got response for addr %#x\n", pkt->getAddr());
            DPRINTF(ECCMemobj, "size %#x\n", pkt->getSize());
            uint8_t* data = pkt->getPtr<uint8_t>();
            size_t size = pkt->getSize();
            int n_bits_data = sizeof(uint8_t) * size;
            int m_bits_parity = static_cast<int>(std::log2(n_bits_data) + 1);
            //pkt基本資訊---------------------------------------------------------------------------------------------

            //map 找對應的hamming code---------------------------------------------------------------------------------------------
            int out_parity = hammingCodeMap[pkt->getAddr()].first;
            DPRINTF(ECCMemobj, "hamming code out: %d \n", out_parity);
            //map 找對應的hamming code---------------------------------------------------------------------------------------------


            //輸出hex_data----------------------------------------------------------------------------------------------

            std::string output_hex;
            for (size_t i = 0; i < size; i++) {
                char hexByte[3];
                snprintf(hexByte, sizeof(hexByte), "%02X", data[i]);
                output_hex += hexByte;
            }
            DPRINTF(ECCMemobj, "data_hex : %s \n", output_hex.c_str());
            //輸出hex_data----------------------------------------------------------------------------------------------


            //輸出binary_data---------------------------------------------------------------------------------------------
            size_t index = 0;
            std::vector<int> data_vector(8 * size);
            for (size_t i = 0; i < size; i++) {
                std::bitset<8> binary(data[i]);
                for (int j = 7; j >= 0; j--) { // 從高位到低位逐位複製
                    data_vector[index++] = static_cast<int>(binary[j]); // 將二進制位轉換為字符
                }
            }
            DPRINTF(ECCMemobj, "data : %s \n", vector_int_to_string(data_vector).c_str());
            //輸出binary_data---------------------------------------------------------------------------------------------



            //尋找錯誤位置並修正data_vector---------------------------------------------------------------------------------------------
            ECC_fix(data_vector, out_parity, n_bits_data, m_bits_parity);
            //尋找錯誤位置並修正data_vector---------------------------------------------------------------------------------------------

            //把修正後的再次設定進Pkt---------------------------------------------------------------------------------------------
            int s = pkt->getSize();
            int idx = 0;
            int output = 0;
            unsigned char c;
            for (int i = 0; i < s; ++i) {
                for (int j = 7; j >= 0; --j) {
                    if (data_vector[idx]) {
                        output += 1 << j;
                    }
                    ++idx;
                }
                //cout<<output<<'\n';
                c = (unsigned char)output;
                output = 0;
                data[i] = c;
                //printf("%x\n", c);
            }

            //輸出fix_binary_data---------------------------------------------------------------------------------------------
            index = 0;
            data_vector.resize(8 * size, 0);
            for (size_t i = 0; i < size; i++) {
                std::bitset<8> binary(data[i]);
                for (int j = 7; j >= 0; j--) { // 從高位到低位逐位複製
                    data_vector[index++] = static_cast<int>(binary[j]); // 將二進制位轉換為字符
                }
            }
            DPRINTF(ECCMemobj, "fix data : %s \n", vector_int_to_string(data_vector).c_str());
            //輸出fix_binary_data---------------------------------------------------------------------------------------------

            //把修正後的再次設定進Pkt---------------------------------------------------------------------------------------------


            DPRINTF(ECCMemobj, "-------------------------------------\n");
        }








        //----------------------------------------------------------------Lab1 mem->cpu修改區域--------------------------------------------------------------------


        // The packet is now done. We're about to put it in the port, no need for
        // this object to continue to stall.
        // We need to free the resource before sending the packet in case the CPU
        // tries to send another request immediately (e.g., in the same callchain).
        blocked = false;

        // Simply forward to the memory port
        if (pkt->req->isInstFetch()) {
            instPort.sendPacket(pkt);
        }
        else {
            dataPort.sendPacket(pkt);
        }

        // For each of the cpu ports, if it needs to send a retry, it should do it
        // now since this memory object may be unblocked now.
        instPort.trySendRetry();
        dataPort.trySendRetry();

        return true;
    }

    void
        ECCMemobj::handleFunctional(PacketPtr pkt)
    {
        // Just pass this on to the memory side to handle for now.
        memPort.sendFunctional(pkt);
    }

    AddrRangeList
        ECCMemobj::getAddrRanges() const
    {
        DPRINTF(ECCMemobj, "Sending new ranges\n");
        // Just use the same ranges as whatever is on the memory side.
        return memPort.getAddrRanges();
    }

    void
        ECCMemobj::sendRangeChange()
    {
        instPort.sendRangeChange();
        dataPort.sendRangeChange();
    }


    int ECCMemobj::generateHammingCode(std::vector<int>& input, int n_bits_data, int m_bits_parity) {
        int parity = 0, idx = 0, cnt = 0; //idx為input的index //i為input+parity位數
        for (int i = 1; i <= n_bits_data + m_bits_parity; ++i) {
            if (i == pow(2, cnt)) {
                ++cnt;
                continue;
            }
            else if (input[idx]) {
                parity ^= i;
                ++idx;
            }
            else
                ++idx;
        }
        return parity;
    }


    void ECCMemobj::ECC_fix(std::vector<int>& input, int parity, int n_bits_data, int m_bits_parity) {
        int combine[n_bits_data + m_bits_parity] = { 0 }, cnt = 0, idx = 0, input_parity = 0;
        for (int i = 1; i <= n_bits_data + m_bits_parity; ++i) {
            if (i == pow(2, cnt)) {
                ++cnt;
                continue;
            }
            else if (input[idx]) {
                input_parity ^= i;
            }

            combine[i] = idx; //conbine只是記input 的 index
            ++idx;

        }

        input_parity ^= parity;
        if (input_parity != 0) {
            DPRINTF(ECCMemobj, "find Error Bit index : %d \n", combine[input_parity] + 1); //從1開始看
            input[combine[input_parity]] ^= 1; //利用combine vector 找回原本input index
        }
    }
    std::string ECCMemobj::vector_int_to_string(std::vector<int>& vector_int) {
        std::string output;
        for (auto i : vector_int) {
            output += std::to_string(i);
        }

        return output;
    }
} // namespace gem5
