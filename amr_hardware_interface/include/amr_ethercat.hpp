#ifndef AMR_ETHERCAT_HPP
#define AMR_ETHERCAT_HPP

#include <soem/ethercat.h>
#include <soem/osal.h>
#include <soem/oshw.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <memory>
#include <exception>

#include "ethercat_exception.hpp"
#include "ethercat_utils.hpp"

namespace EtherCAT
{
    class Slave
    {   
        public:
        Slave(ec_slavet* slave, int slave_num, const std::string& name);
        boolean checkSlave();
        inline int getSlaveNumber() const
        {
            return m_SlaveNumber;
        }
        inline std::string getSlaveName() const
        {
            return m_Name;
        }
        private:
        int m_SlaveNumber;
        std::string m_Name;
    };
    class MasterStack
    {
        public:
        MasterStack();
        // Constructs the Master Stack with the ethernet port "ifname".
        // The EtherCAT slaves are shared with the Master as shared_ptrs.
        MasterStack(const std::string& ifname, const std::vector<std::shared_ptr<Slave>> slaves, bool use_DC = false);

        // Setups the EtherCAT master.
        // Returns TRUE if all checks are passed, else returns FALSE;
        boolean setup_master();

        // 
        MasterStatus start_master();
        
        // Stops active transfer and closes the ec Struct.
        void stop_master();

        private:
        // Ethernet File Name to open with SOEM
        std::string m_ifname;
        
        char m_IOMap[4096];
        // Use table with ec_config_init()
        boolean m_ConfigUseTable = FALSE;
        std::vector<std::shared_ptr<Slave>> m_Slaves;
        
        // Number of slaves, equals to m_Slaves.size();
        int m_NumberOfSlaves;

        // 
        volatile int m_WKC;
        volatile int m_ExpectedWKC;

        //
        int m_CHK = 40;

        boolean m_PDO_OperationalTransferActive = false;

        bool m_UseDC = false;

        // Check each slave in the m_Slaves vector for network config.
        boolean check_slaves();

        void* pdo_transfer(void* ptr);

        // ecat_check in the SOEM Examples.
        //
        OSAL_THREAD_FUNC check_state(void *ptr); 
        
    };
    
}


#endif