#include "../include/amr_ethercat.hpp"

namespace EtherCAT
{

    MasterStatus MasterStack::start_master()
    {
        if(setup_master())
        {
            // Enable slaves:
        }
        else
        {
            // return error code indicating fatal error. 

            return MasterStatus::FATAL_ERROR;  
        }

        return MasterStatus::SUCCESS;
    }

    void MasterStack::stop_master()
    {

    }

    // Implementation of Slave super class.
    boolean Slave::checkSlave()
    {
        if(strcmp(ec_slave[m_SlaveNumber].name, m_Name.c_str()))
        {
            return FALSE;
        }
        return TRUE;
    }        
    /**
     * @brief Implementation of MasterStack class.
     * 
     * @param ifname 
     */
    MasterStack::MasterStack(const std::string& ifname, const std::vector<std::shared_ptr<Slave>> slaves, bool use_DC)
        : m_UseDC{use_DC}
    {
        m_ifname = ifname;
        m_Slaves = slaves;
        
    }
    boolean MasterStack::setup_master()
    {
        if(ec_init(m_ifname.c_str()))
        {
            if(ec_config_init(m_ConfigUseTable) > 0)
            {
                if(!check_slaves())
                {
                    return FALSE;
                }
                int usedmem = ec_config_map(&m_IOMap);

                if(usedmem <= sizeof(m_IOMap))
                {
                    // Not sure:
                    // return FALSE;
                }

                if(m_UseDC)
                {
                    ec_configdc();
                }
                ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

                m_ExpectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

                /* send one valid process data to make outputs in slaves happy*/
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_writestate(0);
                
                while(m_CHK-- && (ec_slave[0].state != EC_STATE_OPERATIONAL))
                {
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
                }

                if(ec_slave[0].state == EC_STATE_OPERATIONAL)
                {
                    
                    m_PDO_OperationalTransferActive = TRUE;
                    return TRUE;
                }
                else
                {
                    
                }
            }
            else
            {
                std::cout << "No slaves found" << std::endl;
            }   
        }
        else
        {
            // No socket connection.
        }

        return FALSE;
    }
    boolean MasterStack::check_slaves()
    {
        if(ec_slavecount < m_NumberOfSlaves)
        {
            return FALSE;
        }
        for(std::size_t i = 0; i < m_Slaves.size(); i++)
        {
            if(m_Slaves[i]->checkSlave())
            {
                std::cout << "Slave " << m_Slaves[i]->getSlaveNumber() << " " << m_Slaves[i]->getSlaveName() << "is OK." << std::endl;
                continue;
            }
            else
            {
                return FALSE;
            }
        }
        return TRUE;
    }

    OSAL_THREAD_FUNC MasterStack::check_state(void* ptr)
    {
        int slave;
        (void)ptr;

        uint8_t currentGroup = 0;

        while(true)
        {
            if(m_PDO_OperationalTransferActive && ((m_WKC < m_ExpectedWKC) || ec_group[currentGroup].docheckstate))
            {
                ec_group[currentGroup].docheckstate = FALSE;

                ec_readstate();

                for(slave = 1; slave <= ec_slavecount; slave++)
                {
                    if(ec_slave[slave].group == currentGroup && ec_slave[slave].state != EC_STATE_OPERATIONAL)
                    {
                        ec_group[currentGroup].docheckstate = TRUE;

                        if(ec_slave[])
                    }
                }
            }
        }
    }
}
